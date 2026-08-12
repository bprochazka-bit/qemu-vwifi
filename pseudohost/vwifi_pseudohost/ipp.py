#
# vwifi-pseudohost — IPP (Internet Printing Protocol) server
#
# SPDX-License-Identifier: GPL-2.0-or-later
#
# How modern Windows (and macOS, and CUPS) discover and print to a
# printer without a vendor driver: "IPP Everywhere" (driverless).  The
# printer advertises _ipp._tcp over mDNS and serves IPP on TCP 631; the
# client sends Get-Printer-Attributes to learn what it is, then Print-Job
# (or Create-Job + Send-Document) with the document in a self-describing
# format (PDF or PWG-Raster).  A packet capture of a real HP OfficeJet
# showed exactly this path — mDNS + IPP, not WSD — so this is what makes
# the simulated printer addable and printable the same way.
#
# IPP is a binary protocol carried in an HTTP POST (RFC 8010/8011, plus
# the IPP Everywhere attribute set from PWG 5100.14).  This implements
# the operations a client uses to add and print: Get-Printer-Attributes,
# Validate-Job, Print-Job, Create-Job and Send-Document; the document
# bytes are handed to the host's print sink untouched.
#
import struct

from .services import TCPService

# delimiter tags
TAG_OPERATION = 0x01
TAG_JOB = 0x02
TAG_END = 0x03
TAG_PRINTER = 0x04
TAG_UNSUPPORTED = 0x05
# value tags
TAG_INTEGER = 0x21
TAG_BOOLEAN = 0x22
TAG_ENUM = 0x23
TAG_OCTETSTR = 0x30
TAG_RESOLUTION = 0x32
TAG_RANGE = 0x33
TAG_TEXT = 0x41
TAG_NAME = 0x42
TAG_KEYWORD = 0x44
TAG_URI = 0x45
TAG_CHARSET = 0x47
TAG_LANGUAGE = 0x48
TAG_MIMETYPE = 0x49

# operation ids
OP_PRINT_JOB = 0x0002
OP_VALIDATE_JOB = 0x0004
OP_CREATE_JOB = 0x0005
OP_SEND_DOCUMENT = 0x0006
OP_GET_JOB_ATTRS = 0x0009
OP_GET_JOBS = 0x000A
OP_GET_PRINTER_ATTRS = 0x000B

# status codes
OK = 0x0000
SERVER_ERROR_OPERATION_NOT_SUPPORTED = 0x0501


def _attr(tag, name, value):
    """One IPP attribute (single value)."""
    if isinstance(name, str):
        name = name.encode()
    return (bytes([tag]) + struct.pack(">H", len(name)) + name
            + struct.pack(">H", len(value)) + value)


def _addl(tag, value):
    """An additional value for the preceding (multi-valued) attribute."""
    return bytes([tag]) + struct.pack(">H", 0) + struct.pack(">H", len(value)) \
        + value


def _int(n):
    return struct.pack(">i", n)


def _bool(b):
    return bytes([1 if b else 0])


def _res(x, y, unit=3):        # 3 = dots per inch
    return struct.pack(">iiB", x, y, unit)


def _keywords(tag, name, values):
    """A multi-valued keyword/mimetype/etc. attribute."""
    vs = [v.encode() if isinstance(v, str) else v for v in values]
    out = _attr(tag, name, vs[0])
    for v in vs[1:]:
        out += _addl(tag, v)
    return out


def _enums(name, values):
    out = _attr(TAG_ENUM, name, _int(values[0]))
    for v in values[1:]:
        out += _addl(TAG_ENUM, _int(v))
    return out


class IPPService(TCPService):
    """An IPP printer endpoint on TCP 631."""

    name = "ipp"
    tcp_ports = (631,)

    def __init__(self):
        super().__init__()
        self._job = 0

    def on_connect(self, conn):
        conn.data["buf"] = bytearray()
        conn.data["expect_sent"] = False

    # -- HTTP framing ------------------------------------------------------
    def on_data(self, conn, data):
        buf = conn.data["buf"]
        buf += data
        he = buf.find(b"\r\n\r\n")
        if he < 0:
            return
        headers = bytes(buf[:he]).decode("latin1", "replace")
        low = headers.lower()
        # Honour Expect: 100-continue, or the client waits forever.
        if "100-continue" in low and not conn.data["expect_sent"]:
            conn.data["expect_sent"] = True
            conn.send(b"HTTP/1.1 100 Continue\r\n\r\n")

        body_start = he + 4
        if "transfer-encoding: chunked" in low:
            body = _dechunk(bytes(buf[body_start:]))
            if body is None:
                return                          # need more chunks
        else:
            clen = _header_int(low, "content-length")
            if clen is None:
                return
            if len(buf) - body_start < clen:
                return                          # need more body
            body = bytes(buf[body_start:body_start + clen])

        resp = self._handle_ipp(body)
        http = (b"HTTP/1.1 200 OK\r\n"
                b"Content-Type: application/ipp\r\n"
                b"Content-Length: " + str(len(resp)).encode() + b"\r\n"
                b"Connection: close\r\n\r\n" + resp)
        conn.send(http)
        conn.close()

    # -- IPP ---------------------------------------------------------------
    def _handle_ipp(self, body):
        if len(body) < 8:
            return self._resp(OK, 0, b"")
        op = struct.unpack_from(">H", body, 2)[0]
        reqid = struct.unpack_from(">I", body, 4)[0]
        doc = _document_after_attrs(body)

        if op == OP_GET_PRINTER_ATTRS:
            self.log("IPP Get-Printer-Attributes")
            return self._resp(OK, reqid, self._printer_attrs())
        if op == OP_VALIDATE_JOB:
            return self._resp(OK, reqid, self._job_attrs(self._job or 1))
        if op == OP_PRINT_JOB:
            self._job += 1
            self._sink(doc, "ipp-job%d" % self._job)
            return self._resp(OK, reqid, self._job_attrs(self._job))
        if op == OP_CREATE_JOB:
            self._job += 1
            return self._resp(OK, reqid, self._job_attrs(self._job))
        if op == OP_SEND_DOCUMENT:
            self._sink(doc, "ipp-job%d" % (self._job or 1))
            return self._resp(OK, reqid, self._job_attrs(self._job or 1))
        if op in (OP_GET_JOBS, OP_GET_JOB_ATTRS):
            return self._resp(OK, reqid, b"")
        return self._resp(SERVER_ERROR_OPERATION_NOT_SUPPORTED, reqid, b"")

    def _sink(self, doc, jobname):
        sink = getattr(self.host, "print_sink", None)
        if sink is not None:
            sink.write_job(doc or b"", jobname=jobname, source="ipp")
        elif doc:
            self.log("received %d-byte job but no print sink" % len(doc))

    def _resp(self, status, reqid, printer_group):
        out = struct.pack(">HHI", 0x0200, status, reqid)      # IPP 2.0
        out += bytes([TAG_OPERATION])
        out += _attr(TAG_CHARSET, "attributes-charset", b"utf-8")
        out += _attr(TAG_LANGUAGE, "attributes-natural-language", b"en")
        out += printer_group
        out += bytes([TAG_END])
        return out

    def _printer_uri(self):
        ip = ".".join(str(x) for x in (self.host.stack.ip or bytes(4)))
        return ("ipp://%s:631/ipp/print" % ip).encode()

    def _printer_attrs(self):
        h = self.host
        model = getattr(h, "wsd_model", getattr(h, "hostname", "Printer"))
        name = getattr(h, "hostname", "Printer")
        uri = self._printer_uri()
        uuid = "urn:uuid:%s" % _uuid(h.mac)
        g = bytes([TAG_PRINTER])
        g += _attr(TAG_URI, "printer-uri-supported", uri)
        g += _attr(TAG_KEYWORD, "uri-authentication-supported", b"none")
        g += _attr(TAG_KEYWORD, "uri-security-supported", b"none")
        g += _attr(TAG_NAME, "printer-name", name.encode())
        g += _attr(TAG_TEXT, "printer-info", model.encode())
        g += _attr(TAG_TEXT, "printer-make-and-model", model.encode())
        g += _attr(TAG_TEXT, "printer-location", b"Front Office")
        g += _attr(TAG_URI, "printer-uuid", uuid.encode())
        g += _attr(TAG_ENUM, "printer-state", _int(3))         # idle
        g += _attr(TAG_KEYWORD, "printer-state-reasons", b"none")
        g += _attr(TAG_BOOLEAN, "printer-is-accepting-jobs", _bool(True))
        g += _attr(TAG_INTEGER, "queued-job-count", _int(0))
        g += _attr(TAG_INTEGER, "printer-up-time", _int(1))
        g += _attr(TAG_BOOLEAN, "color-supported", _bool(True))
        g += _keywords(TAG_KEYWORD, "ipp-versions-supported",
                       ["1.1", "2.0"])
        g += _keywords(TAG_KEYWORD, "ipp-features-supported",
                       ["ipp-everywhere"])
        g += _enums("operations-supported",
                    [OP_PRINT_JOB, OP_VALIDATE_JOB, OP_CREATE_JOB,
                     OP_SEND_DOCUMENT, OP_GET_JOB_ATTRS, OP_GET_JOBS,
                     OP_GET_PRINTER_ATTRS])
        g += _attr(TAG_CHARSET, "charset-configured", b"utf-8")
        g += _attr(TAG_CHARSET, "charset-supported", b"utf-8")
        g += _attr(TAG_LANGUAGE, "natural-language-configured", b"en")
        g += _attr(TAG_LANGUAGE, "generated-natural-language-supported", b"en")
        g += _attr(TAG_MIMETYPE, "document-format-default",
                   b"application/octet-stream")
        g += _keywords(TAG_MIMETYPE, "document-format-supported",
                       ["application/pdf", "image/pwg-raster", "image/urf",
                        "image/jpeg", "application/octet-stream"])
        g += _keywords(TAG_KEYWORD, "compression-supported", ["none"])
        g += _attr(TAG_BOOLEAN, "printer-is-shared", _bool(True))
        # media
        g += _keywords(TAG_KEYWORD, "media-supported",
                       ["iso_a4_210x297mm", "na_letter_8.5x11in"])
        g += _attr(TAG_KEYWORD, "media-default", b"na_letter_8.5x11in")
        g += _keywords(TAG_KEYWORD, "media-ready",
                       ["na_letter_8.5x11in"])
        # sides / color / quality / resolution
        g += _keywords(TAG_KEYWORD, "sides-supported",
                       ["one-sided", "two-sided-long-edge",
                        "two-sided-short-edge"])
        g += _attr(TAG_KEYWORD, "sides-default", b"one-sided")
        g += _keywords(TAG_KEYWORD, "print-color-mode-supported",
                       ["auto", "color", "monochrome"])
        g += _attr(TAG_KEYWORD, "print-color-mode-default", b"auto")
        g += _attr(TAG_RESOLUTION, "printer-resolution-default", _res(600, 600))
        g += _attr(TAG_RESOLUTION, "printer-resolution-supported",
                   _res(600, 600))
        g += _enums("print-quality-supported", [3, 4, 5])
        g += _attr(TAG_ENUM, "print-quality-default", _int(4))
        g += _enums("finishings-supported", [3])               # none
        g += _attr(TAG_ENUM, "finishings-default", _int(3))
        g += _attr(TAG_BOOLEAN, "multiple-document-jobs-supported",
                   _bool(False))
        g += _keywords(TAG_KEYWORD, "job-creation-attributes-supported",
                       ["copies", "sides", "print-color-mode", "media",
                        "printer-resolution", "print-quality"])
        # PWG-Raster / URF descriptors (IPP Everywhere)
        g += _attr(TAG_RESOLUTION, "pwg-raster-document-resolution-supported",
                   _res(600, 600))
        g += _attr(TAG_KEYWORD, "pwg-raster-document-sheet-back", b"normal")
        g += _keywords(TAG_KEYWORD, "pwg-raster-document-type-supported",
                       ["srgb_8", "sgray_8"])
        g += _keywords(TAG_KEYWORD, "urf-supported",
                       ["CP1", "RS600", "SRGB24", "W8", "V1.4", "DM1"])
        g += _attr(TAG_TEXT, "printer-device-id",
                   ("MFG:HP;MDL:%s;CMD:PDF,PWGRaster,URF;CLS:PRINTER;"
                    % model).encode())
        g += _keywords(TAG_KEYWORD, "print-content-optimize-supported",
                       ["auto"])
        g += _keywords(TAG_KEYWORD, "identify-actions-supported", ["display"])
        return g

    def _job_attrs(self, job_id):
        g = bytes([TAG_JOB])
        g += _attr(TAG_INTEGER, "job-id", _int(job_id))
        g += _attr(TAG_URI, "job-uri",
                   (self._printer_uri().decode() + "/%d" % job_id).encode())
        g += _attr(TAG_ENUM, "job-state", _int(9))             # completed
        g += _keywords(TAG_KEYWORD, "job-state-reasons", ["job-completed"])
        return g


# ---- helpers -------------------------------------------------------------
def _header_int(low_headers, name):
    import re
    m = re.search(r"%s:\s*(\d+)" % name, low_headers)
    return int(m.group(1)) if m else None


def _dechunk(body):
    out = bytearray()
    i = 0
    while i < len(body):
        j = body.find(b"\r\n", i)
        if j < 0:
            return None                         # incomplete size line
        try:
            size = int(body[i:j].split(b";")[0], 16)
        except ValueError:
            return None
        if size == 0:
            return bytes(out)                   # last chunk
        start = j + 2
        if start + size + 2 > len(body):
            return None                         # incomplete chunk data
        out += body[start:start + size]
        i = start + size + 2
    return None                                 # no terminating 0-chunk yet


def _document_after_attrs(body):
    """Return the document bytes that follow the end-of-attributes tag."""
    i = 8                                       # skip version/op/request-id
    n = len(body)
    while i < n:
        tag = body[i]
        if tag == TAG_END:
            return body[i + 1:]
        if tag <= 0x05:                         # delimiter tag: no value
            i += 1
            continue
        # value attribute: name-len, name, value-len, value
        if i + 3 > n:
            break
        nlen = struct.unpack_from(">H", body, i + 1)[0]
        i += 3 + nlen
        if i + 2 > n:
            break
        vlen = struct.unpack_from(">H", body, i)[0]
        i += 2 + vlen
    return b""


def _uuid(mac):
    h = mac.hex()
    return "%s-%s-1000-8000-%s" % (h[:8], h[8:12], h)
