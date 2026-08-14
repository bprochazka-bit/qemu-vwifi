#
# vwifi-pseudohost — eSCL (AirScan / Mopria) driverless scan server
#
# SPDX-License-Identifier: GPL-2.0-or-later
#
# eSCL is the standard, driverless network-scan protocol (Apple AirScan /
# Mopria). A device advertises _uscan._tcp over mDNS with a TXT "rs=eSCL"
# resource path, and exposes a small REST/XML API under /eSCL:
#
#   GET  /eSCL/ScannerCapabilities   -> what the scanner can do
#   GET  /eSCL/ScannerStatus         -> Idle / Processing + job states
#   POST /eSCL/ScanJobs              -> start a job; 201 + Location: .../<id>
#   GET  /eSCL/ScanJobs/<id>/NextDocument -> the page image; 404 when done
#
# The real HP OfficeJet advertises _uscan._tcp too, but the capture shows
# its own driver scanning over a *proprietary* HP REST API — which inbox
# clients cannot drive. eSCL is the protocol a driverless client (Windows
# 11's inbox scan, sane-airscan, Mopria) actually speaks, so that is what
# the simulator implements: a real, standards-based scan a generic client
# can complete, returning a one-page PDF of a fixed raster.
#
import re
import zlib

from .services import TCPService

# A US-Letter page at a modest resolution; the returned PDF reports this
# page size. Kept small so the deflated raster is a few KB over the medium.
SCAN_DPI = 200
PAGE_W_PX = int(8.5 * SCAN_DPI)
PAGE_H_PX = int(11 * SCAN_DPI)

NS_SCAN = "http://schemas.hp.com/imaging/escl/2011/05/03"
NS_PWG = "http://www.pwg.org/schemas/2010/12/sm"


def make_pdf(px_w, px_h, dpi=SCAN_DPI, rgb=(228, 228, 228)):
    """A one-page PDF of a solid-colour raster — the pseudo-host's 'scan'.

    The embedded image is a small raster scaled to fill a page whose size
    is px_w/px_h at `dpi`, so the file stays tiny regardless of page size.
    Stdlib only (zlib for FlateDecode)."""
    iw, ih = min(px_w, 128), min(px_h, 166)
    img = zlib.compress((bytes(rgb) * iw) * ih, 6)
    pts_w = px_w * 72.0 / dpi
    pts_h = px_h * 72.0 / dpi
    content = ("q %.2f 0 0 %.2f 0 0 cm /Im0 Do Q" % (pts_w, pts_h)).encode()
    objs = [
        b"<</Type/Catalog/Pages 2 0 R>>",
        b"<</Type/Pages/Kids[3 0 R]/Count 1>>",
        ("<</Type/Page/Parent 2 0 R/MediaBox[0 0 %.2f %.2f]"
         "/Resources<</XObject<</Im0 4 0 R>>>>/Contents 5 0 R>>"
         % (pts_w, pts_h)).encode(),
        (("<</Type/XObject/Subtype/Image/Width %d/Height %d/ColorSpace"
          "/DeviceRGB/BitsPerComponent 8/Filter/FlateDecode/Length %d>>\n"
          "stream\n" % (iw, ih, len(img))).encode() + img + b"\nendstream"),
        (("<</Length %d>>\nstream\n" % len(content)).encode() + content
         + b"\nendstream"),
    ]
    out = b"%PDF-1.4\n"
    offsets = []
    for i, o in enumerate(objs, start=1):
        offsets.append(len(out))
        out += ("%d 0 obj " % i).encode() + o + b" endobj\n"
    xref_off = len(out)
    n = len(objs) + 1
    out += ("xref\n0 %d\n" % n).encode() + b"0000000000 65535 f \n"
    for off in offsets:
        out += ("%010d 00000 n \n" % off).encode()
    out += ("trailer <</Size %d/Root 1 0 R>>\nstartxref\n%d\n%%%%EOF"
            % (n, xref_off)).encode()
    return out


class ESCLService(TCPService):
    """Driverless eSCL scan server on TCP 8080 (rs=eSCL)."""

    name = "escl"
    tcp_ports = (8080,)

    def __init__(self):
        super().__init__()
        self._seq = 0
        self._jobs = {}          # job_id -> pages remaining to hand out

    # -- connection / HTTP framing ----------------------------------------
    def on_connect(self, conn):
        conn.data["buf"] = bytearray()

    def on_data(self, conn, data):
        buf = conn.data["buf"]
        buf += data
        while True:
            he = buf.find(b"\r\n\r\n")
            if he < 0:
                return
            head = bytes(buf[:he]).decode("latin1", "replace")
            m = re.search(r"[Cc]ontent-[Ll]ength:\s*(\d+)", head)
            blen = int(m.group(1)) if m else 0
            if len(buf) < he + 4 + blen:
                return                          # await the full body
            body = bytes(buf[he + 4:he + 4 + blen])
            del buf[:he + 4 + blen]
            self.dump("eSCL request", head.encode())
            self._route(conn, head.split("\r\n")[0], body)

    # -- routing -----------------------------------------------------------
    def _route(self, conn, request_line, _body):
        try:
            method, path, _ = request_line.split(" ", 2)
        except ValueError:
            return self._send(conn, 400, "text/plain", b"bad request")
        path = path.split("?", 1)[0]
        if method == "GET" and path.endswith("/ScannerCapabilities"):
            self.log("eSCL ScannerCapabilities")
            return self._send(conn, 200, "text/xml", self._capabilities())
        if method == "GET" and path.endswith("/ScannerStatus"):
            return self._send(conn, 200, "text/xml", self._status())
        if method == "POST" and path.endswith("/ScanJobs"):
            return self._create_job(conn)
        m = re.search(r"/ScanJobs/([^/]+)/NextDocument$", path)
        if method == "GET" and m:
            return self._next_document(conn, m.group(1))
        # A job DELETE (client cancel) or anything else: acknowledge.
        if method == "DELETE":
            self._jobs.pop(path.rsplit("/", 1)[-1], None)
            return self._send(conn, 200, "text/plain", b"")
        self._send(conn, 404, "text/plain", b"not found")

    def _create_job(self, conn):
        self._seq += 1
        job_id = "sc%d" % self._seq
        self._jobs[job_id] = 1                  # one page (platen)
        self.log("eSCL ScanJobs -> job %s" % job_id)
        loc = "%s/eSCL/ScanJobs/%s" % (self._base(), job_id)
        self._send(conn, 201, "text/plain", b"",
                   extra=["Location: " + loc])

    def _next_document(self, conn, job_id):
        remaining = self._jobs.get(job_id, 0)
        if remaining <= 0:
            # No (more) pages: 404 tells the client the job is complete.
            self._jobs.pop(job_id, None)
            return self._send(conn, 404, "text/plain", b"")
        self._jobs[job_id] = remaining - 1
        pdf = make_pdf(PAGE_W_PX, PAGE_H_PX)
        self.log("eSCL NextDocument %s -> %d-byte page" % (job_id, len(pdf)))
        self._send(conn, 200, "application/pdf", pdf)

    # -- responses ---------------------------------------------------------
    def _send(self, conn, status, ctype, body, extra=None):
        reason = {200: "OK", 201: "Created", 400: "Bad Request",
                  404: "Not Found"}.get(status, "OK")
        lines = ["HTTP/1.1 %d %s" % (status, reason),
                 "Content-Type: " + ctype,
                 "Content-Length: %d" % len(body),
                 "Connection: Keep-Alive"]
        if extra:
            lines += extra
        head = ("\r\n".join(lines) + "\r\n\r\n").encode()
        self.dump("eSCL response", head)
        conn.send(head + body)

    def _base(self):
        ip = ".".join(str(x) for x in (self.host.stack.ip or bytes(4)))
        return "http://%s:8080" % ip

    def _capabilities(self):
        dev = getattr(self.host, "wsd_model", None) or \
            getattr(self.host, "hostname", "Scanner")
        make = getattr(self.host, "wsd_manufacturer", "PseudoHost")
        caps = _caps_body(PAGE_W_PX, PAGE_H_PX)   # platen and ADF are alike
        return (
            '<?xml version="1.0" encoding="UTF-8"?>'
            '<scan:ScannerCapabilities xmlns:scan="%s" xmlns:pwg="%s">'
            '<pwg:Version>2.63</pwg:Version>'
            '<pwg:MakeAndModel>%s %s</pwg:MakeAndModel>'
            '<scan:Platen><scan:PlatenInputCaps>%s</scan:PlatenInputCaps>'
            '</scan:Platen>'
            '<scan:Adf><scan:AdfSimplexInputCaps>%s</scan:AdfSimplexInputCaps>'
            '</scan:Adf>'
            '</scan:ScannerCapabilities>'
            % (NS_SCAN, NS_PWG, _xesc(make), _xesc(dev), caps, caps)).encode()

    def _status(self):
        return (
            '<?xml version="1.0" encoding="UTF-8"?>'
            '<scan:ScannerStatus xmlns:scan="%s" xmlns:pwg="%s">'
            '<pwg:Version>2.63</pwg:Version>'
            '<pwg:State>Idle</pwg:State>'
            '</scan:ScannerStatus>' % (NS_SCAN, NS_PWG)).encode()


def _xesc(s):
    return (str(s).replace("&", "&amp;").replace("<", "&lt;")
            .replace(">", "&gt;"))


def _caps_body(w, h):
    """The per-source capability block (color modes, formats, resolutions)."""
    return (
        '<scan:MaxWidth>%d</scan:MaxWidth>'
        '<scan:MaxHeight>%d</scan:MaxHeight>'
        '<scan:SettingProfiles><scan:SettingProfile>'
        '<scan:ColorModes><scan:ColorMode>RGB24</scan:ColorMode>'
        '<scan:ColorMode>Grayscale8</scan:ColorMode></scan:ColorModes>'
        '<scan:DocumentFormats>'
        '<pwg:DocumentFormat>application/pdf</pwg:DocumentFormat>'
        '<scan:DocumentFormatExt>application/pdf</scan:DocumentFormatExt>'
        '</scan:DocumentFormats>'
        '<scan:SupportedResolutions><scan:DiscreteResolutions>'
        '<scan:DiscreteResolution>'
        '<scan:XResolution>200</scan:XResolution>'
        '<scan:YResolution>200</scan:YResolution></scan:DiscreteResolution>'
        '<scan:DiscreteResolution>'
        '<scan:XResolution>300</scan:XResolution>'
        '<scan:YResolution>300</scan:YResolution></scan:DiscreteResolution>'
        '</scan:DiscreteResolutions></scan:SupportedResolutions>'
        '</scan:SettingProfile></scan:SettingProfiles>' % (w, h))
