#
# vwifi-pseudohost — WSD (Web Services for Devices) discovery
#
# SPDX-License-Identifier: GPL-2.0-or-later
#
# What makes a device show up in Windows "Network" and the "Add a
# printer" network scan.  Stock Windows does not browse mDNS/Bonjour for
# printers; it uses WSD:
#
#   1. Windows multicasts a WS-Discovery Probe to 239.255.255.250:3702.
#   2. The device answers with a unicast ProbeMatch carrying its endpoint
#      UUID and an XAddrs HTTP URL.
#   3. Windows POSTs a WS-Transfer Get to that URL and the device returns
#      its metadata (manufacturer, model, friendly name, and the hosted
#      print service).  That is what populates the name Windows shows.
#
# This module implements those three steps (plus a Hello announcement on
# join and Resolve/ResolveMatch), which is enough for the device to
# appear and be selectable.  It does NOT implement the WSD Print service
# itself (submitting jobs) — that is a much larger surface and a later
# phase; a device that appears but prints to nowhere is still the point
# of a simulation.
#
# The SOAP here is assembled and scraped as text on purpose: the full XML
# stack is not worth pulling in for a handful of fixed message shapes,
# and Windows is lenient about namespace prefixes on the wire.
#
import re

from .services import Service, TCPService

WSD_MCAST = "239.255.255.250"
WSD_PORT = 3702
WSD_HTTP_PORT = 5357

# WS-Discovery / WS-Addressing / DevProf actions and namespaces.
NS_SOAP = "http://www.w3.org/2003/05/soap-envelope"
NS_WSA = "http://schemas.xmlsoap.org/ws/2004/08/addressing"
NS_WSD = "http://schemas.xmlsoap.org/ws/2005/04/discovery"
NS_WSDP = "http://schemas.xmlsoap.org/ws/2006/02/devprof"
NS_WPRT = "http://schemas.microsoft.com/windows/2006/08/wdp/print"
NS_TRANSFER = "http://schemas.xmlsoap.org/ws/2004/09/transfer"

A_HELLO = NS_WSD + "/Hello"
A_BYE = NS_WSD + "/Bye"
A_PROBE = NS_WSD + "/Probe"
A_PROBEMATCH = NS_WSD + "/ProbeMatches"
A_RESOLVE = NS_WSD + "/Resolve"
A_RESOLVEMATCH = NS_WSD + "/ResolveMatches"
A_GET = NS_TRANSFER + "/Get"
A_GETRESPONSE = NS_TRANSFER + "/GetResponse"
# WSD Print (wprt) operations, sent to the device XAddrs (5357).
A_GETELEMENTS_RESP = NS_WPRT + "/GetPrinterElementsResponse"
A_CREATEJOB_RESP = NS_WPRT + "/CreatePrintJobResponse"
A_SENDDOC_RESP = NS_WPRT + "/SendDocumentResponse"
A_ADDDOC_RESP = NS_WPRT + "/AddDocumentResponse"

TO_DISCOVERY = "urn:schemas-xmlsoap-org:ws:2005:04:discovery"
# The device advertises itself as a WSD Device that is a print device.
DEVICE_TYPES = "wsdp:Device wprt:PrintDeviceType"


def uuid_from_mac(mac):
    """A stable urn:uuid built from the MAC (valid hex, so a valid UUID)."""
    h = mac.hex()
    return "urn:uuid:%s-%s-1000-8000-%s" % (h[:8], h[8:12], h)


class _Msg:
    """Small counter shared by a device's WSD messages (MessageNumber)."""

    def __init__(self):
        self.n = 0

    def next(self):
        self.n += 1
        return self.n


def _hdr(action, msg_id, relates_to=None, to=None, seq=None, instance=1):
    parts = [
        '<wsa:Action>%s</wsa:Action>' % action,
        '<wsa:MessageID>%s</wsa:MessageID>' % msg_id,
    ]
    if to:
        parts.insert(0, '<wsa:To>%s</wsa:To>' % to)
    if relates_to:
        parts.append('<wsa:RelatesTo>%s</wsa:RelatesTo>' % relates_to)
    if seq is not None:
        parts.append('<wsd:AppSequence InstanceId="%d" MessageNumber="%d"/>'
                     % (instance, seq))
    return "".join(parts)


def _envelope(header, body):
    return (
        '<?xml version="1.0" encoding="utf-8"?>'
        '<soap:Envelope'
        ' xmlns:soap="%s" xmlns:wsa="%s" xmlns:wsd="%s"'
        ' xmlns:wsdp="%s" xmlns:wprt="%s">'
        '<soap:Header>%s</soap:Header>'
        '<soap:Body>%s</soap:Body></soap:Envelope>'
        % (NS_SOAP, NS_WSA, NS_WSD, NS_WSDP, NS_WPRT, header, body)
    ).encode("utf-8")


def _new_msgid():
    # A per-message urn:uuid; only needs to be unique-ish and well-formed.
    import os
    h = os.urandom(16).hex()
    return "urn:uuid:%s-%s-%s-%s-%s" % (h[:8], h[8:12], h[12:16],
                                        h[16:20], h[20:32])


class WSDDevice:
    """The identity a device presents over WSD, derived from the host."""

    def __init__(self, host):
        self.host = host
        self.uuid = uuid_from_mac(host.mac)
        self.friendly = getattr(host, "hostname", "PseudoDevice")
        self.manufacturer = getattr(host, "wsd_manufacturer", "PseudoHost")
        self.model = getattr(host, "wsd_model", self.friendly)
        self.model_number = getattr(host, "wsd_model_number", "1.0")
        self.instance = 1

    def xaddr(self):
        ip = ".".join(str(x) for x in (self.host.stack.ip or bytes(4)))
        return "http://%s:%d/%s" % (ip, WSD_HTTP_PORT,
                                    self.uuid.split(":")[-1])

    def print_svc_uuid(self):
        # A distinct-but-stable UUID for the hosted print service (the
        # device is ...-8000-..., the service ...-8001-...).
        h = self.host.mac.hex()
        return "urn:uuid:%s-%s-1000-8001-%s" % (h[:8], h[8:12], h)


class WSDiscoveryService(Service):
    """Answers WS-Discovery Probes so Windows finds the device."""

    name = "wsd"
    udp_ports = (WSD_PORT,)

    def __init__(self):
        super().__init__()
        self.dev = None
        self.seq = _Msg()
        self._said_hello = False

    def on_start(self):
        self.dev = WSDDevice(self.host)

    # -- announcement ------------------------------------------------------
    def tick(self):
        # Say Hello once, as soon as we have an address; Windows Function
        # Discovery caches it and shows the device without a manual scan.
        if self._said_hello or self.stack.ip is None:
            return
        self._said_hello = True
        body = ('<wsd:Hello><wsa:EndpointReference><wsa:Address>%s'
                '</wsa:Address></wsa:EndpointReference>'
                '<wsd:Types>%s</wsd:Types><wsd:XAddrs>%s</wsd:XAddrs>'
                '<wsd:MetadataVersion>1</wsd:MetadataVersion></wsd:Hello>'
                % (self.dev.uuid, DEVICE_TYPES, self.dev.xaddr()))
        hdr = _hdr(A_HELLO, _new_msgid(), to=TO_DISCOVERY,
                   seq=self.seq.next(), instance=self.dev.instance)
        self.stack.send_udp(WSD_MCAST, WSD_PORT, _envelope(hdr, body),
                            src_port=WSD_PORT)

    # -- probe / resolve ---------------------------------------------------
    def on_udp(self, src_ip, src_port, dst_ip, dst_port, payload):
        try:
            text = payload.decode("utf-8", "replace")
        except Exception:
            return
        action = _find(text, "Action")
        if action is None:
            return
        msg_id = _find(text, "MessageID") or ""
        if action.endswith("/Probe"):
            self._probe_match(src_ip, src_port, text, msg_id)
        elif action.endswith("/Resolve"):
            self._resolve_match(src_ip, src_port, text, msg_id)

    def _probe_types_ok(self, text):
        # If the probe carries a Types filter, only answer when it names a
        # printer/device type we are; an empty filter matches everything.
        m = re.search(r"<[\w:]*Types[^>]*>(.*?)</[\w:]*Types>", text,
                      re.DOTALL)
        if not m:
            return True
        want = m.group(1)
        return ("PrintDeviceType" in want or "Device" in want
                or want.strip() == "")

    def _probe_match(self, src_ip, src_port, text, msg_id):
        if not self._probe_types_ok(text):
            return
        body = ('<wsd:ProbeMatches><wsd:ProbeMatch>'
                '<wsa:EndpointReference><wsa:Address>%s</wsa:Address>'
                '</wsa:EndpointReference>'
                '<wsd:Types>%s</wsd:Types><wsd:XAddrs>%s</wsd:XAddrs>'
                '<wsd:MetadataVersion>1</wsd:MetadataVersion>'
                '</wsd:ProbeMatch></wsd:ProbeMatches>'
                % (self.dev.uuid, DEVICE_TYPES, self.dev.xaddr()))
        hdr = _hdr(A_PROBEMATCH, _new_msgid(), relates_to=msg_id,
                   to=NS_WSA + "/role/anonymous", seq=self.seq.next(),
                   instance=self.dev.instance)
        self.log("Probe from %s -> ProbeMatch" % _ip(src_ip))
        self.reply_udp(src_ip, src_port, _envelope(hdr, body),
                       src_port=WSD_PORT)

    def _resolve_match(self, src_ip, src_port, text, msg_id):
        # Only answer if the resolve names our endpoint.
        if self.dev.uuid not in text:
            return
        body = ('<wsd:ResolveMatches><wsd:ResolveMatch>'
                '<wsa:EndpointReference><wsa:Address>%s</wsa:Address>'
                '</wsa:EndpointReference>'
                '<wsd:Types>%s</wsd:Types><wsd:XAddrs>%s</wsd:XAddrs>'
                '<wsd:MetadataVersion>1</wsd:MetadataVersion>'
                '</wsd:ResolveMatch></wsd:ResolveMatches>'
                % (self.dev.uuid, DEVICE_TYPES, self.dev.xaddr()))
        hdr = _hdr(A_RESOLVEMATCH, _new_msgid(), relates_to=msg_id,
                   to=NS_WSA + "/role/anonymous", seq=self.seq.next(),
                   instance=self.dev.instance)
        self.log("Resolve -> ResolveMatch")
        self.reply_udp(src_ip, src_port, _envelope(hdr, body),
                       src_port=WSD_PORT)


class WSDHttpService(TCPService):
    """The device's WSD HTTP endpoint (the XAddrs, TCP 5357).

    Handles the WS-Transfer Get (device metadata) that Windows fetches to
    show the device, and the WSD Print operations that let it be added and
    printed to: GetPrinterElements, CreatePrintJob, and SendDocument
    (whose MTOM attachment is the actual document — handed to the host's
    print sink).
    """

    name = "wsd-http"
    tcp_ports = (WSD_HTTP_PORT,)

    def __init__(self):
        super().__init__()
        self._job = 0

    def on_connect(self, conn):
        conn.data["buf"] = bytearray()
        conn.data["need"] = None

    def on_data(self, conn, data):
        buf = conn.data["buf"]
        buf += data
        head_end = buf.find(b"\r\n\r\n")
        if head_end < 0:
            return
        if conn.data["need"] is None:
            m = re.search(rb"content-length:\s*(\d+)", bytes(buf[:head_end]),
                          re.I)
            conn.data["need"] = head_end + 4 + (int(m.group(1)) if m else 0)
        if len(buf) < conn.data["need"]:
            return                              # wait for the whole body
        resp = self._dispatch(bytes(buf), head_end)
        http = (b"HTTP/1.1 200 OK\r\n"
                b"Content-Type: application/soap+xml; charset=utf-8\r\n"
                b"Content-Length: " + str(len(resp)).encode() + b"\r\n"
                b"Connection: close\r\n\r\n" + resp)
        conn.send(http)
        conn.close()

    # -- request dispatch --------------------------------------------------
    def _dispatch(self, raw, head_end):
        headers = raw[:head_end].decode("latin1", "replace")
        body = raw[head_end + 4:]
        soap, binary = _split_mtom(headers, body)
        text = soap.decode("utf-8", "replace")
        action = _find(text, "Action") or ""
        msgid = _find(text, "MessageID") or ""

        if action.endswith("/Get"):
            self.log("metadata Get -> 200")
            return self._metadata(msgid)
        if action.endswith("/GetPrinterElements"):
            self.log("GetPrinterElements -> 200")
            return self._printer_elements(msgid)
        if action.endswith("/CreatePrintJob"):
            self._job += 1
            jobname = _find(text, "JobName") or "job%d" % self._job
            self._cur_jobname = jobname
            self.log("CreatePrintJob '%s' -> job %d" % (jobname, self._job))
            return self._create_job_resp(msgid, self._job)
        if action.endswith("/SendDocument") or action.endswith("/AddDocument"):
            self._accept_document(text, binary)
            resp_action = (A_SENDDOC_RESP if action.endswith("/SendDocument")
                           else A_ADDDOC_RESP)
            return self._doc_resp(msgid, resp_action, self._job)
        # Unknown op: an empty ack keeps the client from erroring hard.
        return self._metadata(msgid)

    def _accept_document(self, text, binary):
        sink = getattr(self.host, "print_sink", None)
        name = getattr(self, "_cur_jobname", None) or _find(text, "JobName") \
            or "wsd-job"
        if sink is not None:
            sink.write_job(binary or b"", jobname=name, source="wsd-print")
        else:
            self.log("SendDocument: %d bytes but no print sink" % len(binary))

    # -- responses ---------------------------------------------------------
    def _metadata(self, relates_to):
        dev = WSDDevice(self.host)
        ip = _ip(self.host.stack.ip or bytes(4))
        this_model = (
            '<wsdp:ThisModel><wsdp:Manufacturer>%s</wsdp:Manufacturer>'
            '<wsdp:ModelName>%s</wsdp:ModelName>'
            '<wsdp:ModelNumber>%s</wsdp:ModelNumber>'
            '<wsdp:PresentationUrl>http://%s/</wsdp:PresentationUrl>'
            '</wsdp:ThisModel>'
            % (dev.manufacturer, dev.model, dev.model_number, ip))
        this_device = (
            '<wsdp:ThisDevice><wsdp:FriendlyName>%s</wsdp:FriendlyName>'
            '<wsdp:FirmwareVersion>1.0</wsdp:FirmwareVersion>'
            '<wsdp:SerialNumber>%s</wsdp:SerialNumber></wsdp:ThisDevice>'
            % (dev.friendly, self.host.mac.hex()))
        # Host is the device; the print service is a *Hosted* service with
        # its own endpoint — this is the shape Windows needs to turn the
        # device into an addable printer.
        relationship = (
            '<wsdp:Relationship Type="%s/host">'
            '<wsdp:Host><wsa:EndpointReference><wsa:Address>%s</wsa:Address>'
            '</wsa:EndpointReference></wsdp:Host>'
            '<wsdp:Hosted><wsa:EndpointReference><wsa:Address>%s</wsa:Address>'
            '</wsa:EndpointReference><wsdp:Types>wprt:PrintServiceType'
            '</wsdp:Types><wsdp:ServiceId>%s</wsdp:ServiceId>'
            '<wsdp:HardwareId>%s</wsdp:HardwareId></wsdp:Hosted>'
            '</wsdp:Relationship>'
            % (NS_WSDP, dev.uuid, dev.print_svc_uuid(), dev.print_svc_uuid(),
               "PseudoPrinter"))
        sections = (
            '<wsdp:MetadataSection Dialect="%s/ThisModel">%s'
            '</wsdp:MetadataSection>'
            '<wsdp:MetadataSection Dialect="%s/ThisDevice">%s'
            '</wsdp:MetadataSection>'
            '<wsdp:MetadataSection Dialect="%s/Relationship">%s'
            '</wsdp:MetadataSection>'
            % (NS_WSDP, this_model, NS_WSDP, this_device,
               NS_WSDP, relationship))
        body = '<wsdp:Metadata>%s</wsdp:Metadata>' % sections
        hdr = _hdr(A_GETRESPONSE, _new_msgid(), relates_to=relates_to,
                   to=NS_WSA + "/role/anonymous")
        return _envelope(hdr, body)

    def _printer_elements(self, relates_to):
        dev = WSDDevice(self.host)
        desc = (
            '<wprt:PrinterDescription>'
            '<wprt:ColorSupported>true</wprt:ColorSupported>'
            '<wprt:DeviceId>MFG:%s;MDL:%s;CLS:PRINTER;'
            'CMD:PCL,PostScript,PDF,URF;</wprt:DeviceId>'
            '<wprt:MultipleDocumentJobsSupported>false'
            '</wprt:MultipleDocumentJobsSupported>'
            '<wprt:PagesPerMinute>20</wprt:PagesPerMinute>'
            '<wprt:PrinterName><wprt:Name xml:lang="en-US">%s</wprt:Name>'
            '</wprt:PrinterName>'
            '<wprt:PrinterInfo><wprt:Name xml:lang="en-US">'
            'Simulated by vwifi-pseudohost</wprt:Name></wprt:PrinterInfo>'
            '</wprt:PrinterDescription>'
            % (dev.manufacturer, dev.model, dev.friendly))
        status = (
            '<wprt:PrinterStatus><wprt:PrinterState>Idle</wprt:PrinterState>'
            '<wprt:PrinterPrimaryStateReason>None'
            '</wprt:PrinterPrimaryStateReason>'
            '<wprt:QueuedJobCount>0</wprt:QueuedJobCount>'
            '</wprt:PrinterStatus>')
        body = (
            '<wprt:GetPrinterElementsResponse><wprt:PrinterElements>'
            '<wprt:ElementData Name="wprt:PrinterDescription" Valid="true">%s'
            '</wprt:ElementData>'
            '<wprt:ElementData Name="wprt:PrinterStatus" Valid="true">%s'
            '</wprt:ElementData>'
            '</wprt:PrinterElements></wprt:GetPrinterElementsResponse>'
            % (desc, status))
        hdr = _hdr(A_GETELEMENTS_RESP, _new_msgid(), relates_to=relates_to,
                   to=NS_WSA + "/role/anonymous")
        return _envelope(hdr, body)

    def _create_job_resp(self, relates_to, job_id):
        body = ('<wprt:CreatePrintJobResponse><wprt:JobId>%d</wprt:JobId>'
                '</wprt:CreatePrintJobResponse>' % job_id)
        hdr = _hdr(A_CREATEJOB_RESP, _new_msgid(), relates_to=relates_to,
                   to=NS_WSA + "/role/anonymous")
        return _envelope(hdr, body)

    def _doc_resp(self, relates_to, action, job_id):
        body = ('<wprt:SendDocumentResponse><wprt:JobId>%d</wprt:JobId>'
                '</wprt:SendDocumentResponse>' % job_id)
        hdr = _hdr(action, _new_msgid(), relates_to=relates_to,
                   to=NS_WSA + "/role/anonymous")
        return _envelope(hdr, body)


# Back-compat alias: the metadata endpoint is now the full HTTP endpoint.
WSDMetadataService = WSDHttpService


def _split_mtom(headers, body):
    """Split a request body into (soap_bytes, binary_bytes).

    A SendDocument is multipart/related (MTOM): the SOAP is the
    application/xop+xml part and the document is the binary part.  Any
    other request is plain SOAP with no attachment.
    """
    m = re.search(r"boundary=\"?([^\";\r\n]+)\"?", headers, re.I)
    if "multipart/related" not in headers.lower() or not m:
        return body, b""
    boundary = ("--" + m.group(1)).encode("latin1")
    soap = b""
    binary = b""
    for part in body.split(boundary):
        if not part or part in (b"--", b"--\r\n", b"\r\n"):
            continue
        he = part.find(b"\r\n\r\n")
        if he < 0:
            continue
        phdr = part[:he].decode("latin1", "replace").lower()
        pbody = part[he + 4:]
        if pbody.endswith(b"\r\n"):
            pbody = pbody[:-2]
        if "xop+xml" in phdr or "soap+xml" in phdr:
            soap = pbody
        elif pbody:
            binary = pbody
    return soap, binary


def _find(text, tag):
    m = re.search(r"<[\w:]*%s[^>]*>(.*?)</[\w:]*%s>" % (tag, tag), text,
                  re.DOTALL)
    return m.group(1).strip() if m else None


def _ip(b):
    return ".".join(str(x) for x in b)


def wsd_services():
    """The pair of services a WSD device runs: discovery + metadata HTTP."""
    return [WSDiscoveryService(), WSDMetadataService()]
