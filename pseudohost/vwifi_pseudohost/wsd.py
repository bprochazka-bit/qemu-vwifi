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
import struct
import time
import zlib

from .services import Service, TCPService

# A modest fixed scan raster (roughly a US-Letter page at ~100 dpi). Small
# enough that a solid-colour PNG is a few KB over the virtual medium, while
# still a plausible page size for Windows' scan UI.
SCAN_W = 850
SCAN_H = 1100


def make_png(width, height, rgb=(224, 224, 224)):
    """A valid 8-bit RGB PNG of a solid colour — the pseudo-host's 'scan'.

    Solid colour so the zlib-compressed IDAT stays tiny regardless of the
    page size. Stdlib only (zlib)."""
    def chunk(typ, data):
        body = typ + data
        return (struct.pack(">I", len(data)) + body
                + struct.pack(">I", zlib.crc32(body) & 0xffffffff))
    sig = b"\x89PNG\r\n\x1a\n"
    ihdr = struct.pack(">IIBBBBB", width, height, 8, 2, 0, 0, 0)  # RGB/8
    row = b"\x00" + bytes(rgb) * width          # filter byte 0 + pixels
    idat = zlib.compress(row * height, 6)
    return (sig + chunk(b"IHDR", ihdr) + chunk(b"IDAT", idat)
            + chunk(b"IEND", b""))

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
# WS-Eventing. Windows subscribes to the printer's PrinterElementsChangeEvent
# while a print queue is open. The subscription MUST be answered with a
# SubscribeResponse — returning anything else (the device metadata, say)
# feeds the spooler's WSD event client a document it can't parse and
# crashes it, which strands the print job in the queue.
NS_EVENTING = "http://schemas.xmlsoap.org/ws/2004/08/eventing"
# WS-MetadataExchange: the Metadata / MetadataSection wrapper elements a
# WS-Transfer Get returns live in *this* namespace, not devprof.  Windows
# silently drops a device whose metadata wrapper is mis-namespaced.
NS_MEX = "http://schemas.xmlsoap.org/ws/2004/09/mex"
# PnP-X (Plug and Play Extensions).  Function Discovery uses the
# <pnpx:DeviceCategory> in ThisModel to categorise a WSD device and
# publish it into the Explorer "Network" folder; a device metadata with
# no PnP-X category is parsed but never rendered as a device tile.
NS_PNPX = "http://schemas.microsoft.com/windows/pnpx/2005/10"
# Device Foundation.  Windows reads <df:DeviceCategory> in ThisModel (a
# space-separated list like "PrintFax.Printer.MFP Imaging.Scanner") in
# addition to the PnP-X category; a real printer carries both.
NS_DF = "http://schemas.microsoft.com/windows/2008/09/devicefoundation"
# The standard PnP-X CompatibleId for a WSD print service. It maps the
# hosted service to Windows' inbox WSD print class driver — the thing that
# turns the discovered device into an installable/printable printer. Real
# HP printers advertise exactly this string.
WSD_PRINT_COMPATIBLE_ID = ("http://schemas.microsoft.com/windows/2006/08/"
                           "wdp/print/PrinterServiceType")
# WSD Scan (WS-Scan) service — the analogue of the print service that makes
# a multifunction show up under Scanners and lets Windows pull a scan. The
# WS-Scan protocol namespace is the 2006/01 one (this is what the operation
# elements and a real HP's metadata use); the CompatibleId that maps the
# hosted service to Windows' inbox WSD-Scan (WIA) driver is a fixed string
# that, by Microsoft's own quirk, carries 2006/08 instead.
NS_WSCN = "http://schemas.microsoft.com/windows/2006/01/wdp/scan"
SCAN_SERVICE_TYPES = "wscn:ScannerServiceType"
WSD_SCAN_COMPATIBLE_ID = ("http://schemas.microsoft.com/windows/2006/08/"
                          "wdp/scan/ScannerServiceType")

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
A_SETEVENTRATE_RESP = NS_WPRT + "/SetEventRateResponse"
A_GETSCANNER_ELEMENTS_RESP = NS_WSCN + "/GetScannerElementsResponse"
A_CREATE_SCAN_JOB_RESP = NS_WSCN + "/CreateScanJobResponse"
A_RETRIEVE_IMAGE_RESP = NS_WSCN + "/RetrieveImageResponse"
A_SUBSCRIBE_RESP = NS_EVENTING + "/SubscribeResponse"
A_UNSUBSCRIBE_RESP = NS_EVENTING + "/UnsubscribeResponse"
A_RENEW_RESP = NS_EVENTING + "/RenewResponse"
A_GETSTATUS_RESP = NS_EVENTING + "/GetStatusResponse"
A_CREATEJOB_RESP = NS_WPRT + "/CreatePrintJobResponse"
A_SENDDOC_RESP = NS_WPRT + "/SendDocumentResponse"
A_ADDDOC_RESP = NS_WPRT + "/AddDocumentResponse"

TO_DISCOVERY = "urn:schemas-xmlsoap-org:ws:2005:04:discovery"
# The device advertises itself as a WSD Device that is a print device.
DEVICE_TYPES = "wsdp:Device wprt:PrintDeviceType"
# The hosted print SERVICE inside the device is a different WSD type from
# the device itself: the device is wprt:PrintDeviceType, the service that
# hosts the print operations is wprt:PrinterServiceType. Windows keys the
# print functionality off the hosted PrinterServiceType, so getting this
# wrong (naming the service PrintDeviceType) leaves the device fetched but
# never turned into a usable printer — Windows re-probes it in a loop.
PRINT_SERVICE_TYPES = "wprt:PrinterServiceType"


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
        ' xmlns:wsdp="%s" xmlns:wprt="%s" xmlns:wscn="%s" xmlns:mex="%s"'
        ' xmlns:pnpx="%s" xmlns:df="%s" xmlns:wse="%s">'
        '<soap:Header>%s</soap:Header>'
        '<soap:Body>%s</soap:Body></soap:Envelope>'
        % (NS_SOAP, NS_WSA, NS_WSD, NS_WSDP, NS_WPRT, NS_WSCN, NS_MEX,
           NS_PNPX, NS_DF, NS_EVENTING, header, body)
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
        # PnP-X device category (space-delimited list); this is what puts
        # the device under the right heading in Explorer's Network folder.
        self.pnpx_category = getattr(host, "pnpx_category", "Printers")
        # Device Foundation category (space-separated). Real printers carry
        # this alongside the PnP-X category; default to a plain printer, and
        # a multifunction profile overrides it (MFP + scanner + fax).
        self.df_category = getattr(host, "df_device_category", "PrintFax.Printer")
        # PnP-X HardwareId / CompatibleId for the hosted print service.
        # Windows REQUIRES both on a hosted element once a DeviceCategory is
        # present, and matches a driver by HardwareId, falling back to the
        # CompatibleId (the inbox WSD print driver) when it doesn't know the
        # exact model — which is what makes the device install and show as a
        # printer. Modelled on a real HP OfficeJet's:
        #   VEN_03F0&DEV_Officejet_Pro_8600&SUBSYS_CM749A
        # 0x03F0 is HP's USB vendor id; a profile can override the id.
        vid = getattr(host, "wsd_vendor_id", 0x03F0)
        model_tok = re.sub(r"[^A-Za-z0-9]+", "_", self.model).strip("_")
        self.hardware_id = getattr(host, "wsd_hardware_id", None) or (
            "VEN_%04X&amp;DEV_%s&amp;SUBSYS_%s"
            % (vid, model_tok, self.model_number))
        self.compatible_id = getattr(host, "pnpx_compatible_id",
                                     WSD_PRINT_COMPATIBLE_ID)
        # A multifunction also hosts a WSD scan service, so it shows up
        # under Scanners as well as Printers. The scanner gets its own
        # PnP-X ids (a distinct HardwareId sub-id, the inbox scan driver's
        # CompatibleId). Off unless the profile sets wsd_scan.
        self.scan = bool(getattr(host, "wsd_scan", False))
        self.scan_hardware_id = getattr(host, "wsd_scan_hardware_id", None) or (
            "VEN_%04X&amp;DEV_%s&amp;SUBSYS_%s_SCAN"
            % (vid, model_tok, self.model_number))
        self.scan_compatible_id = getattr(host, "pnpx_scan_compatible_id",
                                          WSD_SCAN_COMPATIBLE_ID)
        # WS-Discovery AppSequence InstanceId: MUST change each time the
        # device (re)starts so a client discards state cached under a prior
        # instance. A constant "1" means Windows treats every restart as the
        # same boot and can keep serving a stale/rejected metadata version
        # from its cache forever — fatal when iterating on the metadata. A
        # start-time epoch is monotonic across restarts and fits the field.
        self.instance = int(time.time())

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
        self.dump("WSD %s from %s" % (action.rsplit("/", 1)[-1], _ip(src_ip)),
                  payload)
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
        self._subs = []          # active WS-Eventing subscriptions
        self._event_q = []       # pending (dest, http_bytes) to push out
        self._scan_job = 0       # WSD-Scan job counter

    def on_start(self):
        # Register as the printer's event source so the IPP printer can ask
        # us to push a WSD event when a job arrives/completes.
        self.host.wsd_event_source = self

    def on_connect(self, conn):
        conn.data["buf"] = bytearray()

    def on_data(self, conn, data):
        # HTTP keep-alive: Windows sends many WSD requests down one
        # connection and keeps it open. Handle every complete request in the
        # buffer and leave the connection up. The old code answered one
        # request then closed; Windows would send the next request on the
        # connection it still considered open, and the pseudo-host, having
        # forgotten it, answered with a TCP RST — a storm of resets that
        # destabilised the spooler's WSD client. Keeping the connection
        # alive removes them.
        buf = conn.data["buf"]
        buf += data
        while True:
            head_end = buf.find(b"\r\n\r\n")
            if head_end < 0:
                return                          # headers incomplete
            m = re.search(rb"content-length:\s*(\d+)", bytes(buf[:head_end]),
                          re.I)
            need = head_end + 4 + (int(m.group(1)) if m else 0)
            if len(buf) < need:
                return                          # body incomplete
            req = bytes(buf[:need])
            del buf[:need]                      # consume this request
            self.dump("WSD-HTTP request", req)
            resp = self._dispatch(req, head_end)
            # A handler may return (content_type, body) to send a non-SOAP
            # response — RetrieveImage returns an MTOM multipart.
            if isinstance(resp, tuple):
                content_type, body = resp
            else:
                content_type = b"application/soap+xml; charset=utf-8"
                body = resp
            http = (b"HTTP/1.1 200 OK\r\n"
                    b"Content-Type: " + content_type + b"\r\n"
                    b"Content-Length: " + str(len(body)).encode() + b"\r\n"
                    b"Connection: Keep-Alive\r\n\r\n" + body)
            self.dump("WSD-HTTP response", http)
            conn.send(http)
            # loop again for any pipelined request already in the buffer

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
        if action.endswith("/GetScannerElements"):
            self.log("GetScannerElements -> 200")
            return self._scanner_elements(msgid)
        if action.endswith("/CreateScanJob"):
            self._scan_job += 1
            self.log("CreateScanJob -> job %d" % self._scan_job)
            return self._create_scan_job(msgid, text)
        if action.endswith("/RetrieveImage"):
            # Returns MTOM (multipart) rather than a plain SOAP body.
            self.log("RetrieveImage -> image")
            return self._retrieve_image(msgid, text)
        if action.endswith("/SetEventRate"):
            # Windows sets how often it wants printer events while a queue is
            # open. Acknowledge with the rate it asked for; a Fault here (or
            # any non-response) crashes the spooler's WSD print client.
            rate = _find(text, "EventRate") or "1"
            self.log("SetEventRate %s -> ack" % rate)
            return self._set_event_rate_resp(msgid, rate)
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
        # WS-Eventing: Windows subscribes to printer status events while a
        # queue is open. These MUST get their own responses — the spooler's
        # event client crashes on anything else.
        if action.endswith("/eventing/Subscribe"):
            self.log("Subscribe -> SubscribeResponse")
            return self._subscribe_resp(msgid, text)
        if action.endswith("/eventing/Renew"):
            return self._renew_resp(msgid, text)
        if action.endswith("/eventing/GetStatus"):
            return self._getstatus_resp(msgid, text)
        if action.endswith("/eventing/Unsubscribe"):
            return self._unsubscribe_resp(msgid)
        # Truly unknown op: a SOAP Fault is the correct answer. Returning
        # device metadata here (the old behaviour) hands the client a
        # document for the wrong operation and can crash it.
        self.log("unhandled WSD action %r -> Fault" % action)
        return self._fault(msgid, action)

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
        # The whole metadata document is modelled element-for-element on a
        # real HP OfficeJet's — the device Windows actually accepts and
        # renders in the Network folder — because matching a working device
        # is the only reliable target for Windows' opaque WSD validation.
        #
        # ThisDevice first (that is the order the real device uses), with
        # xml:lang on the human-readable strings.
        this_device = (
            '<wsdp:ThisDevice>'
            '<wsdp:FriendlyName xml:lang="en">%s</wsdp:FriendlyName>'
            '<wsdp:FirmwareVersion>1.0</wsdp:FirmwareVersion>'
            '<wsdp:SerialNumber>%s</wsdp:SerialNumber></wsdp:ThisDevice>'
            % (dev.friendly, self.host.mac.hex()))
        # ThisModel: both the PnP-X and Device Foundation categories, no
        # ModelUrl (the real device omits it).
        this_model = (
            '<wsdp:ThisModel>'
            '<wsdp:Manufacturer xml:lang="en">%s</wsdp:Manufacturer>'
            '<wsdp:ManufacturerUrl>http://%s/</wsdp:ManufacturerUrl>'
            '<wsdp:ModelName xml:lang="en">%s</wsdp:ModelName>'
            '<wsdp:ModelNumber>%s</wsdp:ModelNumber>'
            '<wsdp:PresentationUrl>http://%s/</wsdp:PresentationUrl>'
            '<pnpx:DeviceCategory>%s</pnpx:DeviceCategory>'
            '<df:DeviceCategory>%s</df:DeviceCategory>'
            '</wsdp:ThisModel>'
            % (dev.manufacturer, ip, dev.model, dev.model_number, ip,
               dev.pnpx_category, dev.df_category))
        # Relationship: like the real printer, the print service is the only
        # member and there is NO <wsdp:Host> — a <wsdp:Host> marked
        # PrintDeviceType makes Windows look for the print service at the
        # device endpoint (a urn:uuid with no HTTP address) instead of at
        # this Hosted HTTP endpoint. The ServiceId is the http form the real
        # device uses. The Hosted endpoint stays the reachable :5357 address
        # (where this process serves the print operations), and carries the
        # PnP-X HardwareId + CompatibleId Windows needs to build the node.
        uuid_tail = dev.uuid.split(":")[-1]

        def hosted(types, service_id, hwid, cid):
            return (
                '<wsdp:Hosted>'
                '<wsa:EndpointReference><wsa:Address>%s</wsa:Address>'
                '</wsa:EndpointReference>'
                '<wsdp:Types>%s</wsdp:Types>'
                '<wsdp:ServiceId>%s</wsdp:ServiceId>'
                '<pnpx:HardwareId>%s</pnpx:HardwareId>'
                '<pnpx:CompatibleId>%s</pnpx:CompatibleId>'
                '</wsdp:Hosted>'
                % (dev.xaddr(), types, service_id, hwid, cid))

        hosted_services = [
            hosted(PRINT_SERVICE_TYPES,
                   "http://%s/PrintService" % uuid_tail,
                   dev.hardware_id, dev.compatible_id),
        ]
        # A multifunction adds a WSD scan service so Windows also creates a
        # scanner device node (Scanners / Windows Fax and Scan).
        if dev.scan:
            hosted_services.append(hosted(
                SCAN_SERVICE_TYPES,
                "http://%s/ScanService" % uuid_tail,
                dev.scan_hardware_id, dev.scan_compatible_id))

        relationship = (
            '<wsdp:Relationship Type="%s/host">%s</wsdp:Relationship>'
            % (NS_WSDP, "".join(hosted_services)))
        # The Metadata / MetadataSection wrapper elements are WS-Metadata-
        # Exchange (mex:), not devprof: Windows parses the sections by that
        # namespace and drops the device if the wrapper is mis-namespaced.
        sections = (
            '<mex:MetadataSection Dialect="%s/ThisDevice">%s'
            '</mex:MetadataSection>'
            '<mex:MetadataSection Dialect="%s/ThisModel">%s'
            '</mex:MetadataSection>'
            '<mex:MetadataSection Dialect="%s/Relationship">%s'
            '</mex:MetadataSection>'
            % (NS_WSDP, this_device, NS_WSDP, this_model,
               NS_WSDP, relationship))
        body = '<mex:Metadata>%s</mex:Metadata>' % sections
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

    def _scanner_elements(self, relates_to):
        """Minimal WSD-Scan GetScannerElements response.

        Enough for Windows to finish installing the scanner devnode: a
        ScannerDescription (name/info), a ScannerConfiguration advertising a
        flatbed (Platen) with common resolutions, and an Idle status. This
        is the scan analogue of _printer_elements; it does not implement an
        actual scan (CreateScanJob/RetrieveImage) yet — it makes the device
        install and show under Scanners.
        """
        dev = WSDDevice(self.host)
        desc = (
            '<wscn:ScannerDescription>'
            '<wscn:ScannerName xml:lang="en-US">%s</wscn:ScannerName>'
            '<wscn:ScannerInfo xml:lang="en-US">Simulated by '
            'vwifi-pseudohost</wscn:ScannerInfo>'
            '</wscn:ScannerDescription>' % dev.friendly)
        status = (
            '<wscn:ScannerStatus>'
            '<wscn:ScannerCurrentTime>1970-01-01T00:00:00.000Z'
            '</wscn:ScannerCurrentTime>'
            '<wscn:ScannerState>Idle</wscn:ScannerState>'
            '</wscn:ScannerStatus>')
        config = (
            '<wscn:ScannerConfiguration>'
            '<wscn:DeviceSettings>'
            '<wscn:FormatsSupported>'
            '<wscn:FormatValue>jfif</wscn:FormatValue>'
            '<wscn:FormatValue>png</wscn:FormatValue>'
            '</wscn:FormatsSupported>'
            '<wscn:ContentTypesSupported>'
            '<wscn:ContentTypeValue>Auto</wscn:ContentTypeValue>'
            '</wscn:ContentTypesSupported>'
            '</wscn:DeviceSettings>'
            '<wscn:Platen>'
            '<wscn:PlatenOpticalResolution>'
            '<wscn:Width>300</wscn:Width><wscn:Height>300</wscn:Height>'
            '</wscn:PlatenOpticalResolution>'
            '<wscn:PlatenColorSpaces>'
            '<wscn:ColorEntry>RGB24</wscn:ColorEntry>'
            '<wscn:ColorEntry>Grayscale8</wscn:ColorEntry>'
            '</wscn:PlatenColorSpaces>'
            '</wscn:Platen>'
            '</wscn:ScannerConfiguration>')
        body = (
            '<wscn:GetScannerElementsResponse><wscn:ScannerElements>'
            '<wscn:ElementData Name="wscn:ScannerDescription">%s'
            '</wscn:ElementData>'
            '<wscn:ElementData Name="wscn:ScannerConfiguration">%s'
            '</wscn:ElementData>'
            '<wscn:ElementData Name="wscn:ScannerStatus">%s'
            '</wscn:ElementData>'
            '</wscn:ScannerElements></wscn:GetScannerElementsResponse>'
            % (desc, config, status))
        hdr = _hdr(A_GETSCANNER_ELEMENTS_RESP, _new_msgid(),
                   relates_to=relates_to, to=NS_WSA + "/role/anonymous")
        return _envelope(hdr, body)

    # -- WSD-Scan pull flow ------------------------------------------------
    def _create_scan_job(self, relates_to, req_text):
        """Answer CreateScanJob: hand back a job id/token and the final image
        parameters (a fixed PNG raster the pseudo-host will return)."""
        job = self._scan_job
        token = "scantok-%d" % job
        img_info = (
            '<wscn:ImageInformation><wscn:MediaFrontImageInfo>'
            '<wscn:PixelsPerLine>%d</wscn:PixelsPerLine>'
            '<wscn:NumberOfLines>%d</wscn:NumberOfLines>'
            '<wscn:BytesPerLine>%d</wscn:BytesPerLine>'
            '</wscn:MediaFrontImageInfo></wscn:ImageInformation>'
            % (SCAN_W, SCAN_H, SCAN_W * 3))
        final = (
            '<wscn:DocumentFinalParameters>'
            '<wscn:Format>png</wscn:Format>'
            '<wscn:CompressionQualityFactor>0</wscn:CompressionQualityFactor>'
            '<wscn:ImagesToTransfer>1</wscn:ImagesToTransfer>'
            '<wscn:MediaSides><wscn:MediaFront>'
            '<wscn:ScanRegion>'
            '<wscn:ScanRegionWidth>%d</wscn:ScanRegionWidth>'
            '<wscn:ScanRegionHeight>%d</wscn:ScanRegionHeight>'
            '</wscn:ScanRegion>'
            '<wscn:ColorProcessing>RGB24</wscn:ColorProcessing>'
            '<wscn:Resolution><wscn:Width>100</wscn:Width>'
            '<wscn:Height>100</wscn:Height></wscn:Resolution>'
            '</wscn:MediaFront></wscn:MediaSides>'
            '</wscn:DocumentFinalParameters>'
            % (SCAN_W, SCAN_H))
        body = (
            '<wscn:CreateScanJobResponse>'
            '<wscn:JobId>%d</wscn:JobId>'
            '<wscn:JobToken>%s</wscn:JobToken>'
            '%s%s</wscn:CreateScanJobResponse>' % (job, token, img_info, final))
        hdr = _hdr(A_CREATE_SCAN_JOB_RESP, _new_msgid(), relates_to=relates_to,
                   to=NS_WSA + "/role/anonymous")
        return _envelope(hdr, body)

    def _retrieve_image(self, relates_to, _req_text):
        """Answer RetrieveImage with the scan as an MTOM (multipart/related)
        response: a SOAP part whose xop:Include points at the image part."""
        png = make_png(SCAN_W, SCAN_H)
        cid = "scan-%d@pseudohost" % (self._scan_job or 1)
        soap_hdr = _hdr(A_RETRIEVE_IMAGE_RESP, _new_msgid(),
                        relates_to=relates_to, to=NS_WSA + "/role/anonymous")
        soap_body = (
            '<wscn:RetrieveImageResponse><xop:Include '
            'xmlns:xop="http://www.w3.org/2004/08/xop/include" '
            'href="cid:%s"/></wscn:RetrieveImageResponse>' % cid)
        soap = _envelope(soap_hdr, soap_body)
        b = b"MIMEBoundaryPseudohostScan"
        body = (
            b"--" + b + b"\r\n"
            b'Content-Type: application/xop+xml; charset=utf-8; '
            b'type="application/soap+xml"\r\n'
            b"Content-Transfer-Encoding: 8bit\r\n"
            b"Content-ID: <soap@pseudohost>\r\n\r\n" + soap + b"\r\n"
            b"--" + b + b"\r\n"
            b"Content-Type: image/png\r\n"
            b"Content-Transfer-Encoding: binary\r\n"
            b"Content-ID: <" + cid.encode() + b">\r\n\r\n" + png + b"\r\n"
            b"--" + b + b"--\r\n")
        content_type = (
            b'multipart/related; boundary="' + b + b'"; '
            b'type="application/xop+xml"; start="<soap@pseudohost>"; '
            b'start-info="application/soap+xml"')
        return (content_type, body)

    def _set_event_rate_resp(self, relates_to, rate):
        body = ('<wprt:SetEventRateResponse><wprt:EventRate>%s</wprt:EventRate>'
                '</wprt:SetEventRateResponse>' % rate)
        hdr = _hdr(A_SETEVENTRATE_RESP, _new_msgid(), relates_to=relates_to,
                   to=NS_WSA + "/role/anonymous")
        return _envelope(hdr, body)

    # -- WS-Eventing -------------------------------------------------------
    def _subscribe_resp(self, relates_to, req_text):
        """Answer a WS-Eventing Subscribe with a SubscribeResponse.

        We accept the subscription (echoing the requested Expires) and hand
        back a SubscriptionManager the client addresses Renew/Unsubscribe to.
        The pseudo-host does not (yet) push events to the NotifyTo sink, so
        the subscription is effectively a no-op stream — but answering the
        Subscribe correctly is what stops the spooler's event client from
        crashing on a malformed reply, which is what stranded print jobs.
        """
        dev = WSDDevice(self.host)
        expires = _find(req_text, "Expires") or "PT1H"
        sub_id = _new_msgid()

        # Record the subscription so we can push events to it. The NotifyTo
        # address is the sink Windows opened to receive notifications.
        dest = _notify_dest(req_text)
        filt = _find(req_text, "Filter") or ""
        if dest is not None:
            self._subs.append({"id": sub_id, "dest": dest, "filter": filt})
            # Prime the subscriber with the current printer status, which
            # also proves the outbound event path works.
            self._queue_event(dest, self._printer_change_event(sub_id, dest))

        body = (
            '<wse:SubscribeResponse><wse:SubscriptionManager>'
            '<wsa:Address>%s</wsa:Address>'
            '<wsa:ReferenceParameters>'
            '<wse:Identifier>%s</wse:Identifier>'
            '</wsa:ReferenceParameters>'
            '</wse:SubscriptionManager>'
            '<wse:Expires>%s</wse:Expires>'
            '</wse:SubscribeResponse>'
            % (dev.xaddr(), sub_id, expires))
        hdr = _hdr(A_SUBSCRIBE_RESP, _new_msgid(), relates_to=relates_to,
                   to=NS_WSA + "/role/anonymous")
        return _envelope(hdr, body)

    # -- outbound event delivery ------------------------------------------
    def notify_printer_change(self):
        """Queue a PrinterElementsChangeEvent for every subscriber. Called by
        the IPP printer when a job arrives/completes so Windows gets a live
        status push instead of only seeing changes when it next polls."""
        for sub in self._subs:
            self._queue_event(sub["dest"],
                              self._printer_change_event(sub["id"], sub["dest"]))

    def _queue_event(self, dest, soap):
        self._event_q.append((dest, _http_post(dest, soap)))

    def _printer_change_event(self, sub_id, dest):
        queued = self._queued_job_count()
        state = "Processing" if queued else "Idle"
        status = (
            '<wprt:PrinterStatus><wprt:PrinterState>%s</wprt:PrinterState>'
            '<wprt:PrinterPrimaryStateReason>None'
            '</wprt:PrinterPrimaryStateReason>'
            '<wprt:QueuedJobCount>%d</wprt:QueuedJobCount>'
            '</wprt:PrinterStatus>' % (state, queued))
        body = (
            '<wprt:PrinterElementsChangeEvent><wprt:PrinterElements>'
            '<wprt:ElementData Name="wprt:PrinterStatus" Valid="true">%s'
            '</wprt:ElementData></wprt:PrinterElements>'
            '</wprt:PrinterElementsChangeEvent>' % status)
        # The subscription's Identifier is echoed as a reference parameter
        # in the notification header so the client can match it.
        hdr = ('<wsa:To>%s</wsa:To><wse:Identifier>%s</wse:Identifier>'
               % (dest["url"], sub_id))
        hdr += ('<wsa:Action>%s/PrinterElementsChangeEvent</wsa:Action>'
                '<wsa:MessageID>%s</wsa:MessageID>' % (NS_WPRT, _new_msgid()))
        return _envelope(hdr, body)

    def _queued_job_count(self):
        # 0 unless a job is mid-flight; the IPP printer marks jobs complete
        # immediately, so this is a best-effort snapshot.
        return 0

    def tick(self):
        # Flush queued events one connection per tick; the outbound TCP
        # handshake completes across a few poll cycles.
        if not self._event_q:
            return
        tcp = getattr(self.host, "tcp", None)
        if tcp is None:
            self._event_q.clear()
            return
        dest, req = self._event_q.pop(0)
        try:
            tcp.connect(dest["ip"], dest["port"], _EventPoster(req))
        except Exception as e:
            self.log("event push to %s failed: %s" % (dest.get("url"), e))

    def _renew_resp(self, relates_to, req_text):
        expires = _find(req_text, "Expires") or "PT1H"
        body = ('<wse:RenewResponse><wse:Expires>%s</wse:Expires>'
                '</wse:RenewResponse>' % expires)
        hdr = _hdr(A_RENEW_RESP, _new_msgid(), relates_to=relates_to,
                   to=NS_WSA + "/role/anonymous")
        return _envelope(hdr, body)

    def _getstatus_resp(self, relates_to, req_text):
        expires = _find(req_text, "Expires") or "PT1H"
        body = ('<wse:GetStatusResponse><wse:Expires>%s</wse:Expires>'
                '</wse:GetStatusResponse>' % expires)
        hdr = _hdr(A_GETSTATUS_RESP, _new_msgid(), relates_to=relates_to,
                   to=NS_WSA + "/role/anonymous")
        return _envelope(hdr, body)

    def _unsubscribe_resp(self, relates_to):
        hdr = _hdr(A_UNSUBSCRIBE_RESP, _new_msgid(), relates_to=relates_to,
                   to=NS_WSA + "/role/anonymous")
        return _envelope(hdr, "")           # empty body per WS-Eventing

    def _fault(self, relates_to, action):
        """A SOAP 1.2 ActionNotSupported fault for an operation we don't do."""
        body = (
            '<soap:Fault><soap:Code><soap:Value>soap:Sender</soap:Value>'
            '<soap:Subcode><soap:Value>wsa:ActionNotSupported</soap:Value>'
            '</soap:Subcode></soap:Code><soap:Reason>'
            '<soap:Text xml:lang="en">Action not supported: %s</soap:Text>'
            '</soap:Reason></soap:Fault>' % action)
        hdr = _hdr(NS_WSA + "/fault", _new_msgid(), relates_to=relates_to,
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


def _notify_dest(sub_text):
    """Parse the WS-Eventing NotifyTo sink out of a Subscribe request.

    Returns {"url", "ip", "port", "path"} or None. The address looks like
    http://192.168.1.99:5357/<uuid>; only IPv4 literals are handled (the
    virtual network has no DNS)."""
    m = re.search(r"NotifyTo>.*?<[\w:]*Address>\s*([^<\s]+)", sub_text, re.S)
    if not m:
        return None
    url = m.group(1).strip()
    u = re.match(r"https?://(\d+\.\d+\.\d+\.\d+)(?::(\d+))?(/[^\s]*)?$", url)
    if not u:
        return None
    return {"url": url, "ip": u.group(1),
            "port": int(u.group(2) or 80), "path": u.group(3) or "/"}


def _http_post(dest, soap_bytes):
    """Frame a SOAP body as an HTTP POST to a NotifyTo sink."""
    return (
        ("POST %s HTTP/1.1\r\nHost: %s:%d\r\n"
         "Content-Type: application/soap+xml; charset=utf-8\r\n"
         "Content-Length: %d\r\nConnection: close\r\n\r\n"
         % (dest["path"], dest["ip"], dest["port"], len(soap_bytes))).encode()
        + soap_bytes)


class _EventPoster:
    """A one-shot outbound-connection handler: on connect, send the request;
    close as soon as the peer answers (WS-Eventing sinks reply 202)."""

    def __init__(self, request):
        self._req = request

    def on_connect(self, conn):
        conn.send(self._req)

    def on_data(self, conn, _data):
        conn.close()

    def on_close(self, conn):
        pass


def wsd_services():
    """The pair of services a WSD device runs: discovery + metadata HTTP."""
    return [WSDiscoveryService(), WSDMetadataService()]
