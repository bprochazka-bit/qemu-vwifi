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


class WSDMetadataService(TCPService):
    """Serves the WS-Transfer Get (device metadata) Windows fetches from
    the XAddrs URL in the ProbeMatch."""

    name = "wsd-http"
    tcp_ports = (WSD_HTTP_PORT,)

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
            m = re.search(rb"content-length:\s*(\d+)", bytes(buf), re.I)
            conn.data["need"] = head_end + 4 + (int(m.group(1)) if m else 0)
        if len(buf) < conn.data["need"]:
            return                              # wait for the full body
        body_text = bytes(buf).decode("utf-8", "replace")
        req_msgid = _find(body_text, "MessageID") or ""
        resp = self._metadata(req_msgid)
        http = (b"HTTP/1.1 200 OK\r\n"
                b"Content-Type: application/soap+xml; charset=utf-8\r\n"
                b"Content-Length: " + str(len(resp)).encode() + b"\r\n"
                b"Connection: close\r\n\r\n" + resp)
        self.log("metadata Get -> 200")
        conn.send(http)
        conn.close()

    def _metadata(self, relates_to):
        dev = WSDDevice(self.host)
        # ThisModel + ThisDevice + Relationship(Host + hosted PrintService)
        this_model = (
            '<wsdp:ThisModel><wsdp:Manufacturer>%s</wsdp:Manufacturer>'
            '<wsdp:ModelName>%s</wsdp:ModelName>'
            '<wsdp:ModelNumber>%s</wsdp:ModelNumber>'
            '<wsdp:PresentationUrl>http://%s/</wsdp:PresentationUrl>'
            '</wsdp:ThisModel>'
            % (dev.manufacturer, dev.model, dev.model_number,
               _ip(self.host.stack.ip or bytes(4))))
        this_device = (
            '<wsdp:ThisDevice><wsdp:FriendlyName>%s</wsdp:FriendlyName>'
            '<wsdp:FirmwareVersion>1.0</wsdp:FirmwareVersion>'
            '<wsdp:SerialNumber>%s</wsdp:SerialNumber></wsdp:ThisDevice>'
            % (dev.friendly, self.host.mac.hex()))
        svc_id = dev.uuid + "/PrintService"
        relationship = (
            '<wsdp:Relationship Type="%s/host">'
            '<wsdp:Host><wsa:EndpointReference><wsa:Address>%s</wsa:Address>'
            '</wsa:EndpointReference><wsdp:Types>wprt:PrintServiceType'
            '</wsdp:Types><wsdp:ServiceId>%s</wsdp:ServiceId></wsdp:Host>'
            '</wsdp:Relationship>' % (NS_WSDP, dev.uuid, svc_id))
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


def _find(text, tag):
    m = re.search(r"<[\w:]*%s[^>]*>(.*?)</[\w:]*%s>" % (tag, tag), text,
                  re.DOTALL)
    return m.group(1).strip() if m else None


def _ip(b):
    return ".".join(str(x) for x in b)


def wsd_services():
    """The pair of services a WSD device runs: discovery + metadata HTTP."""
    return [WSDiscoveryService(), WSDMetadataService()]
