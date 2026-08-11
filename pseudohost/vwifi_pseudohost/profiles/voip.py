#
# vwifi-pseudohost — VoIP desk phone profile
#
# SPDX-License-Identifier: GPL-2.0-or-later
#
# A SIP hardphone (think Cisco/Polycom/Yealink desk phone).  Its
# recognisable surface is SIP signalling on 5060 (UDP and TCP), TLS on
# 5061, an RTP media range, and a provisioning path (HTTP/HTTPS, TFTP).
# The fingerprint is an embedded Linux stack: TTL 64, a vendor-ish MAC.
#
# The SIP responder below is live over UDP today: it answers OPTIONS —
# the keepalive/probe most SIP scanners and PBXes send — with a 200 OK
# carrying an Allow header and a phone-like User-Agent, so an
# `svmap`/`sipsak` sweep or a PBX options-ping actually sees a phone.
# Registration and call setup are a later phase; the ports are advertised
# so the surface is right now.
#
from ..host import PseudoHost
from ..services import Service

SIP_PORT = 5060


class SipPhoneService(Service):
    """A minimal SIP UAS: answers OPTIONS with 200 OK."""

    name = "sip"
    udp_ports = (SIP_PORT,)
    tcp_ports = (SIP_PORT, 5061, 80, 443)      # SIP/TCP, SIP/TLS, prov UI

    user_agent = "PolyEdge/PseudoPhone-1.0"

    def on_udp(self, src_ip, src_port, dst_ip, dst_port, payload):
        try:
            text = payload.decode("utf-8", "replace")
        except Exception:
            return
        line0 = text.split("\r\n", 1)[0]
        method = line0.split(" ", 1)[0].upper()
        if method != "OPTIONS":
            # A real phone answers more, but only OPTIONS is safe to
            # answer blind without call state; ignore the rest for now.
            return
        headers = self._headers(text)
        via = headers.get("via", "")
        frm = headers.get("from", "")
        to = headers.get("to", "")
        callid = headers.get("call-id", "")
        cseq = headers.get("cseq", "1 OPTIONS")
        contact = "<sip:%s@%s:%d>" % (self.host.hostname,
                                      _ip(self.stack.ip or bytes(4)), SIP_PORT)
        resp = "\r\n".join([
            "SIP/2.0 200 OK",
            "Via: %s" % via,
            "From: %s" % frm,
            "To: %s;tag=ph%04x" % (to, (src_port ^ 0xA5A5) & 0xFFFF),
            "Call-ID: %s" % callid,
            "CSeq: %s" % cseq,
            "Contact: %s" % contact,
            "Allow: INVITE, ACK, CANCEL, OPTIONS, BYE, REFER, NOTIFY",
            "Accept: application/sdp",
            "User-Agent: %s" % self.user_agent,
            "Content-Length: 0",
            "", "",
        ])
        self.log("OPTIONS from %s:%d -> 200 OK" % (_ip(src_ip), src_port))
        self.reply_udp(src_ip, src_port, resp.encode("utf-8"),
                       src_port=dst_port)

    @staticmethod
    def _headers(text):
        out = {}
        for line in text.split("\r\n")[1:]:
            if not line or ":" not in line:
                continue
            k, v = line.split(":", 1)
            out.setdefault(k.strip().lower(), v.strip())
        return out


class VoIPPhone(PseudoHost):
    persona = "voip-phone"
    os_ttl = 64                                # embedded Linux SIP stack
    hostname = "sip-phone-ph01"
    mac_oui = bytes([0x02, 0x1B, 0x77])        # vendor-ish locally-administered
    icmp = True
    services = [SipPhoneService]


def _ip(b):
    return ".".join(str(x) for x in b)
