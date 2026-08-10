#
# vwifi-pseudohost — DHCPv4 client
#
# SPDX-License-Identifier: GPL-2.0-or-later
#
# A textbook DISCOVER / OFFER / REQUEST / ACK exchange (RFC 2131), just
# enough to pull a lease and configure the netstack.  It is driven by
# the netstack's UDP demux: the client registers on port 68 and sends
# broadcast datagrams with an L2 broadcast destination, because until
# the ACK lands there is no IP and no ARP to resolve one.
#
import os
import struct
import time

from . import netstack

DHCP_SERVER_PORT = 67
DHCP_CLIENT_PORT = 68
MAGIC_COOKIE = bytes([99, 130, 83, 99])

# message types
DISCOVER, OFFER, REQUEST, DECLINE, ACK, NAK, RELEASE = range(1, 8)

# options
OPT_SUBNET = 1
OPT_ROUTER = 3
OPT_DNS = 6
OPT_HOSTNAME = 12
OPT_REQ_IP = 50
OPT_LEASE = 51
OPT_MSGTYPE = 53
OPT_SERVER_ID = 54
OPT_PARAM_LIST = 55
OPT_CLIENT_ID = 61
OPT_END = 255


class DHCPClient:
    def __init__(self, stack, hostname="pseudohost", log=None):
        self.stack = stack
        self.hostname = hostname
        self.log = log or (lambda *a: None)
        self.xid = struct.unpack(">I", os.urandom(4))[0]
        self.state = "init"
        self.offer = None
        self.lease = None                  # dict once bound
        self._server_id = None
        self._t_last = 0.0
        stack.register_udp(DHCP_CLIENT_PORT, self._on_udp)

    # ---- public ----------------------------------------------------------
    def start(self):
        self.state = "selecting"
        self._send_discover()

    def bound(self):
        return self.state == "bound"

    def tick(self):
        """Retransmit the current request if it's been too long."""
        if self.state in ("bound", "init"):
            return
        if time.time() - self._t_last > 2.0:
            if self.state == "selecting":
                self._send_discover()
            elif self.state == "requesting":
                self._send_request()

    # ---- packet build ----------------------------------------------------
    def _base(self, msgtype, options):
        # BOOTP header.  broadcast flag set so the server replies to bcast.
        chaddr = bytes(self.stack.mac) + bytes(10)
        hdr = struct.pack(">BBBBIHH4s4s4s4s16s64s128s",
                          1, 1, 6, 0, self.xid, 0, 0x8000,
                          bytes(4), bytes(4), bytes(4), bytes(4),
                          chaddr, bytes(64), bytes(128))
        opts = bytearray(MAGIC_COOKIE)
        opts += bytes([OPT_MSGTYPE, 1, msgtype])
        cid = bytes([0x01]) + bytes(self.stack.mac)
        opts += bytes([OPT_CLIENT_ID, len(cid)]) + cid
        hn = self.hostname.encode()[:63]
        opts += bytes([OPT_HOSTNAME, len(hn)]) + hn
        opts += bytes([OPT_PARAM_LIST, 4, OPT_SUBNET, OPT_ROUTER,
                       OPT_DNS, OPT_LEASE])
        opts += options
        opts += bytes([OPT_END])
        return hdr + bytes(opts)

    def _send_discover(self):
        self._t_last = time.time()
        pkt = self._base(DISCOVER, b"")
        self.log("dhcp: DISCOVER")
        self._broadcast(pkt)

    def _send_request(self):
        self._t_last = time.time()
        req_ip = self.offer["ip"]
        opts = bytes([OPT_REQ_IP, 4]) + req_ip
        opts += bytes([OPT_SERVER_ID, 4]) + self._server_id
        pkt = self._base(REQUEST, opts)
        self.log("dhcp: REQUEST %s" % netstack.ip_str(req_ip))
        self._broadcast(pkt)

    def _broadcast(self, pkt):
        self.stack.send_udp("255.255.255.255", DHCP_SERVER_PORT, pkt,
                            src_port=DHCP_CLIENT_PORT, src_ip=bytes(4),
                            dst_mac=netstack.BROADCAST_MAC)

    # ---- ingress ---------------------------------------------------------
    def _on_udp(self, src_ip, src_port, dst_ip, dst_port, payload):
        if src_port != DHCP_SERVER_PORT or len(payload) < 240:
            return
        xid = struct.unpack_from(">I", payload, 4)[0]
        if xid != self.xid:
            return
        yiaddr = bytes(payload[16:20])
        if payload[236:240] != MAGIC_COOKIE:
            return
        opts = self._parse_opts(payload[240:])
        mtype = opts.get(OPT_MSGTYPE, b"\x00")[0]
        if mtype == OFFER and self.state == "selecting":
            self.offer = {"ip": yiaddr, "opts": opts}
            self._server_id = opts.get(OPT_SERVER_ID, bytes(4))
            self.state = "requesting"
            self.log("dhcp: OFFER %s from %s" % (
                netstack.ip_str(yiaddr),
                netstack.ip_str(self._server_id)))
            self._send_request()
        elif mtype == ACK and self.state == "requesting":
            self._bind(yiaddr, opts)
        elif mtype == NAK:
            self.log("dhcp: NAK — restarting")
            self.state = "selecting"
            self.xid = struct.unpack(">I", os.urandom(4))[0]
            self._send_discover()

    def _bind(self, ip, opts):
        mask = opts.get(OPT_SUBNET, bytes([255, 255, 255, 0]))
        router = opts.get(OPT_ROUTER)
        gw = router[:4] if router else None
        dns = opts.get(OPT_DNS)
        dns1 = dns[:4] if dns else None
        lease_secs = struct.unpack(">I", opts.get(OPT_LEASE,
                                                  b"\x00\x00\x0e\x10"))[0]
        self.lease = {
            "ip": ip, "netmask": mask, "gateway": gw, "dns": dns1,
            "server": self._server_id, "lease_secs": lease_secs,
            "obtained": time.time(),
        }
        self.state = "bound"
        self.stack.configure(ip, mask, gw, dns1)
        self.log("dhcp: bound %s (lease %ds)" % (
            netstack.ip_str(ip), lease_secs))

    @staticmethod
    def _parse_opts(buf):
        out = {}
        i = 0
        n = len(buf)
        while i < n:
            code = buf[i]
            if code == OPT_END:
                break
            if code == 0:
                i += 1
                continue
            ln = buf[i + 1]
            out[code] = bytes(buf[i + 2 : i + 2 + ln])
            i += 2 + ln
        return out
