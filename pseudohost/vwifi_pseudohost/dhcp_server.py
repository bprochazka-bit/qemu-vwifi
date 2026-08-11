#
# vwifi-pseudohost — DHCPv4 server
#
# SPDX-License-Identifier: GPL-2.0-or-later
#
# The lease side of a pseudo-AP: it hands associated stations an address
# so they can finish "coming online" without a real DHCP server on the
# medium.  A small, single-subnet server — a monotonic pool, MAC-keyed
# leases, DISCOVER/REQUEST answered with OFFER/ACK, the standard subnet /
# router / DNS / lease options.  It binds to the AP's netstack UDP:67 and
# replies unicast to the station's MAC (the netstack's L2 override), which
# the AP link turns into a FromDS frame to that station.
#
import struct
import time

from . import dhcp
from . import netstack


class DHCPServer:
    def __init__(self, stack, server_ip, netmask="255.255.255.0",
                 gateway=None, dns=None, pool_start=None, pool_end=None,
                 lease_secs=3600, log=None):
        self.stack = stack
        self.server_ip = netstack.ip_bytes(server_ip)
        self.netmask = netstack.ip_bytes(netmask)
        self.gateway = netstack.ip_bytes(gateway) if gateway else self.server_ip
        self.dns = netstack.ip_bytes(dns) if dns else self.server_ip
        self.lease_secs = lease_secs
        self.log = log or (lambda *a: None)

        base = list(self.server_ip)
        self.pool_start = (netstack.ip_bytes(pool_start) if pool_start
                           else bytes(base[:3] + [100]))
        self.pool_end = (netstack.ip_bytes(pool_end) if pool_end
                         else bytes(base[:3] + [200]))
        self._next = self.pool_start[3]
        self.leases = {}                       # mac -> ip_bytes
        stack.register_udp(dhcp.DHCP_SERVER_PORT, self._on_udp)

    # -- allocation --------------------------------------------------------
    def _allocate(self, mac):
        if mac in self.leases:
            return self.leases[mac]
        if self._next > self.pool_end[3]:
            return None                         # pool exhausted
        ip = bytes(self.pool_start[:3] + bytes([self._next]))
        self._next += 1
        self.leases[mac] = ip
        return ip

    # -- ingress -----------------------------------------------------------
    def _on_udp(self, src_ip, src_port, dst_ip, dst_port, payload):
        if src_port != dhcp.DHCP_CLIENT_PORT or len(payload) < 240:
            return
        if payload[236:240] != dhcp.MAGIC_COOKIE:
            return
        xid = payload[4:8]
        chaddr = bytes(payload[28:34])
        opts = dhcp.DHCPClient._parse_opts(payload[240:])
        mtype = opts.get(dhcp.OPT_MSGTYPE, b"\x00")[0]
        if mtype == dhcp.DISCOVER:
            ip = self._allocate(chaddr)
            if ip:
                self.log("DHCP DISCOVER from %s -> OFFER %s" % (
                    _mac(chaddr), netstack.ip_str(ip)))
                self._reply(xid, chaddr, ip, dhcp.OFFER)
        elif mtype == dhcp.REQUEST:
            req_ip = opts.get(dhcp.OPT_REQ_IP)
            ip = self._allocate(chaddr)
            if req_ip and ip and bytes(req_ip) != ip:
                self._reply(xid, chaddr, ip, dhcp.NAK)
                return
            if ip:
                self.log("DHCP REQUEST from %s -> ACK %s" % (
                    _mac(chaddr), netstack.ip_str(ip)))
                self._reply(xid, chaddr, ip, dhcp.ACK)
        elif mtype == dhcp.RELEASE:
            self.leases.pop(chaddr, None)

    # -- egress ------------------------------------------------------------
    def _reply(self, xid, chaddr, yiaddr, mtype):
        bootp = struct.pack(
            ">BBBBIHH4s4s4s4s16s64s128s",
            2, 1, 6, 0, struct.unpack(">I", xid)[0], 0, 0,
            bytes(4), yiaddr if mtype != dhcp.NAK else bytes(4),
            self.server_ip, bytes(4),
            chaddr + bytes(10), bytes(64), bytes(128))
        opts = bytearray(dhcp.MAGIC_COOKIE)
        opts += bytes([dhcp.OPT_MSGTYPE, 1, mtype])
        opts += bytes([dhcp.OPT_SERVER_ID, 4]) + self.server_ip
        if mtype != dhcp.NAK:
            opts += bytes([dhcp.OPT_SUBNET, 4]) + self.netmask
            opts += bytes([dhcp.OPT_ROUTER, 4]) + self.gateway
            opts += bytes([dhcp.OPT_DNS, 4]) + self.dns
            opts += bytes([dhcp.OPT_LEASE, 4]) + struct.pack(">I",
                                                             self.lease_secs)
        opts += bytes([dhcp.OPT_END])
        payload = bootp + bytes(opts)
        # Reply unicast to the station's hardware address; the AP link
        # turns the L2 override into a FromDS frame to that station.
        self.stack.send_udp("255.255.255.255", dhcp.DHCP_CLIENT_PORT, payload,
                            src_port=dhcp.DHCP_SERVER_PORT,
                            src_ip=self.server_ip, dst_mac=chaddr)


def _mac(b):
    return ":".join("%02x" % x for x in b)
