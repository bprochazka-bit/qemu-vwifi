#
# vwifi-pseudohost — tests for the device profiles and their services
#
# SPDX-License-Identifier: GPL-2.0-or-later
#
# The mDNS and SIP responders are driven directly with a FakeStation +
# NetStack (no medium), and every profile is stood up far enough to bind
# its services, so a broken advert or port list fails here rather than in
# the field.
#
import os
import struct
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from vwifi_pseudohost import mdns, netstack, ssdp          # noqa: E402
from vwifi_pseudohost import ieee80211 as dot11            # noqa: E402
from vwifi_pseudohost.profiles import (PROFILES, get_profile,  # noqa: E402
                                       profile_names)


class FakeStation:
    def __init__(self, mac):
        self.mac = mac
        self.on_eth_rx = None
        self.sent = []

    def send_eth(self, dst, ethertype, sdu):
        self.sent.append((bytes(dst), ethertype, bytes(sdu)))


class FakeHost:
    """Minimal stand-in for a PseudoHost, enough to bind a Service."""

    def __init__(self, hostname, mac, ip="192.168.5.20"):
        self.hostname = hostname
        self.mac = mac
        self.station = FakeStation(mac)
        self.stack = netstack.NetStack(self.station, hostname=hostname)
        self.stack.configure(ip, "255.255.255.0", "192.168.5.1")

    def log(self, msg):
        pass


def deliver_udp(host, src_ip, src_port, dst_port, payload,
                src_mac="02:00:00:00:00:aa"):
    """Feed a UDP datagram in through the real stack ingress.

    Going through _on_eth (rather than calling a service's on_udp
    directly) is what a real frame does: the netstack learns the
    sender's MAC, so a unicast reply can resolve ARP and actually go out.
    """
    src = netstack.ip_bytes(src_ip)
    dst = host.stack.ip
    udp = struct.pack(">HHHH", src_port, dst_port, 8 + len(payload), 0) + payload
    total = 20 + len(udp)
    iph = bytearray(struct.pack(">BBHHHBBH4s4s", 0x45, 0, total, 0, 0,
                                64, netstack.IPPROTO_UDP, 0, src, dst))
    c = netstack._cksum(bytes(iph))
    iph[10], iph[11] = c >> 8, c & 0xFF
    host.station.on_eth_rx(dot11.mac_bytes(src_mac), host.mac,
                           dot11.ETH_P_IP, bytes(iph) + udp)


def _last_udp_payload(station):
    """Pull the UDP payload out of the most recent egress IP frame."""
    for dst_mac, et, sdu in reversed(station.sent):
        if et != dot11.ETH_P_IP or sdu[9] != netstack.IPPROTO_UDP:
            continue
        ihl = (sdu[0] & 0x0F) * 4
        return sdu[ihl + 8:]
    return None


def _mdns_query(qname, qtype=mdns.T_PTR):
    hdr = struct.pack(">HHHHHH", 0, 0, 1, 0, 0, 0)
    q = mdns.encode_name(qname) + struct.pack(">HH", qtype, mdns.CLASS_IN)
    return hdr + q


def _parse_rrs(resp):
    """Return a list of (name, rtype) across answers+additionals."""
    (_id, _fl, qd, an, ns, ar) = struct.unpack_from(">HHHHHH", resp, 0)
    off = 12
    for _ in range(qd):
        _n, off = mdns._read_name(resp, off)
        off += 4
    out = []
    for _ in range(an + ns + ar):
        name, off = mdns._read_name(resp, off)
        rtype, _cls, _ttl, rdlen = struct.unpack_from(">HHIH", resp, off)
        off += 10 + rdlen
        out.append((name.lower(), rtype))
    return out


class TestMDNS(unittest.TestCase):
    def test_name_roundtrip(self):
        enc = mdns.encode_name("_googlecast._tcp.local")
        name, off = mdns._read_name(enc, 0)
        self.assertEqual(name, "_googlecast._tcp.local")
        self.assertEqual(off, len(enc))

    def test_chromecast_browse(self):
        h = FakeHost("Chromecast-Den", dot11.mac_bytes("02:ad:f8:11:22:33"))
        from vwifi_pseudohost.profiles.chromecast import _cast_mdns
        svc = _cast_mdns()
        svc.bind(h)
        svc.on_udp(netstack.ip_bytes("192.168.5.9"), 5353,
                   netstack.ip_bytes("224.0.0.251"), 5353,
                   _mdns_query("_googlecast._tcp.local"))
        resp = _last_udp_payload(h.station)
        self.assertIsNotNone(resp, "no mDNS response emitted")
        rrs = _parse_rrs(resp)
        types = {t for _n, t in rrs}
        self.assertIn(mdns.T_PTR, types)
        self.assertIn(mdns.T_SRV, types)
        self.assertIn(mdns.T_TXT, types)
        self.assertIn(mdns.T_A, types)                 # A resolves to our IP
        names = {n for n, _t in rrs}
        self.assertTrue(any("_googlecast._tcp.local" in n for n in names))

    def test_service_enumeration(self):
        h = FakeHost("HP-OfficeJet-Den", dot11.mac_bytes("02:60:b0:aa:bb:cc"))
        from vwifi_pseudohost.profiles.hp_mfp import _mfp_mdns
        svc = _mfp_mdns()
        svc.bind(h)
        svc.on_udp(netstack.ip_bytes("192.168.5.9"), 5353,
                   netstack.ip_bytes("224.0.0.251"), 5353,
                   _mdns_query(mdns.SERVICE_ENUM))
        resp = _last_udp_payload(h.station)
        self.assertIsNotNone(resp)
        rrs = _parse_rrs(resp)
        # Enumerates each advertised type as a PTR under _services...
        self.assertTrue(all(t == mdns.T_PTR for _n, t in rrs))
        self.assertGreaterEqual(len(rrs), 5)

    def test_uscan_advertised(self):
        # The scanner half is what makes it a multifunction, not a printer.
        h = FakeHost("HP-OfficeJet-Den", dot11.mac_bytes("02:60:b0:aa:bb:cc"))
        from vwifi_pseudohost.profiles.hp_mfp import _mfp_mdns
        svc = _mfp_mdns()
        svc.bind(h)
        svc.on_udp(netstack.ip_bytes("192.168.5.9"), 5353,
                   netstack.ip_bytes("224.0.0.251"), 5353,
                   _mdns_query("_uscan._tcp.local"))
        resp = _last_udp_payload(h.station)
        self.assertIsNotNone(resp, "MFP did not answer a scanner browse")


def _all_mdns_ip_packets(station):
    """Every mDNS egress IP packet (full IP datagram bytes)."""
    out = []
    for _dst, et, sdu in station.sent:
        if et != dot11.ETH_P_IP or sdu[9] != netstack.IPPROTO_UDP:
            continue
        out.append(sdu)
    return out


class FakeClock:
    def __init__(self, t=1000.0):
        self.t = t

    def __call__(self):
        return self.t


class TestMDNSDelivery(unittest.TestCase):
    """The bug this guards: a full record set crammed into one >MTU datagram
    is dropped on the wire, so mDNS discovery silently fails."""

    def _mfp(self):
        h = FakeHost("HP-OfficeJet-PH01", dot11.mac_bytes("00:01:e6:c1:23:45"),
                     ip="192.168.1.127")
        from vwifi_pseudohost.profiles.hp_mfp import _mfp_mdns
        svc = _mfp_mdns()
        svc.bind(h)
        return h, svc

    def test_announcement_split_under_mtu(self):
        h, svc = self._mfp()
        svc._armed = False
        svc.tick()                                   # one announcement pass
        pkts = _all_mdns_ip_packets(h.station)
        self.assertGreater(len(pkts), 1,
                           "announcement should split across packets")
        for sdu in pkts:
            self.assertLessEqual(len(sdu), 1500,
                                 "an mDNS datagram exceeded the link MTU")
            ihl = (sdu[0] & 0x0F) * 4
            self.assertLessEqual(len(sdu) - ihl - 8, mdns.MDNS_MAX_PAYLOAD)

    def test_browse_response_under_mtu(self):
        # An ANY browse of every advertised type returns a large record set.
        h, svc = self._mfp()
        h.station.sent.clear()
        for a in svc.adverts:
            svc.on_udp(netstack.ip_bytes("192.168.1.99"), 5353,
                       netstack.ip_bytes("224.0.0.251"), 5353,
                       _mdns_query(a.stype, mdns.T_ANY))
        for sdu in _all_mdns_ip_packets(h.station):
            self.assertLessEqual(len(sdu), 1500)

    def test_pack_groups_splits_and_preserves(self):
        # 40 groups of a ~200B record each must span multiple <=max packets.
        rr = mdns._rr("x._ipp._tcp.local", mdns.T_TXT, b"k=" + b"v" * 180)
        groups = [([rr], []) for _ in range(40)]
        pkts = mdns.pack_groups(groups, max_payload=1400)
        self.assertGreater(len(pkts), 1)
        for p in pkts:
            self.assertLessEqual(len(p), 1400)
        # Every record still shipped (answer counts sum to 40).
        total = sum(struct.unpack_from(">H", p, 6)[0] for p in pkts)
        self.assertEqual(total, 40)

    def test_startup_burst_then_periodic(self):
        h, svc = self._mfp()
        clock = FakeClock()
        orig = mdns.time.monotonic
        mdns.time.monotonic = clock
        try:
            rounds = 0
            for _ in range(4):                       # first ~4 seconds
                before = len(h.station.sent)
                svc.tick()
                if len(h.station.sent) > before:
                    rounds += 1
                clock.t += 1.0
            self.assertGreaterEqual(rounds, 2,
                                    "expected a startup announcement burst")
            # After the burst, it should not announce again every second.
            quiet_before = len(h.station.sent)
            svc.tick()
            self.assertEqual(len(h.station.sent), quiet_before)
        finally:
            mdns.time.monotonic = orig

    def test_plain_printer_profile_advertises_mdns(self):
        h = FakeHost("HPLJ-PH01", dot11.mac_bytes("02:60:b0:12:34:56"))
        from vwifi_pseudohost.profiles.printer import _printer_mdns
        svc = _printer_mdns()
        svc.bind(h)
        svc.on_udp(netstack.ip_bytes("192.168.5.9"), 5353,
                   netstack.ip_bytes("224.0.0.251"), 5353,
                   _mdns_query("_ipp._tcp.local"))
        resp = _last_udp_payload(h.station)
        self.assertIsNotNone(resp, "plain printer profile answered no IPP browse")
        types = {t for _n, t in _parse_rrs(resp)}
        self.assertIn(mdns.T_SRV, types)
        self.assertIn(mdns.T_TXT, types)


class TestSIP(unittest.TestCase):
    def test_options_200(self):
        h = FakeHost("sip-phone-den", dot11.mac_bytes("02:1b:77:11:22:33"))
        from vwifi_pseudohost.profiles.voip import SipPhoneService, SIP_PORT
        svc = SipPhoneService()
        svc.bind(h)
        req = ("OPTIONS sip:100@192.168.5.20 SIP/2.0\r\n"
               "Via: SIP/2.0/UDP 192.168.5.9:5060;branch=z9hG4bK123\r\n"
               "From: <sip:scan@192.168.5.9>;tag=abc\r\n"
               "To: <sip:100@192.168.5.20>\r\n"
               "Call-ID: deadbeef@192.168.5.9\r\n"
               "CSeq: 1 OPTIONS\r\n"
               "Content-Length: 0\r\n\r\n").encode()
        deliver_udp(h, "192.168.5.9", 5060, SIP_PORT, req)
        resp = _last_udp_payload(h.station)
        self.assertIsNotNone(resp, "no SIP response")
        text = resp.decode()
        self.assertTrue(text.startswith("SIP/2.0 200 OK"))
        self.assertIn("Call-ID: deadbeef@192.168.5.9", text)
        self.assertIn("CSeq: 1 OPTIONS", text)
        self.assertIn("Allow:", text)
        self.assertIn("User-Agent:", text)

    def test_ignores_non_options(self):
        h = FakeHost("sip-phone-den", dot11.mac_bytes("02:1b:77:11:22:33"))
        from vwifi_pseudohost.profiles.voip import SipPhoneService, SIP_PORT
        svc = SipPhoneService()
        svc.bind(h)
        deliver_udp(h, "192.168.5.9", 5060, SIP_PORT,
                    b"INVITE sip:100@x SIP/2.0\r\n\r\n")
        self.assertIsNone(_last_udp_payload(h.station))


def _msearch(st):
    return ("M-SEARCH * HTTP/1.1\r\n"
            "HOST: 239.255.255.250:1900\r\n"
            'MAN: "ssdp:discover"\r\n'
            "MX: 2\r\n"
            "ST: %s\r\n\r\n" % st).encode()


class TestSSDP(unittest.TestCase):
    def test_bambu_msearch_response(self):
        h = FakeHost("3DP-Bambu-Den", dot11.mac_bytes("02:83:8c:11:22:33"))
        from vwifi_pseudohost.profiles.bambu import _bambu_ssdp, BAMBU_NT
        svc = _bambu_ssdp()
        svc.bind(h)
        deliver_udp(h, "192.168.5.9", 51000, ssdp.SSDP_PORT + 90,  # 1990
                    _msearch(BAMBU_NT), src_mac="02:00:00:00:00:bb")
        resp = _last_udp_payload(h.station)
        self.assertIsNotNone(resp, "Bambu did not answer M-SEARCH")
        text = resp.decode()
        self.assertTrue(text.startswith("HTTP/1.1 200 OK"))
        self.assertIn("ST: " + BAMBU_NT, text)
        self.assertIn("DevModel.bambu.com:", text)
        self.assertIn("LOCATION: 192.168.5.20", text)      # bare IP

    def test_smartscreen_mediarenderer_response(self):
        h = FakeHost("SmartDisplay-Den", dot11.mac_bytes("02:71:47:aa:bb:cc"))
        from vwifi_pseudohost.profiles.smart_screen import _screen_ssdp
        svc = _screen_ssdp()
        svc.bind(h)
        deliver_udp(h, "192.168.5.9", 51000, ssdp.SSDP_PORT,
                    _msearch("urn:schemas-upnp-org:device:MediaRenderer:1"),
                    src_mac="02:00:00:00:00:cc")
        resp = _last_udp_payload(h.station)
        self.assertIsNotNone(resp)
        text = resp.decode()
        self.assertIn("MediaRenderer:1", text)
        self.assertIn("LOCATION: http://192.168.5.20:8080/description.xml",
                      text)

    def test_ssdp_all_matches_every_target(self):
        h = FakeHost("SmartDisplay-Den", dot11.mac_bytes("02:71:47:aa:bb:cc"))
        from vwifi_pseudohost.profiles.smart_screen import _screen_ssdp
        svc = _screen_ssdp()
        svc.bind(h)
        # ssdp:all should draw a reply for each advertised target.
        before = len(h.station.sent)
        deliver_udp(h, "192.168.5.9", 51000, ssdp.SSDP_PORT,
                    _msearch("ssdp:all"), src_mac="02:00:00:00:00:cc")
        # 3 targets (rootdevice, MediaRenderer, DIAL) -> 3 responses.
        ip_frames = [s for s in h.station.sent[before:]
                     if s[1] == dot11.ETH_P_IP]
        self.assertGreaterEqual(len(ip_frames), 3)


class TestAllProfilesBind(unittest.TestCase):
    def test_every_profile_instantiates_and_binds(self):
        for name in profile_names():
            cls = get_profile(name)
            h = FakeHost("unit-%s" % name,
                         dot11.mac_bytes("02:00:00:00:00:01"))
            # Bind each of the profile's declared services against a stack.
            for svc in cls.services:
                if isinstance(svc, type):
                    svc = svc()
                elif callable(svc) and not hasattr(svc, "bind"):
                    svc = svc()
                svc.bind(h)
                # port lists must be iterables of ints
                for p in list(svc.udp_ports) + list(svc.tcp_ports):
                    self.assertIsInstance(p, int)

    def test_fingerprints_are_distinct(self):
        ttls = {n: get_profile(n).os_ttl for n in profile_names()}
        self.assertEqual(ttls["chromecast"], 64)
        self.assertEqual(ttls["voip"], 64)
        self.assertEqual(ttls["hp-mfp"], 255)
        self.assertEqual(ttls["bambu"], 64)
        self.assertEqual(ttls["smart-screen"], 64)
        self.assertEqual(ttls["pos"], 64)


if __name__ == "__main__":
    unittest.main()
