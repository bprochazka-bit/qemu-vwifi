#
# vwifi-pseudohost — TCP layer test
#
# SPDX-License-Identifier: GPL-2.0-or-later
#
import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
sys.path.insert(0, os.path.dirname(__file__))

import struct                                           # noqa: E402

from tcp_client import ServerHost, ClientSim          # noqa: E402
from vwifi_pseudohost import netstack                   # noqa: E402
from vwifi_pseudohost import ieee80211 as dot11         # noqa: E402
from vwifi_pseudohost import tcp                         # noqa: E402
from vwifi_pseudohost.services import TCPService       # noqa: E402
from vwifi_pseudohost.netservices import (              # noqa: E402
    LPDService, HTTPService)


class EchoTCP(TCPService):
    name = "echo-tcp"
    tcp_ports = (7,)

    def on_connect(self, conn):
        conn.data["log"] = []

    def on_data(self, conn, data):
        conn.send(data)                        # echo it straight back


class BannerTCP(TCPService):
    name = "banner"
    tcp_ports = (9999,)

    def on_connect(self, conn):
        conn.send(b"HELLO FROM PSEUDOHOST\r\n")


class BigTCP(TCPService):
    """Sends one payload larger than the MSS in a single send() call."""

    name = "big"
    tcp_ports = (1234,)
    PAYLOAD = bytes((i * 7) & 0xFF for i in range(5000))

    def on_connect(self, conn):
        conn.send(self.PAYLOAD)


class TestTCP(unittest.TestCase):
    def _server(self, service, port):
        srv = ServerHost()
        s = service()
        s.bind(srv)
        # TCPService.bind registered the listener on srv.tcp
        return srv

    def test_handshake_and_echo(self):
        srv = self._server(EchoTCP, 7)
        c = ClientSim(srv, dport=7)
        self.assertTrue(c.connect(), "TCP handshake failed")
        c.send(b"round-trip")
        self.assertEqual(c.recv(), b"round-trip")

    def test_banner_on_connect(self):
        srv = self._server(BannerTCP, 9999)
        c = ClientSim(srv, dport=9999)
        self.assertTrue(c.connect())
        self.assertEqual(c.recv(), b"HELLO FROM PSEUDOHOST\r\n")

    def test_multiple_sends(self):
        srv = self._server(EchoTCP, 7)
        c = ClientSim(srv, dport=7)
        self.assertTrue(c.connect())
        c.send(b"one ")
        self.assertEqual(c.recv(), b"one ")
        c.send(b"two")
        self.assertEqual(c.recv(), b"two")

    def test_large_response_is_segmented_to_mss(self):
        # A response bigger than the MSS must go out as several segments,
        # each small enough to fit one un-fragmented link frame, and the
        # client must reassemble the exact bytes.  Regression: a single
        # oversized segment is silently dropped by real peers (WSDAPI).
        srv = self._server(BigTCP, 1234)
        c = ClientSim(srv, dport=1234)
        self.assertTrue(c.connect())
        got = c.recv()
        self.assertEqual(got, BigTCP.PAYLOAD)

        # Inspect the raw egress: every data segment's IP packet must be
        # within a 1500-byte MTU, and it must have taken more than one.
        data_segs = 0
        for _dst, et, sdu in srv.station.sent:
            if et != dot11.ETH_P_IP or sdu[9] != netstack.IPPROTO_TCP:
                continue
            ihl = (sdu[0] & 0x0F) * 4
            seg = sdu[ihl:]
            off_flags = struct.unpack_from(">H", seg, 12)[0]
            payload = seg[(off_flags >> 12) * 4:]
            if payload:
                data_segs += 1
                self.assertLessEqual(len(sdu), 1500, "IP packet exceeds MTU")
                self.assertLessEqual(len(payload), tcp.MSS)
        self.assertGreater(data_segs, 1, "payload was not segmented")

    def test_connection_to_closed_port_resets(self):
        srv = self._server(EchoTCP, 7)
        c = ClientSim(srv, dport=8)            # nothing listening on 8
        # connect() returns False because it gets RST, not SYN|ACK
        self.assertFalse(c.connect())

    def test_no_rst_on_straggler_after_clean_close(self):
        # A retransmitted FIN/ACK arriving after a clean close must be
        # ignored, not answered with a RST. Windows sends such stragglers,
        # and a RST right after a WSD metadata fetch makes it treat the
        # transfer as aborted and refuse to install the device.
        srv = self._server(EchoTCP, 7)
        c = ClientSim(srv, dport=7, sport=55001)
        self.assertTrue(c.connect())
        # client initiates the close
        c._to_server(tcp.FIN | tcp.ACK)
        c.snd_nxt = tcp._u32(c.snd_nxt + 1)
        # take the server's ACK + FIN, then ACK its FIN to finish the close
        for flags, seq, _ack, _p in c._from_server():
            if flags & tcp.FIN:
                c.rcv_nxt = tcp._u32(seq + 1)
        c._to_server(tcp.ACK)                  # completes close -> conn dropped
        # a straggling retransmit of the client's FIN/ACK
        c._to_server(tcp.FIN | tcp.ACK)
        rst = [f for f, _s, _a, _p in c._from_server() if f & tcp.RST]
        self.assertEqual(rst, [], "spurious RST after a clean close")

    def test_syn_to_closed_port_still_resets_after_a_prior_close(self):
        # The TIME_WAIT-style suppression must not swallow a genuine SYN to a
        # port with no listener — that must still fail fast with a RST.
        srv = self._server(EchoTCP, 7)
        c = ClientSim(srv, dport=7, sport=55002)
        self.assertTrue(c.connect())
        c._to_server(tcp.FIN | tcp.ACK)
        c.snd_nxt = tcp._u32(c.snd_nxt + 1)
        for flags, seq, _a, _p in c._from_server():
            if flags & tcp.FIN:
                c.rcv_nxt = tcp._u32(seq + 1)
        c._to_server(tcp.ACK)
        # brand-new connection to a dead port still gets reset
        c2 = ClientSim(srv, dport=8, sport=55003)
        self.assertFalse(c2.connect())


class TestNetServices(unittest.TestCase):
    def test_http_serves_page(self):
        srv = ServerHost()
        HTTPService().bind(srv)
        c = ClientSim(srv, dport=80)
        self.assertTrue(c.connect())
        c.send(b"GET / HTTP/1.0\r\nHost: x\r\n\r\n")
        resp = c.recv()
        self.assertTrue(resp.startswith(b"HTTP/1.0 200 OK"))
        self.assertIn(b"<h1>", resp)

    def test_lpd_receives_a_job(self):
        srv = ServerHost()
        lpd = LPDService()
        lpd.bind(srv)
        c = ClientSim(srv, dport=515)
        self.assertTrue(c.connect())

        # RFC 1179: receive-a-job, then a control file, then a data file.
        c.send(b"\x02lp\n")
        self.assertEqual(c.recv(), b"\x00")

        ctl = b"Hlaptop\nPuser\nfdfA001laptop\n"
        c.send(b"\x02%d cfA001laptop\n" % len(ctl))
        self.assertEqual(c.recv(), b"\x00")
        c.send(ctl + b"\x00")
        self.assertEqual(c.recv(), b"\x00")

        job = b"the print job payload\n"
        c.send(b"\x03%d dfA001laptop\n" % len(job))
        self.assertEqual(c.recv(), b"\x00")
        c.send(job + b"\x00")
        self.assertEqual(c.recv(), b"\x00")

        # The daemon should have accounted the data-file bytes.
        # (one connection -> one conn object in the stack)
        conns = list(srv.tcp.conns.values())
        self.assertTrue(conns)
        self.assertEqual(conns[0].data["job"], len(job))


if __name__ == "__main__":
    unittest.main()
