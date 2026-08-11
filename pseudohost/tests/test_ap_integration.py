#
# vwifi-pseudohost — pseudo-AP <-> pseudo-host end-to-end test
#
# SPDX-License-Identifier: GPL-2.0-or-later
#
# The real proof for the AP side: a PseudoAP and a PseudoHost, each a
# genuine peer, associate to each other across an in-process hub and the
# host pulls a DHCP lease from the AP — for both an open and a WPA2-PSK
# network.  Nothing here is mocked except the hub's fan-out.
#
import os
import sys
import threading
import time
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
sys.path.insert(0, os.path.dirname(__file__))

from pyhub import PyHub                                  # noqa: E402
from vwifi_pseudohost.accesspoint import PseudoAP         # noqa: E402
from vwifi_pseudohost.host import PseudoHost              # noqa: E402
from vwifi_pseudohost.netservices import LPDService       # noqa: E402


class _APThread:
    """Runs a PseudoAP in a background thread for the duration of a test."""

    def __init__(self, hub, **kw):
        self.ap = PseudoAP(hub.sock_path, log=lambda m: None, **kw)
        self._t = None

    def __enter__(self):
        self.ap.start()
        self._t = threading.Thread(target=self.ap.run, daemon=True)
        self._t.start()
        return self.ap

    def __exit__(self, *exc):
        self.ap.stop()
        self._t.join(timeout=2.0)
        self.ap.close()


def _lease(host, timeout=12.0):
    deadline = time.time() + timeout
    while time.time() < deadline:
        host.run(duration=0.2)
        if host.dhcp.bound():
            return True
    return False


class TestOpenAP(unittest.TestCase):
    def test_host_associates_and_leases(self):
        hub = PyHub()
        hub.start()
        try:
            with _APThread(hub, essid="PseudoOpen", channel=6,
                           encryption="open") as ap:
                h = PseudoHost(hub.sock_path, "PseudoOpen",
                               node_id="it-h-open", log=lambda m: None)
                self.assertTrue(h.start(connect_timeout=10.0),
                                "host did not associate")
                self.assertTrue(_lease(h), "no DHCP lease from the AP")
                # leased from the AP's pool, gateway is the AP
                self.assertEqual(h.ip[:3], bytes([192, 168, 4]))
                self.assertGreaterEqual(h.ip[3], 100)
                self.assertEqual(tuple(h.stack.gateway), (192, 168, 4, 1))
                self.assertIn(h.station.mac, ap.stations)
                h.close()
        finally:
            hub.stop()


class TestWPA2AP(unittest.TestCase):
    def test_handshake_and_lease(self):
        hub = PyHub()
        hub.start()
        try:
            with _APThread(hub, essid="PseudoSecure", channel=11,
                           encryption="wpa2", passphrase="hunter2hunter2",
                           bssid="02:de:ad:be:ef:01") as ap:
                h = PseudoHost(hub.sock_path, "PseudoSecure",
                               passphrase="hunter2hunter2",
                               node_id="it-h-wpa2", log=lambda m: None)
                self.assertTrue(h.start(connect_timeout=10.0),
                                "host did not associate/key")
                self.assertIsNotNone(h.station.keys.ptk_aes)
                self.assertIsNotNone(h.station.keys.gtk_aes)
                self.assertTrue(_lease(h), "no DHCP lease over CCMP")
                self.assertEqual(h.ip[:3], bytes([192, 168, 4]))
                # the AP keyed this station
                sta = ap.stations.get(h.station.mac)
                self.assertIsNotNone(sta)
                self.assertTrue(sta.keyed)
                # BSSID we asked for is the one it joined
                self.assertEqual(h.station.bssid,
                                 bytes.fromhex("02deadbeef01"))
                h.close()
        finally:
            hub.stop()

    def test_wrong_passphrase_does_not_associate(self):
        hub = PyHub()
        hub.start()
        try:
            with _APThread(hub, essid="PseudoSecure", channel=1,
                           encryption="wpa2", passphrase="rightpass1"):
                h = PseudoHost(hub.sock_path, "PseudoSecure",
                               passphrase="wrongpass1",
                               node_id="it-h-bad", log=lambda m: None)
                self.assertFalse(h.start(connect_timeout=6.0))
                h.close()
        finally:
            hub.stop()


if __name__ == "__main__":
    unittest.main()
