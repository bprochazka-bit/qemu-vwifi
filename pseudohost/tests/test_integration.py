#
# vwifi-pseudohost — end-to-end integration tests
#
# SPDX-License-Identifier: GPL-2.0-or-later
#
# Stands a pseudo-host up against the in-process mock hub+AP over a real
# Unix socket and drives it through the whole path: scan by ESSID, find
# the channel, associate, (WPA2) run the four-way handshake, and pull a
# DHCP lease.  This is the test that says "the deliverable works".
#
import os
import sys
import time
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
sys.path.insert(0, os.path.dirname(__file__))

from mock_ap import MockAP                              # noqa: E402
from vwifi_pseudohost.profiles import LinuxWorkstation   # noqa: E402
from vwifi_pseudohost.host import PseudoHost             # noqa: E402


def _run_until_lease(hostobj, timeout=12.0):
    deadline = time.time() + timeout
    while time.time() < deadline:
        hostobj.run(duration=0.2)
        if hostobj.dhcp.bound():
            return True
    return False


class TestOpenNetwork(unittest.TestCase):
    def test_connect_and_lease(self):
        ap = MockAP(ssid="OpenLab", passphrase=None)
        ap.start()
        try:
            h = PseudoHost(ap.sock_path, "OpenLab", passphrase=None,
                           node_id="it-open", log=lambda m: None)
            self.assertTrue(h.start(connect_timeout=10.0), "did not associate")
            self.assertEqual(h.station.channel_freq, ap.freq)
            self.assertTrue(_run_until_lease(h), "no DHCP lease")
            self.assertEqual(tuple(h.ip), ap.lease_ip)
            h.close()
        finally:
            ap.stop()


class TestWPA2Network(unittest.TestCase):
    def test_handshake_and_lease(self):
        ap = MockAP(ssid="Lab-AP-1", passphrase="correcthorse1")
        ap.start()
        try:
            h = LinuxWorkstation(ap.sock_path, "Lab-AP-1",
                                 passphrase="correcthorse1",
                                 node_id="it-wpa2", log=lambda m: None)
            self.assertTrue(h.start(connect_timeout=10.0),
                            "did not associate/key")
            # keys installed on both ends
            self.assertIsNotNone(h.station.keys.ptk_aes)
            self.assertIsNotNone(h.station.keys.gtk_aes)
            # give the AP a moment to process msg4 before asserting keyed
            for _ in range(10):
                h.run(duration=0.1)
                if ap.keyed:
                    break
            self.assertTrue(ap.keyed, "AP never installed keys")
            self.assertTrue(_run_until_lease(h), "no DHCP lease over CCMP")
            self.assertEqual(tuple(h.ip), ap.lease_ip)
            self.assertTrue(ap.saw_dhcp_ack)
            h.close()
        finally:
            ap.stop()

    def test_wrong_passphrase_fails(self):
        ap = MockAP(ssid="Lab-AP-1", passphrase="correcthorse1")
        ap.start()
        try:
            h = PseudoHost(ap.sock_path, "Lab-AP-1",
                           passphrase="totallywrong",
                           node_id="it-bad", log=lambda m: None)
            self.assertFalse(h.start(connect_timeout=6.0),
                             "must not associate with the wrong key")
            h.close()
        finally:
            ap.stop()


if __name__ == "__main__":
    unittest.main()
