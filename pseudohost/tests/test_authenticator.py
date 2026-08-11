#
# vwifi-pseudohost — authenticator <-> supplicant handshake test
#
# SPDX-License-Identifier: GPL-2.0-or-later
#
# Runs the real AP-side Authenticator against the real STA-side
# Supplicant, end to end, and checks both derive the same pairwise key
# and the group key crosses intact.  This is the pair a pseudo-AP and a
# pseudo-host actually use, so it is the important one.
#
import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from vwifi_pseudohost import crypto, supplicant             # noqa: E402
from vwifi_pseudohost import ieee80211 as dot11              # noqa: E402
from vwifi_pseudohost.authenticator import Authenticator     # noqa: E402


class TestAuthSupplicant(unittest.TestCase):
    def _run(self, ap_pass, sta_pass, ssid=b"Lab-AP-1"):
        aa = dot11.mac_bytes("02:11:22:33:44:00")
        spa = dot11.mac_bytes("02:aa:bb:cc:dd:ee")
        rsn = dot11.rsn_ie_ccmp_psk()
        gtk = os.urandom(16)

        auth = Authenticator(crypto.wpa_pmk(ap_pass, ssid), aa, spa, gtk,
                             gtk_key_id=2, rsn_ie=rsn)
        supp = supplicant.Supplicant(crypto.wpa_pmk(sta_pass, ssid), spa, aa,
                                     rsn)

        m1 = auth.start()
        m2 = supp.handle(m1)
        self.assertIsNotNone(m2)
        m3 = auth.handle(m2)                    # may raise on bad passphrase
        self.assertIsNotNone(m3)
        m4 = supp.handle(m3)
        self.assertIsNotNone(m4)
        self.assertIsNone(auth.handle(m4))
        return auth, supp, gtk

    def test_matching_passphrase(self):
        auth, supp, gtk = self._run("correcthorse1", "correcthorse1")
        self.assertTrue(auth.completed)
        self.assertTrue(supp.completed)
        self.assertEqual(auth.tk, supp.tk)      # same pairwise key
        self.assertEqual(supp.gtk, gtk)         # group key crossed intact
        self.assertEqual(supp.gtk_key_id, 2)

    def test_mismatched_passphrase_raises(self):
        with self.assertRaises(supplicant.HandshakeError):
            self._run("correcthorse1", "wrongwrong")

    def test_installed_keys_interoperate_over_ccmp(self):
        # The whole point: the keys the two sides install must decrypt
        # each other's CCMP frames.
        auth, supp, _gtk = self._run("correcthorse1", "correcthorse1")
        ap_aes = crypto.AES128(auth.tk)
        sta_aes = crypto.AES128(supp.tk)
        # a dummy 802.11 data frame
        frame = bytearray(24)
        frame[0] = 0x08
        frame[1] = 0x01
        frame[4:10] = dot11.mac_bytes("02:11:22:33:44:00")
        frame[10:16] = dot11.mac_bytes("02:aa:bb:cc:dd:ee")
        frame[16:22] = dot11.mac_bytes("02:00:00:00:00:99")
        frame += b"payload-across-the-link"
        enc = crypto.ccmp_encrypt(sta_aes, bytes(frame), bytes([0, 0, 0, 0, 0, 1]))
        out = crypto.ccmp_decrypt(ap_aes, enc)
        self.assertIsNotNone(out)
        self.assertEqual(out[0], bytes(frame))


if __name__ == "__main__":
    unittest.main()
