#
# vwifi-pseudohost — four-way handshake test
#
# SPDX-License-Identifier: GPL-2.0-or-later
#
# Stands up a minimal Authenticator (the hostapd side) in-process and
# runs a full WPA2-PSK handshake against the Supplicant, checking that
# the two independently derive the same PTK and that the GTK survives
# the AES-key-wrap round trip.
#
import os
import struct
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from vwifi_pseudohost import crypto, supplicant       # noqa: E402
from vwifi_pseudohost import ieee80211 as dot11        # noqa: E402
from vwifi_pseudohost.supplicant import (              # noqa: E402
    KI_TYPE_PAIRWISE, KI_ACK, KI_MIC, KI_INSTALL, KI_SECURE, KI_ENCRYPTED,
    _OFF_KEYINFO, _OFF_NONCE, _OFF_MIC, _OFF_KDLEN, _OFF_KEYDATA)


def build_msg1(anonce, replay):
    info = 2 | KI_TYPE_PAIRWISE | KI_ACK
    kd = b""
    f = bytearray(_OFF_KEYDATA)
    f[0] = supplicant.EAPOL_VERSION
    f[1] = supplicant.EAPOL_TYPE_KEY
    struct.pack_into(">H", f, 2, 95 + len(kd))
    f[4] = supplicant.KEY_DESC_RSN
    struct.pack_into(">H", f, _OFF_KEYINFO, info)
    struct.pack_into(">H", f, 7, 16)
    f[9:17] = replay
    f[_OFF_NONCE : _OFF_NONCE + 32] = anonce
    struct.pack_into(">H", f, _OFF_KDLEN, 0)
    return bytes(f)


def build_msg3(kck, kek, anonce, replay, gtk, gtk_id):
    # GTK KDE: dd len 00-0F-AC 01 keyid rsvd || gtk
    kde_body = bytes([0x00, 0x0F, 0xAC, 0x01, gtk_id & 0x03, 0x00]) + gtk
    kde = bytes([0xDD, len(kde_body)]) + kde_body
    if len(kde) % 8:                            # pad to an 8-byte boundary
        kde += b"\x00" * (8 - len(kde) % 8)
    wrapped = crypto.aes_key_wrap(kek, kde)
    info = 2 | KI_TYPE_PAIRWISE | KI_ACK | KI_MIC | KI_INSTALL | KI_SECURE \
        | KI_ENCRYPTED
    f = bytearray(_OFF_KEYDATA + len(wrapped))
    f[0] = supplicant.EAPOL_VERSION
    f[1] = supplicant.EAPOL_TYPE_KEY
    struct.pack_into(">H", f, 2, 95 + len(wrapped))
    f[4] = supplicant.KEY_DESC_RSN
    struct.pack_into(">H", f, _OFF_KEYINFO, info)
    struct.pack_into(">H", f, 7, 16)
    f[9:17] = replay
    f[_OFF_NONCE : _OFF_NONCE + 32] = anonce
    struct.pack_into(">H", f, _OFF_KDLEN, len(wrapped))
    f[_OFF_KEYDATA:] = wrapped
    f[_OFF_MIC : _OFF_MIC + 16] = b"\x00" * 16
    f[_OFF_MIC : _OFF_MIC + 16] = crypto.eapol_mic(kck, bytes(f))
    return bytes(f)


class TestHandshake(unittest.TestCase):
    def test_full_handshake(self):
        ssid = b"Lab-AP-1"
        passphrase = "correcthorse1"
        pmk = crypto.wpa_pmk(passphrase, ssid)
        aa = dot11.mac_bytes("02:11:22:33:44:00")
        spa = dot11.mac_bytes("02:aa:bb:cc:dd:ee")
        rsn = dot11.rsn_ie_ccmp_psk()

        sup = supplicant.Supplicant(pmk, spa, aa, rsn)

        anonce = os.urandom(32)
        replay = struct.pack(">Q", 1)
        m2 = sup.handle(build_msg1(anonce, replay))
        self.assertIsNotNone(m2)

        # AP independently derives the PTK from the SNonce the STA sent.
        snonce = bytes(m2[_OFF_NONCE : _OFF_NONCE + 32])
        kck, kek, tk = crypto.ptk_from_pmk(pmk, aa, spa, anonce, snonce)
        self.assertEqual(tk, sup.tk)

        # Verify the STA's msg2 MIC with the AP-side KCK.
        chk = bytearray(m2)
        recv_mic = bytes(chk[_OFF_MIC : _OFF_MIC + 16])
        chk[_OFF_MIC : _OFF_MIC + 16] = b"\x00" * 16
        self.assertEqual(crypto.eapol_mic(kck, bytes(chk)), recv_mic)

        # msg3 -> the STA installs keys and answers msg4.
        gtk = os.urandom(16)
        replay3 = struct.pack(">Q", 2)
        m4 = sup.handle(build_msg3(kck, kek, anonce, replay3, gtk, 1))
        self.assertIsNotNone(m4)
        self.assertTrue(sup.completed)
        self.assertEqual(sup.gtk, gtk)

        # msg4 MIC must verify on the AP side too.
        chk4 = bytearray(m4)
        rmic = bytes(chk4[_OFF_MIC : _OFF_MIC + 16])
        chk4[_OFF_MIC : _OFF_MIC + 16] = b"\x00" * 16
        self.assertEqual(crypto.eapol_mic(kck, bytes(chk4)), rmic)

    def test_wrong_passphrase_fails_mic(self):
        ssid = b"Lab-AP-1"
        pmk_ap = crypto.wpa_pmk("correcthorse1", ssid)
        pmk_sta = crypto.wpa_pmk("wrongphrase", ssid)
        aa = dot11.mac_bytes("02:11:22:33:44:00")
        spa = dot11.mac_bytes("02:aa:bb:cc:dd:ee")
        sup = supplicant.Supplicant(pmk_sta, spa, aa,
                                    dot11.rsn_ie_ccmp_psk())
        anonce = os.urandom(32)
        m2 = sup.handle(build_msg1(anonce, struct.pack(">Q", 1)))
        snonce = bytes(m2[_OFF_NONCE : _OFF_NONCE + 32])
        kck_ap, kek_ap, _ = crypto.ptk_from_pmk(pmk_ap, aa, spa, anonce,
                                                snonce)
        # AP builds msg3 with its (correct-passphrase) keys; the STA's KCK
        # differs, so msg3 MIC verification must fail.
        gtk = os.urandom(16)
        m3 = build_msg3(kck_ap, kek_ap, anonce, struct.pack(">Q", 2), gtk, 1)
        with self.assertRaises(supplicant.HandshakeError):
            sup.handle(m3)


class TestKeyWrapRoundtrip(unittest.TestCase):
    def test_wrap_unwrap(self):
        kek = os.urandom(16)
        for n in (16, 24, 32, 40):
            data = os.urandom(n)
            w = crypto.aes_key_wrap(kek, data)
            self.assertEqual(crypto.aes_key_unwrap(kek, w), data)


if __name__ == "__main__":
    unittest.main()
