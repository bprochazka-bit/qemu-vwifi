#
# vwifi-pseudohost — crypto known-answer tests
#
# SPDX-License-Identifier: GPL-2.0-or-later
#
# These anchor the primitives the station's security depends on.  The
# AES and PBKDF2/PTK vectors are the public standards ones; the CCMP
# round-trip mirrors devices/vwifi/tests/crypto.c so a frame this tool
# encrypts is one that codebase's C decrypts and vice versa.
#
import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from vwifi_pseudohost import crypto  # noqa: E402


class TestAES(unittest.TestCase):
    def test_fips197_c1(self):
        key = bytes.fromhex("000102030405060708090a0b0c0d0e0f")
        pt = bytes.fromhex("00112233445566778899aabbccddeeff")
        ct = bytes.fromhex("69c4e0d86a7b0430d8cdb78070b4c55a")
        aes = crypto.AES128(key)
        self.assertEqual(aes.encrypt(pt), ct)
        self.assertEqual(aes.decrypt(ct), pt)


class TestCCMP(unittest.TestCase):
    def _data_frame(self, payload_len=40):
        bssid = bytes([0xAA, 0xBB, 0xCC, 0x00, 0x00, 0x01])
        sta = bytes([0x00, 0x03, 0x7F, 0xCC, 0xDD, 0x10])
        peer = bytes([0x11, 0x22, 0x33, 0x44, 0x55, 0x66])
        f = bytearray(24)
        f[0] = 0x08          # data, subtype 0
        f[1] = 0x01          # ToDS
        f[4:10] = bssid
        f[10:16] = sta
        f[16:22] = peer
        f[22] = 0x10
        f += bytes((0x40 + (i & 0x3F)) for i in range(payload_len))
        return bytes(f)

    def test_roundtrip(self):
        key = bytes.fromhex("0f0e0d0c0b0a09080706050403020100")
        pn = bytes([0, 0, 0, 0, 0, 1])
        aes = crypto.AES128(key)
        frame = self._data_frame(40)
        enc = crypto.ccmp_encrypt(aes, frame, pn, 0)
        self.assertEqual(len(enc), len(frame) + 8 + 8)
        self.assertTrue(enc[1] & 0x40)                       # Protected
        self.assertTrue(enc[24 + 3] & 0x20)                  # ExtIV
        self.assertNotEqual(enc[24 + 8 : 24 + 8 + 40], frame[24:])
        out = crypto.ccmp_decrypt(aes, enc)
        self.assertIsNotNone(out)
        pt, got_pn = out
        self.assertEqual(pt, frame)
        self.assertEqual(got_pn, pn)

    def test_tamper_ciphertext_rejected(self):
        aes = crypto.AES128(bytes(16))
        enc = bytearray(crypto.ccmp_encrypt(aes, self._data_frame(16),
                                            bytes([0, 0, 0, 0, 0, 5])))
        enc[24 + 8] ^= 0x01                                  # flip a ct byte
        self.assertIsNone(crypto.ccmp_decrypt(aes, bytes(enc)))

    def test_tamper_header_rejected(self):
        aes = crypto.AES128(bytes(16))
        enc = bytearray(crypto.ccmp_encrypt(aes, self._data_frame(16),
                                            bytes([0, 0, 0, 0, 0, 6])))
        enc[16] ^= 0x01                                      # rewrite addr3
        self.assertIsNone(crypto.ccmp_decrypt(aes, bytes(enc)))

    def test_wrong_key_rejected(self):
        enc = crypto.ccmp_encrypt(crypto.AES128(bytes(16)),
                                  self._data_frame(16),
                                  bytes([0, 0, 0, 0, 0, 7]))
        self.assertIsNone(crypto.ccmp_decrypt(crypto.AES128(bytes([1]) * 16),
                                              enc))

    def test_ccmp_header_pn_is_low_byte_first(self):
        # The CCMP header stores PN0 (least significant) at offset 0.
        # This is the byte order real 802.11 devices use; getting it
        # backwards is self-consistent but fails interop (it did).
        hdr = crypto.ccmp_build_header(bytes([0, 0, 0, 0, 0, 7]), 0)
        self.assertEqual(hdr.hex(), "0700002000000000")
        pn, kid = crypto.ccmp_parse_header(hdr)
        self.assertEqual(pn, bytes([0, 0, 0, 0, 0, 7]))
        self.assertEqual(kid, 0)

    def test_interop_kat_matches_reference_c(self):
        # Exact bytes produced by devices/vwifi/src/vwifi_crypto.c
        # (vwifi_ccmp_encrypt) for this key/PN/frame.  If our output
        # diverges, a real vwifi device will drop our CCMP frames.
        key = bytes.fromhex("0f0e0d0c0b0a09080706050403020100")
        plain = bytes.fromhex(
            "08010000021122334400" "5254008bc782" "ffffffffffff" "1000"
            "aaaa030000000800" "404142434445464748494a4b4c4d4e4f50515253")
        expect = bytes.fromhex(
            "084100000211223344005254008bc782ffffffffffff1000"
            "0700002000000000539cfdfa5dfb85d0616a79b33f58fb9e"
            "2f2dd311fa21af55e5896e92fbf39945760232b3")
        enc = crypto.ccmp_encrypt(crypto.AES128(key), plain,
                                  bytes([0, 0, 0, 0, 0, 7]), 0)
        self.assertEqual(enc, expect)
        # and the reference ciphertext decrypts back to our plaintext
        out = crypto.ccmp_decrypt(crypto.AES128(key), expect)
        self.assertEqual(out[0], plain)

    def test_interop_kat_qos_matches_reference_c(self):
        # A QoS data frame (subtype 8) encrypted by vwifi_crypto.c.  The
        # CCMP AAD keeps subtype bit b7, which marks the QoS frame; a mask
        # that clears it decrypts non-QoS fine but fails every QoS frame,
        # which is exactly how a real (QoS-using) Windows client broke.
        key = bytes.fromhex("0f0e0d0c0b0a09080706050403020100")
        plain = bytes.fromhex(
            "88010000021122334400" "5254008bc782" "ffffffffffff" "1000"
            "0000"                                  # QoS control (TID 0)
            "aaaa030000000800" "606162636465666768696a6b")
        expect = bytes.fromhex(
            "884100000211223344005254008bc782ffffffffffff1000"
            "00000700002000000000539cfdfa5dfb85d0414a59931f78"
            "dbbe0f0df331b6ab26dd950dbf23")
        enc = crypto.ccmp_encrypt(crypto.AES128(key), plain,
                                  bytes([0, 0, 0, 0, 0, 7]), 0)
        self.assertEqual(enc, expect)
        out = crypto.ccmp_decrypt(crypto.AES128(key), expect)
        self.assertEqual(out[0], plain)

    def test_pn_carry(self):
        self.assertEqual(crypto.pn_increment(bytes([0, 0, 0, 0, 0, 0xFF])),
                         bytes([0, 0, 0, 0, 1, 0]))
        self.assertEqual(
            crypto.pn_increment(bytes([0, 0, 0, 0xFF, 0xFF, 0xFF])),
            bytes([0, 0, 1, 0, 0, 0]))


class TestWPA(unittest.TestCase):
    def test_pmk_vector(self):
        # IEEE 802.11i test vector 2 (RFC-style): passphrase "ThisIsAPassword",
        # SSID "ThisIsASSID" -> known PMK.
        pmk = crypto.wpa_pmk("ThisIsAPassword", "ThisIsASSID")
        self.assertEqual(
            pmk.hex(),
            "0dc0d6eb90555ed6419756b9a15ec3e3209b63df707dd508d14581f8982721af")

    def test_ptk_derivation_shape(self):
        pmk = crypto.wpa_pmk("correcthorse1", "Lab-AP-1")
        aa = bytes.fromhex("021122334400")
        spa = bytes.fromhex("0203040506ff")
        anonce = bytes(range(32))
        snonce = bytes(range(32, 64))
        kck, kek, tk = crypto.ptk_from_pmk(pmk, aa, spa, anonce, snonce)
        self.assertEqual((len(kck), len(kek), len(tk)), (16, 16, 16))
        # Deterministic: swapping the two MACs/nonces must not change the
        # result (the min/max canonicalisation is the point).
        kck2, kek2, tk2 = crypto.ptk_from_pmk(pmk, spa, aa, snonce, anonce)
        self.assertEqual((kck, kek, tk), (kck2, kek2, tk2))


class TestKeyWrap(unittest.TestCase):
    def test_rfc3394_128(self):
        # RFC 3394 §4.1: wrap 128 bits of key data with a 128-bit KEK.
        kek = bytes.fromhex("000102030405060708090a0b0c0d0e0f")
        wrapped = bytes.fromhex(
            "1fa68b0a8112b447aef34bd8fb5a7b829d3e862371d2cfe5")
        expect = bytes.fromhex("00112233445566778899aabbccddeeff")
        self.assertEqual(crypto.aes_key_unwrap(kek, wrapped), expect)


if __name__ == "__main__":
    unittest.main()
