#
# vwifi-pseudohost — frame / wire-format tests
#
# SPDX-License-Identifier: GPL-2.0-or-later
#
import os
import struct
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from vwifi_pseudohost import ieee80211 as dot11  # noqa: E402
from vwifi_pseudohost import medium               # noqa: E402


class TestMediumHeader(unittest.TestCase):
    def test_header_is_40_bytes_at_right_offsets(self):
        self.assertEqual(medium._HDR.size, 40)
        hdr = medium._HDR.pack(
            medium.VWIFI_MAGIC, 2, 100, b"\x01\x02\x03\x04\x05\x06",
            0x0B, -30, 0x11223344, 0x55667788, 0,
            2437, 0x0005, 0, 0, 0, b"\x00\x00")
        self.assertEqual(struct.unpack_from("<I", hdr, 0)[0], medium.VWIFI_MAGIC)
        self.assertEqual(struct.unpack_from("<H", hdr, 4)[0], 2)      # version
        self.assertEqual(struct.unpack_from("<H", hdr, 6)[0], 100)    # frame_len
        self.assertEqual(hdr[8:14], b"\x01\x02\x03\x04\x05\x06")      # tx_mac
        self.assertEqual(hdr[14], 0x0B)                               # rate
        self.assertEqual(struct.unpack_from("b", hdr, 15)[0], -30)    # rssi
        self.assertEqual(struct.unpack_from("<H", hdr, 28)[0], 2437)  # ch freq

    def test_parse_roundtrip(self):
        frame = dot11.build_auth(b"\x02\x00\x00\x00\x00\x01",
                                 b"\x02\x11\x22\x33\x44\x00", 0)
        hdr = medium._HDR.pack(
            medium.VWIFI_MAGIC, 2, len(frame), b"\x02\x11\x22\x33\x44\x00",
            0x0B, -42, 0, 0, 0, 2437, 0x0005, 0, 0, 0, b"\x00\x00")
        rx = medium.MediumClient._parse(hdr + frame)
        self.assertEqual(rx.tx_mac, b"\x02\x11\x22\x33\x44\x00")
        self.assertEqual(rx.rssi, -42)
        self.assertEqual(rx.channel_freq, 2437)
        self.assertEqual(rx.frame, frame)


class TestChannels(unittest.TestCase):
    def test_roundtrip(self):
        for ch in (1, 6, 11, 13, 14, 36, 149):
            self.assertEqual(dot11.freq_to_chan(dot11.chan_to_freq(ch)), ch)


class TestIEs(unittest.TestCase):
    def test_parse_beacon_ssid_channel_rsn(self):
        # Synthesize a beacon the way hostapd would frame one.
        bssid = b"\x02\x11\x22\x33\x44\x00"
        b = dot11.mgmt_header(dot11.STYPE_BEACON, dot11.BROADCAST, bssid,
                              bssid, 0)
        b += struct.pack("<QHH", 0, 100, 0x0431)     # tsf, beacon int, cap
        b += dot11.ie(dot11.EID_SSID, b"Lab-AP-1")
        b += dot11.supp_rate_ies()
        b += dot11.ie(dot11.EID_DS_PARAMS, bytes([6]))
        b += dot11.rsn_ie_ccmp_psk()
        parsed = dot11.Beacon(bytes(b), medium_freq=2437)
        self.assertEqual(parsed.ssid, b"Lab-AP-1")
        self.assertEqual(parsed.channel_freq, 2437)
        self.assertTrue(parsed.privacy)
        self.assertIsNotNone(parsed.rsn)
        self.assertIn(dot11.AKM_PSK, parsed.rsn["akms"])
        self.assertIn(dot11.CIPHER_CCMP, parsed.rsn["pairwise"])

    def test_data_frame_eth_roundtrip(self):
        bssid = b"\x02\x11\x22\x33\x44\x00"
        us = b"\x02\xaa\xbb\xcc\xdd\xee"
        peer = b"\x02\x00\x00\x00\x00\x99"
        payload = b"hello-payload"
        # STA -> AP (ToDS)
        f = dot11.build_data_frame(bssid, us, peer, (dot11.ETH_P_IP, payload))
        # Reflect it back as AP -> STA (FromDS) to exercise the other path.
        b = bytearray(f)
        b[1] = dot11.FC1_FROMDS
        b[4:10] = us            # addr1 = DA (us)
        b[10:16] = bssid        # addr2 = TA (BSSID)
        b[16:22] = peer         # addr3 = SA
        out = dot11.parse_data_frame(bytes(b))
        self.assertIsNotNone(out)
        sa, da, ethertype, sdu = out
        self.assertEqual(sa, peer)
        self.assertEqual(da, us)
        self.assertEqual(ethertype, dot11.ETH_P_IP)
        self.assertEqual(sdu, payload)


if __name__ == "__main__":
    unittest.main()
