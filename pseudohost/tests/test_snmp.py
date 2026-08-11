#
# vwifi-pseudohost — SNMP agent tests
#
# SPDX-License-Identifier: GPL-2.0-or-later
#
# Exercises the agent the way the Windows TCP/IP port monitor does: a
# GetRequest for sysDescr and a GetNext walk from the printer group.
#
import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
sys.path.insert(0, os.path.dirname(__file__))

from vwifi_pseudohost import snmp                       # noqa: E402
from vwifi_pseudohost import ieee80211 as dot11          # noqa: E402
from test_profiles import FakeHost, deliver_udp, _last_udp_payload  # noqa: E402


def build_request(pdu_tag, oids, community=b"public", reqid=1, version=1):
    vbs = b""
    for oid in oids:
        vbs += snmp._tlv(snmp.T_SEQ,
                         snmp._enc_oid(oid) + snmp._tlv(snmp.T_NULL, b""))
    pdu = (snmp._tlv(snmp.T_INT, bytes([reqid]))
           + snmp._enc_int(0) + snmp._enc_int(0)
           + snmp._tlv(snmp.T_SEQ, vbs))
    return snmp._tlv(snmp.T_SEQ,
                     snmp._enc_int(version) + snmp._tlv(snmp.T_OCTSTR, community)
                     + snmp._tlv(pdu_tag, pdu))


def parse_first_varbind(resp):
    tag, body, _ = snmp._rd_tlv(resp, 0)
    i = 0
    _t, _ver, i = snmp._rd_tlv(body, i)
    _t, _comm, i = snmp._rd_tlv(body, i)
    _pt, pdu, i = snmp._rd_tlv(body, i)
    j = 0
    _t, _rid, j = snmp._rd_tlv(pdu, j)
    _t, _es, j = snmp._rd_tlv(pdu, j)
    _t, _ei, j = snmp._rd_tlv(pdu, j)
    _t, vbl, j = snmp._rd_tlv(pdu, j)
    _t, vb, _ = snmp._rd_tlv(vbl, 0)
    ot, oidb, k = snmp._rd_tlv(vb, 0)
    vtag, vval, _ = snmp._rd_tlv(vb, k)
    return snmp._dec_oid(oidb), vtag, vval


class TestSNMP(unittest.TestCase):
    def _host(self):
        h = FakeHost("HP-OfficeJet-Den", dot11.mac_bytes("02:60:b0:aa:bb:cc"),
                     ip="10.1.2.107")
        h.wsd_model = "HP OfficeJet Pro 9015"
        snmp.SNMPAgent().bind(h)
        return h

    def test_get_sysdescr(self):
        h = self._host()
        req = build_request(snmp.PDU_GET, [(1, 3, 6, 1, 2, 1, 1, 1, 0)])
        deliver_udp(h, "10.1.2.5", 47000, 161, req, src_mac="02:0:0:0:0:5")
        resp = _last_udp_payload(h.station)
        self.assertIsNotNone(resp)
        oid, vtag, vval = parse_first_varbind(resp)
        self.assertEqual(oid, (1, 3, 6, 1, 2, 1, 1, 1, 0))
        self.assertEqual(vval, b"HP OfficeJet Pro 9015")

    def test_hrdevice_type_is_printer(self):
        h = self._host()
        req = build_request(snmp.PDU_GET,
                            [(1, 3, 6, 1, 2, 1, 25, 3, 2, 1, 2, 1)])
        deliver_udp(h, "10.1.2.5", 47001, 161, req, src_mac="02:0:0:0:0:5")
        resp = _last_udp_payload(h.station)
        oid, vtag, vval = parse_first_varbind(resp)
        self.assertEqual(vtag, snmp.T_OID)
        # hrDeviceType == the "printer" device-type OID
        self.assertEqual(snmp._dec_oid(vval),
                         (1, 3, 6, 1, 2, 1, 25, 3, 1, 5))

    def test_getnext_walks(self):
        h = self._host()
        # GetNext(sysDescr) -> the next OID in the table (sysObjectID).
        req = build_request(snmp.PDU_GETNEXT, [(1, 3, 6, 1, 2, 1, 1, 1, 0)])
        deliver_udp(h, "10.1.2.5", 47002, 161, req, src_mac="02:0:0:0:0:5")
        resp = _last_udp_payload(h.station)
        oid, vtag, vval = parse_first_varbind(resp)
        self.assertEqual(oid, (1, 3, 6, 1, 2, 1, 1, 2, 0))       # sysObjectID
        self.assertEqual(vtag, snmp.T_OID)

    def test_printer_name(self):
        h = self._host()
        req = build_request(snmp.PDU_GET,
                            [(1, 3, 6, 1, 2, 1, 43, 5, 1, 1, 16, 1)])
        deliver_udp(h, "10.1.2.5", 47003, 161, req, src_mac="02:0:0:0:0:5")
        oid, vtag, vval = parse_first_varbind(_last_udp_payload(h.station))
        self.assertEqual(vval, b"HP-OfficeJet-Den")


if __name__ == "__main__":
    unittest.main()
