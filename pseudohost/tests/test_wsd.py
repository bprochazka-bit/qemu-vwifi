#
# vwifi-pseudohost — WSD discovery tests
#
# SPDX-License-Identifier: GPL-2.0-or-later
#
import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
sys.path.insert(0, os.path.dirname(__file__))

from vwifi_pseudohost import netstack, wsd              # noqa: E402
from vwifi_pseudohost import ieee80211 as dot11          # noqa: E402
from tcp_client import ServerHost, ClientSim             # noqa: E402
from test_profiles import FakeHost, deliver_udp, _last_udp_payload  # noqa: E402

PROBE = (
    '<?xml version="1.0"?><soap:Envelope '
    'xmlns:soap="http://www.w3.org/2003/05/soap-envelope" '
    'xmlns:wsa="http://schemas.xmlsoap.org/ws/2004/08/addressing" '
    'xmlns:wsd="http://schemas.xmlsoap.org/ws/2005/04/discovery" '
    'xmlns:wprt="http://schemas.microsoft.com/windows/2006/08/wdp/print">'
    '<soap:Header>'
    '<wsa:Action>http://schemas.xmlsoap.org/ws/2005/04/discovery/Probe'
    '</wsa:Action>'
    '<wsa:MessageID>urn:uuid:probe-0001</wsa:MessageID></soap:Header>'
    '<soap:Body><wsd:Probe><wsd:Types>wprt:PrintDeviceType</wsd:Types>'
    '</wsd:Probe></soap:Body></soap:Envelope>').encode()


class TestWSDiscovery(unittest.TestCase):
    def _svc(self):
        h = FakeHost("HP-OfficeJet-Den", dot11.mac_bytes("02:60:b0:aa:bb:cc"),
                     ip="10.1.2.100")
        svc = wsd.WSDiscoveryService()
        svc.bind(h)
        return h, svc

    def test_probe_match(self):
        h, svc = self._svc()
        deliver_udp(h, "10.1.2.5", 50000, wsd.WSD_PORT, PROBE,
                    src_mac="02:00:00:00:00:05")
        resp = _last_udp_payload(h.station)
        self.assertIsNotNone(resp, "no ProbeMatch emitted")
        text = resp.decode()
        self.assertIn("ProbeMatches", text)
        self.assertIn(wsd.uuid_from_mac(h.mac), text)             # our EPR
        self.assertIn("10.1.2.100:5357", text)                    # XAddrs
        self.assertIn("urn:uuid:probe-0001", text)                # RelatesTo

    def test_probe_wrong_type_ignored(self):
        h, svc = self._svc()
        probe = PROBE.replace(b"wprt:PrintDeviceType", b"foo:SomethingElse")
        deliver_udp(h, "10.1.2.5", 50000, wsd.WSD_PORT, probe,
                    src_mac="02:00:00:00:00:05")
        self.assertIsNone(_last_udp_payload(h.station))

    def test_hello_on_tick(self):
        h, svc = self._svc()
        svc.tick()
        resp = _last_udp_payload(h.station)
        self.assertIsNotNone(resp)
        self.assertIn("Hello", resp.decode())
        # only once
        h.station.sent.clear()
        svc.tick()
        self.assertIsNone(_last_udp_payload(h.station))


class TestWSDMetadata(unittest.TestCase):
    def test_get_returns_device_metadata(self):
        srv = ServerHost(ip="10.1.2.100")
        srv.hostname = "HP-OfficeJet-Den"
        srv.wsd_manufacturer = "HP"
        srv.wsd_model = "HP OfficeJet Pro 9015"
        srv.wsd_model_number = "9015"
        wsd.WSDMetadataService().bind(srv)

        c = ClientSim(srv, dport=wsd.WSD_HTTP_PORT)
        self.assertTrue(c.connect())
        body = ('<soap:Envelope '
                'xmlns:soap="http://www.w3.org/2003/05/soap-envelope" '
                'xmlns:wsa="http://schemas.xmlsoap.org/ws/2004/08/addressing">'
                '<soap:Header><wsa:MessageID>urn:uuid:get-1</wsa:MessageID>'
                '<wsa:Action>http://schemas.xmlsoap.org/ws/2004/09/transfer/Get'
                '</wsa:Action></soap:Header><soap:Body/></soap:Envelope>')
        req = ("POST /dev HTTP/1.1\r\nHost: 10.1.2.100:5357\r\n"
               "Content-Type: application/soap+xml\r\n"
               "Content-Length: %d\r\n\r\n%s" % (len(body), body)).encode()
        c.send(req)
        resp = c.recv()
        self.assertTrue(resp.startswith(b"HTTP/1.1 200"))
        text = resp.decode()
        self.assertIn("HP OfficeJet Pro 9015", text)              # model
        self.assertIn("<wsdp:Manufacturer>HP</wsdp:Manufacturer>", text)
        self.assertIn("HP-OfficeJet-Den", text)                   # friendly
        self.assertIn("PrintDeviceType", text)                   # hosted svc
        self.assertIn("urn:uuid:get-1", text)                     # RelatesTo
        # The metadata wrapper MUST be WS-MetadataExchange (mex:), not
        # devprof: Windows drops the device from the Network folder if the
        # Metadata / MetadataSection elements are mis-namespaced.
        self.assertIn('xmlns:mex="http://schemas.xmlsoap.org/ws/2004/09/mex"',
                      text)
        self.assertIn("<mex:Metadata>", text)
        self.assertIn("mex:MetadataSection", text)
        self.assertNotIn("<wsdp:Metadata>", text)
        # PnP-X DeviceCategory: without it Windows parses the metadata but
        # never publishes the device into the Explorer "Network" folder.
        self.assertIn('xmlns:pnpx="http://schemas.microsoft.com/windows/'
                      'pnpx/2005/10"', text)
        self.assertIn("<pnpx:DeviceCategory>Printers</pnpx:DeviceCategory>",
                      text)


if __name__ == "__main__":
    unittest.main()
