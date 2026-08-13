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

    def test_appsequence_instance_id_is_per_boot(self):
        # A constant InstanceId lets Windows keep stale metadata cached
        # across restarts; it must reflect the boot (a time epoch here).
        _h, svc = self._svc()
        self.assertGreater(svc.dev.instance, 1)
        import re as _re
        h2, svc2 = self._svc()
        deliver_udp(h2, "10.1.2.5", 50000, wsd.WSD_PORT, PROBE,
                    src_mac="02:00:00:00:00:05")
        text = _last_udp_payload(h2.station).decode()
        m = _re.search(r'InstanceId="(\d+)"', text)
        self.assertIsNotNone(m)
        self.assertGreater(int(m.group(1)), 1)

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
        self.assertIn('<wsdp:Manufacturer xml:lang="en">HP</wsdp:Manufacturer>',
                      text)
        self.assertIn("HP-OfficeJet-Den", text)                   # friendly
        # Modelled on the real HP: the Relationship carries the print
        # service as the only member, with NO <wsdp:Host> (a Host marked
        # PrintDeviceType makes Windows look for print at the device
        # endpoint), and the hosted service is a wprt:PrinterServiceType.
        self.assertNotIn("<wsdp:Host>", text)
        self.assertIn(
            "<wsdp:Types>wprt:PrinterServiceType</wsdp:Types>", text)  # svc
        # PnP-X: Windows needs a HardwareId + CompatibleId on the hosted
        # print service (once a DeviceCategory is present) to build the
        # printer's device node — without them it re-fetches the metadata
        # in a loop and never shows the device. Modelled on a real HP.
        self.assertIn("<pnpx:HardwareId>", text)
        self.assertIn("VEN_", text)
        self.assertIn("<pnpx:CompatibleId>http://schemas.microsoft.com/"
                      "windows/2006/08/wdp/print/PrinterServiceType"
                      "</pnpx:CompatibleId>", text)
        # Device Foundation category alongside the PnP-X one, like a real
        # printer, and its namespace must be declared.
        self.assertIn("<df:DeviceCategory>", text)
        self.assertIn(
            'xmlns:df="http://schemas.microsoft.com/windows/2008/09/'
            'devicefoundation"', text)
        self.assertIn("urn:uuid:get-1", text)                     # RelatesTo
        # The metadata wrapper MUST be WS-MetadataExchange (mex:), not
        # devprof: Windows drops the device from the Network folder if the
        # Metadata / MetadataSection elements are mis-namespaced.
        self.assertIn('xmlns:mex="http://schemas.xmlsoap.org/ws/2004/09/mex"',
                      text)
        self.assertIn("<mex:Metadata>", text)
        self.assertIn("mex:MetadataSection", text)
        # PnP-X DeviceCategory: without it Windows parses the metadata but
        # never publishes the device into the Explorer "Network" folder.
        self.assertIn('xmlns:pnpx="http://schemas.microsoft.com/windows/'
                      'pnpx/2005/10"', text)
        self.assertIn("<pnpx:DeviceCategory>Printers</pnpx:DeviceCategory>",
                      text)

    def test_multifunction_advertises_scan_service(self):
        # A device with wsd_scan carries a second hosted service so Windows
        # also creates a scanner node.
        h = FakeHost("HP-OfficeJet-Den", dot11.mac_bytes("00:01:e6:aa:bb:cc"),
                     ip="10.1.2.100")
        h.wsd_scan = True
        h.wsd_model = "HP OfficeJet Pro 9015"
        srv = wsd.WSDHttpService()
        srv.bind(h)
        meta = srv._metadata("urn:uuid:x").decode()
        self.assertIn("<wsdp:Types>wscn:ScannerServiceType</wsdp:Types>", meta)
        self.assertIn("wdp/scan/ScannerServiceType", meta)   # scan compat id
        self.assertIn("<wsdp:Types>wprt:PrinterServiceType</wsdp:Types>", meta)
        # A print-only device has no scan service.
        h2 = FakeHost("Plain", dot11.mac_bytes("02:60:b0:aa:bb:cc"),
                      ip="10.1.2.101")
        srv2 = wsd.WSDHttpService()
        srv2.bind(h2)
        self.assertNotIn("ScannerServiceType", srv2._metadata("urn:uuid:y").decode())

    def test_get_scanner_elements_response(self):
        h = FakeHost("HP-OfficeJet-Den", dot11.mac_bytes("00:01:e6:aa:bb:cc"),
                     ip="10.1.2.100")
        h.wsd_scan = True
        srv = wsd.WSDHttpService()
        srv.bind(h)
        resp = srv._scanner_elements("urn:uuid:s-1").decode()
        self.assertIn("GetScannerElementsResponse", resp)
        self.assertIn("wscn:ScannerState>Idle", resp)
        self.assertIn("urn:uuid:s-1", resp)                  # RelatesTo


if __name__ == "__main__":
    unittest.main()
