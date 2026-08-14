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
        # The wscn prefix MUST bind to the 2006/08 namespace: the
        # ScannerServiceType QName is resolved through it, and Windows only
        # treats the hosted service as a scanner when it is
        # {2006/08}ScannerServiceType. Binding wscn to 2006/01 (an earlier
        # regression) left the device showing only as a printer.
        self.assertIn(
            'xmlns:wscn="http://schemas.microsoft.com/windows/2006/08/'
            'wdp/scan"', meta)
        self.assertIn("<wsdp:Types>wprt:PrinterServiceType</wsdp:Types>", meta)
        # A print-only device has no scan service.
        h2 = FakeHost("Plain", dot11.mac_bytes("02:60:b0:aa:bb:cc"),
                      ip="10.1.2.101")
        srv2 = wsd.WSDHttpService()
        srv2.bind(h2)
        self.assertNotIn("ScannerServiceType", srv2._metadata("urn:uuid:y").decode())

    def _dispatch_action(self, action, extra_body=""):
        h = FakeHost("HP-OfficeJet-Den", dot11.mac_bytes("00:01:e6:aa:bb:cc"),
                     ip="10.1.2.100")
        srv = wsd.WSDHttpService()
        srv.bind(h)
        req = ('<soap:Envelope '
               'xmlns:soap="http://www.w3.org/2003/05/soap-envelope" '
               'xmlns:wsa="http://schemas.xmlsoap.org/ws/2004/08/addressing" '
               'xmlns:wse="http://schemas.xmlsoap.org/ws/2004/08/eventing">'
               '<soap:Header><wsa:MessageID>urn:uuid:req-1</wsa:MessageID>'
               '<wsa:Action>%s</wsa:Action></soap:Header>'
               '<soap:Body>%s</soap:Body></soap:Envelope>'
               % (action, extra_body)).encode()
        raw = (b"POST /x HTTP/1.1\r\nContent-Type: application/soap+xml\r\n"
               b"Content-Length: " + str(len(req)).encode() + b"\r\n\r\n" + req)
        return srv._dispatch(raw, raw.find(b"\r\n\r\n")).decode()

    def test_eventing_subscribe_gets_subscriberesponse(self):
        # The bug that crashed the spooler: a Subscribe used to fall through
        # to device metadata. It must return a WS-Eventing SubscribeResponse.
        resp = self._dispatch_action(
            "http://schemas.xmlsoap.org/ws/2004/08/eventing/Subscribe",
            "<wse:Subscribe><wse:Expires>PT1H</wse:Expires></wse:Subscribe>")
        self.assertIn("<wse:SubscribeResponse>", resp)
        self.assertIn("<wse:SubscriptionManager>", resp)
        self.assertIn("<wse:Identifier>", resp)
        self.assertIn("<wse:Expires>PT1H</wse:Expires>", resp)
        self.assertIn("urn:uuid:req-1", resp)                 # RelatesTo
        self.assertNotIn("ThisModel", resp)                   # not metadata!

    def test_unknown_action_returns_fault_not_metadata(self):
        resp = self._dispatch_action(
            "http://example.com/some/UnknownOperation")
        self.assertIn("<soap:Fault>", resp)
        self.assertIn("ActionNotSupported", resp)
        self.assertNotIn("<wsdp:ThisModel>", resp)

    def test_notify_dest_parsing(self):
        sub = ('<wse:Subscribe><wse:Delivery><wse:NotifyTo><wsa:Address>'
               'http://192.168.1.99:5357/sink-uuid</wsa:Address>'
               '</wse:NotifyTo></wse:Delivery></wse:Subscribe>')
        d = wsd._notify_dest(sub)
        self.assertEqual(d["ip"], "192.168.1.99")
        self.assertEqual(d["port"], 5357)
        self.assertEqual(d["path"], "/sink-uuid")
        self.assertIsNone(wsd._notify_dest("<no notifyto/>"))

    def _host_with_tcp(self):
        import time
        from vwifi_pseudohost import tcp
        h = FakeHost("HP-OfficeJet-PH01", dot11.mac_bytes("00:01:e6:c1:23:45"),
                     ip="192.168.1.127")
        h.tcp = tcp.TCPStack(h.stack)
        # Peer MAC already learned (as it is from the incoming Subscribe).
        h.stack.arp_cache[netstack.ip_bytes("192.168.1.99")] = (
            dot11.mac_bytes("52:54:00:8b:c7:82"), time.time() + 120)
        return h

    def test_subscribe_records_sub_and_queues_event(self):
        h = self._host_with_tcp()
        svc = wsd.WSDHttpService()
        svc.bind(h)
        req = ('<soap:Envelope xmlns:wse="http://schemas.xmlsoap.org/ws/'
               '2004/08/eventing" xmlns:wsa="http://schemas.xmlsoap.org/ws/'
               '2004/08/addressing"><soap:Body><wse:Subscribe><wse:Delivery>'
               '<wse:NotifyTo><wsa:Address>http://192.168.1.99:5357/sink'
               '</wsa:Address></wse:NotifyTo></wse:Delivery>'
               '<wse:Expires>PT1H</wse:Expires></wse:Subscribe></soap:Body>'
               '</soap:Envelope>')
        svc._subscribe_resp("urn:uuid:s", req)
        self.assertEqual(len(svc._subs), 1)
        self.assertEqual(len(svc._event_q), 1)     # initial status primed

    def test_tick_opens_outbound_connection_to_sink(self):
        import struct
        h = self._host_with_tcp()
        svc = wsd.WSDHttpService()
        svc.bind(h)
        req = ('<wse:Subscribe xmlns:wse="x" xmlns:wsa="y"><wse:NotifyTo>'
               '<wsa:Address>http://192.168.1.99:5357/sink</wsa:Address>'
               '</wse:NotifyTo></wse:Subscribe>')
        # Minimal record; _notify_dest tolerates the namespaces above.
        dest = wsd._notify_dest(req)
        svc._subs.append({"dest": dest, "filter": ""})
        svc._queue_event(dest, svc._event(
            dest, wsd.A_ELEMENTS_CHANGE_EVENT, svc._elements_change_body()))
        h.station.sent.clear()
        svc.tick()
        syn = None
        for _dst, et, sdu in h.station.sent:
            if et != dot11.ETH_P_IP or sdu[9] != netstack.IPPROTO_TCP:
                continue
            ihl = (sdu[0] & 0xF) * 4
            dip = ".".join(str(x) for x in sdu[16:20])
            flags = struct.unpack(">H", sdu[ihl + 12:ihl + 14])[0] & 0x3F
            if flags & 0x02:                       # SYN
                syn = dip
        self.assertEqual(syn, "192.168.1.99")      # active open to the sink

    def test_job_end_event_routed_only_to_jobend_subscriber(self):
        # Windows subscribes once per event type. A JobEndStateEvent must be
        # delivered to the subscription that filtered for it (and echo that
        # subscription's NotifyTo Identifier), not to a status subscriber.
        h = self._host_with_tcp()
        svc = wsd.WSDHttpService()
        svc.bind(h)
        jobend = {"dest": {"url": "http://192.168.1.99:5357/je",
                           "ip": "192.168.1.99", "port": 5357, "path": "/je",
                           "identifier": "urn:uuid:je-id"},
                  "filter": wsd.A_JOBEND_EVENT}
        status = {"dest": {"url": "http://192.168.1.99:5357/ps",
                           "ip": "192.168.1.99", "port": 5357, "path": "/ps",
                           "identifier": "urn:uuid:ps-id"},
                  "filter": wsd.A_STATUS_SUMMARY_EVENT}
        svc._subs = [jobend, status]
        svc._cur_jobname = "Test Page"
        svc._cur_jobuser = "user"
        svc._queue_job_completion(job_id=2, nbytes=318 * 1024)

        posts = [req.decode("latin1") for _dest, req in svc._event_q]
        jobend_posts = [p for p in posts if "JobEndStateEvent" in p]
        self.assertEqual(len(jobend_posts), 1)
        je = jobend_posts[0]
        self.assertIn("POST /je ", je)                  # to the jobend sink
        self.assertIn("<wprt:JobId>2</wprt:JobId>", je)
        self.assertIn("Completed", je)
        self.assertIn("JobCompletedSuccessfully", je)
        self.assertIn("<wprt:JobName>Test Page</wprt:JobName>", je)
        self.assertIn("urn:uuid:je-id", je)             # echoes its Identifier
        # the status sink got the Processing/Idle summaries, not the job end
        status_posts = [p for p in posts if "POST /ps " in p]
        self.assertTrue(status_posts)
        self.assertTrue(all("JobEndStateEvent" not in p for p in status_posts))
        self.assertTrue(any("PrinterStatusSummaryEvent" in p
                            for p in status_posts))

    def test_send_document_queues_job_end_event(self):
        # End to end through _dispatch: CreatePrintJob then an MTOM
        # SendDocument, with a JobEndStateEvent subscription active, must
        # queue a terminal job-end notification so Windows clears the queue.
        from vwifi_pseudohost.printing import PrintSink
        h = FakeHost("HP-OfficeJet-Den", dot11.mac_bytes("00:01:e6:aa:bb:cc"),
                     ip="10.1.2.100")
        h.print_sink = PrintSink(None)
        svc = wsd.WSDHttpService()
        svc.bind(h)
        svc._subs = [{"dest": {"url": "http://192.168.1.99:5357/je",
                               "ip": "192.168.1.99", "port": 5357,
                               "path": "/je", "identifier": "urn:uuid:je"},
                      "filter": wsd.A_JOBEND_EVENT}]

        def post(raw):
            return svc._dispatch(raw, raw.find(b"\r\n\r\n"))

        cj = ('<soap:Envelope '
              'xmlns:soap="http://www.w3.org/2003/05/soap-envelope" '
              'xmlns:wsa="http://schemas.xmlsoap.org/ws/2004/08/addressing" '
              'xmlns:wprt="%s"><soap:Header>'
              '<wsa:MessageID>urn:uuid:c</wsa:MessageID>'
              '<wsa:Action>%s/CreatePrintJob</wsa:Action></soap:Header>'
              '<soap:Body><wprt:CreatePrintJobRequest><wprt:JobDescription>'
              '<wprt:JobName>Report</wprt:JobName>'
              '<wprt:JobOriginatingUserName>alice'
              '</wprt:JobOriginatingUserName></wprt:JobDescription>'
              '</wprt:CreatePrintJobRequest></soap:Body></soap:Envelope>'
              % (wsd.NS_WPRT, wsd.NS_WPRT)).encode()
        post(b"POST /x HTTP/1.1\r\nContent-Type: application/soap+xml\r\n"
             b"Content-Length: " + str(len(cj)).encode() + b"\r\n\r\n" + cj)

        doc = b"%PDF-1.7\nhello\n%%EOF"
        soap = ('<soap:Envelope '
                'xmlns:soap="http://www.w3.org/2003/05/soap-envelope" '
                'xmlns:wsa="http://schemas.xmlsoap.org/ws/2004/08/addressing" '
                'xmlns:wprt="%s"><soap:Header>'
                '<wsa:MessageID>urn:uuid:d</wsa:MessageID>'
                '<wsa:Action>%s/SendDocument</wsa:Action></soap:Header>'
                '<soap:Body/></soap:Envelope>' % (wsd.NS_WPRT, wsd.NS_WPRT))
        boundary = "B"
        body = (("--%s\r\n" % boundary).encode() +
                b'Content-Type: application/xop+xml\r\nContent-ID: <s>\r\n\r\n'
                + soap.encode() + ("\r\n--%s\r\n" % boundary).encode() +
                b"Content-Type: application/octet-stream\r\n"
                b"Content-ID: <d>\r\n\r\n" + doc +
                ("\r\n--%s--\r\n" % boundary).encode())
        raw = (b"POST /x HTTP/1.1\r\nContent-Type: multipart/related; "
               b'boundary="' + boundary.encode() + b'"; '
               b'type="application/xop+xml"\r\nContent-Length: ' +
               str(len(body)).encode() + b"\r\n\r\n" + body)
        resp = post(raw)
        self.assertIn(b"SendDocumentResponse", resp)

        posts = [req.decode("latin1") for _d, req in svc._event_q]
        je = [p for p in posts if "JobEndStateEvent" in p]
        self.assertEqual(len(je), 1)
        self.assertIn("<wprt:JobName>Report</wprt:JobName>", je[0])
        self.assertIn("<wprt:JobOriginatingUserName>alice", je[0])
        self.assertIn("<wprt:JobId>1</wprt:JobId>", je[0])

    def test_set_event_rate_acknowledged(self):
        # Windows sends SetEventRate while a print queue is open; a Fault
        # (the old behaviour) crashes the spooler. It must be acked.
        resp = self._dispatch_action(
            "http://schemas.microsoft.com/windows/2006/08/wdp/print/"
            "SetEventRate",
            "<wprt:SetEventRateRequest><wprt:EventRate>2</wprt:EventRate>"
            "</wprt:SetEventRateRequest>")
        self.assertIn("SetEventRateResponse", resp)
        self.assertIn("<wprt:EventRate>2</wprt:EventRate>", resp)
        self.assertNotIn("<soap:Fault>", resp)

    def test_http_keep_alive_two_requests_one_connection(self):
        # Two WSD requests down one connection both get answered and the
        # connection is not closed (no Connection: close, no RST storm).
        srv = ServerHost(ip="10.1.2.100")
        srv.hostname = "HP-OfficeJet-Den"
        wsd.WSDHttpService().bind(srv)
        c = ClientSim(srv, dport=wsd.WSD_HTTP_PORT)
        self.assertTrue(c.connect())

        def wsd_get():
            body = ('<soap:Envelope xmlns:soap="http://www.w3.org/2003/05/'
                    'soap-envelope" xmlns:wsa="http://schemas.xmlsoap.org/ws/'
                    '2004/08/addressing"><soap:Header>'
                    '<wsa:MessageID>urn:uuid:g</wsa:MessageID>'
                    '<wsa:Action>http://schemas.xmlsoap.org/ws/2004/09/'
                    'transfer/Get</wsa:Action></soap:Header>'
                    '<soap:Body/></soap:Envelope>')
            return ("POST /dev HTTP/1.1\r\nContent-Type: application/soap+xml"
                    "\r\nContent-Length: %d\r\n\r\n%s"
                    % (len(body), body)).encode()

        c.send(wsd_get())
        r1 = c.recv()
        self.assertIn(b"HTTP/1.1 200", r1)
        self.assertIn(b"Keep-Alive", r1)
        self.assertNotIn(b"Connection: close", r1)
        # Second request on the SAME connection must also be answered.
        c.send(wsd_get())
        r2 = c.recv()
        self.assertIn(b"GetResponse", r2)

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

    def test_make_png_is_valid(self):
        import struct
        png = wsd.make_png(80, 60)
        self.assertEqual(png[:8], b"\x89PNG\r\n\x1a\n")
        self.assertEqual(struct.unpack(">II", png[16:24]), (80, 60))
        self.assertEqual(png[-8:], b"IEND\xae\x42\x60\x82")  # IEND + its CRC

    def test_create_scan_job_response(self):
        h = FakeHost("HP-OfficeJet-Den", dot11.mac_bytes("00:01:e6:aa:bb:cc"),
                     ip="10.1.2.100")
        srv = wsd.WSDHttpService()
        srv.bind(h)
        srv._scan_job = 1
        resp = srv._create_scan_job("urn:uuid:c", "<x/>").decode()
        self.assertIn("<wscn:CreateScanJobResponse>", resp)
        self.assertIn("<wscn:JobId>1</wscn:JobId>", resp)
        self.assertIn("<wscn:JobToken>", resp)
        self.assertIn("<wscn:ImageInformation>", resp)
        self.assertIn("<wscn:PixelsPerLine>%d" % wsd.SCAN_W, resp)
        self.assertIn("<wscn:DocumentFinalParameters>", resp)

    def test_retrieve_image_is_mtom_with_png(self):
        h = FakeHost("HP-OfficeJet-Den", dot11.mac_bytes("00:01:e6:aa:bb:cc"),
                     ip="10.1.2.100")
        srv = wsd.WSDHttpService()
        srv.bind(h)
        srv._scan_job = 1
        content_type, body = srv._retrieve_image("urn:uuid:r", "")
        self.assertIn(b"multipart/related", content_type)
        self.assertIn(b'type="application/xop+xml"', content_type)
        self.assertIn(b"xop:Include", body)
        self.assertIn(b"Content-Type: image/png", body)
        self.assertIn(b"\x89PNG\r\n\x1a\n", body)            # the image part
        self.assertIn(b"RetrieveImageResponse", body)


if __name__ == "__main__":
    unittest.main()
