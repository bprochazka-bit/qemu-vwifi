#
# vwifi-pseudohost — print sink, raw 9100, and WSD print tests
#
# SPDX-License-Identifier: GPL-2.0-or-later
#
import os
import sys
import tempfile
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
sys.path.insert(0, os.path.dirname(__file__))

from vwifi_pseudohost import wsd                       # noqa: E402
from vwifi_pseudohost.printing import PrintSink, JetDirectService  # noqa: E402
from tcp_client import ServerHost, ClientSim            # noqa: E402


class TestPrintSink(unittest.TestCase):
    def test_writes_file(self):
        d = tempfile.mkdtemp(prefix="ph-print-")
        sink = PrintSink(d)
        path = sink.write_job(b"%PDF-1.4 hello", jobname="My Doc")
        self.assertTrue(path and os.path.exists(path))
        self.assertTrue(path.endswith(".pdf"))
        with open(path, "rb") as f:
            self.assertEqual(f.read(), b"%PDF-1.4 hello")

    def test_null_sink_discards(self):
        sink = PrintSink(None)
        self.assertIsNone(sink.write_job(b"data", jobname="x"))

    def test_sniffs_postscript(self):
        d = tempfile.mkdtemp(prefix="ph-print-")
        p = PrintSink(d).write_job(b"%!PS-Adobe-3.0\n...", jobname="ps")
        self.assertTrue(p.endswith(".ps"))


class TestJetDirect(unittest.TestCase):
    def test_raw_job_to_sink(self):
        d = tempfile.mkdtemp(prefix="ph-print-")
        srv = ServerHost(ip="10.1.2.100")
        srv.print_sink = PrintSink(d)
        JetDirectService().bind(srv)

        c = ClientSim(srv, dport=9100)
        self.assertTrue(c.connect())
        c.send(b"\x1b%-12345X@PJL\nraw print payload\n")
        c.close()                              # EOF -> job flushed to sink
        files = os.listdir(d)
        self.assertEqual(len(files), 1)
        with open(os.path.join(d, files[0]), "rb") as f:
            self.assertIn(b"raw print payload", f.read())


class TestWSDPrint(unittest.TestCase):
    def _server(self, print_dir=None):
        srv = ServerHost(ip="10.1.2.100")
        srv.hostname = "HP-OfficeJet-Den"
        srv.wsd_manufacturer = "HP"
        srv.wsd_model = "HP OfficeJet Pro 9015"
        srv.print_sink = PrintSink(print_dir)
        wsd.WSDHttpService().bind(srv)
        return srv

    _sport = 40000

    def _post(self, srv, body_bytes, ctype="application/soap+xml"):
        # A fresh source port per request (real clients do this; the test
        # helper otherwise reuses one 4-tuple and collides).
        TestWSDPrint._sport += 1
        c = ClientSim(srv, dport=wsd.WSD_HTTP_PORT, sport=TestWSDPrint._sport)
        self.assertTrue(c.connect())
        req = (b"POST /dev HTTP/1.1\r\nHost: x\r\nContent-Type: " +
               ctype.encode() + b"\r\nContent-Length: " +
               str(len(body_bytes)).encode() + b"\r\n\r\n" + body_bytes)
        c.send(req)
        return c.recv()

    def _soap(self, action, extra=""):
        return (
            '<soap:Envelope '
            'xmlns:soap="http://www.w3.org/2003/05/soap-envelope" '
            'xmlns:wsa="http://schemas.xmlsoap.org/ws/2004/08/addressing" '
            'xmlns:wprt="http://schemas.microsoft.com/windows/2006/08/wdp/'
            'print"><soap:Header>'
            '<wsa:MessageID>urn:uuid:req-1</wsa:MessageID>'
            '<wsa:Action>%s</wsa:Action></soap:Header>'
            '<soap:Body>%s</soap:Body></soap:Envelope>' % (action, extra)
        ).encode()

    def test_get_printer_elements(self):
        srv = self._server()
        resp = self._post(srv, self._soap(wsd.NS_WPRT + "/GetPrinterElements"))
        text = resp.decode()
        self.assertIn("GetPrinterElementsResponse", text)
        self.assertIn("HP OfficeJet Pro 9015", text)
        self.assertIn("<wprt:PrinterState>Idle</wprt:PrinterState>", text)

    def test_create_job_and_send_document(self):
        d = tempfile.mkdtemp(prefix="ph-wsd-")
        srv = self._server(print_dir=d)

        cj = self._post(srv, self._soap(
            wsd.NS_WPRT + "/CreatePrintJob",
            "<wprt:CreatePrintJobRequest><wprt:PrintTicket/>"
            "<wprt:JobDescription><wprt:JobName>Quarterly Report"
            "</wprt:JobName></wprt:JobDescription>"
            "</wprt:CreatePrintJobRequest>"))
        self.assertIn(b"CreatePrintJobResponse", cj)
        self.assertIn(b"<wprt:JobId>1</wprt:JobId>", cj)

        # SendDocument as MTOM: SOAP part + binary document part.
        doc = b"%PDF-1.7\nthis is the printed document\n%%EOF"
        boundary = "MIMEBoundary123"
        soap = self._soap(wsd.NS_WPRT + "/SendDocument").decode()
        mtom = (
            ("--%s\r\n" % boundary) +
            "Content-Type: application/xop+xml; charset=UTF-8; "
            "type=\"application/soap+xml\"\r\n"
            "Content-Transfer-Encoding: 8bit\r\n"
            "Content-ID: <soap@x>\r\n\r\n" + soap + "\r\n" +
            ("--%s\r\n" % boundary) +
            "Content-Type: application/octet-stream\r\n"
            "Content-Transfer-Encoding: binary\r\n"
            "Content-ID: <doc@x>\r\n\r\n"
        ).encode() + doc + ("\r\n--%s--\r\n" % boundary).encode()

        resp = self._post(srv, mtom,
                          ctype='multipart/related; boundary="%s"; '
                                'type="application/xop+xml"' % boundary)
        self.assertIn(b"SendDocumentResponse", resp)

        files = os.listdir(d)
        self.assertEqual(len(files), 1)
        self.assertTrue(files[0].endswith(".pdf"))
        with open(os.path.join(d, files[0]), "rb") as f:
            self.assertEqual(f.read(), doc)     # exact bytes preserved


if __name__ == "__main__":
    unittest.main()
