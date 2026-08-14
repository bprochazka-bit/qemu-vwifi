#
# vwifi-pseudohost — eSCL (driverless scan) server tests
#
# SPDX-License-Identifier: GPL-2.0-or-later
#
import os
import re
import sys
import unittest
import xml.dom.minidom as minidom

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
sys.path.insert(0, os.path.dirname(__file__))

from vwifi_pseudohost import escl                        # noqa: E402
from tcp_client import ServerHost, ClientSim             # noqa: E402


class TestMakePDF(unittest.TestCase):
    def test_pdf_is_structurally_valid(self):
        pdf = escl.make_pdf(1700, 2200)
        self.assertEqual(pdf[:5], b"%PDF-")
        self.assertTrue(pdf.rstrip().endswith(b"%%EOF"))
        self.assertIn(b"/Subtype/Image", pdf)
        # every xref offset must point at "<n> 0 obj"
        sx = int(pdf.rsplit(b"startxref", 1)[1].split(b"%%EOF")[0].strip())
        lines = pdf[sx:].split(b"\n")
        n = int(lines[1].split()[1])
        for i, ln in enumerate(lines[2:2 + n]):
            parts = ln.split()
            if len(parts) >= 3 and parts[2] == b"n":
                off = int(parts[0])
                self.assertRegex(pdf[off:off + 12], rb"%d 0 obj" % i)


class TestESCL(unittest.TestCase):
    _sport = 46000

    def _server(self):
        h = ServerHost(ip="10.1.2.100")
        h.wsd_model = "HP OfficeJet Pro 8600"
        h.wsd_manufacturer = "HP"
        srv = escl.ESCLService()
        srv.bind(h)
        return h, srv

    def _req(self, h, method, path, body=b""):
        TestESCL._sport += 1
        c = ClientSim(h, dport=8080, sport=TestESCL._sport)
        self.assertTrue(c.connect())
        req = ("%s %s HTTP/1.1\r\nHost: x\r\nContent-Length: %d\r\n\r\n"
               % (method, path, len(body))).encode() + body
        c.send(req)
        return c.recv()

    def test_capabilities_is_well_formed_and_advertises_platen_and_adf(self):
        _h, srv = self._server()
        caps = srv._capabilities()
        minidom.parseString(caps)               # raises on malformed XML
        self.assertIn(b"PlatenInputCaps", caps)
        self.assertIn(b"AdfSimplexInputCaps", caps)
        self.assertIn(b"application/pdf", caps)
        self.assertIn(b"HP OfficeJet Pro 8600", caps)

    def test_capabilities_over_http(self):
        h, _srv = self._server()
        resp = self._req(h, "GET", "/eSCL/ScannerCapabilities")
        self.assertIn(b"200 OK", resp)
        self.assertIn(b"ScannerCapabilities", resp)

    def test_full_scan_job_flow(self):
        h, _srv = self._server()
        # start a job
        resp = self._req(h, "POST", "/eSCL/ScanJobs", b"<ScanSettings/>")
        self.assertIn(b"201 Created", resp)
        m = re.search(rb"Location:\s*(\S+)", resp)
        self.assertIsNotNone(m)
        job_path = m.group(1).decode().split("8080", 1)[1]

        # first NextDocument returns the page as a PDF
        resp = self._req(h, "GET", job_path + "/NextDocument")
        self.assertIn(b"200 OK", resp)
        self.assertIn(b"application/pdf", resp)
        self.assertIn(b"%PDF-", resp)

        # second NextDocument reports the job complete (404)
        resp = self._req(h, "GET", job_path + "/NextDocument")
        self.assertIn(b"404", resp)

    def test_unknown_path_404(self):
        h, _srv = self._server()
        resp = self._req(h, "GET", "/eSCL/Nonsense")
        self.assertIn(b"404", resp)

    def test_status_reports_idle(self):
        _h, srv = self._server()
        st = srv._status()
        minidom.parseString(st)
        self.assertIn(b"Idle", st)


if __name__ == "__main__":
    unittest.main()
