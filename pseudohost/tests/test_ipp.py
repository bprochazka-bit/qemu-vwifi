#
# vwifi-pseudohost — IPP server tests
#
# SPDX-License-Identifier: GPL-2.0-or-later
#
import os
import struct
import sys
import tempfile
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
sys.path.insert(0, os.path.dirname(__file__))

from vwifi_pseudohost import ipp                       # noqa: E402
from vwifi_pseudohost.printing import PrintSink         # noqa: E402
from tcp_client import ServerHost, ClientSim            # noqa: E402


def ipp_request(op, reqid=1, doc=b""):
    body = struct.pack(">HHI", 0x0200, op, reqid)
    body += bytes([ipp.TAG_OPERATION])
    body += ipp._attr(ipp.TAG_CHARSET, "attributes-charset", b"utf-8")
    body += ipp._attr(ipp.TAG_LANGUAGE, "attributes-natural-language", b"en")
    body += ipp._attr(ipp.TAG_URI, "printer-uri",
                      b"ipp://10.1.2.100:631/ipp/print")
    body += bytes([ipp.TAG_END])
    body += doc
    return body


def parse_ipp_response(resp):
    """Return (status, {attr_name: value_bytes}) from an IPP response."""
    assert len(resp) >= 8
    status = struct.unpack_from(">H", resp, 2)[0]
    attrs = {}
    i = 8
    n = len(resp)
    while i < n:
        tag = resp[i]
        if tag == ipp.TAG_END:
            break
        if tag <= 0x05:
            i += 1
            continue
        nlen = struct.unpack_from(">H", resp, i + 1)[0]
        name = resp[i + 3:i + 3 + nlen]
        i += 3 + nlen
        vlen = struct.unpack_from(">H", resp, i)[0]
        val = resp[i + 2:i + 2 + vlen]
        i += 2 + vlen
        if name:
            attrs[name.decode()] = val
    return status, attrs


class TestIPP(unittest.TestCase):
    _sport = 41000

    def _server(self, print_dir=None):
        srv = ServerHost(ip="10.1.2.100")
        srv.hostname = "HP-OfficeJet-PH01"
        srv.wsd_model = "HP OfficeJet Pro 9015"
        srv.print_sink = PrintSink(print_dir)
        ipp.IPPService().bind(srv)
        return srv

    def _post(self, srv, body):
        TestIPP._sport += 1
        c = ClientSim(srv, dport=631, sport=TestIPP._sport)
        self.assertTrue(c.connect())
        req = (b"POST /ipp/print HTTP/1.1\r\nHost: 10.1.2.100:631\r\n"
               b"Content-Type: application/ipp\r\n"
               b"Content-Length: " + str(len(body)).encode() +
               b"\r\n\r\n" + body)
        c.send(req)
        resp = c.recv()
        # strip HTTP headers
        hb = resp.find(b"\r\n\r\n")
        return resp[hb + 4:]

    def test_get_printer_attributes(self):
        srv = self._server()
        ipp_resp = self._post(srv, ipp_request(ipp.OP_GET_PRINTER_ATTRS))
        status, attrs = parse_ipp_response(ipp_resp)
        self.assertEqual(status, ipp.OK)
        self.assertEqual(attrs.get("printer-make-and-model"),
                         b"HP OfficeJet Pro 9015")
        self.assertEqual(attrs.get("printer-uri-supported"),
                         b"ipp://10.1.2.100:631/ipp/print")
        self.assertIn(b"application/pdf", attrs.get("document-format-supported"))
        self.assertEqual(attrs.get("printer-state"), struct.pack(">i", 3))
        self.assertEqual(attrs.get("printer-is-accepting-jobs"), b"\x01")
        self.assertIn(b"ipp-everywhere", attrs.get("ipp-features-supported"))

    def test_print_job_reaches_sink(self):
        d = tempfile.mkdtemp(prefix="ph-ipp-")
        srv = self._server(print_dir=d)
        doc = b"%PDF-1.7\nprinted via IPP\n%%EOF"
        ipp_resp = self._post(srv,
                              ipp_request(ipp.OP_PRINT_JOB, reqid=7, doc=doc))
        status, attrs = parse_ipp_response(ipp_resp)
        self.assertEqual(status, ipp.OK)
        self.assertIn(b"job-id", ipp_resp)
        files = os.listdir(d)
        self.assertEqual(len(files), 1)
        self.assertTrue(files[0].endswith(".pdf"))
        with open(os.path.join(d, files[0]), "rb") as f:
            self.assertEqual(f.read(), doc)

    def test_get_job_attributes_reports_completed(self):
        # The bug that crashed the spooler: Get-Job-Attributes returned an
        # empty body. It must carry the job's id and a terminal state so
        # Windows sees the job done and clears the queue.
        srv = self._server()
        self._post(srv, ipp_request(ipp.OP_CREATE_JOB, reqid=1))
        resp = self._post(srv, ipp_request(ipp.OP_GET_JOB_ATTRS, reqid=2))
        status, attrs = parse_ipp_response(resp)
        self.assertEqual(status, ipp.OK)
        self.assertIn(b"job-id", resp)
        self.assertEqual(attrs.get("job-state"), struct.pack(">i", 9))  # done
        self.assertIn(b"job-completed", resp)

    def test_create_job_is_pending_not_completed(self):
        srv = self._server()
        resp = self._post(srv, ipp_request(ipp.OP_CREATE_JOB, reqid=1))
        _status, attrs = parse_ipp_response(resp)
        self.assertEqual(attrs.get("job-state"), struct.pack(">i", 3))  # pend

    def test_validate_job_has_no_job_group(self):
        srv = self._server()
        resp = self._post(srv, ipp_request(ipp.OP_VALIDATE_JOB, reqid=1))
        status, attrs = parse_ipp_response(resp)
        self.assertEqual(status, ipp.OK)
        self.assertNotIn("job-state", attrs)
        self.assertNotIn("job-id", attrs)

    def test_chunked_print_job(self):
        d = tempfile.mkdtemp(prefix="ph-ipp-")
        srv = self._server(print_dir=d)
        doc = b"%PDF-1.7\nchunked job\n%%EOF"
        body = ipp_request(ipp.OP_PRINT_JOB, reqid=8, doc=doc)
        # chunk the whole IPP body
        chunked = (b"%X\r\n%s\r\n0\r\n\r\n" % (len(body), body))
        TestIPP._sport += 1
        c = ClientSim(srv, dport=631, sport=TestIPP._sport)
        self.assertTrue(c.connect())
        req = (b"POST /ipp/print HTTP/1.1\r\nHost: x\r\n"
               b"Content-Type: application/ipp\r\n"
               b"Transfer-Encoding: chunked\r\n\r\n" + chunked)
        c.send(req)
        resp = c.recv()
        self.assertIn(b"job-id", resp)
        files = os.listdir(d)
        self.assertEqual(len(files), 1)
        with open(os.path.join(d, files[0]), "rb") as f:
            self.assertEqual(f.read(), doc)


if __name__ == "__main__":
    unittest.main()
