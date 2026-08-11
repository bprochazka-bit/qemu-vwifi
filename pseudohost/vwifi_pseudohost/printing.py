#
# vwifi-pseudohost — print job sink and raw (JetDirect) printing
#
# SPDX-License-Identifier: GPL-2.0-or-later
#
# Where accepted print jobs go, and the simplest way to accept one.
#
#   PrintSink        writes each job to a file under a directory, or
#                    discards it (to "null") when no directory is set.
#   JetDirectService raw TCP 9100 — the printer reads the connection to
#                    EOF and hands the bytes to the sink.  This is the
#                    path a Windows "Standard TCP/IP Port (Raw)" prints
#                    over, and it always works: no protocol to get wrong.
#
# The document is whatever PDL the client sends (PostScript, PCL, PDF,
# XPS); the sink sniffs a sensible file extension but never transforms
# the bytes — a simulator records what was sent, it does not render it.
#
import os
import re

from .services import TCPService


class PrintSink:
    def __init__(self, directory=None, log=None):
        self.directory = directory
        self.log = log or (lambda *a: None)
        self.count = 0

    def write_job(self, data, jobname="job", source="print"):
        if not data:
            return None
        self.count += 1
        ext = _sniff_ext(data)
        if not self.directory:
            self.log("%s: discarded %d-byte job '%s' -> null"
                     % (source, len(data), jobname))
            return None
        try:
            os.makedirs(self.directory, exist_ok=True)
            safe = re.sub(r"[^A-Za-z0-9._-]+", "_", jobname).strip("_")[:60] \
                or "job"
            path = os.path.join(self.directory,
                                "%04d-%s.%s" % (self.count, safe, ext))
            with open(path, "wb") as f:
                f.write(data)
            self.log("%s: wrote %d bytes -> %s" % (source, len(data), path))
            return path
        except OSError as e:
            self.log("%s: could not write job: %s" % (source, e))
            return None


def _sniff_ext(data):
    head = data[:64]
    if head[:4] == b"%PDF":
        return "pdf"
    if head[:2] == b"%!" or b"PostScript" in head:
        return "ps"
    if head[:4] == b"PK\x03\x04" or b"application/oxps" in data[:512] \
            or b"[Content_Types].xml" in data[:2048]:
        return "xps"           # XPS/OXPS is a ZIP container
    if head[:1] == b"\x1b":
        return "pcl"           # escape -> PCL/PJL
    if head[:3] == b"\x1b%-" or b"@PJL" in head:
        return "pjl"
    return "prn"


class JetDirectService(TCPService):
    """Raw print server on TCP 9100 (HP JetDirect / 'Standard TCP/IP')."""

    name = "jetdirect"
    tcp_ports = (9100,)

    def on_connect(self, conn):
        conn.data["buf"] = bytearray()

    def on_data(self, conn, data):
        conn.data["buf"] += data

    def on_close(self, conn):
        data = bytes(conn.data.get("buf", b""))
        sink = getattr(self.host, "print_sink", None)
        if sink is not None:
            sink.write_job(data, jobname="raw9100", source="jetdirect")
        elif data:
            self.log("received %d bytes but no print sink configured"
                     % len(data))
