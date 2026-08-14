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
        self._seeded = False

    def _seed_count(self):
        """Resume numbering past the highest job already on disk.

        The counter lives only in memory, so without this a restarted
        pseudo-host would begin again at 0001 and overwrite the jobs a
        previous run had written.  Scan the output directory once and pick
        up after its largest ``NNNN-`` prefix so indices are never reused.
        """
        self._seeded = True
        if not self.directory:
            return
        hi = 0
        try:
            for n in os.listdir(self.directory):
                m = re.match(r"(\d+)-", n)
                if m:
                    hi = max(hi, int(m.group(1)))
        except OSError:
            return
        self.count = hi

    def write_job(self, data, jobname="job", source="print"):
        if not data:
            return None
        if not self._seeded:
            self._seed_count()
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
            # Belt and suspenders: if this index is somehow already on disk
            # (e.g. files created between seeding and now), step past it so a
            # job is never silently overwritten.
            while _index_taken(self.directory, self.count):
                self.count += 1
            path = os.path.join(self.directory,
                                "%04d-%s.%s" % (self.count, safe, ext))
            with open(path, "wb") as f:
                f.write(data)
            self.log("%s: wrote %d bytes -> %s" % (source, len(data), path))
            return path
        except OSError as e:
            self.log("%s: could not write job: %s" % (source, e))
            return None


def _index_taken(directory, idx):
    prefix = "%04d-" % idx
    try:
        return any(n.startswith(prefix) for n in os.listdir(directory))
    except OSError:
        return False


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
