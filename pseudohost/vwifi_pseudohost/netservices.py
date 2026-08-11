#
# vwifi-pseudohost — TCP-backed network services
#
# SPDX-License-Identifier: GPL-2.0-or-later
#
# Concrete services that a pseudo-host or pseudo-AP can run, now that
# there is a TCP layer under them:
#
#   LPDService     an RFC 1179 line-printer daemon (TCP 515) — accepts a
#                  print job and records it
#   HTTPService    a tiny HTTP/1.0 server (one canned page) — the admin
#                  UI a printer / NAS / smart screen would expose
#   NASService     an SMB-ish presence: accepts on 139/445, answers an
#                  HTTP admin page, and advertises _smb/_http over mDNS
#
# These are simulations sized for a lab: LPD really does speak the
# protocol (a real `lpr` job is accepted and logged), HTTP serves a real
# page, and the NAS is convincing to a scan without pretending to be a
# full SMB stack.  Each is a TCPService, so it becomes live wherever a
# TCP stack is attached and is advertisement-only where one is not.
#
from .services import TCPService
from . import netstack


# ---------------------------------------------------------------------------
# LPD — RFC 1179 line printer daemon
# ---------------------------------------------------------------------------

class LPDService(TCPService):
    """A line-printer daemon.  Accepts jobs and records them.

    RFC 1179 is a simple binary line protocol on TCP 515:
      \\x02<queue>\\n            receive-a-job
      then, per file:
        \\x02<len> <ctlname>\\n  control file follows
        \\x01<len> <dataname>\\n data file follows
      each sub-transfer is ACKed with a single \\x00 byte, and the file
    body is terminated by a trailing \\x00.  We track just enough state to
    ACK correctly and to count the bytes of the job.
    """

    name = "lpd"
    tcp_ports = (515,)

    def on_connect(self, conn):
        conn.data["phase"] = "cmd"
        conn.data["want"] = 0          # bytes remaining in current subfile
        conn.data["job"] = 0           # data bytes seen this job
        conn.data["buf"] = bytearray()

    def on_data(self, conn, data):
        d = conn.data
        buf = d["buf"]
        buf += data
        while buf:
            if d["want"] > 0:
                take = min(d["want"], len(buf))
                body = bytes(buf[:take])
                del buf[:take]
                d["want"] -= take
                if d["want"] == 0:
                    body = body[:-1]           # drop the trailing NUL
                    conn.send(b"\x00")         # ACK the completed subfile
                if d.get("is_data"):
                    d["job"] += len(body)
                continue
            nl = buf.find(b"\n")
            if nl < 0:
                break
            line = bytes(buf[:nl])
            del buf[: nl + 1]
            self._command(conn, line)

    def _command(self, conn, line):
        if not line:
            return
        code = line[0]
        rest = line[1:]
        if code == 0x02 and (b" " not in rest):
            # 0x02 <queue> : receive-a-job
            conn.data["queue"] = rest.decode("ascii", "replace")
            self.log("print job -> queue '%s'" % conn.data["queue"])
            conn.send(b"\x00")
        elif code == 0x02:
            # 0x02 <len> <name> : control file follows
            self._begin_subfile(conn, rest, is_data=False)
        elif code == 0x03:
            # 0x03 <len> <name> : data file follows
            self._begin_subfile(conn, rest, is_data=True)
        elif code == 0x01:
            # 0x01 : abort job
            conn.send(b"\x00")
        else:
            conn.send(b"\x00")

    def _begin_subfile(self, conn, rest, is_data):
        try:
            length_s, _name = rest.split(b" ", 1)
            length = int(length_s)
        except (ValueError, IndexError):
            conn.send(b"\x01")                  # negative ack
            return
        conn.data["want"] = length + 1          # +1 for the trailing NUL
        conn.data["is_data"] = is_data
        conn.send(b"\x00")

    def on_close(self, conn):
        if conn.data.get("job"):
            self.log("job complete: %d data bytes from queue '%s'" % (
                conn.data["job"], conn.data.get("queue", "?")))


# ---------------------------------------------------------------------------
# HTTP — a one-page embedded web server
# ---------------------------------------------------------------------------

class HTTPService(TCPService):
    """Answers any HTTP/1.0-ish request with one canned page.

    Enough to be the device's admin/status UI to a browser or a scanner
    grabbing a banner; not a real web server.
    """

    name = "http"
    tcp_ports = (80,)
    server_banner = "PseudoHost/1.0"
    title = "PseudoHost"

    def page(self):
        return ("<!doctype html><html><head><title>%s</title></head>"
                "<body><h1>%s</h1><p>This device is simulated by "
                "vwifi-pseudohost.</p></body></html>" % (
                    self.title, self.title)).encode()

    def on_data(self, conn, data):
        buf = conn.data.setdefault("buf", bytearray())
        buf += data
        if b"\r\n\r\n" not in buf and b"\n\n" not in buf:
            return                              # wait for end of headers
        body = self.page()
        resp = ("HTTP/1.0 200 OK\r\n"
                "Server: %s\r\n"
                "Content-Type: text/html\r\n"
                "Content-Length: %d\r\n"
                "Connection: close\r\n\r\n" % (
                    self.server_banner, len(body))).encode() + body
        conn.send(resp)
        conn.close()


# ---------------------------------------------------------------------------
# NAS — an SMB-style presence
# ---------------------------------------------------------------------------

class NASHTTPService(HTTPService):
    name = "nas-http"
    tcp_ports = (5000,)                        # Synology-DSM-style admin port
    server_banner = "nginx"
    title = "PseudoNAS"

    def page(self):
        return (b"<!doctype html><html><head><title>PseudoNAS</title></head>"
                b"<body><h1>PseudoNAS</h1><p>Login</p>"
                b"<form><input name=username><input name=password "
                b"type=password></form></body></html>")


class SMBService(TCPService):
    """Accepts SMB connections and answers with a minimal banner.

    A real SMB negotiate is out of scope; what a scan sees here is an
    open 445/139 that completes the TCP handshake and returns a short,
    SMB-shaped byte string rather than silence — enough to read as "a
    file server is here".  Discovery is handled by advertising _smb._tcp
    over mDNS in the NAS profile.
    """

    name = "smb"
    tcp_ports = (139, 445)

    def on_data(self, conn, data):
        # NetBIOS session / SMB2 clients send a request first; answer with
        # a tiny NetBIOS-framed blob so the socket isn't just dead air.
        if conn.data.get("greeted"):
            return
        conn.data["greeted"] = True
        # NetBIOS session keep-alive (0x85) — harmless, and enough to make
        # the port look alive to a probe.
        conn.send(bytes([0x85, 0x00, 0x00, 0x00]))
