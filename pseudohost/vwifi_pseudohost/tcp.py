#
# vwifi-pseudohost — a minimal server-side TCP
#
# SPDX-License-Identifier: GPL-2.0-or-later
#
# Just enough TCP for a pseudo-host or pseudo-AP to *host* services:
# passive open (listen), the three-way handshake, in-order data both
# ways, and an orderly close.  It is server-side and single-segment-at-a-
# time; the medium is virtual and effectively lossless, so there is no
# retransmission timer, no congestion control, and no reassembly of
# out-of-order segments — a gap is simply re-ACKed and the sender resends.
# That is a deliberate scope: it makes LPD, a small HTTP page, and an
# SMB-style banner work, which is what the service layer needs, without
# dragging a full stack in.
#
# A TCPService (services.py) registers its ports here; on an established
# connection the stack calls the service's on_connect/on_data/on_close,
# and the service writes back through conn.send().
#
import os
import struct

from . import netstack

# flags
FIN = 0x01
SYN = 0x02
RST = 0x04
PSH = 0x08
ACK = 0x10

# connection states
LISTEN = "LISTEN"
SYN_RCVD = "SYN_RCVD"
ESTABLISHED = "ESTABLISHED"
CLOSE_WAIT = "CLOSE_WAIT"
LAST_ACK = "LAST_ACK"
FIN_WAIT_1 = "FIN_WAIT_1"
CLOSED = "CLOSED"

WINDOW = 64240
# We never fragment IP (DF is set) and advertise no MSS option, so every
# outbound segment must fit a single link frame on its own: keep the
# payload comfortably under a 1500-byte MTU (1400 + 20 TCP + 20 IP = 1440).
# A response larger than this (e.g. WSD device metadata, an IPP attribute
# set) is split across several segments; without this a client such as
# Windows' WSDAPI silently drops the oversized packet and the exchange
# looks like it "completed" while the peer got nothing usable.
MSS = 1400


def _u32(x):
    return x & 0xFFFFFFFF


class TCPConn:
    """One TCP connection.  Handed to a service; the service calls send()
    and close()."""

    def __init__(self, stack, local_ip, local_port, remote_ip, remote_port,
                 tcpstack):
        self.stack = stack
        self.local_ip = local_ip
        self.local_port = local_port
        self.remote_ip = remote_ip
        self.remote_port = remote_port
        self._tcp = tcpstack
        self.state = LISTEN
        self.iss = struct.unpack(">I", os.urandom(4))[0]
        self.snd_nxt = self.iss
        self.snd_una = self.iss
        self.rcv_nxt = 0
        self.service = None
        self.data = {}          # opaque per-connection scratch for services

    @property
    def key(self):
        return (self.remote_ip, self.remote_port, self.local_port)

    # -- outbound ----------------------------------------------------------
    def _segment(self, flags, payload=b""):
        seg = _build_segment(self.local_port, self.remote_port,
                             self.snd_nxt, self.rcv_nxt, flags, payload,
                             self.local_ip, self.remote_ip)
        self.stack.send_ip(self.remote_ip, netstack.IPPROTO_TCP, seg,
                           src_ip=self.local_ip)

    def send(self, data):
        """Send application data to the peer, split into MSS-sized segments.

        The medium is lossless and in-order, so we can stream the segments
        back to back with no retransmit timer; PSH is set only on the last
        one.  Splitting is mandatory, not an optimisation: an oversized
        single segment would need IP fragmentation we do not do, and the
        peer drops it.
        """
        if self.state not in (ESTABLISHED, CLOSE_WAIT):
            return
        if not data:
            return
        off, n = 0, len(data)
        while off < n:
            chunk = data[off:off + MSS]
            off += len(chunk)
            flags = ACK | (PSH if off >= n else 0)
            self._segment(flags, chunk)
            self.snd_nxt = _u32(self.snd_nxt + len(chunk))

    def close(self):
        if self.state in (ESTABLISHED,):
            self._segment(FIN | ACK)
            self.snd_nxt = _u32(self.snd_nxt + 1)
            self.state = FIN_WAIT_1
        elif self.state == CLOSE_WAIT:
            self._segment(FIN | ACK)
            self.snd_nxt = _u32(self.snd_nxt + 1)
            self.state = LAST_ACK

    def _ack(self):
        self._segment(ACK)


class TCPStack:
    def __init__(self, stack, log=None):
        self.stack = stack
        self.log = log or (lambda *a: None)
        self.listeners = {}                    # port -> service
        self.conns = {}                        # (rip, rport, lport) -> conn
        stack.attach_tcp(self)

    def listen(self, port, service):
        self.listeners[port] = service

    # -- ingress -----------------------------------------------------------
    def on_segment(self, src_ip, dst_ip, seg):
        if len(seg) < 20:
            return
        (sport, dport, seq, ack, off_flags, _win, _sum, _urg) = \
            struct.unpack_from(">HHIIHHHH", seg, 0)
        data_off = (off_flags >> 12) * 4
        flags = off_flags & 0x3F
        payload = seg[data_off:]
        key = (src_ip, sport, dport)
        conn = self.conns.get(key)

        if conn is None:
            if (flags & SYN) and not (flags & ACK) and dport in self.listeners:
                self._accept(src_ip, sport, dport, seq)
            elif not (flags & RST):
                # unknown connection -> reset the peer
                self._send_rst(dst_ip, src_ip, dport, sport, ack, seq,
                               flags, len(payload))
            return

        if flags & RST:
            self._drop(conn)
            return
        self._deliver(conn, seq, ack, flags, payload)

    def _accept(self, rip, rport, lport, seq):
        conn = TCPConn(self.stack, self.stack.ip, lport, rip, rport, self)
        conn.rcv_nxt = _u32(seq + 1)
        conn.state = SYN_RCVD
        conn.service = self.listeners[lport]
        conn._segment(SYN | ACK)
        conn.snd_nxt = _u32(conn.snd_nxt + 1)
        self.conns[conn.key] = conn

    def _deliver(self, conn, seq, ack, flags, payload):
        if conn.state == SYN_RCVD:
            if (flags & ACK) and ack == conn.snd_nxt:
                conn.snd_una = ack
                conn.state = ESTABLISHED
                self._call(conn, "on_connect")
            # A client that sends data with the handshake ACK falls
            # through to the ESTABLISHED handling below.
            if conn.state != ESTABLISHED:
                return

        if flags & ACK:
            conn.snd_una = ack

        if payload:
            if seq == conn.rcv_nxt:
                conn.rcv_nxt = _u32(conn.rcv_nxt + len(payload))
                conn._ack()
                self._call(conn, "on_data", payload)
            else:
                conn._ack()                    # out of order: re-ACK, drop

        if flags & FIN:
            if seq == conn.rcv_nxt or True:
                conn.rcv_nxt = _u32(conn.rcv_nxt + 1)
                conn._ack()
                if conn.state == ESTABLISHED:
                    conn.state = CLOSE_WAIT
                    self._call(conn, "on_close")
                    if conn.state == CLOSE_WAIT:
                        conn.close()           # send our FIN
                elif conn.state == FIN_WAIT_1:
                    conn.state = CLOSED
                    self._drop(conn)

        if conn.state == LAST_ACK and (flags & ACK) and not payload:
            self._drop(conn)

    def _call(self, conn, hook, *args):
        svc = conn.service
        fn = getattr(svc, hook, None)
        if fn:
            try:
                fn(conn, *args)
            except Exception as e:                     # a broken service
                self.log("tcp: service %s raised: %s" % (hook, e))
                conn.close()

    def _drop(self, conn):
        conn.state = CLOSED
        self.conns.pop(conn.key, None)

    def _send_rst(self, local_ip, remote_ip, lport, rport, ack, seq, flags,
                  plen):
        # RST to a stray segment; seq/ack per RFC 793.
        if flags & ACK:
            rseq, rack, rflags = ack, 0, RST
        else:
            rseq, rack, rflags = 0, _u32(seq + plen + (1 if flags & SYN else 0)), \
                RST | ACK
        seg = _build_segment(lport, rport, rseq, rack, rflags, b"",
                             local_ip, remote_ip)
        self.stack.send_ip(remote_ip, netstack.IPPROTO_TCP, seg,
                           src_ip=local_ip)


def _build_segment(sport, dport, seq, ack, flags, payload, src_ip, dst_ip):
    hdr = struct.pack(">HHIIHHHH", sport, dport, _u32(seq), _u32(ack),
                      (5 << 12) | flags, WINDOW, 0, 0)
    seg = hdr + payload
    pseudo = (src_ip + dst_ip + bytes([0, netstack.IPPROTO_TCP])
              + struct.pack(">H", len(seg)))
    c = netstack._cksum(pseudo + seg)
    return seg[:16] + struct.pack(">H", c) + seg[18:]
