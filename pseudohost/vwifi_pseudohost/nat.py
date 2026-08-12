#
# vwifi-pseudohost — userspace NAT (masquerade to the real host network)
#
# SPDX-License-Identifier: GPL-2.0-or-later
#
# A pseudo-AP is the default gateway for its associated stations.  On its
# own it terminates IP only for its own address, so a station can reach the
# AP and its peers but nothing beyond the virtual medium.  This module gives
# the AP the other half of a home router's job: it masquerades a station's
# off-subnet traffic out onto the *real* host network and relays the answers
# back — so a station on the virtual air can reach the actual internet, the
# same way `iptables -t nat -A POSTROUTING -j MASQUERADE` does for a real AP.
#
# It is a transport-layer NAPT built entirely on ordinary host sockets, so
# it needs no root, no TUN device and no raw sockets for the common case:
#
#   - TCP  — each station connection is terminated here and bridged to a
#            fresh host socket to the real destination; bytes are relayed
#            both ways.  (A minimal, lossless-medium TCP, like tcp.py.)
#   - UDP  — each station flow gets a connected host datagram socket; the
#            payload is forwarded and replies are mapped back to the station.
#   - ICMP — echo requests are forwarded through a Linux "ping" datagram
#            socket when the platform allows it; if it does not (no
#            privilege), ICMP forwarding is quietly disabled and the rest
#            keeps working.
#
# The source address the AP uses on the real network is the host's default
# unless an interface (or source IP) is pinned — that is the "specify an
# interface on the host it will use" knob, wired from `pseudoap
# --nat-interface`.
#
# Integration is via a select loop: the owner (the AP) folds rlist()/wlist()
# into its own select() and calls handle_readable()/handle_writable() and
# tick().  Reply packets are injected back toward the station through the
# AP's NetStack, which ARP-resolves and (on a secured network) CCMP-encrypts
# them per the destination station, exactly like any other frame it sends.
#
import errno
import socket
import struct
import time

from . import netstack
from . import tcp

IPPROTO_ICMP = netstack.IPPROTO_ICMP
IPPROTO_TCP = netstack.IPPROTO_TCP
IPPROTO_UDP = netstack.IPPROTO_UDP

# TCP flags (mirror tcp.py).
_FIN = 0x01
_SYN = 0x02
_RST = 0x04
_PSH = 0x08
_ACK = 0x10

# Keep station-bound TCP segments comfortably inside a plausible link MTU so
# the medium never has to fragment; the real socket hands us data in chunks
# and we re-segment to this.
_MSS = 1400

# Idle lifetimes for the stateless protocols (seconds).
_UDP_IDLE = 30.0
_ICMP_IDLE = 10.0

# A sanity cap so a misbehaving (or hostile) station can't exhaust host fds.
_MAX_FLOWS = 512


def _u32(x):
    return x & 0xFFFFFFFF


def resolve_bind_ip(spec):
    """Turn a --nat-interface value into a source IP string to bind to.

    Accepts a dotted-quad ("10.0.0.7", bound as the source address) or an
    interface name ("eth0", resolved to its IPv4 via SIOCGIFADDR on Linux).
    Returns the IP string, or raises ValueError if it can't be resolved.
    """
    if spec is None:
        return None
    spec = spec.strip()
    # Already an IPv4 literal?
    try:
        socket.inet_aton(spec)
        return spec
    except OSError:
        pass
    # Otherwise treat it as an interface name and look up its address.
    try:
        import fcntl
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            SIOCGIFADDR = 0x8915
            packed = fcntl.ioctl(
                s.fileno(), SIOCGIFADDR,
                struct.pack("256s", spec.encode()[:15]))
            return socket.inet_ntoa(packed[20:24])
        finally:
            s.close()
    except (ImportError, OSError) as e:
        raise ValueError("cannot resolve interface %r to an IPv4 address: %s"
                         % (spec, e))


class _TCPFlow:
    """One station TCP connection, terminated here and bridged to a host
    socket.  We are the server toward the station and the client toward the
    real world; two independent sequence spaces meet in the middle."""

    __slots__ = ("nat", "sip", "sport", "dip", "dport", "sock",
                 "iss", "snd_nxt", "rcv_nxt", "connected", "established",
                 "fin_from_real", "fin_from_sta", "pending")

    def __init__(self, nat, sip, sport, dip, dport):
        self.nat = nat
        self.sip = sip                 # station IP (bytes)
        self.sport = sport             # station port
        self.dip = dip                 # real destination IP (bytes)
        self.dport = dport             # real destination port
        self.sock = None
        self.iss = struct.unpack(">I", _rand4())[0]
        self.snd_nxt = self.iss        # next seq we send to the station
        self.rcv_nxt = 0               # next seq we expect from the station
        self.connected = False         # host socket finished connecting
        self.established = False        # station completed its handshake
        self.fin_from_real = False
        self.fin_from_sta = False
        self.pending = b""             # station data buffered pre-connect

    @property
    def key(self):
        return (self.sip, self.sport, self.dip, self.dport)

    # -- segments toward the station --------------------------------------
    def _emit(self, flags, payload=b""):
        # From the station's view this comes from dip:dport (the server).
        seg = tcp._build_segment(self.dport, self.sport, self.snd_nxt,
                                 self.rcv_nxt, flags, payload,
                                 self.dip, self.sip)
        self.nat.stack.send_ip(self.sip, IPPROTO_TCP, seg, src_ip=self.dip)

    def _rst(self):
        self._emit(_RST | _ACK)

    # -- station -> here ---------------------------------------------------
    def on_syn(self, seq):
        self.rcv_nxt = _u32(seq + 1)
        self.sock = self.nat._open_stream(self.dip, self.dport)
        if self.sock is None:
            self._rst()
            self.nat._drop_tcp(self)
            return
        # Watch for the connect to finish (writable), then SYN|ACK.
        self.nat._tcp_by_sock[self.sock] = self
        self.nat._writers.add(self.sock)

    def on_connected(self):
        self.connected = True
        self.nat._writers.discard(self.sock)
        self.nat._readers.add(self.sock)
        self._emit(_SYN | _ACK)
        self.snd_nxt = _u32(self.snd_nxt + 1)
        if self.pending:
            self._to_real(self.pending)
            self.pending = b""

    def on_connect_failed(self):
        self._rst()
        self.nat._drop_tcp(self)

    def on_segment(self, seq, ack, flags, payload):
        if flags & _RST:
            self.nat._drop_tcp(self)
            return
        if flags & _ACK:
            self.established = True
        if payload:
            if seq == self.rcv_nxt:                # in order: relay + ACK
                self.rcv_nxt = _u32(self.rcv_nxt + len(payload))
                self._emit(_ACK)
                self._to_real(payload)
            else:
                self._emit(_ACK)                   # out of order: re-ACK
        if flags & _FIN:
            self.rcv_nxt = _u32(self.rcv_nxt + 1)
            self._emit(_ACK)
            self.fin_from_sta = True
            self._half_close_real()
            if self.fin_from_real:
                self.nat._drop_tcp(self)

    def _to_real(self, data):
        if not self.connected:
            self.pending += data                   # flush once connected
            return
        try:
            self.sock.sendall(data)
        except OSError:
            self._rst()
            self.nat._drop_tcp(self)

    def _half_close_real(self):
        try:
            self.sock.shutdown(socket.SHUT_WR)
        except OSError:
            pass

    # -- real socket -> station -------------------------------------------
    def on_readable(self):
        try:
            data = self.sock.recv(65535)
        except (BlockingIOError, InterruptedError):
            return
        except OSError:
            self._rst()
            self.nat._drop_tcp(self)
            return
        if not data:                               # real side closed
            self.fin_from_real = True
            self._emit(_FIN | _ACK)
            self.snd_nxt = _u32(self.snd_nxt + 1)
            self.nat._readers.discard(self.sock)
            if self.fin_from_sta:
                self.nat._drop_tcp(self)
            return
        for i in range(0, len(data), _MSS):
            chunk = data[i:i + _MSS]
            self._emit(_PSH | _ACK, chunk)
            self.snd_nxt = _u32(self.snd_nxt + len(chunk))


class _UDPFlow:
    __slots__ = ("sip", "sport", "dip", "dport", "sock", "last")

    def __init__(self, sip, sport, dip, dport, sock):
        self.sip = sip
        self.sport = sport
        self.dip = dip
        self.dport = dport
        self.sock = sock
        self.last = 0.0

    @property
    def key(self):
        return (self.sip, self.sport, self.dip, self.dport)


class _ICMPFlow:
    __slots__ = ("sip", "ident", "dip", "sock", "last")

    def __init__(self, sip, ident, dip, sock):
        self.sip = sip
        self.ident = ident
        self.dip = dip
        self.sock = sock
        self.last = 0.0

    @property
    def key(self):
        return (self.sip, self.ident, self.dip)


class NAT:
    """Masquerade a NetStack's off-subnet traffic onto the real host.

    Register with stack.set_forward_handler(self.forward); the owner folds
    rlist()/wlist() into its select() and calls handle_readable(),
    handle_writable() and tick().
    """

    def __init__(self, stack, bind_ip=None, log=None, verbose=False,
                 icmp=True):
        self.stack = stack
        self.bind_ip = bind_ip
        self.log = log or (lambda *a: None)
        self.verbose = verbose
        self._icmp_ok = icmp                      # may flip off on first fail

        self._tcp = {}                            # key -> _TCPFlow
        self._tcp_by_sock = {}                    # sock -> _TCPFlow
        self._udp = {}                            # key -> _UDPFlow
        self._udp_by_sock = {}
        self._icmp = {}                           # key -> _ICMPFlow
        self._icmp_by_sock = {}
        self._readers = set()                     # socks watched for read
        self._writers = set()                     # socks watched for write

        stack.set_forward_handler(self.forward)

    # ---- select integration --------------------------------------------
    def rlist(self):
        return list(self._readers)

    def wlist(self):
        return list(self._writers)

    def handle_readable(self, sock):
        flow = self._tcp_by_sock.get(sock)
        if flow is not None:
            flow.on_readable()
            return
        flow = self._udp_by_sock.get(sock)
        if flow is not None:
            self._udp_reply(flow)
            return
        flow = self._icmp_by_sock.get(sock)
        if flow is not None:
            self._icmp_reply(flow)

    def handle_writable(self, sock):
        flow = self._tcp_by_sock.get(sock)
        if flow is None:
            self._writers.discard(sock)
            return
        err = sock.getsockopt(socket.SOL_SOCKET, socket.SO_ERROR)
        if err == 0:
            if self.verbose:
                self.log("nat tcp: %s connected" % _flowname(flow))
            flow.on_connected()
        else:
            self.log("nat tcp: connect to %s:%d failed: %s" % (
                netstack.ip_str(flow.dip), flow.dport, errno.errorcode.get(
                    err, err)))
            flow.on_connect_failed()

    def tick(self):
        now = _now()
        for flow in list(self._udp.values()):
            if now - flow.last > _UDP_IDLE:
                self._drop_udp(flow)
        for flow in list(self._icmp.values()):
            if now - flow.last > _ICMP_IDLE:
                self._drop_icmp(flow)

    def close(self):
        for flow in list(self._tcp.values()):
            self._drop_tcp(flow)
        for flow in list(self._udp.values()):
            self._drop_udp(flow)
        for flow in list(self._icmp.values()):
            self._drop_icmp(flow)

    # ---- ingress from the stack ----------------------------------------
    def forward(self, src, dst, proto, body):
        # Only masquerade genuinely off-subnet unicast; never loop the AP's
        # own subnet or its address back out to the world.
        if self.stack.same_subnet(dst) or dst == self.stack.ip:
            return
        if proto == IPPROTO_TCP:
            self._on_tcp(src, dst, body)
        elif proto == IPPROTO_UDP:
            self._on_udp(src, dst, body)
        elif proto == IPPROTO_ICMP:
            self._on_icmp(src, dst, body)

    def _at_capacity(self):
        return (len(self._tcp) + len(self._udp) + len(self._icmp)
                >= _MAX_FLOWS)

    # ---- TCP ------------------------------------------------------------
    def _on_tcp(self, src, dst, seg):
        if len(seg) < 20:
            return
        sport, dport, seq, ack, off_flags = \
            struct.unpack_from(">HHIIH", seg, 0)
        data_off = (off_flags >> 12) * 4
        flags = off_flags & 0x3F
        payload = seg[data_off:]
        key = (src, sport, dst, dport)
        flow = self._tcp.get(key)
        if flow is None:
            if not (flags & _SYN) or (flags & _ACK):
                return                             # only a fresh SYN opens one
            if self._at_capacity():
                self.log("nat: flow table full, dropping SYN to %s:%d"
                         % (netstack.ip_str(dst), dport))
                return
            flow = _TCPFlow(self, src, sport, dst, dport)
            self._tcp[key] = flow
            if self.verbose:
                self.log("nat tcp: open %s" % _flowname(flow))
            flow.on_syn(seq)
            return
        flow.on_segment(seq, ack, flags, payload)

    def _open_stream(self, dip, dport):
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            s.setblocking(False)
            if self.bind_ip:
                s.bind((self.bind_ip, 0))
            try:
                s.connect((netstack.ip_str(dip), dport))
            except BlockingIOError:
                pass
            except OSError as e:
                if e.errno not in (errno.EINPROGRESS, errno.EWOULDBLOCK):
                    raise
            return s
        except OSError as e:
            self.log("nat tcp: cannot open socket to %s:%d: %s" % (
                netstack.ip_str(dip), dport, e))
            return None

    def _drop_tcp(self, flow):
        self._tcp.pop(flow.key, None)
        if flow.sock is not None:
            self._tcp_by_sock.pop(flow.sock, None)
            self._readers.discard(flow.sock)
            self._writers.discard(flow.sock)
            try:
                flow.sock.close()
            except OSError:
                pass
        if self.verbose:
            self.log("nat tcp: close %s" % _flowname(flow))

    # ---- UDP ------------------------------------------------------------
    def _on_udp(self, src, dst, body):
        if len(body) < 8:
            return
        sport, dport, length = struct.unpack_from(">HHH", body, 0)
        payload = body[8:]
        key = (src, sport, dst, dport)
        flow = self._udp.get(key)
        if flow is None:
            if self._at_capacity():
                return
            try:
                s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                s.setblocking(False)
                if self.bind_ip:
                    s.bind((self.bind_ip, 0))
                s.connect((netstack.ip_str(dst), dport))
            except OSError as e:
                self.log("nat udp: cannot open socket to %s:%d: %s" % (
                    netstack.ip_str(dst), dport, e))
                return
            flow = _UDPFlow(src, sport, dst, dport, s)
            self._udp[key] = flow
            self._udp_by_sock[s] = flow
            self._readers.add(s)
            if self.verbose:
                self.log("nat udp: open %s" % _flowname(flow))
        flow.last = _now()
        try:
            flow.sock.send(payload)
        except OSError as e:
            self.log("nat udp: send to %s:%d failed: %s" % (
                netstack.ip_str(dst), dport, e))
            self._drop_udp(flow)

    def _udp_reply(self, flow):
        try:
            data = flow.sock.recv(65535)
        except (BlockingIOError, InterruptedError):
            return
        except OSError:
            self._drop_udp(flow)
            return
        flow.last = _now()
        # Back to the station as if from dip:dport.
        self.stack.send_udp(flow.sip, flow.sport, data,
                            src_port=flow.dport, src_ip=flow.dip)

    def _drop_udp(self, flow):
        self._udp.pop(flow.key, None)
        self._udp_by_sock.pop(flow.sock, None)
        self._readers.discard(flow.sock)
        try:
            flow.sock.close()
        except OSError:
            pass

    # ---- ICMP echo ------------------------------------------------------
    def _on_icmp(self, src, dst, body):
        if not self._icmp_ok or len(body) < 8 or body[0] != 8:
            return                                 # only forward echo request
        ident = (body[4] << 8) | body[5]
        key = (src, ident, dst)
        flow = self._icmp.get(key)
        if flow is None:
            if self._at_capacity():
                return
            s = self._open_icmp()
            if s is None:
                return
            try:
                s.connect((netstack.ip_str(dst), 0))
            except OSError as e:
                self.log("nat icmp: connect to %s failed: %s"
                         % (netstack.ip_str(dst), e))
                s.close()
                return
            flow = _ICMPFlow(src, ident, dst, s)
            self._icmp[key] = flow
            self._icmp_by_sock[s] = flow
            self._readers.add(s)
            if self.verbose:
                self.log("nat icmp: open echo id=%d -> %s"
                         % (ident, netstack.ip_str(dst)))
        flow.last = _now()
        # A ping datagram socket rewrites the id to its own port and fixes
        # the checksum; we just hand it the ICMP message.
        try:
            flow.sock.send(bytes(body))
        except OSError as e:
            self.log("nat icmp: send failed: %s" % e)
            self._drop_icmp(flow)

    def _open_icmp(self):
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, IPPROTO_ICMP)
            s.setblocking(False)
            if self.bind_ip:
                s.bind((self.bind_ip, 0))
            return s
        except OSError:
            # No unprivileged ping and no raw-socket privilege: give up on
            # ICMP forwarding for good, but say so once and keep TCP/UDP.
            if self._icmp_ok:
                self.log("nat icmp: forwarding unavailable on this host "
                         "(needs a ping socket or root) — disabling it")
            self._icmp_ok = False
            return None

    def _icmp_reply(self, flow):
        try:
            data = flow.sock.recv(65535)
        except (BlockingIOError, InterruptedError):
            return
        except OSError:
            self._drop_icmp(flow)
            return
        flow.last = _now()
        # The kernel hands us the ICMP echo reply (type 0); restore the
        # station's original identifier and re-checksum before delivering.
        reply = bytearray(data)
        if len(reply) >= 8 and reply[0] == 0:
            reply[4] = flow.ident >> 8
            reply[5] = flow.ident & 0xFF
            reply[2] = reply[3] = 0
            c = netstack._cksum(bytes(reply))
            reply[2] = c >> 8
            reply[3] = c & 0xFF
        self.stack.send_ip(flow.sip, IPPROTO_ICMP, bytes(reply),
                           src_ip=flow.dip)

    def _drop_icmp(self, flow):
        self._icmp.pop(flow.key, None)
        self._icmp_by_sock.pop(flow.sock, None)
        self._readers.discard(flow.sock)
        try:
            flow.sock.close()
        except OSError:
            pass


def _flowname(flow):
    if isinstance(flow, _UDPFlow):
        proto = "udp"
    else:
        proto = "tcp"
    return "%s %s:%d -> %s:%d" % (
        proto, netstack.ip_str(flow.sip), flow.sport,
        netstack.ip_str(flow.dip), flow.dport)


# Date.now()/os.urandom wrappers kept tiny so tests can monkeypatch if ever
# needed; time is a host concern here (the NAT lives on the real machine).
def _now():
    return time.time()


def _rand4():
    import os
    return os.urandom(4)
