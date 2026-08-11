#
# vwifi-pseudohost — an in-process medium hub for tests
#
# SPDX-License-Identifier: GPL-2.0-or-later
#
# A pure-Python stand-in for vwifi-medium: it accepts peers on a Unix
# socket, absorbs their hello, and forwards each frame to the other peers
# on a matching channel — channel 0 (unknown/broadcast) matches anything,
# exactly like the real hub's filter.  It exists so a PseudoAP and a
# PseudoHost can associate to each other end to end without the C hub or
# QEMU.  It is a test helper, not the product (the product is the real
# vwifi-medium).
#
import os
import selectors
import socket
import struct
import tempfile
import threading

import sys
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from vwifi_pseudohost import medium as med       # noqa: E402


class _Peer:
    def __init__(self, sock):
        self.sock = sock
        self.buf = bytearray()
        self.channel = 0                # 0 = unknown/broadcast


def _match(a, b):
    return a == 0 or b == 0 or a == b


class PyHub(threading.Thread):
    def __init__(self):
        super().__init__(daemon=True)
        d = tempfile.mkdtemp(prefix="vwifi-pyhub-")
        self.sock_path = os.path.join(d, "hub.sock")
        self._srv = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        self._srv.bind(self.sock_path)
        self._srv.listen(8)
        self._srv.setblocking(False)
        self._sel = selectors.DefaultSelector()
        self._sel.register(self._srv, selectors.EVENT_READ, None)
        self._peers = {}
        self._stop = threading.Event()

    def stop(self):
        self._stop.set()

    def run(self):
        try:
            while not self._stop.is_set():
                for key, _mask in self._sel.select(timeout=0.1):
                    if key.data is None:
                        self._accept()
                    else:
                        self._readable(key.data)
        finally:
            self._sel.close()
            self._srv.close()
            for p in list(self._peers.values()):
                p.sock.close()

    def _accept(self):
        try:
            conn, _ = self._srv.accept()
        except OSError:
            return
        conn.setblocking(False)
        peer = _Peer(conn)
        self._peers[conn.fileno()] = peer
        self._sel.register(conn, selectors.EVENT_READ, peer)

    def _readable(self, peer):
        try:
            data = peer.sock.recv(65536)
        except OSError:
            data = b""
        if not data:
            self._drop(peer)
            return
        peer.buf += data
        self._drain(peer)

    def _drop(self, peer):
        try:
            self._sel.unregister(peer.sock)
        except KeyError:
            pass
        self._peers.pop(peer.sock.fileno(), None)
        peer.sock.close()

    def _drain(self, peer):
        buf = peer.buf
        while len(buf) >= 4:
            (mlen,) = struct.unpack_from("!I", buf, 0)
            if len(buf) < 4 + mlen:
                break
            msg = bytes(buf[4 : 4 + mlen])
            del buf[: 4 + mlen]
            self._dispatch(peer, msg)

    def _dispatch(self, peer, msg):
        if len(msg) < 4:
            return
        (magic,) = struct.unpack_from("<I", msg, 0)
        if magic == med.HELLO_MAGIC:
            return                              # absorb registration
        if magic != med.VWIFI_MAGIC:
            return
        # Learn the sender's channel from the frame header.
        version = struct.unpack_from("<H", msg, 4)[0]
        if version >= 2 and len(msg) >= med.HDR_SIZE:
            peer.channel = struct.unpack_from("<H", msg, 28)[0]
        wire = struct.pack("!I", len(msg)) + msg
        for other in list(self._peers.values()):
            if other is peer:
                continue
            if _match(peer.channel, other.channel):
                try:
                    other.sock.sendall(wire)
                except OSError:
                    self._drop(other)
