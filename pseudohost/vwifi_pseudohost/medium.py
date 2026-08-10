#
# vwifi-pseudohost — medium (hub) transport
#
# SPDX-License-Identifier: GPL-2.0-or-later
#
# The wire side of a pseudo-host: a Unix-domain stream socket to the
# vwifi-medium hub, speaking the v2 length-prefixed protocol from
# abi/vwifi.h.  Everything endianness-sensitive is pinned here so the
# layers above deal only in 802.11 frames and channel numbers.
#
#   length prefix ......... network byte order  (struct '!I')
#   frame header .......... host/little-endian   (matches QEMU on x86)
#   hello magic ........... little-endian raw     (the hub compares the
#                           raw uint32 without ntohl — see abi/vwifi.h)
#
import socket
import struct

# Mirrors abi/vwifi.h.
VWIFI_MAGIC = 0x46495756          # "VWIF"
VWIFI_VERSION = 2
HELLO_MAGIC = 0x52495756          # "VWIR"
HELLO_FLAG_PHYSICAL = 0x01
HDR_SIZE = 40
HDR_SIZE_V1 = 28
MAX_FRAME = 8192
MAX_MSG = HDR_SIZE + MAX_FRAME

DEFAULT_RSSI = -30
DEFAULT_RATE = 0x0B               # 6 Mbps OFDM

CHAN_FLAG_2GHZ = 0x0001
CHAN_FLAG_5GHZ = 0x0002
CHAN_FLAG_HT20 = 0x0004

_HDR = struct.Struct("<IHH6sBbIIIHHHHH2s")


class RxFrame:
    """One frame received from the medium, with the header fields a
    station cares about."""

    __slots__ = ("tx_mac", "rate_code", "rssi", "channel_freq", "frame")

    def __init__(self, tx_mac, rate_code, rssi, channel_freq, frame):
        self.tx_mac = tx_mac
        self.rate_code = rate_code
        self.rssi = rssi
        self.channel_freq = channel_freq
        self.frame = frame


class MediumClient:
    """A peer connection to the vwifi hub.

    Blocking connect and hello; frame TX is a single synchronous write
    (the hub's stream is length-prefixed, so a partial write would
    corrupt it — same constraint the QEMU devices live under).  RX is
    incremental: feed whatever the socket returns into recv_frames()
    and it yields the complete frames buffered so far.
    """

    def __init__(self, sock_path, node_id="pseudohost", physical=False):
        self.sock_path = sock_path
        self.node_id = node_id
        self.physical = physical
        self.sock = None
        self._buf = bytearray()
        # The channel we stamp on outgoing frames.  0 = broadcast/unknown,
        # which the hub treats as "hear every channel" — the scan state.
        self.tx_channel_freq = 0
        self.tx_channel_flags = 0

    # -- lifecycle ---------------------------------------------------------
    def connect(self):
        self.sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        self.sock.connect(self.sock_path)
        return self

    def close(self):
        if self.sock:
            try:
                self.sock.close()
            finally:
                self.sock = None

    def fileno(self):
        return self.sock.fileno()

    # -- hello -------------------------------------------------------------
    def send_hello(self):
        nid = self.node_id.encode("utf-8")[:120] + b"\x00"
        magic = struct.pack("<I", HELLO_MAGIC)          # host byte order
        payload = magic + nid
        if self.physical:
            payload += bytes([HELLO_FLAG_PHYSICAL])
        self.sock.sendall(struct.pack("!I", len(payload)) + payload)

    # -- channel -----------------------------------------------------------
    def set_channel(self, freq, flags=None):
        """Pin the channel stamped on every outgoing frame.

        The hub learns a peer's channel from the frames it sends and then
        only forwards same-channel traffic to it, so this is also what
        selects which channel the station hears.
        """
        self.tx_channel_freq = freq
        if flags is not None:
            self.tx_channel_flags = flags
        elif freq:
            self.tx_channel_flags = (CHAN_FLAG_2GHZ if freq < 5000
                                     else CHAN_FLAG_5GHZ) | CHAN_FLAG_HT20
        else:
            self.tx_channel_flags = 0

    # -- transmit ----------------------------------------------------------
    def send_frame(self, frame, tx_mac, channel_freq=None, rate_code=None,
                   tsf_us=0):
        if len(frame) > MAX_FRAME:
            raise ValueError("frame too large for the medium")
        freq = self.tx_channel_freq if channel_freq is None else channel_freq
        flags = self.tx_channel_flags
        hdr = _HDR.pack(
            VWIFI_MAGIC, VWIFI_VERSION, len(frame), bytes(tx_mac),
            rate_code or DEFAULT_RATE, DEFAULT_RSSI,
            tsf_us & 0xFFFFFFFF, (tsf_us >> 32) & 0xFFFFFFFF, 0,
            freq, flags, 0, 0, 0, b"\x00\x00")
        msg = hdr + frame
        self.sock.sendall(struct.pack("!I", len(msg)) + msg)

    # -- receive -----------------------------------------------------------
    def feed(self, data):
        self._buf += data

    def recv_frames(self):
        """Read what's available on the socket and yield RxFrame objects.

        Returns after draining one recv() worth of data; call it whenever
        the socket is readable.  Raises ConnectionError if the hub closed.
        """
        data = self.sock.recv(65536)
        if not data:
            raise ConnectionError("hub closed the connection")
        self._buf += data
        yield from self._drain()

    def _drain(self):
        buf = self._buf
        while len(buf) >= 4:
            (mlen,) = struct.unpack_from("!I", buf, 0)
            if mlen > MAX_MSG or mlen < HDR_SIZE_V1:
                # A desynced stream can't be trusted; drop it and let the
                # caller reconnect rather than emit garbage frames.
                self._buf = bytearray()
                raise ConnectionError("framing error from hub (len=%d)" % mlen)
            if len(buf) < 4 + mlen:
                break
            msg = bytes(buf[4 : 4 + mlen])
            del buf[: 4 + mlen]
            rx = self._parse(msg)
            if rx is not None:
                yield rx

    @staticmethod
    def _parse(msg):
        if len(msg) < HDR_SIZE_V1:
            return None
        version = struct.unpack_from("<H", msg, 4)[0]
        frame_len = struct.unpack_from("<H", msg, 6)[0]
        tx_mac = bytes(msg[8:14])
        rate_code = msg[14]
        rssi = struct.unpack_from("b", msg, 15)[0]
        channel_freq = 0
        if version >= 2 and len(msg) >= HDR_SIZE:
            channel_freq = struct.unpack_from("<H", msg, 28)[0]
            hdr_size = HDR_SIZE
        else:
            hdr_size = HDR_SIZE_V1
        frame = msg[hdr_size:]
        if frame_len <= len(frame):
            frame = frame[:frame_len]
        return RxFrame(tx_mac, rate_code, rssi, channel_freq, frame)
