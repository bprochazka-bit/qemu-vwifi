#!/usr/bin/env python3
"""
vwifi_dump.py — attach to the vwifi medium hub and capture every frame.

Connects to the hub as an ordinary peer, prints a decoded one-line
summary of each frame, and optionally writes a pcap with radiotap
headers for Wireshark.

Usage:
    python3 vwifi_dump.py /tmp/vwifi.sock
    python3 vwifi_dump.py /tmp/vwifi.sock -v
    python3 vwifi_dump.py /tmp/vwifi.sock -w capture.pcap
    python3 vwifi_dump.py /tmp/vwifi.sock -w - | wireshark -k -i -

Why this peer stays anonymous
-----------------------------
It never sends a hello and never transmits, and both matter:

  * The hub learns a peer's channel from the frames that peer sends.
    Having sent none, this one's channel stays 0, which the hub's
    channel filter treats as "matches anything" -- so the capture sees
    every channel at once rather than whichever one it last claimed.

  * A hello binds a peer to a node, and a node is subject to the
    propagation model. A tap that silently loses frames because the
    simulation decided they were too weak is worse than no tap at all.

The cost is that the hub logs it as an unnamed `local-qemu` peer. That
is the right trade: this is a wiretap on the medium, not a radio in it.

pcap output
-----------
Link type 127 (LINKTYPE_IEEE802_11_RADIOTAP). Each frame gets a
radiotap header carrying the TSF, the channel, the signal level, and
the rate -- as a legacy rate when the medium's rate code is one, and as
a radiotap MCS field when it is an HT code. An unrecognised code emits
neither rather than a plausible-looking 6 Mbps, because a wrong rate in
a capture is harder to catch than a missing one.

The frames carry NO FCS (see abi/vwifi.h), so no FCS flag is set and
Wireshark is told not to expect four trailing bytes.
"""

import socket
import struct
import sys
import time
import os

MAGIC = 0x46495756  # "VWIF"
HDR_SIZE_V1 = 28
HDR_SIZE_V2 = 40

# 802.11 frame type/subtype decoding
FRAME_TYPES = {
    0: "Mgmt",
    1: "Ctrl",
    2: "Data",
    3: "Ext",
}

MGMT_SUBTYPES = {
    0: "AssocReq", 1: "AssocResp", 2: "ReassocReq", 3: "ReassocResp",
    4: "ProbeReq", 5: "ProbeResp", 6: "TimingAdv", 7: "Reserved",
    8: "Beacon", 9: "ATIM", 10: "Disassoc", 11: "Auth",
    12: "Deauth", 13: "Action", 14: "ActionNoAck", 15: "Reserved",
}

DATA_SUBTYPES = {
    0: "Data", 1: "Data+CF-Ack", 2: "Data+CF-Poll", 3: "Data+CF-Ack+CF-Poll",
    4: "Null", 5: "CF-Ack", 6: "CF-Poll", 7: "CF-Ack+CF-Poll",
    8: "QoS-Data", 9: "QoS-Data+CF-Ack", 10: "QoS-Data+CF-Poll",
    11: "QoS-Data+CF-Ack+CF-Poll", 12: "QoS-Null", 13: "Reserved",
    14: "QoS-CF-Poll", 15: "QoS-CF-Ack+CF-Poll",
}

# Radiotap constants
RADIOTAP_HEADER_REVISION = 0

# Radiotap present flags (bitmask)
RADIOTAP_TSFT          = (1 << 0)
RADIOTAP_FLAGS         = (1 << 1)
RADIOTAP_RATE          = (1 << 2)
RADIOTAP_CHANNEL       = (1 << 3)
RADIOTAP_DBM_ANTSIGNAL = (1 << 5)
RADIOTAP_ANTENNA       = (1 << 11)
RADIOTAP_MCS           = (1 << 19)

# Radiotap channel flags
RADIOTAP_CHAN_TURBO  = 0x0010
RADIOTAP_CHAN_CCK    = 0x0020
RADIOTAP_CHAN_OFDM   = 0x0040
RADIOTAP_CHAN_2GHZ   = 0x0080
RADIOTAP_CHAN_5GHZ   = 0x0100
RADIOTAP_CHAN_PASSIVE = 0x0200
RADIOTAP_CHAN_DYN    = 0x0400

# PCAP constants
PCAP_MAGIC       = 0xA1B2C3D4
PCAP_VERSION_MAJ = 2
PCAP_VERSION_MIN = 4
PCAP_SNAPLEN     = 65535
PCAP_LINKTYPE_RADIOTAP = 127

# Rate code to Mbps*2 mapping (for radiotap rate field, in 500kbps units)
# These are the legacy OFDM/CCK rate codes
RATE_CODE_TO_500KBPS = {
    # OFDM rates
    0x0B: 12,   # 6 Mbps
    0x0F: 18,   # 9 Mbps
    0x0A: 24,   # 12 Mbps
    0x0E: 36,   # 18 Mbps
    0x09: 48,   # 24 Mbps
    0x0D: 72,   # 36 Mbps
    0x08: 96,   # 48 Mbps
    0x0C: 108,  # 54 Mbps
    # CCK rates
    0x1B: 2,    # 1 Mbps
    0x1A: 4,    # 2 Mbps
    0x19: 11,   # 5.5 Mbps
    0x18: 22,   # 11 Mbps
}


def decode_frame_type(fc):
    ftype = (fc >> 2) & 0x3
    subtype = (fc >> 4) & 0xF
    type_str = FRAME_TYPES.get(ftype, "?")
    if ftype == 0:
        sub_str = MGMT_SUBTYPES.get(subtype, "?")
    elif ftype == 2:
        sub_str = DATA_SUBTYPES.get(subtype, "?")
    else:
        sub_str = f"sub={subtype}"
    return f"{type_str}/{sub_str}"


def mac_str(data, offset):
    return ":".join(f"{data[offset+i]:02x}" for i in range(6))


def hexdump(data, max_bytes=64):
    lines = []
    for i in range(0, min(len(data), max_bytes), 16):
        chunk = data[i:i+16]
        hex_part = " ".join(f"{b:02x}" for b in chunk)
        ascii_part = "".join(chr(b) if 32 <= b < 127 else "." for b in chunk)
        lines.append(f"  {i:04x}: {hex_part:<48s} {ascii_part}")
    if len(data) > max_bytes:
        lines.append(f"  ... ({len(data) - max_bytes} more bytes)")
    return "\n".join(lines)


def freq_to_channel(freq):
    """Convert frequency in MHz to 802.11 channel number."""
    if freq == 0:
        return 0
    if 2412 <= freq <= 2472:
        return (freq - 2407) // 5
    if freq == 2484:
        return 14
    if 5180 <= freq <= 5825:
        return (freq - 5000) // 5
    return 0


# CCK rate codes, needed for the radiotap channel flags.
CCK_RATE_CODES = (0x1B, 0x1A, 0x19, 0x18)


def decode_rate(code):
    """
    Split a medium rate code into what radiotap can express.

    Mirrors vwifi_decode_rate() in the Linux driver's monitor path --
    the two read the same field off the same wire and disagreeing about
    it would make a guest capture and a hub capture of one frame report
    different rates.

    Returns a dict; legacy_500kbps is 0 and mcs is None when the code
    has no representation, which is the caller's cue to omit the field.
    """
    out = {"legacy_500kbps": 0, "cck": False, "mcs": None, "ht40": False}

    if code < 0x80:
        out["legacy_500kbps"] = RATE_CODE_TO_500KBPS.get(code, 0)
        out["cck"] = code in CCK_RATE_CODES
        return out

    if code < 0xA0:
        # HT: bit 4 selects HT40, bit 3 selects NSS 2, which in
        # radiotap's MCS numbering is simply MCS + 8.
        out["mcs"] = (code & 0x07) + (8 if code & 0x08 else 0)
        out["ht40"] = bool(code & 0x10)
        return out

    # VHT/HE. radiotap has fields for these, but the medium's rate codes
    # do not carry enough to fill them honestly (no GI, no coding, and
    # NSS only by inference), so report nothing rather than a guess.
    return out


def build_radiotap_header(tsf_lo, tsf_hi, rate_code, rssi_dbm,
                          channel_freq=0, channel_flags_medium=0):
    """
    Build a radiotap header for one captured frame.

    The field set is not fixed: radiotap requires present fields to
    appear in bit order and at their natural alignment, so the header is
    assembled incrementally. Fields we cannot fill honestly are left
    out -- a rate code the medium uses for an HT MCS has no legacy
    equivalent, and emitting "6 Mbps" for it would put a number in
    Wireshark that no one would think to doubt.

    Layout, when everything is present:
      0   u8  version          8   u64 TSFT      (bit 0, 8-aligned)
      1   u8  pad             16   u8  flags     (bit 1)
      2   u16 length          17   u8  rate      (bit 2)
      4   u32 present         18   u16 chan freq (bit 3, 2-aligned)
                              20   u16 chan flags
                              22   s8  signal    (bit 5)
                              23   u8  antenna   (bit 11)
                              24   u8  mcs known (bit 19, 1-aligned)
                              25   u8  mcs flags
                              26   u8  mcs index
    """
    ri = decode_rate(rate_code)

    present = RADIOTAP_TSFT | RADIOTAP_FLAGS | RADIOTAP_CHANNEL
    present |= RADIOTAP_DBM_ANTSIGNAL | RADIOTAP_ANTENNA
    if ri["legacy_500kbps"]:
        present |= RADIOTAP_RATE
    if ri["mcs"] is not None:
        present |= RADIOTAP_MCS

    body = b""

    # TSFT — 8-byte aligned, and it is the first field, so offset 8.
    body += struct.pack("<Q", (tsf_hi << 32) | tsf_lo)

    # Flags. Deliberately NOT setting RADIOTAP_F_FCS: frames on this
    # medium carry no FCS, and claiming one points Wireshark at four
    # bytes of payload and makes it report a checksum error.
    body += struct.pack("<B", 0)

    if ri["legacy_500kbps"]:
        body += struct.pack("<B", ri["legacy_500kbps"])

    # Channel — 2-byte aligned.
    if len(body) % 2:
        body += b"\x00"
    freq = channel_freq or 2412
    chan_flags = RADIOTAP_CHAN_2GHZ if freq < 5000 else RADIOTAP_CHAN_5GHZ
    chan_flags |= RADIOTAP_CHAN_CCK if ri["cck"] else RADIOTAP_CHAN_OFDM
    body += struct.pack("<HH", freq, chan_flags)

    body += struct.pack("<b", max(-128, min(127, rssi_dbm)))   # signal
    body += struct.pack("<B", 0)                               # antenna

    if ri["mcs"] is not None:
        # known: bandwidth (0x01) + MCS index (0x02)
        mcs_flags = 0x01 if ri["ht40"] else 0x00     # 0 = 20 MHz, 1 = 40
        body += struct.pack("<BBB", 0x03, mcs_flags, ri["mcs"])

    hdr = struct.pack("<BBHI", RADIOTAP_HEADER_REVISION, 0,
                      8 + len(body), present)
    return hdr + body


class PcapWriter:
    """Write a pcap stream with the radiotap link type.

    Every record is flushed. The point of this tool is watching a live
    medium, and a capture that only becomes readable when the process
    exits cannot be piped into Wireshark or tailed while a bug is
    happening.
    """

    def __init__(self, filename, stream=None):
        if stream is not None:
            self.f = stream
            self.own_fd = False
        else:
            self.f = open(filename, 'wb')
            self.own_fd = True
        # Write global header
        self.f.write(struct.pack('<IHHiIII',
                                 PCAP_MAGIC,
                                 PCAP_VERSION_MAJ,
                                 PCAP_VERSION_MIN,
                                 0,              # thiszone
                                 0,              # sigfigs
                                 PCAP_SNAPLEN,
                                 PCAP_LINKTYPE_RADIOTAP))
        self.f.flush()

    def write_packet(self, radiotap_hdr, frame_data, ts_sec=None, ts_usec=None):
        """Write one packet record: radiotap header + 802.11 frame."""
        if ts_sec is None:
            now = time.time()
            ts_sec = int(now)
            ts_usec = int((now - ts_sec) * 1_000_000)
        pkt = radiotap_hdr + frame_data
        # Packet header: ts_sec(4) + ts_usec(4) + incl_len(4) + orig_len(4)
        self.f.write(struct.pack('<IIII',
                                 ts_sec, ts_usec,
                                 len(pkt), len(pkt)))
        self.f.write(pkt)
        self.f.flush()

    def close(self):
        self.f.flush()
        if self.own_fd:
            self.f.close()


def main():
    if len(sys.argv) < 2:
        print(f"Usage: {sys.argv[0]} <socket_path> [options]")
        print()
        print("Options:")
        print("  -v              Verbose (hex dump of each frame)")
        print("  -w <file.pcap>  Write captured frames to pcap file")
        print("                  (with radiotap headers for Wireshark)")
        print("  -w -            Write the pcap to stdout, for piping:")
        print("                    ... -w - | wireshark -k -i -")
        print()
        print("Examples:")
        print(f"  {sys.argv[0]} /tmp/vwifi.sock")
        print(f"  {sys.argv[0]} /tmp/vwifi.sock -v")
        print(f"  {sys.argv[0]} /tmp/vwifi.sock -w capture.pcap")
        print(f"  {sys.argv[0]} /tmp/vwifi.sock -w capture.pcap -v")
        sys.exit(1)

    sock_path = sys.argv[1]
    verbose = "-v" in sys.argv
    pcap_file = None
    pcap_writer = None

    # Parse -w argument
    for i, arg in enumerate(sys.argv):
        if arg == "-w" and i + 1 < len(sys.argv):
            pcap_file = sys.argv[i + 1]
            break

    # With the pcap on stdout, every human-readable line has to go to
    # stderr instead or it lands in the middle of the capture file.
    # Grab the binary stream first: after the redirect, sys.stdout is
    # stderr and its buffer is the wrong file.
    pcap_stream = None
    if pcap_file == "-":
        pcap_stream = sys.stdout.buffer
        sys.stdout = sys.stderr

    sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    sock.connect(sock_path)
    print(f"Connected to {sock_path}")
    if pcap_file:
        pcap_writer = PcapWriter(pcap_file, pcap_stream)
        print(f"Writing pcap to {pcap_file} (radiotap + 802.11)")
    print(f"Listening for frames... (Ctrl+C to stop)\n")

    buf = b""
    frame_count = 0

    try:
        while True:
            data = sock.recv(4096)
            if not data:
                print("Connection closed")
                break
            buf += data

            while len(buf) >= 4:
                msg_len = struct.unpack("!I", buf[:4])[0]
                # VWIFI_MAX_MSG_SIZE from abi/vwifi.h: header + frame.
                if msg_len > 40 + 8192:
                    print(f"ERROR: absurd msg_len {msg_len}, resetting")
                    buf = b""
                    break
                if len(buf) < 4 + msg_len:
                    break  # need more data

                msg = buf[4:4+msg_len]
                buf = buf[4+msg_len:]

                if len(msg) < HDR_SIZE_V1:
                    print(f"WARNING: short message ({len(msg)} bytes)")
                    continue

                # Parse medium header (native little-endian from QEMU)
                magic = struct.unpack("<I", msg[0:4])[0]
                version = struct.unpack("<H", msg[4:6])[0]
                frame_len = struct.unpack("<H", msg[6:8])[0]
                tx_mac = msg[8:14]
                rate_code = msg[14]
                rssi = struct.unpack("b", bytes([msg[15]]))[0]
                tsf_lo = struct.unpack("<I", msg[16:20])[0]
                tsf_hi = struct.unpack("<I", msg[20:24])[0]
                flags = struct.unpack("<I", msg[24:28])[0]

                # Parse v2 channel fields if present
                channel_freq = 0
                channel_flags = 0
                channel_bond_freq = 0
                if version >= 2 and len(msg) >= HDR_SIZE_V2:
                    channel_freq = struct.unpack("<H", msg[28:30])[0]
                    channel_flags = struct.unpack("<H", msg[30:32])[0]
                    channel_bond_freq = struct.unpack("<H", msg[32:34])[0]

                # Determine actual header size used
                if version >= 2 and len(msg) >= HDR_SIZE_V2:
                    hdr_size = HDR_SIZE_V2
                else:
                    hdr_size = HDR_SIZE_V1

                # Trust frame_len over "whatever is left": a peer that
                # pads its message would otherwise get the padding
                # written into the capture as part of the frame.
                frame = msg[hdr_size:]
                if frame_len <= len(frame):
                    frame = frame[:frame_len]
                frame_count += 1

                tx_mac_str = ":".join(f"{b:02x}" for b in tx_mac)

                # Decode 802.11 frame type
                frame_info = ""
                if len(frame) >= 24:
                    fc = frame[0] | (frame[1] << 8)
                    frame_info = decode_frame_type(fc)
                    da = mac_str(frame, 4)
                    sa = mac_str(frame, 10)
                    frame_info += f" DA={da} SA={sa}"
                elif len(frame) >= 2:
                    fc = frame[0] | (frame[1] << 8)
                    frame_info = decode_frame_type(fc)

                # Channel info string
                chan_str = ""
                if channel_freq != 0:
                    ch_num = freq_to_channel(channel_freq)
                    chan_str = f" ch={ch_num}({channel_freq}MHz)"
                    if channel_bond_freq != 0:
                        chan_str += f"+{channel_bond_freq}MHz"

                ts = time.strftime("%H:%M:%S")
                print(f"[{ts}] #{frame_count} from={tx_mac_str} "
                      f"len={frame_len} rate=0x{rate_code:02x} "
                      f"rssi={rssi}dBm{chan_str}")
                print(f"  Type: {frame_info}")

                if verbose:
                    print(hexdump(frame))
                print()

                # Write to pcap if requested
                if pcap_writer:
                    radiotap = build_radiotap_header(
                        tsf_lo, tsf_hi, rate_code, rssi,
                        channel_freq, channel_flags)
                    pcap_writer.write_packet(radiotap, frame)

    except KeyboardInterrupt:
        print(f"\n{frame_count} frames captured")
    finally:
        sock.close()
        if pcap_writer:
            pcap_writer.close()
            print(f"Pcap written to {pcap_file}")


if __name__ == "__main__":
    main()
