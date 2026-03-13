#!/usr/bin/env python3
"""
medium_dump.py — Tap into the ath9k virtual medium hub and dump frames.

Connects to the hub as a regular client and prints hex dumps of all
frames seen on the medium. Optionally writes a pcap file with proper
radiotap headers for analysis in Wireshark.

Usage:
    python3 medium_dump.py /tmp/medium_hub_1.sock
    python3 medium_dump.py /tmp/medium_hub_1.sock -v
    python3 medium_dump.py /tmp/medium_hub_1.sock -w capture.pcap
    python3 medium_dump.py /tmp/medium_hub_1.sock -w capture.pcap -v

Notes on pcap output:
    The pcap file uses link type 127 (LINKTYPE_IEEE802_11_RADIOTAP).
    Each packet is prefixed with a radiotap header containing:
      - TSFT (timestamp from the medium header TSF fields)
      - Rate (from the medium header rate_code)
      - Antenna signal (dBm, from the medium header rssi)
      - Channel frequency and flags (from v2 medium header, or defaults)
    This allows Wireshark to display channel info, signal strength,
    data rate, and timestamps alongside the decoded 802.11 frames.
"""

import socket
import struct
import sys
import time
import os

MAGIC = 0x41394B57  # "A9KW"
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
RADIOTAP_RATE          = (1 << 2)
RADIOTAP_CHANNEL       = (1 << 3)
RADIOTAP_DBM_ANTSIGNAL = (1 << 5)
RADIOTAP_ANTENNA       = (1 << 11)

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
# These are the ath9k OFDM/CCK rate codes
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


def rate_is_ofdm(rate_code):
    """Check if a rate code is OFDM (vs CCK)."""
    return rate_code in (0x0B, 0x0F, 0x0A, 0x0E, 0x09, 0x0D, 0x08, 0x0C)


def build_radiotap_header(tsf_lo, tsf_hi, rate_code, rssi_dbm,
                          channel_freq=0, channel_flags_medium=0):
    """
    Build a radiotap header for one captured frame.

    Returns bytes of the complete radiotap header.

    Layout (must respect radiotap alignment rules):
      offset 0:  u8  version (0)
      offset 1:  u8  pad (0)
      offset 2:  u16 length (total header length, LE)
      offset 4:  u32 present flags (LE)
      -- present fields follow, in order of bit position --
      offset 8:  u64 TSFT (bit 0) — 8-byte aligned
      offset 16: u8  rate (bit 2) — 500kbps units
      offset 17: u8  pad for channel alignment
      offset 18: u16 channel freq (bit 3)
      offset 20: u16 channel flags (bit 3)
      offset 22: s8  antenna signal dBm (bit 5)
      offset 23: u8  antenna index (bit 11)
      Total: 24 bytes
    """
    present = (RADIOTAP_TSFT | RADIOTAP_RATE |
               RADIOTAP_CHANNEL | RADIOTAP_DBM_ANTSIGNAL |
               RADIOTAP_ANTENNA)

    # TSF as 64-bit microsecond counter
    tsf = (tsf_hi << 32) | tsf_lo

    # Rate in 500kbps units
    rate_500k = RATE_CODE_TO_500KBPS.get(rate_code, 12)  # default 6 Mbps

    # Channel frequency and flags
    if channel_freq == 0:
        channel_freq = 2412  # default channel 1
    chan_flags = RADIOTAP_CHAN_2GHZ if channel_freq < 5000 else RADIOTAP_CHAN_5GHZ
    if rate_is_ofdm(rate_code):
        chan_flags |= RADIOTAP_CHAN_OFDM
    else:
        chan_flags |= RADIOTAP_CHAN_CCK

    # Clamp RSSI
    rssi = max(-128, min(127, rssi_dbm))

    # Pack: version(1) + pad(1) + len(2) + present(4) +
    #        tsft(8) + rate(1) + pad(1) + chan_freq(2) + chan_flags(2) +
    #        signal(1) + antenna(1) = 24 bytes
    hdr = struct.pack('<BBhI',
                      RADIOTAP_HEADER_REVISION,  # version
                      0,                          # pad
                      24,                         # length
                      present)                    # present flags
    hdr += struct.pack('<Q', tsf)                 # TSFT (8 bytes, aligned)
    hdr += struct.pack('<B', rate_500k)           # rate
    hdr += struct.pack('<x')                      # pad for channel alignment
    hdr += struct.pack('<Hh', channel_freq, chan_flags)  # channel
    hdr += struct.pack('<bB', rssi, 0)            # signal + antenna

    assert len(hdr) == 24
    return hdr


class PcapWriter:
    """Write pcap file with radiotap link type."""

    def __init__(self, filename):
        self.f = open(filename, 'wb')
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
        self.f.close()


def main():
    if len(sys.argv) < 2:
        print(f"Usage: {sys.argv[0]} <socket_path> [options]")
        print()
        print("Options:")
        print("  -v              Verbose (hex dump of each frame)")
        print("  -w <file.pcap>  Write captured frames to pcap file")
        print("                  (with radiotap headers for Wireshark)")
        print()
        print("Examples:")
        print(f"  {sys.argv[0]} /tmp/ath9k.sock")
        print(f"  {sys.argv[0]} /tmp/ath9k.sock -v")
        print(f"  {sys.argv[0]} /tmp/ath9k.sock -w capture.pcap")
        print(f"  {sys.argv[0]} /tmp/ath9k.sock -w capture.pcap -v")
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

    sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    sock.connect(sock_path)
    print(f"Connected to {sock_path}")
    if pcap_file:
        pcap_writer = PcapWriter(pcap_file)
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
                if msg_len > 8192 + 64:
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

                frame = msg[hdr_size:]
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
