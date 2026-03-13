#!/usr/bin/env python3
"""
medium_test.py — Transport-layer test for the ath9k virtual wireless medium.

Connects to the hub as a fake "device" and tests frame delivery,
integrity, ordering, and buffer stress — without involving any
ath9k driver or QEMU at all.

Modes:
  sender   — Sends test frames to the medium
  receiver — Listens for test frames and validates them
  bidir    — Two connections: sends AND receives simultaneously
  pair     — Launches sender+receiver in threads for a self-test
  stress   — Rapid-fire frames of varying sizes

Usage:
  # Terminal 1 (or on VM-B):
  python3 medium_test.py receiver /tmp/medium_hub_1.sock

  # Terminal 2 (or on VM-A):
  python3 medium_test.py sender /tmp/medium_hub_1.sock

  # Self-test with two connections:
  python3 medium_test.py pair /tmp/medium_hub_1.sock

  # Stress test:
  python3 medium_test.py stress /tmp/medium_hub_1.sock
"""

import socket
import struct
import sys
import os
import time
import threading
import hashlib
import random
import json

# ── Protocol constants (must match ath9k_medium.h) ──────────────────
MAGIC       = 0x41394B57   # "A9KW"
VERSION     = 1
HDR_SIZE    = 28           # sizeof(ath9k_medium_frame_hdr)
MAX_FRAME   = 8192
HDR_STRUCT  = '<IHH6sBbIII'  # little-endian to match QEMU native

# Test frame marker in the 802.11 payload
TEST_MAGIC  = b'MTEST'

# ── Wire helpers ─────────────────────────────────────────────────────

def make_mac(node_id):
    """Generate a fake MAC from a node ID byte."""
    return bytes([0xDE, 0xAD, 0x00, 0x00, 0x00, node_id])

def mac_str(mac_bytes):
    return ':'.join(f'{b:02x}' for b in mac_bytes)

def build_medium_msg(tx_mac, payload):
    """Build a complete wire message: [u32 len BE][header LE][payload]."""
    frame_len = len(payload)
    hdr = struct.pack(HDR_STRUCT,
        MAGIC,              # magic
        VERSION,            # version
        frame_len,          # frame_len
        tx_mac,             # tx_mac (6 bytes)
        0x0B,               # rate_code
        -30,                # rssi
        0,                  # tsf_lo
        0,                  # tsf_hi
        0,                  # flags
    )
    assert len(hdr) == HDR_SIZE, f"Header size mismatch: {len(hdr)} != {HDR_SIZE}"
    msg = hdr + payload
    return struct.pack('!I', len(msg)) + msg

def parse_medium_msg(msg):
    """Parse a medium message (without length prefix). Returns (hdr_dict, payload)."""
    if len(msg) < HDR_SIZE:
        return None, None
    fields = struct.unpack(HDR_STRUCT, msg[:HDR_SIZE])
    hdr = {
        'magic':     fields[0],
        'version':   fields[1],
        'frame_len': fields[2],
        'tx_mac':    fields[3],
        'rate_code': fields[4],
        'rssi':      fields[5],
        'tsf_lo':    fields[6],
        'tsf_hi':    fields[7],
        'flags':     fields[8],
    }
    payload = msg[HDR_SIZE:HDR_SIZE + hdr['frame_len']]
    return hdr, payload

def build_test_payload(seq, size, sender_id):
    """
    Build a test payload with:
      [5B magic][4B seq LE][1B sender_id][2B total_size][16B sha256 of rest]
      [remaining fill bytes = pattern based on seq]
    """
    # Minimum overhead: 5 + 4 + 1 + 2 + 16 = 28 bytes
    min_size = 28
    if size < min_size:
        size = min_size
    fill_len = size - min_size
    # Deterministic fill pattern based on seq
    pattern = bytes([(seq + i) & 0xFF for i in range(fill_len)])
    # Build without checksum first
    core = (TEST_MAGIC +
            struct.pack('<I', seq) +
            struct.pack('B', sender_id) +
            struct.pack('<H', size) +
            pattern)
    # Compute checksum over core
    cksum = hashlib.sha256(core).digest()[:16]
    return core[:min_size - 16] + cksum + pattern

def parse_test_payload(payload):
    """Parse and validate a test payload. Returns (seq, sender_id, size, valid)."""
    if len(payload) < 28:
        return None, None, None, False
    if payload[:5] != TEST_MAGIC:
        return None, None, None, False

    seq = struct.unpack('<I', payload[5:9])[0]
    sender_id = payload[9]
    size = struct.unpack('<H', payload[10:12])[0]
    cksum_rx = payload[12:28]
    pattern = payload[28:]

    # Rebuild core without checksum
    fill_len = size - 28
    expected_pattern = bytes([(seq + i) & 0xFF for i in range(fill_len)])
    core = (TEST_MAGIC +
            struct.pack('<I', seq) +
            struct.pack('B', sender_id) +
            struct.pack('<H', size) +
            expected_pattern)
    cksum_expected = hashlib.sha256(core).digest()[:16]

    valid = (cksum_rx == cksum_expected and
             len(pattern) == fill_len and
             pattern == expected_pattern)
    return seq, sender_id, size, valid

# ── Socket connection ────────────────────────────────────────────────

def connect_to_hub(sock_path):
    """Connect to the medium hub and return the socket."""
    sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    sock.connect(sock_path)
    return sock

def recv_messages(sock, timeout=5.0):
    """Generator that yields (hdr, payload) from the socket."""
    buf = b''
    sock.settimeout(timeout)
    while True:
        try:
            data = sock.recv(16384)
        except socket.timeout:
            return
        if not data:
            return
        buf += data
        while len(buf) >= 4:
            msg_len = struct.unpack('!I', buf[:4])[0]
            if msg_len > MAX_FRAME + HDR_SIZE + 100:
                print(f"  ERROR: absurd msg_len {msg_len}")
                buf = b''
                break
            if len(buf) < 4 + msg_len:
                break
            msg = buf[4:4 + msg_len]
            buf = buf[4 + msg_len:]
            hdr, payload = parse_medium_msg(msg)
            if hdr:
                yield hdr, payload

# ── Test modes ───────────────────────────────────────────────────────

def run_sender(sock_path, sender_id=1, count=100, sizes=None, delay=0.01,
               quiet=False):
    """Send test frames to the medium."""
    if sizes is None:
        sizes = [28, 64, 128, 256, 512, 1024, 1500, 2048, 4096]

    sock = connect_to_hub(sock_path)
    tx_mac = make_mac(sender_id)
    sent = 0
    errors = 0

    if not quiet:
        print(f"[SENDER {sender_id}] Connected as {mac_str(tx_mac)}")
        print(f"[SENDER {sender_id}] Sending {count} frames, sizes: {sizes}")

    for seq in range(count):
        size = sizes[seq % len(sizes)]
        payload = build_test_payload(seq, size, sender_id)
        wire_msg = build_medium_msg(tx_mac, payload)
        try:
            sock.sendall(wire_msg)
            sent += 1
        except (BrokenPipeError, OSError) as e:
            if not quiet:
                print(f"[SENDER {sender_id}] Send error at seq={seq}: {e}")
            errors += 1
            break
        if delay > 0:
            time.sleep(delay)

    if not quiet:
        print(f"[SENDER {sender_id}] Done: {sent} sent, {errors} errors")
    sock.close()
    return {'sent': sent, 'errors': errors, 'sender_id': sender_id}


def run_receiver(sock_path, sender_id=1, expect_count=100, timeout=10.0,
                 my_id=2, quiet=False):
    """Receive test frames and validate integrity."""
    sock = connect_to_hub(sock_path)
    rx_mac = make_mac(my_id)

    received = 0
    valid = 0
    corrupt = 0
    out_of_order = 0
    wrong_size = 0
    duplicates = 0
    seen_seqs = set()
    last_seq = -1
    size_stats = {}

    if not quiet:
        print(f"[RECEIVER {my_id}] Connected as {mac_str(rx_mac)}, "
              f"waiting for frames from sender {sender_id} "
              f"(timeout={timeout}s)")

    start = time.time()
    for hdr, payload in recv_messages(sock, timeout=timeout):
        # Skip our own frames (hub broadcasts to all including sender)
        if hdr['tx_mac'] == rx_mac:
            continue
        # Only look at test frames from the expected sender
        seq, sid, size, is_valid = parse_test_payload(payload)
        if seq is None:
            continue  # not a test frame (e.g. beacon from a VM)
        if sid != sender_id:
            continue

        received += 1
        if is_valid:
            valid += 1
        else:
            corrupt += 1
            if not quiet:
                print(f"  CORRUPT: seq={seq} size={size} "
                      f"payload_len={len(payload)}")

        if seq in seen_seqs:
            duplicates += 1
        seen_seqs.add(seq)

        if seq != last_seq + 1 and last_seq >= 0:
            out_of_order += 1
        last_seq = seq

        actual_len = len(payload)
        size_stats.setdefault(actual_len, {'count': 0, 'valid': 0})
        size_stats[actual_len]['count'] += 1
        if is_valid:
            size_stats[actual_len]['valid'] += 1

        if received >= expect_count:
            break

    elapsed = time.time() - start
    sock.close()

    results = {
        'received':     received,
        'valid':        valid,
        'corrupt':      corrupt,
        'out_of_order': out_of_order,
        'duplicates':   duplicates,
        'expected':     expect_count,
        'missing':      expect_count - len(seen_seqs),
        'elapsed':      round(elapsed, 3),
        'fps':          round(received / elapsed, 1) if elapsed > 0 else 0,
        'size_stats':   size_stats,
    }

    if not quiet:
        print(f"\n[RECEIVER {my_id}] Results:")
        print(f"  Received:     {received}/{expect_count}")
        print(f"  Valid:        {valid}")
        print(f"  Corrupt:      {corrupt}")
        print(f"  Out-of-order: {out_of_order}")
        print(f"  Duplicates:   {duplicates}")
        print(f"  Missing:      {results['missing']}")
        print(f"  Elapsed:      {elapsed:.3f}s")
        print(f"  Throughput:   {results['fps']} frames/sec")
        if size_stats:
            print(f"  Size breakdown:")
            for sz in sorted(size_stats):
                s = size_stats[sz]
                print(f"    {sz:5d} bytes: {s['count']:4d} rx, "
                      f"{s['valid']:4d} valid")
        overall = "PASS" if (valid == received == expect_count and
                             corrupt == 0) else "FAIL"
        print(f"\n  Overall: {overall}")

    return results


def run_pair(sock_path, count=200, delay=0.005):
    """Self-test: open two connections, send from one, receive on the other."""
    print("=" * 60)
    print("PAIR TEST: sender(id=1) -> hub -> receiver(id=2)")
    print(f"  Frames: {count}, delay: {delay}s")
    print("=" * 60)

    sizes = [28, 50, 64, 100, 128, 200, 256, 512, 1024, 1500, 2000, 2346, 4096]

    results = {}
    errors = []

    def do_recv():
        results['rx'] = run_receiver(sock_path, sender_id=1,
                                      expect_count=count,
                                      timeout=max(count * delay * 3, 10),
                                      my_id=2, quiet=False)

    def do_send():
        time.sleep(0.5)  # let receiver connect first
        results['tx'] = run_sender(sock_path, sender_id=1, count=count,
                                    sizes=sizes, delay=delay, quiet=False)

    t_rx = threading.Thread(target=do_recv)
    t_tx = threading.Thread(target=do_send)
    t_rx.start()
    t_tx.start()
    t_tx.join()
    t_rx.join()

    print("\n" + "=" * 60)
    tx = results.get('tx', {})
    rx = results.get('rx', {})
    sent = tx.get('sent', 0)
    received = rx.get('received', 0)
    valid = rx.get('valid', 0)
    corrupt = rx.get('corrupt', 0)
    missing = rx.get('missing', 0)

    ok = (sent == count and received == count and valid == count and
          corrupt == 0)
    print(f"PAIR RESULT: {'PASS' if ok else 'FAIL'}")
    print(f"  Sent={sent}  Received={received}  Valid={valid}  "
          f"Corrupt={corrupt}  Missing={missing}")
    print("=" * 60)
    return ok


def run_stress(sock_path, duration=10, max_size=4096):
    """
    Stress test: blast frames as fast as possible at varying sizes.
    Measures throughput, checks for corruption and dropped frames.
    """
    print("=" * 60)
    print(f"STRESS TEST: {duration}s, max_size={max_size}")
    print("=" * 60)

    # Random sizes including edge cases
    test_sizes = [28, 29, 30, 31, 32, 48, 63, 64, 65, 100, 127, 128, 129,
                  255, 256, 257, 500, 512, 1000, 1024, 1500, 2000, 2346,
                  3000, 4000]
    test_sizes = [s for s in test_sizes if s <= max_size]

    results = {}

    def do_recv():
        results['rx'] = run_receiver(
            sock_path, sender_id=1,
            expect_count=999999,  # run until timeout
            timeout=duration + 3,
            my_id=2, quiet=True
        )

    def do_send():
        time.sleep(0.5)
        sock = connect_to_hub(sock_path)
        tx_mac = make_mac(1)
        sent = 0
        errors = 0
        total_bytes = 0
        start = time.time()

        while time.time() - start < duration:
            size = random.choice(test_sizes)
            payload = build_test_payload(sent, size, 1)
            wire_msg = build_medium_msg(tx_mac, payload)
            try:
                sock.sendall(wire_msg)
                sent += 1
                total_bytes += len(payload)
            except (BrokenPipeError, OSError):
                errors += 1
                break

        elapsed = time.time() - start
        sock.close()
        results['tx'] = {
            'sent': sent, 'errors': errors,
            'elapsed': round(elapsed, 3),
            'total_bytes': total_bytes,
            'fps': round(sent / elapsed, 1) if elapsed > 0 else 0,
            'mbps': round(total_bytes * 8 / elapsed / 1e6, 2) if elapsed > 0 else 0,
        }

    t_rx = threading.Thread(target=do_recv)
    t_tx = threading.Thread(target=do_send)
    t_rx.start()
    t_tx.start()
    t_tx.join()
    t_rx.join()

    tx = results.get('tx', {})
    rx = results.get('rx', {})

    print(f"\n{'='*60}")
    print(f"STRESS RESULTS ({duration}s):")
    print(f"  TX: {tx.get('sent',0)} frames, "
          f"{tx.get('fps',0)} fps, "
          f"{tx.get('mbps',0)} Mbps, "
          f"{tx.get('errors',0)} errors")
    print(f"  RX: {rx.get('received',0)} frames, "
          f"{rx.get('valid',0)} valid, "
          f"{rx.get('corrupt',0)} corrupt, "
          f"{rx.get('missing', '?')} missing, "
          f"{rx.get('duplicates',0)} duplicates")

    loss = tx.get('sent', 0) - rx.get('received', 0)
    loss_pct = (loss / tx['sent'] * 100) if tx.get('sent', 0) > 0 else 0
    print(f"  Loss: {loss} frames ({loss_pct:.1f}%)")
    print(f"  Integrity: {'PASS' if rx.get('corrupt',1)==0 else 'FAIL'}")

    if rx.get('size_stats'):
        print(f"  Size breakdown:")
        for sz in sorted(rx['size_stats']):
            s = rx['size_stats'][sz]
            status = "OK" if s['valid'] == s['count'] else "CORRUPT"
            print(f"    {sz:5d}B: {s['count']:5d} rx, "
                  f"{s['valid']:5d} valid  [{status}]")

    ok = rx.get('corrupt', 1) == 0 and rx.get('received', 0) > 0
    print(f"\n  Overall: {'PASS' if ok else 'FAIL'}")
    print(f"{'='*60}")
    return ok


def run_bidir(sock_path, count=100, delay=0.01):
    """
    Bidirectional test: two connections, each sends AND receives.
    Validates that traffic flows correctly in both directions
    simultaneously.
    """
    print("=" * 60)
    print(f"BIDIRECTIONAL TEST: {count} frames each way")
    print("=" * 60)

    sizes = [28, 64, 128, 256, 512, 1024, 1500]
    results = {}

    def node_a():
        """Node A: sender_id=1, receives from sender_id=2"""
        # Send
        r_tx = run_sender(sock_path, sender_id=1, count=count,
                          sizes=sizes, delay=delay, quiet=True)
        results['a_tx'] = r_tx

    def node_a_rx():
        r_rx = run_receiver(sock_path, sender_id=2,
                            expect_count=count,
                            timeout=max(count * delay * 4, 10),
                            my_id=1, quiet=True)
        results['a_rx'] = r_rx

    def node_b():
        """Node B: sender_id=2, receives from sender_id=1"""
        r_tx = run_sender(sock_path, sender_id=2, count=count,
                          sizes=sizes, delay=delay, quiet=True)
        results['b_tx'] = r_tx

    def node_b_rx():
        r_rx = run_receiver(sock_path, sender_id=1,
                            expect_count=count,
                            timeout=max(count * delay * 4, 10),
                            my_id=2, quiet=True)
        results['b_rx'] = r_rx

    threads = [
        threading.Thread(target=node_a_rx),
        threading.Thread(target=node_b_rx),
    ]
    for t in threads:
        t.start()
    time.sleep(0.5)  # let receivers connect

    senders = [
        threading.Thread(target=node_a),
        threading.Thread(target=node_b),
    ]
    for t in senders:
        t.start()
    for t in senders + threads:
        t.join()

    print(f"\nA→B: sent={results.get('a_tx',{}).get('sent',0)}  "
          f"received={results.get('b_rx',{}).get('received',0)}  "
          f"valid={results.get('b_rx',{}).get('valid',0)}  "
          f"corrupt={results.get('b_rx',{}).get('corrupt',0)}")
    print(f"B→A: sent={results.get('b_tx',{}).get('sent',0)}  "
          f"received={results.get('a_rx',{}).get('received',0)}  "
          f"valid={results.get('a_rx',{}).get('valid',0)}  "
          f"corrupt={results.get('a_rx',{}).get('corrupt',0)}")

    ok_ab = (results.get('b_rx', {}).get('valid', 0) == count and
             results.get('b_rx', {}).get('corrupt', 0) == 0)
    ok_ba = (results.get('a_rx', {}).get('valid', 0) == count and
             results.get('a_rx', {}).get('corrupt', 0) == 0)
    print(f"\n  A→B: {'PASS' if ok_ab else 'FAIL'}")
    print(f"  B→A: {'PASS' if ok_ba else 'FAIL'}")
    print(f"  Overall: {'PASS' if ok_ab and ok_ba else 'FAIL'}")
    print("=" * 60)
    return ok_ab and ok_ba


# ── Main ─────────────────────────────────────────────────────────────

def usage():
    print(f"""Usage: {sys.argv[0]} <mode> <socket_path> [options]

Modes:
  sender   <sock>  [--count N] [--delay S] [--id N]
      Send N test frames (default 100) with inter-frame delay S seconds.

  receiver <sock>  [--count N] [--timeout S] [--from N] [--id N]
      Receive up to N frames from sender id (default 1), timeout S seconds.

  pair     <sock>  [--count N] [--delay S]
      Self-test: sender(1) → hub → receiver(2). Single-direction.

  bidir    <sock>  [--count N] [--delay S]
      Bidirectional: two nodes send and receive simultaneously.

  stress   <sock>  [--duration S] [--maxsize N]
      Blast frames for S seconds at maximum rate, check integrity.

  dump     <sock>
      Passive listener — prints all frames (like medium_dump.py -v).

Examples:
  # Quick self-test:
  python3 {sys.argv[0]} pair /tmp/medium_hub_1.sock

  # Stress test for 30 seconds:
  python3 {sys.argv[0]} stress /tmp/medium_hub_1.sock --duration 30

  # Manual sender/receiver (two terminals):
  python3 {sys.argv[0]} receiver /tmp/medium_hub_1.sock --count 500
  python3 {sys.argv[0]} sender /tmp/medium_hub_1.sock --count 500
""")
    sys.exit(1)


def parse_args(argv):
    opts = {}
    i = 0
    while i < len(argv):
        if argv[i].startswith('--'):
            key = argv[i][2:]
            if i + 1 < len(argv):
                opts[key] = argv[i + 1]
                i += 2
            else:
                opts[key] = True
                i += 1
        else:
            i += 1
    return opts


def run_dump(sock_path):
    """Passive frame dump (like medium_dump.py but integrated)."""
    sock = connect_to_hub(sock_path)
    print(f"Connected to {sock_path}, dumping frames...\n")
    count = 0
    try:
        for hdr, payload in recv_messages(sock, timeout=3600):
            count += 1
            ts = time.strftime("%H:%M:%S")
            tx = mac_str(hdr['tx_mac'])
            print(f"[{ts}] #{count} from={tx} len={hdr['frame_len']} "
                  f"rate=0x{hdr['rate_code']:02x} rssi={hdr['rssi']}dBm")
            # Check if it's a test frame
            seq, sid, size, valid = parse_test_payload(payload)
            if seq is not None:
                status = "VALID" if valid else "CORRUPT"
                print(f"  TEST: seq={seq} sender={sid} size={size} [{status}]")
            else:
                # Show first few bytes
                hex_bytes = ' '.join(f'{b:02x}' for b in payload[:32])
                print(f"  DATA: {hex_bytes}")
            print()
    except KeyboardInterrupt:
        print(f"\n{count} frames captured")
    sock.close()


def main():
    if len(sys.argv) < 3:
        usage()

    mode = sys.argv[1]
    sock_path = sys.argv[2]
    opts = parse_args(sys.argv[3:])

    if mode == 'sender':
        run_sender(sock_path,
                   sender_id=int(opts.get('id', 1)),
                   count=int(opts.get('count', 100)),
                   delay=float(opts.get('delay', 0.01)))

    elif mode == 'receiver':
        run_receiver(sock_path,
                     sender_id=int(opts.get('from', 1)),
                     expect_count=int(opts.get('count', 100)),
                     timeout=float(opts.get('timeout', 10)),
                     my_id=int(opts.get('id', 2)))

    elif mode == 'pair':
        ok = run_pair(sock_path,
                      count=int(opts.get('count', 200)),
                      delay=float(opts.get('delay', 0.005)))
        sys.exit(0 if ok else 1)

    elif mode == 'bidir':
        ok = run_bidir(sock_path,
                       count=int(opts.get('count', 100)),
                       delay=float(opts.get('delay', 0.01)))
        sys.exit(0 if ok else 1)

    elif mode == 'stress':
        ok = run_stress(sock_path,
                        duration=int(opts.get('duration', 10)),
                        max_size=int(opts.get('maxsize', 4096)))
        sys.exit(0 if ok else 1)

    elif mode == 'dump':
        run_dump(sock_path)

    else:
        print(f"Unknown mode: {mode}")
        usage()


if __name__ == '__main__':
    main()
