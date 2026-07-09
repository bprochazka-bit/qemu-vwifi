#!/usr/bin/env python3
"""
Userspace regression harness for the audit fixes on this branch.

Spawns ./vwifi-medium on temporary sockets and exercises every fix that
can be exercised without a real kernel module / QEMU VM. Kernel-side
fixes (driver V1 acceptance H2, driver locking H3, relay buffer H6)
need a real kernel + module + VM and are NOT covered here -- see
tests/README.md for the manual procedure.

Usage:
    make userspace
    python3 tests/harness.py
    # or
    make test

Exit code: 0 = all pass, 1 = at least one fail, 2 = harness setup error.
"""

import os
import shutil
import signal
import socket
import struct
import subprocess
import sys
import tempfile
import time
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent


# =====================================================================
# Wire-format helpers (must match vwifi.h)
# =====================================================================
VWIFI_MAGIC = 0x46495756   # "VWIF"
HDR_V2_FMT  = '<I H H 6s B b I I I H H H H H 2s'  # 40 bytes
HDR_V2_SIZE = struct.calcsize(HDR_V2_FMT)
assert HDR_V2_SIZE == 40, HDR_V2_SIZE

# Channel flags (mirror vwifi.h VWIFI_CHAN_FLAG_*)
CHF_2GHZ      = 0x0001
CHF_5GHZ      = 0x0002
CHF_HT20      = 0x0004
CHF_HT40PLUS  = 0x0008
CHF_HT40MINUS = 0x0010
CHF_VHT80     = 0x0020
CHF_VHT160    = 0x0040
CHF_VHT80_80  = 0x0080
CHF_HE20      = 0x0100
CHF_HE40      = 0x0200
CHF_HE80      = 0x0400
CHF_HE160     = 0x0800

# Common channel center frequencies (MHz)
CH1, CH6, CH11             = 2412, 2437, 2462
CH36, CH40, CH44, CH48     = 5180, 5200, 5220, 5240
CH52, CH56, CH60, CH64     = 5260, 5280, 5300, 5320
CH149                      = 5745

# VHT80 block centers (5 GHz)
VHT80_CENTER_36_48 = 5210   # block covering ch36-ch48
VHT80_CENTER_52_64 = 5290   # block covering ch52-ch64
VHT160_CENTER_36_64 = 5250  # 160 MHz block covering ch36-ch64


def make_frame(tx_mac: bytes,
               channel_freq: int = 0,
               channel_bond_freq: int = 0,
               channel_flags: int = 0,
               center_freq1: int = 0,
               center_freq2: int = 0,
               payload: bytes = b'\x00' * 32,
               rate_code: int = 0x0B) -> bytes:
    """Build a complete wire message: [len_be][hdr V2][payload].

    All channel fields default to 0 which is the "unknown / broadcast"
    sentinel; pass real values to exercise the hub's channel-aware
    filter.
    """
    assert len(tx_mac) == 6
    frame_len = len(payload)
    hdr = struct.pack(
        HDR_V2_FMT,
        VWIFI_MAGIC,
        2,                      # version
        frame_len,
        tx_mac,
        rate_code,
        -30,                    # rssi
        0, 0,                   # tsf_lo, tsf_hi
        0,                      # flags (TTL=0; hub stamps it)
        channel_freq,
        channel_flags,
        channel_bond_freq,
        center_freq1,
        center_freq2,
        b'\x00\x00',            # reserved
    )
    wire = hdr + payload
    return struct.pack('!I', len(wire)) + wire


HELLO_MAGIC = 0x52495756   # "VWIR"
HELLO_FLAG_PHYSICAL = 0x01


def make_hello(node_id: str, physical: bool = False) -> bytes:
    """Build a hello/registration message: [len_be][HELLO_MAGIC][node_id\\0][flags?].

    `physical=True` appends the flag byte that marks the peer as a real
    radio (the phys bridge), so the hub exempts its links from the
    simulated physics model.
    """
    payload = struct.pack('<I', HELLO_MAGIC) + node_id.encode() + b'\x00'
    if physical:
        payload += bytes([HELLO_FLAG_PHYSICAL])
    return struct.pack('!I', len(payload)) + payload


# =====================================================================
# Harness
# =====================================================================
class Harness:
    def __init__(self):
        self.tmp = Path(tempfile.mkdtemp(prefix='vwifi-test-'))
        self.sock = self.tmp / 'medium.sock'
        self.ctl  = self.tmp / 'medium.ctl'
        self.log  = self.tmp / 'hub.log'
        self.hub: subprocess.Popen | None = None
        self.tcp_port: int | None = None
        self.passed = 0
        self.failed = 0
        self.failures: list[tuple[str, str]] = []

    # ---- hub lifecycle ----
    def start_hub(self, tcp_port: int | None = None):
        binary = REPO_ROOT / 'vwifi-medium'
        if not binary.exists():
            sys.exit(f'FATAL: {binary} not built. Run "make userspace" first.')
        # remove leftover sockets if a previous run died
        for p in (self.sock, self.ctl):
            if p.exists():
                p.unlink()
        self.tcp_port = tcp_port
        cmd = [str(binary), str(self.sock), '-c', str(self.ctl)]
        if tcp_port is not None:
            cmd += ['-t', str(tcp_port)]
        self.hub = subprocess.Popen(
            cmd,
            stdout=open(self.log, 'w'),
            stderr=subprocess.STDOUT,
            cwd=str(self.tmp),
        )
        # wait for both sockets to appear
        deadline = time.time() + 2.0
        while time.time() < deadline:
            if self.sock.exists() and self.ctl.exists():
                # one extra beat so listen() is in place
                time.sleep(0.05)
                return
            if self.hub.poll() is not None:
                sys.exit(f'FATAL: hub exited early. Log:\n{self.log.read_text()}')
            time.sleep(0.02)
        sys.exit(f'FATAL: hub did not bind sockets in 2s. Log:\n{self.log.read_text()}')

    def stop_hub(self):
        if not self.hub:
            return
        if self.hub.poll() is None:
            self.hub.terminate()
            try:
                self.hub.wait(timeout=2)
            except subprocess.TimeoutExpired:
                self.hub.kill()
                self.hub.wait()
        self.hub = None

    def restart_hub(self, tcp_port: int | None = None):
        self.stop_hub()
        self.start_hub(tcp_port=tcp_port)

    def bridge_conn(self) -> socket.socket:
        """Open a TCP connection to the hub's bridge port. The hub treats
        TCP peers as inter-hub bridges (is_bridge=1)."""
        assert self.tcp_port is not None, 'hub started without a TCP port'
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.connect(('127.0.0.1', self.tcp_port))
        return s

    def cleanup(self):
        self.stop_hub()
        shutil.rmtree(self.tmp, ignore_errors=True)

    # ---- control protocol ----
    def ctl_cmd(self, cmd: str, timeout: float = 2.0) -> str:
        """Send one command on the control socket; return the response text.

        Reads until the socket closes (server-side does close on QUIT)
        or until no data has arrived for ~150 ms after the first byte.
        """
        s = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        s.settimeout(timeout)
        s.connect(str(self.ctl))
        s.sendall((cmd + '\n').encode())
        chunks = []
        s.settimeout(0.3)
        try:
            while True:
                d = s.recv(8192)
                if not d:
                    break
                chunks.append(d)
        except socket.timeout:
            pass
        finally:
            s.close()
        return b''.join(chunks).decode(errors='replace')

    def data_conn(self) -> socket.socket:
        s = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        s.connect(str(self.sock))
        return s

    @staticmethod
    def drain(sock: socket.socket, settle: float = 0.1):
        """Drain any pending bytes on a non-blocking socket."""
        time.sleep(settle)
        sock.setblocking(False)
        try:
            while sock.recv(8192):
                pass
        except (BlockingIOError, OSError):
            pass
        sock.setblocking(True)

    # ---- assertions ----
    def expect(self, cond: bool, what: str, why: str = ''):
        if cond:
            self.passed += 1
            print(f'   PASS  {what}')
        else:
            self.failed += 1
            self.failures.append((what, why))
            print(f'   FAIL  {what}' + (f'  -- {why}' if why else ''))

    def section(self, name: str):
        print(f'\n== {name} ==')

    def report(self) -> int:
        print()
        print(f'==== {self.passed} passed, {self.failed} failed ====')
        if self.failed:
            print()
            print('Failures:')
            for what, why in self.failures:
                print(f'  - {what}: {why}')
            print()
            print(f'Hub log: {self.log}')
            return 1
        return 0


# =====================================================================
# Test cases (each takes a Harness)
# =====================================================================
def test_c4_socket_perms(h: Harness):
    h.section('C4: socket permissions')
    ctl_mode = h.ctl.stat().st_mode & 0o777
    h.expect(ctl_mode == 0o600, 'control socket is 0600',
             f'got 0{ctl_mode:o}')
    sock_mode = h.sock.stat().st_mode & 0o777
    h.expect(sock_mode == 0o666, 'data socket is 0666',
             f'got 0{sock_mode:o}')


def test_list_peers_empty(h: Harness):
    h.section('LIST_PEERS empty')
    r = h.ctl_cmd('LIST_PEERS')
    h.expect('0 nodes' in r, 'fresh hub reports 0 nodes', repr(r[:120]))


def test_l6_tx_drop_column(h: Harness):
    h.section('L6: tx_drop column visible')
    h.ctl_cmd('SET_POS testnode 1 2 3')
    r = h.ctl_cmd('LIST_PEERS')
    h.expect('tx_drop=' in r, 'tx_drop column present', repr(r[:200]))
    h.expect('testnode' in r, 'testnode visible after SET_POS', repr(r[:200]))


def test_c3_maclist_capacity(h: Harness):
    h.section('C3: maclist holds many MACs without truncation')
    # We can't easily force 16 MACs onto one node without a long-running
    # data session, so just check the buffer accepts a typical-width line.
    # A node with one MAC should render the macs=[xx:xx:..] block fully.
    # (The buffer was 256 bytes / max 16 MACs * 18 = 288, now 512.)
    r = h.ctl_cmd('LIST_PEERS')
    if 'testnode' in r:
        line = next(l for l in r.splitlines() if 'testnode' in l)
        h.expect('macs=[' in line and ']' in line,
                 'maclist brackets present and closed',
                 line[:200])


def test_c2_load_config_recursion(h: Harness):
    h.section('C2: LOAD_CONFIG recursion cap')
    loop = h.tmp / 'loop.cfg'
    loop.write_text(f'LOAD_CONFIG {loop}\n')
    r = h.ctl_cmd(f'LOAD_CONFIG {loop}')
    h.expect('too deeply nested' in r,
             'self-referential config returns nesting error',
             repr(r[:200]))
    h.expect(h.hub.poll() is None, 'hub still running after recursion bomb',
             'hub exited')


def test_h5_quit_in_load_config(h: Harness):
    h.section('H5: QUIT inside LOAD_CONFIG terminates cleanly')
    cfg = h.tmp / 'quit.cfg'
    cfg.write_text(
        'SET_POS alice 1 2 3\n'
        'QUIT\n'
        'SET_POS bob 4 5 6\n'
    )
    h.ctl_cmd(f'LOAD_CONFIG {cfg}')
    r = h.ctl_cmd('LIST_PEERS')
    h.expect('alice' in r, 'commands before QUIT executed', repr(r[:300]))
    h.expect('bob' not in r, 'commands after QUIT did NOT execute',
             repr(r[:300]))


def test_h5_orig_large_response(h: Harness):
    h.section('H5-orig: large LIST_PEERS not truncated under EAGAIN risk')
    for i in range(120):
        h.ctl_cmd(f'SET_POS auto{i} {i} 0 0')
    r = h.ctl_cmd('LIST_PEERS', timeout=5.0)
    # one header line + at least one line per node
    lines = r.count('\n')
    h.expect(lines >= 120,
             f'LIST_PEERS returned >=120 lines (got {lines})')


def test_c1_channel_mismatch_drops(h: Harness):
    h.section('C1: channel mismatch drops frames')
    h.restart_hub()
    a = h.data_conn()
    b = h.data_conn()
    try:
        mac_a = b'\x02\x00\x00\x00\x00\xaa'
        mac_b = b'\x02\x00\x00\x00\x00\xbb'

        # B registers channel 11 (2462), A registers channel 1 (2412)
        b.sendall(make_frame(mac_b, channel_freq=2462))
        a.sendall(make_frame(mac_a, channel_freq=2412))
        h.drain(b)

        # A sends on ch1, B (ch11) must NOT receive
        a.sendall(make_frame(mac_a, channel_freq=2412,
                             payload=b'X' * 64))
        b.settimeout(0.3)
        try:
            got = b.recv(8192)
        except socket.timeout:
            got = b''
        h.expect(got == b'',
                 'peer on ch11 did NOT receive frame from ch1 sender',
                 f'received {len(got)} bytes')
    finally:
        a.close(); b.close()


def test_c1_channel_match_delivers(h: Harness):
    h.section('C1: channel match delivers frames')
    h.restart_hub()
    a = h.data_conn()
    b = h.data_conn()
    try:
        mac_a = b'\x02\x00\x00\x00\x00\xaa'
        mac_b = b'\x02\x00\x00\x00\x00\xbb'

        # Both peers on channel 1
        b.sendall(make_frame(mac_b, channel_freq=2412))
        a.sendall(make_frame(mac_a, channel_freq=2412))
        h.drain(b)

        # A sends another ch1 frame; B (ch1) must receive
        a.sendall(make_frame(mac_a, channel_freq=2412,
                             payload=b'Y' * 64))
        b.settimeout(0.5)
        try:
            got = b.recv(8192)
        except socket.timeout:
            got = b''
        h.expect(len(got) > 0,
                 'peer on ch1 received frame from ch1 sender',
                 f'received {len(got)} bytes')
    finally:
        a.close(); b.close()


def test_c1_v1_broadcast(h: Harness):
    h.section('C1: V1 senders (channel_freq=0) broadcast to all')
    h.restart_hub()
    a = h.data_conn()
    b = h.data_conn()
    try:
        mac_a = b'\x02\x00\x00\x00\x00\xaa'
        mac_b = b'\x02\x00\x00\x00\x00\xbb'

        # B registers ch11, A has channel_freq=0 (unknown / V1-style)
        b.sendall(make_frame(mac_b, channel_freq=2462))
        a.sendall(make_frame(mac_a, channel_freq=0))
        h.drain(b)

        # A sends with channel_freq=0 -- per spec this is broadcast;
        # B should receive even though its channel is 11.
        a.sendall(make_frame(mac_a, channel_freq=0,
                             payload=b'Z' * 64))
        b.settimeout(0.5)
        try:
            got = b.recv(8192)
        except socket.timeout:
            got = b''
        h.expect(len(got) > 0,
                 'V1/unknown sender broadcasts despite receiver channel',
                 f'received {len(got)} bytes')
    finally:
        a.close(); b.close()


# ---------------------------------------------------------------------
# Channel-filter coverage: HT40 plus/minus, VHT80 center, multi-peer
# fanout, mid-stream channel changes. The single-pair sender/receiver
# helper below keeps each test compact; each peer "tunes" by sending
# one registration frame so the hub learns its channel, and the
# receiver's pre-test backlog is drained before the assertion frame.
# ---------------------------------------------------------------------

def _tune(sock, mac, **chan_kwargs):
    """Tell the hub which channel this peer is on by transmitting one
    frame with the desired channel fields. Returns immediately; caller
    is responsible for draining the receiver afterwards."""
    sock.sendall(make_frame(mac, **chan_kwargs))


def _drain_all(socks, settle=0.1):
    """Drain any pending bytes on a list of sockets."""
    time.sleep(settle)
    for s in socks:
        s.setblocking(False)
        try:
            while s.recv(8192):
                pass
        except (BlockingIOError, OSError):
            pass
        s.setblocking(True)


def _recv_with_marker(sock, marker, timeout=0.4):
    """Read up to one chunk and return True iff `marker` appears in it."""
    sock.settimeout(timeout)
    try:
        got = sock.recv(8192)
    except socket.timeout:
        return False, 0
    return marker in got, len(got)


def test_c1_ht40_plus_minus_dont_collide(h: Harness):
    h.section('C1: HT40+ and HT40- on the same primary do NOT collide')
    h.restart_hub()
    a = h.data_conn()
    b = h.data_conn()
    try:
        mac_a = b'\x02\x00\x00\x00\x40\xaa'
        mac_b = b'\x02\x00\x00\x00\x40\xbb'

        # A is on ch6 HT40+ (primary=2437, bond=2457).
        # B is on ch6 HT40- (primary=2437, bond=2417).
        # Same primary, different bond pair -> filter must drop.
        _tune(a, mac_a, channel_freq=CH6, channel_bond_freq=2457,
              channel_flags=CHF_2GHZ | CHF_HT40PLUS)
        _tune(b, mac_b, channel_freq=CH6, channel_bond_freq=2417,
              channel_flags=CHF_2GHZ | CHF_HT40MINUS)
        _drain_all([a, b])

        marker = b'PLUSMINUS-NEVER-DELIVER-' + os.urandom(16)
        a.sendall(make_frame(mac_a, channel_freq=CH6, channel_bond_freq=2457,
                             channel_flags=CHF_2GHZ | CHF_HT40PLUS,
                             payload=marker.ljust(64, b'\x00')))

        got, n = _recv_with_marker(b, marker)
        h.expect(not got,
                 'HT40+ frame is NOT delivered to HT40- peer',
                 f'unexpectedly received {n} bytes containing marker')
    finally:
        a.close(); b.close()


def test_c1_ht40_to_ht20_delivers(h: Harness):
    h.section('C1: HT40+ frame IS delivered to a HT20 receiver on the same primary')
    h.restart_hub()
    a = h.data_conn()
    b = h.data_conn()
    try:
        mac_a = b'\x02\x00\x00\x00\x41\xaa'
        mac_b = b'\x02\x00\x00\x00\x41\xbb'

        # A on ch1 HT40+ (primary=2412, bond=2432). B on ch1 HT20 (bond=0).
        # Mixed width: only one side declares a bond pair, so the bond
        # check is skipped and the shared primary governs -- mirroring
        # real 802.11 where an HT20 station hears an HT40 AP on the
        # primary. The bond pair only separates two *bonded* peers
        # (see test_c1_ht40_plus_minus_dont_collide).
        _tune(a, mac_a, channel_freq=CH1, channel_bond_freq=2432,
              channel_flags=CHF_2GHZ | CHF_HT40PLUS)
        _tune(b, mac_b, channel_freq=CH1, channel_bond_freq=0,
              channel_flags=CHF_2GHZ | CHF_HT20)
        _drain_all([a, b])

        marker = b'HT40-TO-HT20-' + os.urandom(16)
        a.sendall(make_frame(mac_a, channel_freq=CH1, channel_bond_freq=2432,
                             channel_flags=CHF_2GHZ | CHF_HT40PLUS,
                             payload=marker.ljust(64, b'\x00')))

        got, n = _recv_with_marker(b, marker)
        h.expect(got,
                 'HT40+ frame is delivered to a HT20 receiver on shared primary',
                 f'received {n} bytes; marker not found')
    finally:
        a.close(); b.close()


def test_c1_ht40_match_delivers(h: Harness):
    h.section('C1: HT40+ peers on matching primary+bond deliver')
    h.restart_hub()
    a = h.data_conn()
    b = h.data_conn()
    try:
        mac_a = b'\x02\x00\x00\x00\x42\xaa'
        mac_b = b'\x02\x00\x00\x00\x42\xbb'

        # Both on ch1 HT40+ (primary=2412, bond=2432).
        _tune(a, mac_a, channel_freq=CH1, channel_bond_freq=2432,
              channel_flags=CHF_2GHZ | CHF_HT40PLUS)
        _tune(b, mac_b, channel_freq=CH1, channel_bond_freq=2432,
              channel_flags=CHF_2GHZ | CHF_HT40PLUS)
        _drain_all([a, b])

        marker = b'HT40-MATCH-' + os.urandom(16)
        a.sendall(make_frame(mac_a, channel_freq=CH1, channel_bond_freq=2432,
                             channel_flags=CHF_2GHZ | CHF_HT40PLUS,
                             payload=marker.ljust(64, b'\x00')))

        got, n = _recv_with_marker(b, marker)
        h.expect(got,
                 'HT40+ frame is delivered to a matching HT40+ peer',
                 f'received {n} bytes; marker not found')
    finally:
        a.close(); b.close()


def test_c1_multichannel_fanout_isolation(h: Harness):
    h.section('C1: 5-peer fanout: only same-channel receiver gets the frame')
    h.restart_hub()
    # One sender (ch1), four receivers (ch1, ch6, ch11, ch36). The
    # one on ch1 should receive; the other three should not.
    sender = h.data_conn()
    rx_ch1  = h.data_conn()
    rx_ch6  = h.data_conn()
    rx_ch11 = h.data_conn()
    rx_ch36 = h.data_conn()
    all_socks = [sender, rx_ch1, rx_ch6, rx_ch11, rx_ch36]

    try:
        macs = {
            'send':  b'\x02\x00\x00\x00\x50\x01',
            'ch1':   b'\x02\x00\x00\x00\x50\x02',
            'ch6':   b'\x02\x00\x00\x00\x50\x03',
            'ch11':  b'\x02\x00\x00\x00\x50\x04',
            'ch36':  b'\x02\x00\x00\x00\x50\x05',
        }

        _tune(sender,  macs['send'], channel_freq=CH1,  channel_flags=CHF_2GHZ | CHF_HT20)
        _tune(rx_ch1,  macs['ch1'],  channel_freq=CH1,  channel_flags=CHF_2GHZ | CHF_HT20)
        _tune(rx_ch6,  macs['ch6'],  channel_freq=CH6,  channel_flags=CHF_2GHZ | CHF_HT20)
        _tune(rx_ch11, macs['ch11'], channel_freq=CH11, channel_flags=CHF_2GHZ | CHF_HT20)
        _tune(rx_ch36, macs['ch36'], channel_freq=CH36, channel_flags=CHF_5GHZ | CHF_HT20)
        _drain_all(all_socks)

        marker = b'FANOUT-CH1-' + os.urandom(16)
        sender.sendall(make_frame(macs['send'],
                                  channel_freq=CH1,
                                  channel_flags=CHF_2GHZ | CHF_HT20,
                                  payload=marker.ljust(64, b'\x00')))

        got_ch1, _  = _recv_with_marker(rx_ch1, marker)
        got_ch6, _  = _recv_with_marker(rx_ch6, marker)
        got_ch11, _ = _recv_with_marker(rx_ch11, marker)
        got_ch36, _ = _recv_with_marker(rx_ch36, marker)

        h.expect(got_ch1,      'ch1 receiver got the frame')
        h.expect(not got_ch6,  'ch6 receiver did NOT get the frame')
        h.expect(not got_ch11, 'ch11 receiver did NOT get the frame')
        h.expect(not got_ch36, 'ch36 receiver did NOT get the frame')
    finally:
        for s in all_socks:
            s.close()


def test_c1_vht80_center_mismatch_drops(h: Harness):
    h.section('C1: VHT80 peers with same primary but different center_freq1 do NOT collide')
    h.restart_hub()
    a = h.data_conn()
    b = h.data_conn()
    try:
        mac_a = b'\x02\x00\x00\x00\x80\xaa'
        mac_b = b'\x02\x00\x00\x00\x80\xbb'

        # Both peers on primary=5180 declaring VHT80, but the test
        # forces different center_freq1 values. Real hardware can't
        # actually be in this state (primary determines the 80-MHz
        # block), but the filter logic must still distinguish if a
        # crafted frame arrives -- otherwise overlapping 80-MHz blocks
        # would silently cross-deliver.
        _tune(a, mac_a, channel_freq=CH36,
              channel_bond_freq=CH40,
              channel_flags=CHF_5GHZ | CHF_VHT80,
              center_freq1=VHT80_CENTER_36_48)
        _tune(b, mac_b, channel_freq=CH36,
              channel_bond_freq=CH40,
              channel_flags=CHF_5GHZ | CHF_VHT80,
              center_freq1=VHT80_CENTER_52_64)  # synthetic mismatch
        _drain_all([a, b])

        marker = b'VHT80-CENTER-MISMATCH-' + os.urandom(16)
        a.sendall(make_frame(mac_a, channel_freq=CH36,
                             channel_bond_freq=CH40,
                             channel_flags=CHF_5GHZ | CHF_VHT80,
                             center_freq1=VHT80_CENTER_36_48,
                             payload=marker.ljust(64, b'\x00')))

        got, n = _recv_with_marker(b, marker)
        h.expect(not got,
                 'VHT80 frame with mismatched center_freq1 is dropped',
                 f'unexpectedly received {n} bytes containing marker')
    finally:
        a.close(); b.close()


def test_c1_vht80_center_match_delivers(h: Harness):
    h.section('C1: VHT80 peers with matching primary+center_freq1 deliver')
    h.restart_hub()
    a = h.data_conn()
    b = h.data_conn()
    try:
        mac_a = b'\x02\x00\x00\x00\x81\xaa'
        mac_b = b'\x02\x00\x00\x00\x81\xbb'

        _tune(a, mac_a, channel_freq=CH36, channel_bond_freq=CH40,
              channel_flags=CHF_5GHZ | CHF_VHT80,
              center_freq1=VHT80_CENTER_36_48)
        _tune(b, mac_b, channel_freq=CH36, channel_bond_freq=CH40,
              channel_flags=CHF_5GHZ | CHF_VHT80,
              center_freq1=VHT80_CENTER_36_48)
        _drain_all([a, b])

        marker = b'VHT80-CENTER-MATCH-' + os.urandom(16)
        a.sendall(make_frame(mac_a, channel_freq=CH36,
                             channel_bond_freq=CH40,
                             channel_flags=CHF_5GHZ | CHF_VHT80,
                             center_freq1=VHT80_CENTER_36_48,
                             payload=marker.ljust(64, b'\x00')))

        got, n = _recv_with_marker(b, marker)
        h.expect(got,
                 'VHT80 frame with matching center_freq1 is delivered',
                 f'received {n} bytes; marker not found')
    finally:
        a.close(); b.close()


def test_a_vht80_mixed_bond_delivers(h: Harness):
    h.section('A: VHT80 VM-style (bond set) and bridge-style (bond=0) on the '
              'same block deliver')
    h.restart_hub()
    a = h.data_conn()
    b = h.data_conn()
    try:
        mac_a = b'\x02\x00\x00\x03\x80\xaa'
        mac_b = b'\x02\x00\x00\x03\x80\xbb'

        # A models the kernel driver, which sets channel_bond_freq for
        # VHT80 (the implicit HT40 secondary). B models vwifi-phys-bridge,
        # which leaves bond_freq=0 for VHT80 and relies on center_freq1.
        # Both are genuinely on the same 80-MHz block (ch36-48, center
        # 5210); they MUST hear each other. Pre-fix, the strict bond rule
        # split them apart and broke the physical bridge for VHT/HE.
        _tune(a, mac_a, channel_freq=CH36, channel_bond_freq=CH40,
              channel_flags=CHF_5GHZ | CHF_VHT80,
              center_freq1=VHT80_CENTER_36_48)
        _tune(b, mac_b, channel_freq=CH36, channel_bond_freq=0,
              channel_flags=CHF_5GHZ | CHF_VHT80,
              center_freq1=VHT80_CENTER_36_48)
        _drain_all([a, b])

        marker = b'VHT80-MIXED-BOND-' + os.urandom(16)
        a.sendall(make_frame(mac_a, channel_freq=CH36, channel_bond_freq=CH40,
                             channel_flags=CHF_5GHZ | CHF_VHT80,
                             center_freq1=VHT80_CENTER_36_48,
                             payload=marker.ljust(64, b'\x00')))

        got, n = _recv_with_marker(b, marker)
        h.expect(got,
                 'VHT80 frame crosses a bond_freq encoding mismatch when '
                 'center_freq1 agrees',
                 f'received {n} bytes; marker not found')
    finally:
        a.close(); b.close()


def test_a_vht80_mixed_bond_center_mismatch_still_drops(h: Harness):
    h.section('A: VHT80 mixed bond but different center_freq1 still drops '
              '(center disambiguation intact)')
    h.restart_hub()
    a = h.data_conn()
    b = h.data_conn()
    try:
        mac_a = b'\x02\x00\x00\x03\x81\xaa'
        mac_b = b'\x02\x00\x00\x03\x81\xbb'

        # Same primary, one side bond-set / one side bond=0, but the
        # 80-MHz centers differ. Relaxing the bond rule must NOT collapse
        # wide-channel center disambiguation: this still drops.
        _tune(a, mac_a, channel_freq=CH36, channel_bond_freq=CH40,
              channel_flags=CHF_5GHZ | CHF_VHT80,
              center_freq1=VHT80_CENTER_36_48)
        _tune(b, mac_b, channel_freq=CH36, channel_bond_freq=0,
              channel_flags=CHF_5GHZ | CHF_VHT80,
              center_freq1=VHT80_CENTER_52_64)  # synthetic center mismatch
        _drain_all([a, b])

        marker = b'VHT80-MIXED-BOND-MISMATCH-' + os.urandom(16)
        a.sendall(make_frame(mac_a, channel_freq=CH36, channel_bond_freq=CH40,
                             channel_flags=CHF_5GHZ | CHF_VHT80,
                             center_freq1=VHT80_CENTER_36_48,
                             payload=marker.ljust(64, b'\x00')))

        got, n = _recv_with_marker(b, marker)
        h.expect(not got,
                 'VHT80 frame with mismatched center_freq1 still drops despite '
                 'mixed bond encoding',
                 f'unexpectedly received {n} bytes containing marker')
    finally:
        a.close(); b.close()


def _count_frames_with_prefix(sock, marker_prefix, settle=0.4):
    """Drain a non-blocking socket and count occurrences of a byte
    prefix in the combined buffer. Used by statistical FER tests
    where the count of delivered frames is the assertion."""
    time.sleep(settle)
    sock.setblocking(False)
    chunks = []
    try:
        while True:
            d = sock.recv(65536)
            if not d:
                break
            chunks.append(d)
    except (BlockingIOError, OSError):
        pass
    sock.setblocking(True)
    return b''.join(chunks).count(marker_prefix)


def _mac_str(mac_bytes):
    return ':'.join(f'{b:02x}' for b in mac_bytes)


# ---------------------------------------------------------------------
# HE / VHT specific channel-filter coverage
# ---------------------------------------------------------------------

def test_c1_he80_center_match_delivers(h: Harness):
    h.section('HE80: matching primary+center_freq1 delivers')
    h.restart_hub()
    a = h.data_conn()
    b = h.data_conn()
    try:
        mac_a = b'\x02\x00\x00\x00\xe1\xaa'
        mac_b = b'\x02\x00\x00\x00\xe1\xbb'
        _tune(a, mac_a, channel_freq=CH36, channel_bond_freq=CH40,
              channel_flags=CHF_5GHZ | CHF_HE80,
              center_freq1=VHT80_CENTER_36_48)
        _tune(b, mac_b, channel_freq=CH36, channel_bond_freq=CH40,
              channel_flags=CHF_5GHZ | CHF_HE80,
              center_freq1=VHT80_CENTER_36_48)
        _drain_all([a, b])

        marker = b'HE80-MATCH-' + os.urandom(16)
        a.sendall(make_frame(mac_a, channel_freq=CH36,
                             channel_bond_freq=CH40,
                             channel_flags=CHF_5GHZ | CHF_HE80,
                             center_freq1=VHT80_CENTER_36_48,
                             rate_code=0xE0,           # HE80 NSS=1 MCS0
                             payload=marker.ljust(64, b'\x00')))
        got, n = _recv_with_marker(b, marker)
        h.expect(got, 'HE80 frame delivered to matching HE80 peer',
                 f'received {n} bytes; marker not found')
    finally:
        a.close(); b.close()


def test_c1_he80_center_mismatch_drops(h: Harness):
    h.section('HE80: same primary, different center_freq1 -> dropped')
    h.restart_hub()
    a = h.data_conn()
    b = h.data_conn()
    try:
        mac_a = b'\x02\x00\x00\x00\xe2\xaa'
        mac_b = b'\x02\x00\x00\x00\xe2\xbb'

        _tune(a, mac_a, channel_freq=CH36, channel_bond_freq=CH40,
              channel_flags=CHF_5GHZ | CHF_HE80,
              center_freq1=VHT80_CENTER_36_48)
        # Synthetic: same primary, different center_freq1
        _tune(b, mac_b, channel_freq=CH36, channel_bond_freq=CH40,
              channel_flags=CHF_5GHZ | CHF_HE80,
              center_freq1=VHT80_CENTER_52_64)
        _drain_all([a, b])

        marker = b'HE80-CENTER-MISMATCH-' + os.urandom(16)
        a.sendall(make_frame(mac_a, channel_freq=CH36,
                             channel_bond_freq=CH40,
                             channel_flags=CHF_5GHZ | CHF_HE80,
                             center_freq1=VHT80_CENTER_36_48,
                             rate_code=0xE5,           # HE80 NSS=1 MCS5
                             payload=marker.ljust(64, b'\x00')))
        got, n = _recv_with_marker(b, marker)
        h.expect(not got,
                 'HE80 frame with mismatched center_freq1 is dropped',
                 f'unexpectedly received {n} bytes')
    finally:
        a.close(); b.close()


def test_c1_he40_to_he20_delivers(h: Harness):
    h.section('HE40+ frame IS delivered to a HE20 receiver on the same primary')
    h.restart_hub()
    a = h.data_conn()
    b = h.data_conn()
    try:
        mac_a = b'\x02\x00\x00\x00\xe3\xaa'
        mac_b = b'\x02\x00\x00\x00\xe3\xbb'

        # A: HE40+ on ch36 (primary=5180, bond=5200)
        _tune(a, mac_a, channel_freq=CH36, channel_bond_freq=CH40,
              channel_flags=CHF_5GHZ | CHF_HE40)
        # B: HE20 on ch36 (primary=5180, no bond). Mixed width: only one
        # side sets bond, so the shared primary governs and the frame is
        # heard -- the HE analogue of test_c1_ht40_to_ht20_delivers.
        _tune(b, mac_b, channel_freq=CH36, channel_bond_freq=0,
              channel_flags=CHF_5GHZ | CHF_HE20)
        _drain_all([a, b])

        marker = b'HE40-TO-HE20-' + os.urandom(16)
        a.sendall(make_frame(mac_a, channel_freq=CH36,
                             channel_bond_freq=CH40,
                             channel_flags=CHF_5GHZ | CHF_HE40,
                             payload=marker.ljust(64, b'\x00')))
        got, n = _recv_with_marker(b, marker)
        h.expect(got,
                 'HE40+ frame is delivered to a HE20 receiver on shared primary',
                 f'received {n} bytes; marker not found')
    finally:
        a.close(); b.close()


def test_c1_vht160_center_match_delivers(h: Harness):
    h.section('VHT160: matching primary+center_freq1 delivers')
    h.restart_hub()
    a = h.data_conn()
    b = h.data_conn()
    try:
        mac_a = b'\x02\x00\x00\x01\x60\xaa'
        mac_b = b'\x02\x00\x00\x01\x60\xbb'

        # Both on ch36 VHT160 (primary=5180, bond=5200, 160-MHz center=5250)
        _tune(a, mac_a, channel_freq=CH36, channel_bond_freq=CH40,
              channel_flags=CHF_5GHZ | CHF_VHT160,
              center_freq1=VHT160_CENTER_36_64)
        _tune(b, mac_b, channel_freq=CH36, channel_bond_freq=CH40,
              channel_flags=CHF_5GHZ | CHF_VHT160,
              center_freq1=VHT160_CENTER_36_64)
        _drain_all([a, b])

        marker = b'VHT160-MATCH-' + os.urandom(16)
        a.sendall(make_frame(mac_a, channel_freq=CH36,
                             channel_bond_freq=CH40,
                             channel_flags=CHF_5GHZ | CHF_VHT160,
                             center_freq1=VHT160_CENTER_36_64,
                             rate_code=0xC0,           # VHT160 NSS=1 MCS0
                             payload=marker.ljust(64, b'\x00')))
        got, n = _recv_with_marker(b, marker)
        h.expect(got, 'VHT160 frame delivered to matching VHT160 peer',
                 f'received {n} bytes; marker not found')
    finally:
        a.close(); b.close()


def test_c1_vht160_center_mismatch_drops(h: Harness):
    h.section('VHT160: same primary, different center_freq1 -> dropped')
    h.restart_hub()
    a = h.data_conn()
    b = h.data_conn()
    try:
        mac_a = b'\x02\x00\x00\x01\x61\xaa'
        mac_b = b'\x02\x00\x00\x01\x61\xbb'

        _tune(a, mac_a, channel_freq=CH36, channel_bond_freq=CH40,
              channel_flags=CHF_5GHZ | CHF_VHT160,
              center_freq1=VHT160_CENTER_36_64)
        # Synthetic 160-MHz "block" with a different center
        _tune(b, mac_b, channel_freq=CH36, channel_bond_freq=CH40,
              channel_flags=CHF_5GHZ | CHF_VHT160,
              center_freq1=5410)
        _drain_all([a, b])

        marker = b'VHT160-MISMATCH-' + os.urandom(16)
        a.sendall(make_frame(mac_a, channel_freq=CH36,
                             channel_bond_freq=CH40,
                             channel_flags=CHF_5GHZ | CHF_VHT160,
                             center_freq1=VHT160_CENTER_36_64,
                             rate_code=0xC9,           # VHT160 NSS=1 MCS9
                             payload=marker.ljust(64, b'\x00')))
        got, n = _recv_with_marker(b, marker)
        h.expect(not got,
                 'VHT160 frame with mismatched center_freq1 is dropped',
                 f'unexpectedly received {n} bytes')
    finally:
        a.close(); b.close()


def test_c1_vht80_80_center2_match_delivers(h: Harness):
    h.section('VHT80+80: matching primary+center_freq1+center_freq2 delivers')
    h.restart_hub()
    a = h.data_conn()
    b = h.data_conn()
    try:
        mac_a = b'\x02\x00\x00\x01\x80\xaa'
        mac_b = b'\x02\x00\x00\x01\x80\xbb'

        # 80+80: primary ch36 block (center1=5210) + secondary block at ch149 (center2=5775)
        _tune(a, mac_a, channel_freq=CH36, channel_bond_freq=CH40,
              channel_flags=CHF_5GHZ | CHF_VHT80_80,
              center_freq1=VHT80_CENTER_36_48,
              center_freq2=5775)
        _tune(b, mac_b, channel_freq=CH36, channel_bond_freq=CH40,
              channel_flags=CHF_5GHZ | CHF_VHT80_80,
              center_freq1=VHT80_CENTER_36_48,
              center_freq2=5775)
        _drain_all([a, b])

        marker = b'VHT80+80-MATCH-' + os.urandom(16)
        a.sendall(make_frame(mac_a, channel_freq=CH36,
                             channel_bond_freq=CH40,
                             channel_flags=CHF_5GHZ | CHF_VHT80_80,
                             center_freq1=VHT80_CENTER_36_48,
                             center_freq2=5775,
                             payload=marker.ljust(64, b'\x00')))
        got, n = _recv_with_marker(b, marker)
        h.expect(got, 'VHT80+80 frame delivered to matching peer',
                 f'received {n} bytes; marker not found')
    finally:
        a.close(); b.close()


def test_c1_vht80_80_center2_mismatch_drops(h: Harness):
    h.section('VHT80+80: matching center1 but different center2 -> dropped')
    h.restart_hub()
    a = h.data_conn()
    b = h.data_conn()
    try:
        mac_a = b'\x02\x00\x00\x01\x81\xaa'
        mac_b = b'\x02\x00\x00\x01\x81\xbb'

        _tune(a, mac_a, channel_freq=CH36, channel_bond_freq=CH40,
              channel_flags=CHF_5GHZ | CHF_VHT80_80,
              center_freq1=VHT80_CENTER_36_48,
              center_freq2=5775)        # secondary at ch149 block
        _tune(b, mac_b, channel_freq=CH36, channel_bond_freq=CH40,
              channel_flags=CHF_5GHZ | CHF_VHT80_80,
              center_freq1=VHT80_CENTER_36_48,
              center_freq2=5530)        # secondary at ch100 block (different)
        _drain_all([a, b])

        marker = b'VHT80+80-MISMATCH-' + os.urandom(16)
        a.sendall(make_frame(mac_a, channel_freq=CH36,
                             channel_bond_freq=CH40,
                             channel_flags=CHF_5GHZ | CHF_VHT80_80,
                             center_freq1=VHT80_CENTER_36_48,
                             center_freq2=5775,
                             payload=marker.ljust(64, b'\x00')))
        got, n = _recv_with_marker(b, marker)
        h.expect(not got,
                 'VHT80+80 frame with mismatched center_freq2 is dropped',
                 f'unexpectedly received {n} bytes')
    finally:
        a.close(); b.close()


def test_m3_he_mcs_rate_thresholds(h: Harness):
    h.section('M3: HE MCS rate codes drive FER thresholds (MCS0 delivers, MCS9 drops)')
    h.restart_hub()
    a = h.data_conn()
    b = h.data_conn()
    try:
        mac_a = b'\x02\x00\x00\x02\xe0\xaa'
        mac_b = b'\x02\x00\x00\x02\xe0\xbb'

        chan_kwargs = dict(
            channel_freq=CH36,
            channel_bond_freq=CH40,
            channel_flags=CHF_5GHZ | CHF_HE80,
            center_freq1=VHT80_CENTER_36_48,
        )
        _tune(a, mac_a, **chan_kwargs)
        _tune(b, mac_b, **chan_kwargs)
        _drain_all([a, b])

        # Pin SNR to 20 dB symmetric. SET_SNR auto-switches the hub to
        # MODEL_SNR_TABLE so per-frame FER will fire.
        r = h.ctl_cmd(f'SET_SNR {_mac_str(mac_a)} {_mac_str(mac_b)} 20 20')
        h.expect('OK' in r, 'SET_SNR accepted via MAC strings', repr(r[:120]))

        # Batch 1: HE80 NSS=1 MCS0 (rate_code 0xE0, min_snr=5 dB)
        # margin = 20 - 5 = 15 dB, > 10 -> FER = 0 -> all deliver
        N = 50
        prefix0 = b'HEMCS0-'
        for i in range(N):
            payload = prefix0 + i.to_bytes(2, 'big') + b'\x00' * 32
            a.sendall(make_frame(mac_a, rate_code=0xE0, payload=payload,
                                 **chan_kwargs))
        delivered0 = _count_frames_with_prefix(b, prefix0)
        h.expect(delivered0 >= N - 2,
                 f'HE80 MCS0 (margin=+15 dB) delivers ~all frames',
                 f'got {delivered0}/{N}')

        # Batch 2: HE80 NSS=1 MCS9 (rate_code 0xE9, min_snr=30 dB)
        # margin = 20 - 30 = -10 dB, < -3 -> FER = 1 -> all drop
        prefix9 = b'HEMCS9-'
        for i in range(N):
            payload = prefix9 + i.to_bytes(2, 'big') + b'\x00' * 32
            a.sendall(make_frame(mac_a, rate_code=0xE9, payload=payload,
                                 **chan_kwargs))
        delivered9 = _count_frames_with_prefix(b, prefix9)
        h.expect(delivered9 == 0,
                 f'HE80 MCS9 (margin=-10 dB) drops every frame',
                 f'got {delivered9}/{N}')

        # Batch 3: HE80 MCS11 (rate_code 0xEB, min_snr=35 dB, 1024-QAM 5/6)
        # margin = 20 - 35 = -15 dB -> FER = 1
        prefix11 = b'HEMCS11-'
        for i in range(N):
            payload = prefix11 + i.to_bytes(2, 'big') + b'\x00' * 32
            a.sendall(make_frame(mac_a, rate_code=0xEB, payload=payload,
                                 **chan_kwargs))
        delivered11 = _count_frames_with_prefix(b, prefix11)
        h.expect(delivered11 == 0,
                 f'HE80 MCS11 (1024-QAM 5/6, margin=-15 dB) drops every frame',
                 f'got {delivered11}/{N}')
    finally:
        a.close(); b.close()


def test_c1_channel_change_mid_stream(h: Harness):
    h.section('C1: peer that changes channel starts being heard on the new channel')
    h.restart_hub()
    a = h.data_conn()
    b = h.data_conn()
    try:
        mac_a = b'\x02\x00\x00\x00\x60\xaa'
        mac_b = b'\x02\x00\x00\x00\x60\xbb'

        # A starts on ch1, B starts on ch6.
        _tune(a, mac_a, channel_freq=CH1,
              channel_flags=CHF_2GHZ | CHF_HT20)
        _tune(b, mac_b, channel_freq=CH6,
              channel_flags=CHF_2GHZ | CHF_HT20)
        _drain_all([a, b])

        # Frame 1: A still on ch1 -> B must NOT receive.
        m1 = b'CHANGE-PRE-CH1-' + os.urandom(16)
        a.sendall(make_frame(mac_a, channel_freq=CH1,
                             channel_flags=CHF_2GHZ | CHF_HT20,
                             payload=m1.ljust(64, b'\x00')))
        got1, _ = _recv_with_marker(b, m1, timeout=0.3)
        h.expect(not got1, 'B does not hear A while A is on ch1')

        # A switches to ch6 (next TX carries the new channel info).
        m2 = b'CHANGE-POST-CH6-' + os.urandom(16)
        a.sendall(make_frame(mac_a, channel_freq=CH6,
                             channel_flags=CHF_2GHZ | CHF_HT20,
                             payload=m2.ljust(64, b'\x00')))
        got2, _ = _recv_with_marker(b, m2, timeout=0.5)
        h.expect(got2, 'B hears A after A switches to ch6')
    finally:
        a.close(); b.close()


def test_h1h2_h7_node_recycling(h: Harness):
    h.section('H1+H2 / H7: node and peer recycling under churn')
    h.restart_hub()
    # 5 cycles of 20 connects; each connection sends one frame to be
    # auto-bound, then disconnects. After all cycles, the node table
    # should not have 100 orphaned active entries.
    for cycle in range(5):
        conns = [h.data_conn() for _ in range(20)]
        for i, c in enumerate(conns):
            mac = bytes([0x02, 0, 0, cycle, 0, i])
            c.sendall(make_frame(mac, channel_freq=0))
        time.sleep(0.1)
        for c in conns:
            c.close()
        time.sleep(0.1)

    # The hub processes disconnects asynchronously on the next poll
    # cycle. Re-query LIST_PEERS for up to 2 s until the "(N peers
    # online)" header reports zero so the test isn't racy against
    # poll scheduling.
    import re
    pattern = re.compile(r'\((\d+) peers online\)')
    deadline = time.time() + 2.0
    online = -1
    r = ''
    while time.time() < deadline:
        r = h.ctl_cmd('LIST_PEERS')
        m = pattern.search(r)
        online = int(m.group(1)) if m else -1
        if online == 0:
            break
        time.sleep(0.1)

    h.expect(online == 0,
             'all churned peers are offline after disconnect',
             f'still {online} marked online after 2 s')
    header = r.splitlines()[0] if r else ''
    h.expect('nodes' in header,
             'LIST_PEERS header has node count', repr(header))


def test_b_bridge_trunk_not_channel_filtered(h: Harness):
    h.section('B: a TCP bridge (multi-channel trunk) is not channel-filtered')
    h.restart_hub(tcp_port=54330)
    a = h.data_conn()       # local single-radio peer on ch1
    br = h.bridge_conn()    # TCP peer == inter-hub bridge (is_bridge=1)
    try:
        mac_a  = b'\x02\x00\x00\x04\x00\xaa'
        mac_br = b'\x02\x00\x00\x04\x00\xbb'

        # The bridge last relayed a ch36 frame, so the hub caches ch36 as
        # "the bridge's channel". A real trunk carries every channel, so
        # this cached value must NOT gate what the bridge receives.
        br.sendall(make_frame(mac_br, channel_freq=CH36,
                              channel_flags=CHF_5GHZ | CHF_HT20))
        # A is a single radio on ch1.
        _tune(a, mac_a, channel_freq=CH1, channel_flags=CHF_2GHZ | CHF_HT20)
        _drain_all([a, br])

        # A transmits on ch1. The bridge is on a different cached channel
        # (ch36) but must still receive it -- downstream hubs do their own
        # per-receiver filtering. Pre-fix, the frame was dropped at the
        # trunk and cross-channel multi-hub traffic vanished.
        marker = b'BRIDGE-TRUNK-CH1-' + os.urandom(16)
        a.sendall(make_frame(mac_a, channel_freq=CH1,
                             channel_flags=CHF_2GHZ | CHF_HT20,
                             payload=marker.ljust(64, b'\x00')))

        got, n = _recv_with_marker(br, marker, timeout=0.5)
        h.expect(got,
                 'bridge receives a ch1 frame despite its cached ch36 channel',
                 f'received {n} bytes; marker not found')
    finally:
        a.close(); br.close()


def test_phys_bridge_exempt_from_physics(h: Harness):
    h.section('Bridge: a physical-radio peer is exempt from the simulated '
              'physics model')
    h.restart_hub()
    ap = h.data_conn()
    physbr = h.data_conn()
    normal = h.data_conn()
    try:
        mac_ap = b'\x02\x00\x00\x05\x00\xaa'
        mac_pb = b'\x02\x00\x00\x05\x00\xbb'
        mac_nm = b'\x02\x00\x00\x05\x00\xcc'

        # Name the nodes via hello; the bridge announces itself physical.
        ap.sendall(make_hello('apnode'))
        physbr.sendall(make_hello('physbr', physical=True))
        normal.sendall(make_hello('normalnode'))

        # Reveal channels (all ch6 HT20) so the channel filter passes.
        _tune(ap,     mac_ap, channel_freq=CH6, channel_flags=CHF_2GHZ | CHF_HT20)
        _tune(physbr, mac_pb, channel_freq=CH6, channel_flags=CHF_2GHZ | CHF_HT20)
        _tune(normal, mac_nm, channel_freq=CH6, channel_flags=CHF_2GHZ | CHF_HT20)

        # 100% simulated loss from the AP toward both receivers. Only the
        # physical peer is exempt and should still hear the frame; the
        # ordinary peer is the control that proves the loss link is live.
        r1 = h.ctl_cmd('SET_LOSS apnode physbr 1.0')
        r2 = h.ctl_cmd('SET_LOSS apnode normalnode 1.0')
        h.expect('OK' in r1 and 'OK' in r2, 'both 100% loss links set',
                 repr((r1[:60], r2[:60])))
        _drain_all([ap, physbr, normal])

        marker = b'PHYS-EXEMPT-' + os.urandom(16)
        ap.sendall(make_frame(mac_ap, channel_freq=CH6,
                              channel_flags=CHF_2GHZ | CHF_HT20,
                              payload=marker.ljust(64, b'\x00')))

        got_pb, _ = _recv_with_marker(physbr, marker, timeout=0.5)
        got_nm, _ = _recv_with_marker(normal, marker, timeout=0.5)
        h.expect(got_pb,
                 'physical-radio peer receives frame despite 100% simulated loss')
        h.expect(not got_nm,
                 'non-physical peer is dropped by the 100% loss link (control)')
    finally:
        ap.close(); physbr.close(); normal.close()


def test_hub_survives_garbage(h: Harness):
    h.section('Defensive: hub survives garbage on data socket')
    s = h.data_conn()
    try:
        # Bad length prefix (huge) - hub should disconnect peer, stay up
        s.sendall(b'\xff\xff\xff\xff' + b'A' * 32)
        time.sleep(0.1)
        h.expect(h.hub.poll() is None,
                 'hub survives oversized length prefix',
                 'hub exited')
    finally:
        s.close()

    s = h.data_conn()
    try:
        # Truncated header
        s.sendall(b'\x00\x00\x00\x10' + b'\x00' * 8)
        time.sleep(0.1)
        h.expect(h.hub.poll() is None,
                 'hub survives truncated message',
                 'hub exited')
    finally:
        s.close()


# =====================================================================
# Main
# =====================================================================
TESTS = [
    test_c4_socket_perms,
    test_list_peers_empty,
    test_l6_tx_drop_column,
    test_c3_maclist_capacity,
    test_c2_load_config_recursion,
    test_h5_quit_in_load_config,
    test_h5_orig_large_response,
    # Channel-filter family
    test_c1_channel_mismatch_drops,
    test_c1_channel_match_delivers,
    test_c1_v1_broadcast,
    test_c1_ht40_plus_minus_dont_collide,
    test_c1_ht40_to_ht20_delivers,
    test_c1_ht40_match_delivers,
    test_c1_multichannel_fanout_isolation,
    test_c1_vht80_center_mismatch_drops,
    test_c1_vht80_center_match_delivers,
    # A: wide-channel bond-encoding mismatch (VM vs phys-bridge)
    test_a_vht80_mixed_bond_delivers,
    test_a_vht80_mixed_bond_center_mismatch_still_drops,
    # HE / VHT160 / VHT80+80 + rate-table physics
    test_c1_he80_center_match_delivers,
    test_c1_he80_center_mismatch_drops,
    test_c1_he40_to_he20_delivers,
    test_c1_vht160_center_match_delivers,
    test_c1_vht160_center_mismatch_drops,
    test_c1_vht80_80_center2_match_delivers,
    test_c1_vht80_80_center2_mismatch_drops,
    test_m3_he_mcs_rate_thresholds,
    test_c1_channel_change_mid_stream,
    # B: inter-hub bridge trunk must not be channel-filtered
    test_b_bridge_trunk_not_channel_filtered,
    # Physical bridge: real-radio links bypass the simulated physics
    test_phys_bridge_exempt_from_physics,
    # Resource lifecycle + defensive
    test_h1h2_h7_node_recycling,
    test_hub_survives_garbage,
]


def main():
    h = Harness()
    try:
        h.start_hub()
        for fn in TESTS:
            try:
                fn(h)
            except Exception as e:
                h.failed += 1
                h.failures.append((fn.__name__, f'crash: {e!r}'))
                print(f'   CRASH {fn.__name__}: {e!r}')
                # If the hub died, restart so subsequent tests can run
                if h.hub is None or h.hub.poll() is not None:
                    print('   (hub died; restarting)')
                    h.start_hub()
    finally:
        rc = h.report()
        h.cleanup()
    sys.exit(rc)


if __name__ == '__main__':
    main()
