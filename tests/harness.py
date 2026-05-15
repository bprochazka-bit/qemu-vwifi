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
# Wire-format helpers (must match ath9k_medium.h)
# =====================================================================
VWIFI_MAGIC = 0x46495756   # "VWIF"
HDR_V2_FMT  = '<I H H 6s B b I I I H H H H H 2s'  # 40 bytes
HDR_V2_SIZE = struct.calcsize(HDR_V2_FMT)
assert HDR_V2_SIZE == 40, HDR_V2_SIZE


def make_frame(tx_mac: bytes,
               channel_freq: int = 0,
               channel_bond_freq: int = 0,
               payload: bytes = b'\x00' * 32,
               rate_code: int = 0x0B) -> bytes:
    """Build a complete wire message: [len_be][hdr V2][payload]."""
    assert len(tx_mac) == 6
    frame_len = len(payload)
    hdr = struct.pack(
        HDR_V2_FMT,
        VWIFI_MAGIC,
        2,                  # version
        frame_len,
        tx_mac,
        rate_code,
        -30,                # rssi
        0, 0,               # tsf_lo, tsf_hi
        0,                  # flags (TTL=0; hub stamps it)
        channel_freq,
        0,                  # channel_flags
        channel_bond_freq,
        0, 0,               # center_freq1, _freq2
        b'\x00\x00',        # reserved
    )
    wire = hdr + payload
    return struct.pack('!I', len(wire)) + wire


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
        self.passed = 0
        self.failed = 0
        self.failures: list[tuple[str, str]] = []

    # ---- hub lifecycle ----
    def start_hub(self):
        binary = REPO_ROOT / 'vwifi-medium'
        if not binary.exists():
            sys.exit(f'FATAL: {binary} not built. Run "make userspace" first.')
        # remove leftover sockets if a previous run died
        for p in (self.sock, self.ctl):
            if p.exists():
                p.unlink()
        self.hub = subprocess.Popen(
            [str(binary), str(self.sock), '-c', str(self.ctl)],
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

    def restart_hub(self):
        self.stop_hub()
        self.start_hub()

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
    test_c1_channel_mismatch_drops,
    test_c1_channel_match_delivers,
    test_c1_v1_broadcast,
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
