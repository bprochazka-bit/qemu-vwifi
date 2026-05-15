# qemu-vwifi regression harness

`harness.py` is a self-contained Python test runner for the audit
fixes on the `claude/802-11-ack-packet-drops-qTLMc` branch. It spawns
`./ath9k_hub` against temporary sockets and exercises everything that
can be verified in userspace, no kernel module or QEMU VM required.

## Run

```
make userspace      # builds ath9k_hub + ath9k_host_relay + ath9k_phys_bridge
make test           # runs python3 tests/harness.py
```

Exit code is `0` on all-pass, `1` if any test fails, `2` for setup
errors. Failed tests print the assertion context plus the path to
the hub's stderr log in the temp directory.

## What's covered (userspace only)

| Test                  | Fix       | What it proves                                    |
|-----------------------|-----------|---------------------------------------------------|
| socket permissions    | C4        | control 0600, data 0666                           |
| LIST_PEERS empty      | -         | hub starts clean                                  |
| tx_drop column        | L6        | new column rendered                               |
| maclist capacity      | C3        | MAC list block isn't truncated                    |
| LOAD_CONFIG recursion | C2        | self-referential config refused, hub survives     |
| QUIT in LOAD_CONFIG   | H5        | -1 propagates, post-QUIT lines skipped            |
| large LIST_PEERS      | H5-orig   | 120-node dump arrives intact (POLLOUT drain)      |
| channel mismatch      | C1        | ch1 sender to ch11 receiver -> dropped            |
| channel match         | C1        | ch1 -> ch1 delivered                              |
| V1 broadcast          | C1        | channel_freq=0 still broadcasts                   |
| node recycling        | H1+H2/H7  | 100 connect/disconnect cycles leave 0 online      |
| garbage tolerance     | -         | bad length / truncated frames don't kill hub      |

## What's NOT covered (needs a real kernel + VM)

These three fixes touch the kernel driver or relay's chardev interaction
and can't be exercised from userspace alone:

**H2 — driver accepts V1 headers.** The hub already emits V2; only an
older V1 sender (or a forced `ATH9K_MEDIUM_VERSION=1` build) triggers
the original bug. To validate:

```
sudo insmod ath9k_medium_host.ko
./ath9k_host_relay /dev/ath9k_medium /tmp/medium.sock
# Run a V1-emitting peer (or patch the hub to downgrade outgoing
# headers); confirm dmesg has no "rx: short message" warnings and
# that the VM still receives frames.
```

**H3 — driver locking.** Needs the module loaded with
`CONFIG_PROVE_LOCKING=y` (and ideally `CONFIG_KASAN=y`). Repeatedly
attach/detach the relay while TX is hot:

```
for i in $(seq 1 50); do
  killall ath9k_host_relay
  ./ath9k_host_relay /dev/ath9k_medium /tmp/medium.sock &
  sleep 0.5
done
sudo dmesg | grep -iE "BUG|lockdep|kasan"   # should be empty
```

**H6 — relay buffer sizing for jumbo AMSDU.** Needs the kernel module
plus a peer that can emit max-size frames (~8 KB). With the previous
buffer sizing, a max-size frame caused `chrdev_read` to return
`-EMSGSIZE` and `kfree_skb` it silently. With the fix, the relay
should forward it and the receiver should see it.

## Extending the harness

Each test is a top-level function `test_*` that takes a `Harness`. To
add one, append it to the `TESTS` list at the bottom of `harness.py`.
The harness handles hub lifecycle (start/restart/cleanup), control-
socket commands (`h.ctl_cmd`), data-socket clients (`h.data_conn`),
and assertion bookkeeping (`h.expect`).

`make_frame()` builds wire-format V2 frames; the existing channel-
filter tests are templates for any new data-path assertion.
