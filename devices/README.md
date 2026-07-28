# devices — QEMU device models

Each subdirectory is one PCI device a guest can be given. Both attach to
the same medium, at the same time, in the same lab.

| Device | Directory | Guest driver | For |
|---|---|---|---|
| `vwifi-ath9k` | [`ath9k/`](ath9k/) | in-tree Linux `ath9k`, unmodified | Linux guests, stock images |
| `vwifi-virt` | [`vwifi/`](vwifi/) | ours, see [`../drivers/vwifi/`](../drivers/vwifi/) | Windows guests; Linux and macOS planned |

## The split

`vwifi-ath9k` emulates real silicon. Everything it does is constrained
by what the AR9285's driver expects to find — register semantics,
EEPROM layout, reset sequencing, descriptor rings, hardware crypto. The
payoff is that a stock Linux guest needs nothing installed.

`vwifi-virt` emulates nothing. Its ABI is designed rather than
discovered, so the device can own the 802.11 state machine and the guest
driver can stay thin. The payoff is that the interesting logic — scan,
association, CCMP, SoftAP — is testable on the host with no guest, no
QEMU, and no driver SDK in the loop.

## Both are built inside a QEMU tree

Neither device builds standalone; QEMU's Meson build compiles them. Each
has an `integrate.sh` that copies its sources plus the shared `abi/`
headers into `hw/net/<device>/` inside a QEMU source tree and patches
that tree's `meson.build` and `Kconfig`. Both scripts are idempotent.

Drive it from the repository root, which handles one device or both in a
single QEMU build:

```bash
make qemu QEMU_SRC=/path/to/qemu                # both
make qemu QEMU_SRC=/path/to/qemu DEVICES=ath9k  # one
make qemu-help                                  # every target and variable
```

They share one build directory and one output binary — each device has
its own subdirectory and its own Kconfig symbol, so they never collide,
and you choose per VM at runtime with `-device`.

`make qemu-install` will not overwrite an existing QEMU at the install
prefix; `make qemu-upgrade` will. That pair exists because the default
prefix is `/usr/local`, where something else may already have installed
a `qemu-system` binary other things depend on.

Each device Makefile also still works on its own
(`make -C devices/ath9k integrate build QEMU_SRC=...`), with the same
`install`/`upgrade` semantics, if you want to drive one in isolation.

The copies inside the QEMU tree are outputs, not sources. Edit here,
re-run `integrate`.

## Both speak one wire protocol

`abi/vwifi.h`, the v2 medium protocol: a 40-byte header carrying tx MAC,
rate code, RSSI, TSF and the full channel tuple, behind a big-endian
length prefix, over a Unix socket to the hub.

A device that gets that header wrong does not fail loudly — the hub
drops its frames and logs nothing. `devices/vwifi/tests/medium_proto.c`
asserts the format field by field; treat it as the reference for what
"correct on the wire" means, and add a peer to it when you add a peer.

## Attaching a device to a medium

The two devices reach the hub differently, and the command lines are
**not** interchangeable.

`vwifi-ath9k` opens the hub's Unix socket itself, so it takes a path:

```bash
qemu-system-x86_64 ... \
  -device vwifi-ath9k,medium=/tmp/vwifi.sock,node_id=linux-vm
```

`vwifi-virt` goes through a QEMU chardev, so it takes a chardev id:

```bash
qemu-system-x86_64 ... \
  -chardev socket,id=medium,path=/tmp/vwifi.sock,server=off,reconnect-ms=2000 \
  -device vwifi-virt,chardev=medium,node_id=virt-guest
```

`vwifi-ath9k` has no `chardev` property — it registers exactly
`medium`, `macaddr` and `node_id` — so borrowing the second form for it
fails at startup with "Property 'vwifi-ath9k.chardev' not found".

Both survive a hub restart, by different means: `vwifi-ath9k` runs its
own 2-second reconnect timer, and `vwifi-virt` gets the same from the
chardev layer's `reconnect-ms`. Omit `reconnect-ms` and the vwifi-virt
guest stays dead after the hub goes away.

`node_id` is what the peer calls itself in the hub's `LIST_PEERS`,
`SURVEY` and `SET_POS` output. Give every peer a distinct one — an
unnamed peer is very hard to find in a busy lab.
