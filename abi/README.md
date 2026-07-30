# abi — the shared contracts

Three headers, each the single authoritative definition of an interface
that more than one component depends on. Nothing here is compiled on its
own; everything here is included by something that is.

| Header | Contract between | Consumers |
|---|---|---|
| `vwifi.h` | Every peer on the medium | `medium/`, `host/`, `devices/ath9k/`, `devices/vwifi/` |
| `vwifi_abi.h` | The `vwifi-virt` device and its guest driver | `devices/vwifi/`, `drivers/vwifi/` |
| `vwifi_host_ioctl.h` | `vwifi_host.ko` and its controllers | `host/` |

## Why one copy

This is not tidiness. It is a bug that has already happened twice in
this codebase.

**The device and the driver drifted.** During development of
`vwifi-virt`, `vwifi_abi.h` existed in both `device/src/` and
`driver/src/`. The device gained `VWIFI_BSS_F_BEACON` and the driver's
copy did not. A missing define fails to compile, which is the *good*
outcome; a changed field offset compiles fine and produces silent memory
corruption across the guest/host boundary.

**The device and the medium drifted.** `vwifi-virt` was written against
a hand-transcribed stub of the medium header, in a tree that didn't have
the real one. The stub got the struct layout right and the magic value
wrong, and byte-swapped the hello. Both are invisible at runtime: the
hub simply never registers the peer, no error is logged anywhere, and
frames vanish. `devices/vwifi/tests/medium_proto.c` now asserts the wire
format field by field precisely because nothing else would have caught
it.

Consolidating these repositories was largely about making that class of
bug structurally impossible. Every build here reaches into this
directory; nobody copies out of it.

## How each consumer picks it up

| Consumer | Mechanism |
|---|---|
| Top-level Makefile (hub, relay, ctl, bridges) | `-I$(TOPDIR)/abi` |
| Host kernel module | `ccflags-y += -I$(src)/../abi` |
| QEMU devices | `scripts/integrate.sh` copies the headers into the QEMU tree |
| Windows driver (MSBuild) | add `..\..\..\abi` to *Additional Include Directories* |
| Device unit tests | `-I../../../abi` |

The QEMU devices copy rather than include across the tree boundary
because a QEMU source tree has to build standalone once integrated — it
cannot depend on a path outside itself. `integrate.sh` is the only thing
that makes those copies, and re-running it refreshes them. **Never edit
the copy inside a QEMU tree.**

## Rules for changing a contract

1. Never remove or reorder a field in an existing struct.
2. Add new fields only at the end, or into an existing `_reserved` hole.
3. New opcodes get fresh numbers. Deprecated opcodes stay allocated and
   return `-ENOSYS`.
4. Bump the version on any layout change — `VWIFI_ABI_VERSION` for
   `vwifi_abi.h`, `VWIFI_VERSION` for the medium protocol. The guest
   driver reads `VWIFI_REG_ABI_VERSION` at init and refuses to bind on
   mismatch; that check is the last line of defence against exactly the
   drift described above.
5. Changing `vwifi.h` means rebuilding *and reloading* every peer. A hub
   that speaks v2 and a module that speaks v1 will run, exchange
   nothing useful, and log nothing about it.

## Byte order, once, clearly

On the medium, only the outer message length prefix is in network byte
order. Everything else — the frame header's fields, the hello magic —
goes out in **host** byte order, because the hub compares and copies
them raw. Sending `htonl(VWIFI_HELLO_MAGIC)` on a little-endian host
produces a hello the hub silently ignores. This has bitten one
implementation already.

## Why the host ioctls are separate

`vwifi_host_ioctl.h` was split out of `vwifi.h` so the wire protocol
carries no Linux-only includes. QEMU device models include `vwifi.h`,
and QEMU builds on Windows and macOS hosts, where `<sys/ioctl.h>` does
not exist. Only `vwifi_host.ko` and `vwifi-ctl` need the ioctl ABI.
