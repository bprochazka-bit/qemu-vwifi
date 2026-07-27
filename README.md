# qemu-vwifi — a virtual 802.11 medium for QEMU, and the radios on it

One shared virtual radio medium, several virtual radios that attach to
it, and the guest drivers that make those radios look real to the OS
inside the VM.

A guest sees a wireless NIC. It scans, associates, encrypts, roams —
the whole 802.11 state machine — against other VMs, against the host's
own `wlanX`, and optionally against a real radio bridged in over the
air. No RF hardware required for any of it.

## The four layers

```
 drivers/    what the guest OS binds to        vwifi.sys (Windows), in-tree ath9k (Linux)
     |
 devices/    what QEMU exposes on the PCI bus  vwifi-virt, vwifi-ath9k
     |
 medium/     the shared air                    vwifi-medium (the hub)
     |
 host/       the host's own radio on that air  vwifi_host.ko + relay + ctl

 abi/        the contracts every layer agrees on
```

Each layer talks to the next through exactly one contract, and each
contract has exactly one copy, in `abi/`. That is the rule the layout
exists to enforce — see [`abi/README.md`](abi/README.md) for why.

## Repository layout

| Path | What lives there |
|---|---|
| [`abi/`](abi/) | The shared contracts. `vwifi.h` (medium wire protocol), `vwifi_abi.h` (vwifi-virt device ↔ guest driver), `vwifi_host_ioctl.h` (host module control) |
| [`medium/`](medium/) | The hub (`vwifi-medium`), the tools that attach to it (phys bridge, linkbench), the web controller, and the regression harness |
| [`host/`](host/) | `vwifi_host.ko` — mac80211 radios for the host machine — plus `vwifi-host-relay` and `vwifi-ctl` |
| [`devices/ath9k/`](devices/ath9k/) | `vwifi-ath9k`: a QEMU model of an Atheros AR9285, driven by the unmodified in-tree Linux `ath9k` driver |
| [`devices/vwifi/`](devices/vwifi/) | `vwifi-virt`: a QEMU device with a purpose-built ABI, for guests where no suitable real NIC can be emulated |
| [`drivers/vwifi/`](drivers/vwifi/) | Guest drivers for `vwifi-virt` — Windows today, Linux and macOS planned |
| [`docs/`](docs/) | Development plans and lab notes |
| [`scripts/`](scripts/), [`examples/`](examples/) | Lab bring-up helpers and sample hostapd configs |

## The two devices, and why there are two

They are not redundant. They solve different problems, and both attach
to the same medium at the same time.

**`vwifi-ath9k`** emulates real silicon (Atheros AR9285, PCI ID
168C:002B). Its whole point is that the guest needs *no special driver*:
Linux already ships `ath9k`, so a stock distro image boots and finds
Wi-Fi. The device therefore has to be faithful at the register level —
EEPROM layout, reset sequencing, descriptor DMA, hardware crypto.

**`vwifi-virt`** emulates nothing. It exposes a clean ring-and-opcode
ABI of our own design, and the guest needs a driver we write. That is
the right trade when no emulatable real NIC has a usable driver in the
target OS — Windows being the case that motivated it. Because the ABI
is ours, the device can own the 802.11 state machine (scan, assoc,
CCMP, SoftAP) and the guest driver stays thin. That in turn means the
interesting logic is testable without ever booting the guest.

| | `vwifi-ath9k` | `vwifi-virt` |
|---|---|---|
| Guest driver | in-tree Linux `ath9k`, unmodified | ours ([`drivers/vwifi/`](drivers/vwifi/)) |
| Guest OS | Linux | Windows now; Linux and macOS planned |
| 802.11 state machine | in the guest (mac80211) | in the device |
| Testable without a guest | crypto and A-MPDU units | the entire device core |

## Architecture

```
 ┌─────────────── Host machine ────────────────────────────────┐
 │                                                             │
 │  hostapd / wpa_supplicant / iw / NetworkManager             │
 │        │                                                    │
 │   mac80211 (wlanX)                                          │
 │        │                                                    │
 │   vwifi_host.ko          ← host/                            │
 │        │  /dev/<radio>                                      │
 │   vwifi-host-relay       ← host/tools/                      │
 │        │                                                    │
 └────────┼────────────────────────────────────────────────────┘
          │ unix socket
    vwifi-medium            ← medium/     the shared air:
          │                                channel-aware, per-link SNR,
          │                                frame-error model, airtime survey
    ┌─────┼───────────────┬────────────────────┐
    │     │               │                    │
 ┌──▼──┐ ┌▼─────┐   ┌─────▼──────┐   ┌─────────▼─────────┐
 │Linux│ │Windows│   │ other VMs │   │ vwifi-phys-bridge │
 │ VM  │ │  VM   │   │           │   │ (real radio in    │
 │     │ │       │   │           │   │  monitor mode)    │
 │ath9k│ │vwifi. │   │           │   └───────────────────┘
 │ ↑   │ │ sys ↑ │   │           │
 │vwifi│ │vwifi- │   │           │
 │-ath9k│ │virt  │   │           │
 └─────┘ └───────┘   └───────────┘
```

Every peer on the medium speaks the same 40-byte v2 frame header from
`abi/vwifi.h`. The hub filters by channel (including HT40 bonding and
VHT/HE center frequencies), models per-link SNR and frame error, and
charges airtime per channel. A peer that registers as *physical* — the
phys bridge — is exempt from the simulated propagation model, because
real-world RF is already the channel.

## Quick start

Nothing here needs QEMU, a guest image, or kernel headers.

```bash
make            # userspace binaries -> build/
make test       # medium harness + both devices' unit tests
```

`make test` runs the hub's 58-assertion regression harness and both
device cores against mock backends — scan, association, WPA2/CCMP,
monitor mode, SoftAP, A-MPDU, WEP/TKIP/CCMP known-answer vectors.

Then bring up a medium and put the host on it:

```bash
./build/vwifi-medium /tmp/vwifi.sock -c /tmp/vwifi.ctl   # the air

make module && sudo insmod host/vwifi_host.ko            # host radio
sudo ./build/vwifi-ctl create --dev vwifi-lab --ifname vwm-lab
sudo ./build/vwifi-host-relay /tmp/vwifi.sock /dev/vwifi-lab
```

`vwm-lab` is now a full mac80211 radio: hostapd, wpa_supplicant,
monitor mode, mesh and ad-hoc all work on it, and everything it
transmits reaches every VM on the medium.

To put a guest on the medium, build QEMU with one of the devices and
point it at the same socket:

```bash
make -C devices/ath9k integrate build QEMU_SRC=/path/to/qemu   # Linux guests
make -C devices/vwifi integrate build QEMU_SRC=/path/to/qemu   # Windows guests

qemu-system-x86_64 -machine q35 -m 2048 -drive file=vm.qcow2,format=qcow2 \
  -chardev socket,id=medium,path=/tmp/vwifi.sock,server=off \
  -device vwifi-ath9k,chardev=medium,node_id=linux-vm
```

## Where to go next

| You want to | Read |
|---|---|
| Run the hub, drive it at runtime, bridge hosts or a real radio | [`medium/README.md`](medium/README.md) |
| Give the host machine its own radios | [`host/README.md`](host/README.md) |
| Build the Linux (AR9285) device | [`devices/ath9k/README.md`](devices/ath9k/README.md) |
| Build the vwifi-virt device | [`devices/vwifi/README.md`](devices/vwifi/README.md) |
| Write or build a guest driver | [`drivers/README.md`](drivers/README.md) |
| Change anything on the wire | [`abi/README.md`](abi/README.md) |
| Watch a medium in a browser | [`medium/controller/README.md`](medium/controller/README.md) |

## License

GPL-2.0-or-later, matching QEMU and the Linux kernel.
