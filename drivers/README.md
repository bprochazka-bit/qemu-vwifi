# drivers — guest drivers

What runs *inside* the VM and binds to a device from
[`../devices/`](../devices/).

| Device | Guest OS | Driver | Status |
|---|---|---|---|
| `vwifi-ath9k` | Linux | in-tree `ath9k` | Nothing to write — see [`ath9k/`](ath9k/) |
| `vwifi-virt` | Windows 10/11 | [`vwifi/windows/`](vwifi/windows/) | Written, not yet compiled |
| `vwifi-virt` | Linux | [`vwifi/linux/`](vwifi/linux/) | Planned |
| `vwifi-virt` | macOS | [`vwifi/macos/`](vwifi/macos/) | Planned |

## The layout is per-device, then per-OS

```
drivers/
  ath9k/          notes only; the driver ships with Linux
  vwifi/
    windows/      WDI miniport (vwifi.sys)
    linux/        planned
    macos/        planned
```

A device's drivers sit together because they share one thing that
matters more than their OS: `abi/vwifi_abi.h`, the register map, ring
layout and opcode set they must all implement identically. When that
contract changes, every driver under a device changes with it. Grouping
by OS first would scatter that.

## What a `vwifi-virt` driver has to do

Less than you would expect, and that is deliberate. The device owns the
802.11 state machine — scanning, association, CCMP, 802.3↔802.11
framing, SoftAP. A driver:

1. Binds to PCI `1AF4:0E00`, maps BAR0, checks `VWIFI_REG_SIGNATURE` and
   `VWIFI_REG_ABI_VERSION`, and refuses to bind on mismatch.
2. Allocates the four rings (ctrl request, ctrl response, TX, RX) in
   guest memory and programs their base addresses and sizes.
3. Sets up MSI-X and routes the vectors.
4. Translates the OS's Wi-Fi API into control opcodes — `SET_OP_MODE`,
   `SET_CHANNEL`, `SCAN`, `CONNECT`, `SET_KEY`, `START_AP` — and
   translates the resulting events and responses back.
5. Moves 802.3 frames across the TX and RX rings.

That is a plumbing job. The hard parts of 802.11 are already done and
already tested on the host side, which is why a new OS port is a
tractable amount of work rather than a research project.

See [`../abi/README.md`](../abi/README.md) for the contract and the
rules for changing it, and
[`../devices/vwifi/README.md`](../devices/vwifi/README.md) for what the
device does with each opcode.
