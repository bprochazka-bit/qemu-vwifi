# Linux driver for vwifi-virt — planned

Nothing here yet. This file records the shape of the port so it can be
picked up without re-deriving the decisions.

## Do you actually want this?

For most Linux guests: no. Use
[`vwifi-ath9k`](../../../devices/ath9k/) instead — the guest needs no
driver at all, because Linux has shipped `ath9k` since 2009. That is
strictly less work than anything described below.

This port earns its keep in three cases:

- **Testing the `vwifi_abi.h` contract from a second implementation.**
  A Linux driver is far cheaper to iterate on than the Windows one, and
  a second consumer of the ABI finds specification gaps that a single
  consumer never will.
- **Guests where `ath9k` is unavailable** — a stripped or embedded
  kernel with no `CONFIG_ATH9K`.
- **Exercising device-owned 802.11.** `vwifi-ath9k` puts the state
  machine in mac80211; `vwifi-virt` puts it in the device. Comparing the
  two on one medium is a useful thing to be able to do.

## Shape of the port

The device owns scanning, association, key management and framing, so
this is a **full-MAC** driver: register `cfg80211_ops` directly, do not
pull in mac80211. mac80211 would insist on running a state machine the
device has already run.

```
  cfg80211
     |  .scan, .connect, .add_key, .start_ap, ...
  vwifi_virt.ko
     |  MMIO + MSI-X + 4 DMA rings
  vwifi-virt (QEMU)
```

Rough mapping — the device side of each is already implemented and
tested in `devices/vwifi/`:

| `cfg80211_ops` | Control opcode |
|---|---|
| `.scan` | `VWIFI_OP_SCAN`, then `VWIFI_EV_SCAN_COMPLETE` + BSS entries |
| `.connect` | `VWIFI_OP_CONNECT` → `VWIFI_EV_ASSOC_RESULT` |
| `.disconnect` | `VWIFI_OP_DISCONNECT` |
| `.add_key` / `.del_key` | `VWIFI_OP_SET_KEY` / `VWIFI_OP_DEL_KEY` |
| `.set_monitor_channel` | `VWIFI_OP_SET_CHANNEL` + `SET_OP_MODE` |
| `.start_ap` / `.stop_ap` | `VWIFI_OP_START_AP` / `VWIFI_OP_STOP_AP` |

The data path is a `netdev` moving 802.3 frames over the TX and RX
rings. Monitor mode is the exception: raw 802.11 with radiotap, gated by
`VWIFI_OP_SET_RAW_FILTER`.

## Suggested order

1. PCI probe, BAR0 map, signature and ABI-version check, MSI-X.
2. Ring allocation and the control request/response path. `GET_CAPS`
   round-tripping is the first real milestone.
3. `cfg80211` registration with scan only — `iw dev wlanX scan` against
   a hub with a SoftAP peer on it.
4. Connect, key install, and the data path.
5. Monitor mode and injection.
6. SoftAP.

## Before writing any of it

Read [`../../../abi/vwifi_abi.h`](../../../abi/vwifi_abi.h) — it is the
contract, and it is the specification. Then run
`make -C devices/vwifi test`: the device core is driven end to end
against a mock backend there, and those tests are the clearest available
statement of what the device does with each opcode.
