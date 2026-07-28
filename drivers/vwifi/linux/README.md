# vwifi — Linux driver for vwifi-virt

A full-MAC `cfg80211` driver. Builds `vwifi.ko`, which binds to PCI
`1AF4:0E00` and gives the guest a `wlan0` that scans, associates with
WPA2, and passes traffic.

## Status, honestly

| | |
|---|---|
| Compiles | **Yes** — clean at `W=1` against kernel 6.8 headers |
| Run in a guest | **Not yet.** Never loaded, never bound to a live device |
| STA mode: scan, connect, WPA2, data | Implemented |
| Monitor mode | Mode switch works; RX frames are dropped (no radiotap path) |
| SoftAP | Not implemented — the device supports it, the driver does not |

Treat "compiles" as what it is. Every ring interaction below is written
against the device's actual implementation rather than guessed at, but
the first `insmod` is still the first time any of it executes.

## Why full-MAC, not mac80211

The device runs the 802.11 state machine: it scans channels, parses
beacons, does auth/assoc, encrypts with CCMP, and translates
802.3↔802.11. Registering with mac80211 would put a second state machine
on top of one that has already done all of that — two schedulers fighting
over one radio.

So this driver registers `cfg80211_ops` directly. Each operation is one
control opcode; each device event is one cfg80211 notification. That
symmetry is essentially the whole driver:

| cfg80211 | opcode |
|---|---|
| `.scan` | `SCAN` → `EV_BSS_FOUND` ×N, `EV_SCAN_COMPLETE` |
| `.connect` | `CONNECT` → `EV_ASSOC_RESULT` |
| `.disconnect` | `DISCONNECT` → `EV_DISCONNECTED` |
| `.add_key` / `.del_key` | `SET_KEY` / `DEL_KEY` |
| `.change_virtual_intf` | `SET_OP_MODE` |

## Layout

```
vwifi_drv.h        driver-private state; ring and buffer sizes
vwifi_main.c       PCI attach, MMIO, MSI-X, the four rings, control transport
vwifi_cfg80211.c   wiphy setup, cfg80211_ops, device events -> notifications
vwifi_net.c        netdev: 802.3 in and out of the TX/RX rings
```

The contract is `../../../abi/vwifi_abi.h`, included verbatim — the same
header the device and the Windows driver compile against. Nothing in it
is restated here. If it changes, everything changes together, which is
the point.

## Two things worth knowing before you change anything

**Ring ownership is asymmetric.** The control-request and TX rings are
driver-produced: fill the descriptor, set `OWN`, ring the doorbell, and
the device clears `OWN` when it consumes. The control-response and RX
rings are the other way round — the *driver* pre-arms slots with a
buffer and `OWN` set, and the device only writes into a slot that has
`OWN` set, clearing it as it does. So on those two rings "OWN clear"
means "completed", and **a slot you forget to re-arm is a slot lost
forever**.

**There is no TX completion interrupt.** The device raises only
`VEC_CTRL_RSP` and `VEC_RX`; it drains the whole TX ring synchronously
on each doorbell write. That is why `ndo_start_xmit` drops on a full
ring instead of calling `netif_stop_queue()` — a stopped queue would
have nothing to wake it and transmit would wedge permanently.

## Building

```bash
sudo apt install linux-headers-$(uname -r)
make                        # against the running kernel
make KDIR=/path/to/kernel   # against another tree
sudo make install           # modules_install + depmod
```

Build it in the guest, or on a host with matching headers and copy
`vwifi.ko` in.

## Using it

Start a hub and a peer to talk to first — see
[`../../../docs/testing-guests.md`](../../../docs/testing-guests.md).
Then, in a guest booted with `-device vwifi-virt`:

```bash
sudo insmod vwifi.ko
dmesg | tail                # expect "vwifi-virt bound: ABI 1, caps 0x..., hub link up"

ip link set wlan0 up
iw dev wlan0 scan | grep SSID

wpa_passphrase Lab-AP-1 correcthorse1 > /tmp/wpa.conf
sudo wpa_supplicant -B -i wlan0 -c /tmp/wpa.conf
iw dev wlan0 link
sudo dhclient wlan0
```

If `insmod` succeeds but no `wlan0` appears, the probe failed — `dmesg`
will say why. The two interesting failures are a bad signature (BAR not
decoding) and an ABI mismatch (device and driver built from different
versions of `vwifi_abi.h`), both of which refuse to bind rather than
limp along.

## What to do first when it misbehaves

Reach for `devices/vwifi/tools/vwifi-probe` before the kernel debugger.
It reads the same registers from userspace with nothing bound, so it
separates "the device is broken" from "the driver is broken" in one
command. If `vwifi-probe` shows `LINK_UP` and healthy counters but the
driver sees nothing, the fault is on this side.

`dev_dbg()` output needs `CONFIG_DYNAMIC_DEBUG`:

```bash
echo 'module vwifi +p' | sudo tee /sys/kernel/debug/dynamic_debug/control
```

## Not done yet

- **Monitor mode RX.** `SET_OP_MODE` works and the device will send raw
  frames with `RX_F_RAW`, but there is no monitor interface or radiotap
  header, so those frames are counted and dropped.
- **SoftAP.** `START_AP`/`STOP_AP` exist in the ABI and the device
  implements them; no `.start_ap` here yet.
- **Management frame TX/RX to userspace.** Needed for SAE (WPA3). The
  device already reports `EV_MGMT_RX`; the handler is a stub.
- **Multiple interfaces.** One netdev per device, one device per wiphy.
