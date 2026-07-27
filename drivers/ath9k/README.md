# ath9k guest driver — there isn't one, and that's the point

`vwifi-ath9k` emulates an Atheros AR9285 (PCI `168C:002B`). Linux has
shipped a driver for that chip since 2009. A stock guest image boots,
`ath9k` binds, and there is Wi-Fi — nothing to install, nothing to sign,
nothing to keep in step with a kernel version.

That constraint is the whole design. Every register the device
implements exists because the real driver reads or writes it; the
device's job is to be indistinguishable from silicon, not to be
convenient.

## Checking it bound

```
$ lspci -nn | grep -i atheros
00:04.0 Network controller [0280]: Qualcomm Atheros AR9285 [168c:002b]

$ dmesg | grep ath9k
ath9k 0000:00:04.0: Atheros AR9285 Rev:2 mem=0x..., irq=11

$ iw dev
        Interface wlan0
                type managed
```

If `lspci` shows the device but `dmesg` shows no `ath9k`, the guest
kernel lacks the module — `modprobe ath9k`, or use a kernel built with
`CONFIG_ATH9K=m`. If `ath9k` loads and then errors during
`ath9k_hw_init()`, that is a device bug, not a guest one: see
[`../../devices/ath9k/README.md`](../../devices/ath9k/README.md) and
the register trace it describes how to capture.

## What the guest side owns

Because this is real-silicon emulation, the 802.11 state machine lives
in the guest, in mac80211 — the opposite of `vwifi-virt`. Scanning,
association, key management and rate control are the guest kernel's
work. The device provides descriptor DMA, hardware crypto (WEP, TKIP,
CCMP), A-MPDU aggregation with Block-Ack, and the medium transport.

Anything you would do to a real ath9k card works here: `iw`, `hostapd`,
`wpa_supplicant`, monitor mode, injection, mesh.

## Other operating systems

Windows and macOS have no usable in-box AR9285 driver, and writing one
would mean implementing the full 802.11 state machine against an
emulated 2009 register interface. That is why `vwifi-virt` exists — see
[`../vwifi/`](../vwifi/). For Linux guests, use this device.
