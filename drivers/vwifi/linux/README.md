# vwifi — Linux driver for vwifi-virt

A full-MAC `cfg80211` driver. Builds `vwifi.ko`, which binds to PCI
`1AF4:0E00` and gives the guest a `wlan0` that scans, associates with
WPA2, and passes traffic.

## Status, honestly

| | |
|---|---|
| Compiles | **Yes** — clean at `W=1` against kernel 6.8 headers |
| Loads in a guest | Fixed after a first attempt failed — see below. Bound-and-working is still unconfirmed |
| Reviewed | Yes — an adversarial pass found 15 issues; the real ones are fixed |
| STA mode: scan, connect, WPA2 (CCMP), data | Implemented |
| Monitor mode + radiotap RX | Implemented |
| Raw frame injection | Implemented |
| SoftAP | Not implemented — the device supports it, the driver does not |

Treat "compiles" as what it is. Every ring interaction is written
against the device's actual implementation rather than guessed at, and
a review pass caught three memory-safety bugs before anyone ran it —
but the first `insmod` is still the first time any of it executes.

### Things the first load attempt caught

- **cfg80211 rejects a single-interface `iface_combination`.**
  `wiphy_verify_combinations()` warns and returns `-EINVAL` for a
  combination with `max_interfaces < 2` and no radar detection —
  "combinations with just one interface aren't real". This driver
  supports one interface at a time, so it declares no combinations at
  all, which is how that is expressed. Symptom was a `WARNING` at
  `net/wireless/core.c:625` and `wiphy_register failed: -22`.
- **`asm/unaligned.h` became `linux/unaligned.h` in 6.12.** Sidestepped
  by not using `put_unaligned_le*()` at all — the two stores it was
  doing are into a local buffer, where a `memcpy` is equivalent and
  carries no version dependency.

### Things the review caught, worth knowing about

Not history for its own sake — each of these is a trap the next person
to touch this code could fall back into.

- **The device controls `payload_len` on every control response.**
  Clamping it to the ring buffer is not enough; the destination is the
  caller's buffer, which is smaller. `ctrl_rsp_cap` now bounds the copy.
  That is the trust boundary, and it is the one place a hostile or buggy
  device gets a memory-corruption primitive.
- **`CTRL_ENABLE` is edge-triggered on the device side.** Ring base
  addresses are latched only on a 0→1 transition, so a driver attaching
  to an already-enabled device (kexec, re-bind, a predecessor that died)
  would have its rings ignored while the device kept DMAing into the
  previous incarnation's memory. Probe now resets first.
- **`free_irq()` does not flush work an interrupt already queued.**
  Both the probe error path and `remove()` have to `cancel_work_sync()`,
  and `remove()` has to quiesce *before* `unregister_netdev()` frees the
  netdev that the work and the RX path both dereference.
- **Do not advertise what the device rejects.** It accepts only
  CCMP-128 keys. Offering WEP/TKIP lets a supplicant negotiate a cipher
  that fails at `.add_key` mid-handshake, leaving an associated but
  unkeyed link and a deauth loop.

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
vwifi_monitor.c    radiotap on RX, radiotap stripping + injection on TX
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

## Installing with DKMS (recommended in a guest)

```bash
sudo apt install dkms linux-headers-$(uname -r)

# from the repository root, or from this directory
sudo make dkms              # stage, add, build, install
sudo modprobe vwifi

sudo make dkms-remove       # unregister and clean /usr/src
```

Use this rather than `make install` in any guest whose kernel gets
updated. A plain `make install` module is built for exactly one kernel
version and silently stops loading after the next upgrade — which
presents as "the Wi-Fi disappeared", with nothing obviously to blame.
DKMS rebuilds it as part of the upgrade.

### The one wrinkle: the shared ABI header

DKMS copies exactly one directory into `/usr/src`, and these sources are
not self-contained — they include `abi/vwifi_abi.h` from the top of the
repository, the same file the QEMU device and the Windows driver compile
against. A plain `dkms add .` would stage a tree that cannot build.

`dkms-install.sh` therefore stages the sources *and* drops a copy of that
header beside them, with a note saying what it is. The copy is a build
artifact of packaging, exactly like the headers `integrate.sh` puts into
a QEMU tree: generated, never edited, replaced on every install. The one
authoritative copy stays in `abi/`.

The Makefile works in both layouts — it adds `-I../../../abi` only when
that directory exists, and the quoted `#include` finds a sibling copy on
its own in the DKMS tree.

The version lives in `dkms.conf`, and `dkms-install.sh` refuses to stage
if it disagrees with `VWIFI_DRV_VERSION` in `vwifi_drv.h`. Two files that
must agree, with nothing else to enforce it.

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

## Debugging an empty scan

`iw dev wlan0 scan` returning nothing has three possible causes, and the
point of the logging below is to tell them apart in one run rather than
three.

Turn on both sides:

```bash
# host: let the DEVICE say what it sees on the medium
-device vwifi-virt,chardev=medium,node_id=virt-guest,verbose=on

# guest: let the DRIVER say what it did with it
echo 'module vwifi +p' | sudo tee /sys/kernel/debug/dynamic_debug/control
sudo iw dev wlan0 scan
dmesg | tail -20
```

Then read it in this order:

| What you see | Where the problem is |
|---|---|
| QEMU log has no `BSS_FOUND` lines | The device never saw a beacon. Nothing is transmitting on the scanned channel, or the hub is not delivering to this peer — check `LIST_PEERS` on the hub's control socket for a second node, and that it shows `mode=AP` on a channel the scan covers |
| QEMU logs `BSS_FOUND`, dmesg says `no BSS reported by the device` | The events are not reaching the driver — a control-response ring problem |
| dmesg says `on unknown freq N MHz -- dropped` | The device is reporting a channel the wiphy does not have. The channel list is built from `GET_CAPS`; compare against `vwifi-probe` output |
| dmesg says `cfg80211 rejected BSS` | The frame reached cfg80211 and it did not like it — usually a malformed beacon |
| `scan complete: no BSS reported` and nothing else | The scan ran and the air was genuinely empty |

The most common answer by far is the first row: **a medium with only one
peer on it has nothing to find.** Something has to be beaconing — the
host radio running hostapd, or another guest in SoftAP mode. See
[`../../../docs/testing-guests.md`](../../../docs/testing-guests.md)
Part 2.

## About `iw dev <dev> info`

**No `txpower` line** was a missing `.get_tx_power` — nl80211 asks the
driver, and a driver that does not implement it gets the field omitted.
Implemented now. Note it is advertised, not enforced: the medium models
propagation from node positions and per-node TX power set on the hub's
control socket, so what the guest asks for does not change how far its
frames actually reach.

**No `TXQ` lines** is correct and will not change. Those come from
mac80211's intermediate software queues, and this is a full-MAC driver —
there is no mac80211 in the path to have them. `ath9k` shows them
because it is a mac80211 driver; this one never will.

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

## Monitor mode and injection

```bash
sudo ip link set wlan0 down
sudo iw dev wlan0 set type monitor
sudo ip link set wlan0 up
sudo iw dev wlan0 set channel 6

sudo tcpdump -i wlan0 -e -n            # every frame on the channel
```

The interface becomes `ARPHRD_IEEE80211_RADIOTAP`, and each frame
arrives with a radiotap header carrying the TSF, rate, channel and RSSI
the device recovered from the medium. Wireshark and tcpdump read it as
an ordinary 802.11 capture.

The mode switch is refused while the interface is up (`-EBUSY`). That is
deliberate: libpcap latches the link type at open, so changing it under
a running capture yields frames that decode as the wrong protocol rather
than an error.

**Injection** works on the same interface — write a radiotap header
followed by an 802.11 frame to a packet socket, and the driver strips
the radiotap and marks the descriptor `VWIFI_TX_F_INJECT`. The device
then puts the frame on the medium verbatim: no 802.3 translation, no
encryption, and the transmitter address taken from the frame's own
`addr2` rather than the station MAC. Anything `aireplay-ng`, `packetforge-ng`
or a raw socket can build will go out as written.

That is what makes the medium testable from inside a guest: forge a
beacon, a deauth, or a deliberately malformed frame and watch what the
other peers do with it.

### Rate reporting

Full, across legacy, HT and VHT. Radiotap's `RATE` field really is
legacy-only — the spec says it must be absent for HT and above — but
radiotap carries dedicated `MCS` (bit 19) and `VHT` (bit 21) fields for
exactly that case, and the driver emits them.

Nothing is lost on the wire: the medium's rate-code namespace already
encodes bandwidth, spatial streams and MCS index in the code itself
(`abi/vwifi.h`), so it is all recoverable at this end.

| Medium code | Reported as |
|---|---|
| `0x00`–`0x1F` | `RATE`, in 500 kbps units, with the CCK/OFDM channel flag |
| `0x80`–`0x9F` | `MCS`, with MCS 0–15 and 20/40 MHz bandwidth |
| `0xA0`–`0xDF` | `VHT`, with MCS, NSS and 80/160 MHz bandwidth |
| `0xE0`–`0xFB` | `VHT` at 80 MHz — see below |

HE codes report as VHT80. Radiotap's HE field is a six-word structure
whose useful subset needs more than the medium encodes, and a
half-filled HE field is read as wrong data rather than as missing data.
Since these frames are VHT80-equivalent on this medium, that is what
they are reported as. Widening the medium's rate encoding is the
prerequisite for doing better, not a change on this side.

Because which fields are present now varies per frame, the header is
built rather than copied from a fixed struct. Radiotap requires each
field aligned to its own size relative to the start of the header; the
builder is the only thing that moves the cursor, and the resulting
layouts (23 / 26 / 36 bytes for legacy / HT / VHT) were checked against
that rule field by field.

### No FCS, and that is correct

Frames on this medium carry no FCS **by convention, not by accident**:
`vwifi-phys-bridge` strips it from real captures on the way in, and the
QEMU devices never add one — `vwifi-ath9k` explicitly sends
`frame_len - FCS_LEN` to the medium and appends a dummy FCS only on the
guest-facing side, because the `ath9k` driver expects
`RX_INCLUDES_FCS`.

So the radiotap flags do not set `IEEE80211_RADIOTAP_F_FCS`. Claiming
one would point a capture tool at four bytes of payload. Synthesising a
real CRC-32 here would be easy and pointless — it would be a value this
driver computed, not the one a transmitter sent, so it could never fail
and would carry no information.

## Not done yet

- **SoftAP.** `START_AP`/`STOP_AP` exist in the ABI and the device
  implements them; no `.start_ap` here yet.
- **Management frame TX/RX to userspace.** Needed for SAE (WPA3). The
  device already reports `EV_MGMT_RX`; the handler is a stub.
- **Multiple interfaces.** One netdev per device, one device per wiphy.
