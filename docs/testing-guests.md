# Bringing up guests on the medium

How to get Linux and Windows VMs onto a `vwifi-medium` and see the
devices work — and, just as importantly, what each guest can and cannot
show you today.

## Read this first: what actually works

The two devices are at very different stages, and picking the wrong one
for a guest wastes an afternoon.

| | Linux guest | Windows guest |
|---|---|---|
| **`vwifi-ath9k`** | **Proven.** Stock `ath9k` binds, you get a real `wlan0` | No usable in-box driver |
| **`vwifi-virt`** | Driver compiles, **not yet run** — plus a no-driver probe | Driver written, **never compiled** |

So:

- **For a known-good Wi-Fi VM, use `vwifi-ath9k` with a Linux guest.**
  That is the path with mileage on it, and it exercises the whole stack:
  hub, medium protocol, channel model, association, encryption.
- **`vwifi-virt` now has a Linux driver**
  ([`../drivers/vwifi/linux/`](../drivers/vwifi/linux/)) that builds
  clean but has never been loaded. Bringing it up is Part 4 below —
  expect to debug it, and use `vwifi-probe` to tell device faults from
  driver faults.
- **The Windows driver is still the open work item.** It has never been
  through a compiler. Expect to fix member names on the first build —
  see [`../drivers/vwifi/windows/README.md`](../drivers/vwifi/windows/README.md).

---

## Part 1 — Start the medium

Everything attaches to a hub. Start it first: `vwifi-virt` uses a QEMU
chardev, and unless you pass `reconnect-ms`, QEMU refuses to start if
the socket is not already listening.

```bash
cd /opt/projects/qemu-vwifi
make                                              # userspace binaries

./build/vwifi-medium /tmp/vwifi.sock -c /tmp/vwifi.ctl
```

Leave it running. In another terminal you can watch it live:

```bash
echo LIST_PEERS | socat - UNIX-CONNECT:/tmp/vwifi.ctl
echo SURVEY     | socat - UNIX-CONNECT:/tmp/vwifi.ctl
```

`LIST_PEERS` is the single most useful command in this whole system: if
a guest is not listed there, it is not on the medium, and nothing else
you do inside the guest matters.

## Part 2 — Give the medium something to talk to

A lone VM on a medium has nobody to hear. Put the host on it as an AP,
so a guest has something to scan for and associate with.

```bash
sudo apt install linux-headers-$(uname -r) hostapd
make module
sudo insmod host/vwifi_host.ko

sudo ./build/vwifi-ctl create --dev vwifi-ap --ifname vwm-ap
sudo ./build/vwifi-host-relay /tmp/vwifi.sock /dev/vwifi-ap &

sudo hostapd examples/hostapd/lab-ap1.conf
```

`examples/hostapd/lab-ap1.conf` is a WPA2-PSK AP on channel 6,
SSID `Lab-AP-1`, passphrase `correcthorse1`. Point its `interface=` line
at `vwm-ap`.

Check the host radio joined:

```bash
echo LIST_PEERS | socat - UNIX-CONNECT:/tmp/vwifi.ctl
# expect a line with mode=AP chan=6
```

---

## Part 3 — Linux guest with `vwifi-ath9k` (the working path)

### 3a. Build a guest image

The repo ships an image builder with a ready-made Wi-Fi recipe —
Debian trixie with `hostapd`, `wpasupplicant`, `iw` preinstalled:

```bash
sudo apt install qemu-utils debootstrap parted kpartx e2fsprogs
cd devices/ath9k/tools
sudo python3 build_debian_qcow2.py --recipe wifi-1.recipe.json
```

That produces `wifi-1.qcow2` (root password `changeme`, serial console
enabled). Any Debian/Ubuntu cloud image works too, as long as it has
`iw` and `wpasupplicant`.

### 3b. Boot it on the medium

```bash
/usr/local/bin/qemu-system-x86_64 \
  -machine q35 -m 2048 -smp 2 -enable-kvm \
  -drive file=wifi-1.qcow2,format=qcow2,if=virtio \
  -device vwifi-ath9k,medium=/tmp/vwifi.sock,node_id=linux-sta \
  -nographic
```

Note `medium=` — the ath9k device opens the socket itself and
reconnects on its own, so it does not need a `-chardev`.

### 3c. Associate from inside the guest

```bash
ip link set wlan0 up
iw dev wlan0 scan | grep -E 'SSID|signal'      # should show Lab-AP-1

wpa_passphrase Lab-AP-1 correcthorse1 > /tmp/wpa.conf
wpa_supplicant -B -i wlan0 -c /tmp/wpa.conf
iw dev wlan0 link                               # expect "Connected to ..."
```

Back on the host, both peers should now be visible with traffic
counters climbing:

```bash
echo LIST_PEERS | socat - UNIX-CONNECT:/tmp/vwifi.ctl
```

That is the full loop: guest → device → hub → relay → host radio →
hostapd, with real WPA2 in between.

---

## Part 4 — Linux guest with `vwifi-virt`

Two things to do here, in this order: prove the device works with no
driver at all, then load the driver. Doing it in that order means that
when something breaks you already know which half to blame.

### 4a. Build the probe tool

```bash
make -C devices/vwifi probe          # -> devices/vwifi/build/vwifi-probe
```

Copy it into the guest (`scp`, or mount the qcow2 with `guestmount`).

### 4b. Boot with the device attached

`vwifi-virt` takes a `-chardev`, not a path:

```bash
/usr/local/bin/qemu-system-x86_64 \
  -machine q35 -m 2048 -smp 2 -enable-kvm \
  -drive file=wifi-1.qcow2,format=qcow2,if=virtio \
  -chardev socket,id=medium,path=/tmp/vwifi.sock,server=off,reconnect-ms=2000 \
  -device vwifi-virt,chardev=medium,node_id=virt-guest \
  -nographic
```

`reconnect-ms=2000` makes QEMU retry rather than die if the hub is not
up yet. Without it, start the hub first.

### 4c. Probe it

```bash
lspci -nn | grep 1af4:0e00        # device should enumerate
sudo ./vwifi-probe
```

Expected:

```
vwifi-virt found at 0000:00:04.0

-- identity --
  signature       0x46495756  OK
  abi version     1           matches this build
  caps            0x0000031f  MONITOR INJECT STA SOFTAP WPA2 HT VHT
  msix vectors    4

-- state --
  status          0x00000002  LINK_UP
  ctrl            0x00000000
```

What each line proves:

| Line | What it tells you |
|---|---|
| device found | PCI enumeration and the QOM registration work |
| `signature OK` | BAR0 decodes and MMIO reaches the device model |
| `abi version matches` | guest and device agree on `vwifi_abi.h` |
| `LINK_UP` | the chardev is connected to the hub — the medium path is live |
| counters | frames actually crossing the medium |

`STATUS` without `LINK_UP` means the hub socket is not connected: check
the hub is running and the paths match. A signature of `0xFFFFFFFF`
means the BAR is not decoding, not that the device is broken.

Run `sudo ./vwifi-probe -w` while the host AP is beaconing and the RX
counter should climb — the device is parsing beacons off the medium with
no guest driver involved at all.

### 4d. Load the driver

```bash
# In the guest. DKMS so the module survives kernel upgrades:
sudo apt install dkms linux-headers-$(uname -r)
sudo make -C drivers/vwifi/linux dkms
sudo modprobe vwifi

# Or, for a one-off test against the running kernel only:
#   make -C drivers/vwifi/linux && sudo insmod drivers/vwifi/linux/vwifi.ko

dmesg | tail
```

Expect `vwifi-virt bound: ABI 1, caps 0x..., hub link up` and a `wlan0`.
Then the same sequence as the ath9k guest:

```bash
ip link set wlan0 up
iw dev wlan0 scan | grep SSID
wpa_passphrase Lab-AP-1 correcthorse1 > /tmp/wpa.conf
sudo wpa_supplicant -B -i wlan0 -c /tmp/wpa.conf
iw dev wlan0 link
```

This driver has never been run. If it misbehaves, `vwifi-probe` still
works alongside it and tells you whether the device is healthy —
see [`../drivers/vwifi/linux/README.md`](../drivers/vwifi/linux/README.md).

---

## Part 5 — Windows guest

### 5a. What you get without a driver

Install Windows 10/11 normally, then add:

```bash
/usr/local/bin/qemu-system-x86_64 \
  -machine q35 -m 4096 -smp 4 -enable-kvm \
  -drive file=win11.qcow2,format=qcow2,if=virtio \
  -chardev socket,id=medium,path=/tmp/vwifi.sock,server=off,reconnect-ms=2000 \
  -device vwifi-virt,chardev=medium,node_id=win-guest \
  -vga std
```

Device Manager will show an unknown device under "Other devices". Check
its hardware ID is `PCI\VEN_1AF4&DEV_0E00`. That confirms the device
enumerates — and is as far as you get until the driver is built.

### 5b. Building the driver (the real work item)

This has never been compiled. Budget time for it.

1. On a Windows machine or VM, install the **latest WDK** (or the EWDK,
   which needs no Visual Studio install).
2. Create a KMDF driver project and add everything from
   `drivers/vwifi/windows/src/`.
3. Add `..\..\..\abi` to *C/C++ → General → Additional Include
   Directories* so `vwifi_abi.h` resolves.
4. Target `_NT_TARGET_VERSION = 0x0A000008` (NTDDI_WIN10_VB). Do **not**
   use an old WDK to support old Windows — that is backwards for
   drivers; `docs/vwifi-virt-development-plan.md` explains why.
5. Build. Expect to fix a handful of TLV struct member names: the
   message shapes came from the WDK's own `WABIModel.xml`, so the
   structure is right, but how the code generator spells the resulting C
   members could not be checked without a WDK on disk. That is a fixup,
   not a redesign.

### 5c. Installing it in the guest

Windows will not load an unsigned kernel driver. In the guest, as
Administrator:

```
bcdedit /set testsigning on
shutdown /r /t 0
```

Then create a test certificate, sign the `.sys` and `.cat`, and install
via Device Manager → Update Driver → point at the `.inf`
(`drivers/vwifi/windows/inf/vwifi.inx` is preprocessed to `vwifi.inf` at
build time).

For debugging, add a serial pipe to QEMU and attach WinDbg:

```
-serial pipe:/tmp/winkd
```

---

## Troubleshooting

**Guest not in `LIST_PEERS`.** The device never reached the hub. For
`vwifi-virt` check `vwifi-probe` for `LINK_UP`; for `vwifi-ath9k` check
QEMU's stderr for `medium connected to ...`.

**`chan=-` in `LIST_PEERS`.** The peer is sending frames with
`channel_freq=0`. Run `DIAG` on the control socket; see the
troubleshooting section in [`../medium/README.md`](../medium/README.md).

**Guest sees no networks.** Confirm something is transmitting: `SURVEY`
on the control socket shows per-channel airtime. If it is empty, nothing
is beaconing — the host AP is not up.

**Everything on the same channel?** The hub filters by channel. An AP on
6 and a station scanning 1 will never see each other. `LIST_PEERS` shows
each peer's channel.

**QEMU exits immediately with a chardev error.** The hub was not
running. Start `vwifi-medium` first, or add `reconnect-ms=2000`.

**The guest's MAC is not the one passed to `mac=`.** The device hands
its address to the guest in `GET_CAPS`, and the driver publishes it as
the interface's *permanent* address. Check each hop in turn:

```bash
iw phy phy0 info | head -2      # wiphy perm_addr — what the device gave us
ethtool -P wlan0                # netdev perm_addr — same value
ip link show wlan0              # current address, which userspace may change
```

If the first two agree with `mac=` but `ip link` disagrees, something in
the guest changed it after the fact — NetworkManager randomizes Wi-Fi
MACs for scanning by default. `nmcli device show wlan0` reports both the
permanent and the in-use address; `wifi.cloned-mac-address=permanent`
pins it. If the first two are all-zero, the driver predates this being
published — rebuild it.
