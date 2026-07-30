# host — the host machine's own radios

`vwifi_host.ko` is a mac80211 driver. It registers wiphys that behave
like real radios to everything above them — hostapd, wpa_supplicant,
`iw`, NetworkManager, monitor mode, mesh, ad-hoc — but whose PHY is the
virtual medium instead of an antenna.

That is what lets the host machine be a peer in its own lab: run the AP
on the host and the stations in VMs, or the reverse, or sniff the whole
medium from the host with tcpdump.

Radios are created and destroyed at runtime, so a controller can add and
remove host interfaces on demand rather than reloading the module.

## What's here

| Path | What it is |
|---|---|
| `vwifi_host.c` | The kernel module. Builds to `vwifi_host.ko` |
| `tools/vwifi_host_relay.c` | Bridges one radio's char device to a hub socket |
| `tools/vwifi_ctl.c` | Creates and destroys radios via `/dev/vwifi-ctl` |
| `ackmon/` | AR9271 firmware hook for hardware-ACKing a virtual AP (see `../docs/ar9271-phys-bridge-lab.md`) |

## Building

The userspace tools need no kernel headers and come from the top-level
`make`. The module needs headers for the kernel you will load it into:

```bash
# Debian/Ubuntu
sudo apt install linux-headers-$(uname -r)

make module                          # from the repository root
make module KDIR=/path/to/kernel     # against a different kernel
sudo make install                    # modules_install + depmod
sudo make install SKIP_SIGN=1        # ... skipping the signing step
```

`SKIP_SIGN=1` exists because stock Debian and Ubuntu kernel-headers
packages ship no private signing key, so Kbuild's post-install
`sign-file` step fails noisily and harmlessly. The module is simply left
unsigned, which is fine unless Secure Boot is enabled.

The module compiles against `../abi/vwifi.h` and
`../abi/vwifi_host_ioctl.h`. There is one copy of each; do not fork them
into this directory.

## Loading the module

```bash
sudo insmod host/vwifi_host.ko
```

Loading the module creates the control node `/dev/vwifi-ctl` but **no radio
yet** — radios are created on demand (see below). To get the classic
single-radio behaviour of older builds, load with `default_radio=1`:

```bash
# One radio at /dev/vwifi with the macaddr= address, ready immediately:
sudo insmod host/vwifi_host.ko default_radio=1 macaddr=00:03:7F:CC:DD:02
```

## Creating a radio

Each radio is an independent mac80211 wiphy (its own `wlanX`) backed by its
own char device at `/dev/<name>`. Create one with `vwifi-ctl`:

```bash
# Radio at /dev/vwifi-lab, netdev renamed to vwm-lab and brought up:
sudo ./build/vwifi-ctl create --dev vwifi-lab --ifname vwm-lab
# ...prints: created radio 'vwifi-lab' id=0 phy=phyN mac=... dev=/dev/vwifi-lab
```

`--mac aa:bb:cc:dd:ee:ff` sets the MAC explicitly; omitted, a stable
locally-administered MAC is derived from the name. Tear a radio down (after
its relay has stopped) with:

```bash
sudo ./build/vwifi-ctl destroy --dev vwifi-lab
```

You can create as many radios as you like; each joins the medium as a
separate node through its own relay.

## Starting the relay

```bash
sudo ./build/vwifi-host-relay /tmp/vwifi.sock /dev/vwifi-lab
```

The relay bridges a radio's char device to the hub's Unix socket. Full
options:

```
Usage: ./build/vwifi-host-relay <hub-socket-path> [chardev-path]

  hub-socket-path  Path to vwifi-medium Unix socket
  chardev-path     Path to the radio's char device
                   (default: /dev/vwifi, i.e. the default_radio node)
```

It logs to stderr:

```
relay: chardev /dev/vwifi-lab opened (fd=3)
relay: connected to hub /tmp/vwifi.sock (fd=4)
relay: bridging /dev/vwifi-lab ↔ /tmp/vwifi.sock
```

## Using the host interface

The host's `wlanX` is now on the same wireless medium as the guests.

Host as AP, guest as STA:

```bash
# Host
sudo ip link set wlan0 up
sudo hostapd /etc/hostapd/hostapd.conf

# Inside the VM
sudo wpa_supplicant -i wlan0 -c /etc/wpa_supplicant.conf
```

Host as STA, guest as AP:

```bash
# Inside the VM: run hostapd to create the AP
# Host
sudo iw dev wlan0 scan
sudo wpa_supplicant -i wlan0 -c /etc/wpa_supplicant.conf
```

Host as monitor:

```bash
sudo iw dev wlan0 set type monitor
sudo ip link set wlan0 up
sudo tcpdump -i wlan0 -e -n
```

## Dynamic radios

The module registers a control node, `/dev/vwifi-ctl`. Userspace issues two
ioctls on it — `VWIFI_IOC_CREATE_RADIO` and `VWIFI_IOC_DESTROY_RADIO` (see
[`../abi/vwifi_host_ioctl.h`](../abi/vwifi_host_ioctl.h)) — to add and remove radios at runtime. `vwifi-ctl` is the CLI
front-end. Each radio:

- is an independent mac80211 wiphy with its own `wlanX` and MAC;
- exposes its own data char device at `/dev/<name>`, which exactly one
  `vwifi-host-relay` bridges to a medium hub;
- can be created and destroyed without touching any other radio, and
  without reloading the module.

Destroying a radio is refused (`EBUSY`) while a relay still holds its char
device open, so stop the relay first. This is the model Nyxus drives to add
and remove host interfaces per medium on demand.

## Module parameters

| Parameter | Default | Description |
|---|---|---|
| `default_radio` | `0` | Create one radio at load time, exposed at `/dev/vwifi` with the `macaddr` address (classic single-radio behaviour). Off by default so a controller managing radios via `/dev/vwifi-ctl` gets no stray interface. |
| `macaddr` | `00:03:7F:CC:DD:01` | MAC address for the `default_radio`. Dynamically created radios set their own MAC via `vwifi-ctl --mac` (or derive one from the radio name). |

Every radio's MAC must be unique on the medium.

## How it works

### TX path (host → VMs)

1. Application sends data through `wlanX`.
2. mac80211 builds an 802.11 frame and calls the module's `.tx`.
3. The module wraps the frame in a `vwifi_frame_hdr` (40-byte v2
   header) with the wire-protocol length prefix. The header
   carries `tx_mac`, `rate_code` (translated from
   `tx_info->control.rates[0]` to our HT/VHT MCS namespace), and
   the current operating channel (`channel_freq`,
   `channel_bond_freq`, `channel_flags`, `center_freq{1,2}`).
4. The frame is enqueued for the relay daemon. If the queue
   crosses the high watermark, the module calls
   `ieee80211_stop_queues()` so mac80211 stops handing us frames
   until the relay drains.
5. The relay daemon's `poll()` wakes, `read()`s the message from
   `/dev/<radio>`, and `write()`s it to the hub's Unix socket.
6. The hub looks up each peer's last known channel and fans the
   frame out only to peers whose channel matches (including HT40
   bond_freq and VHT80+ center_freq disambiguation; the HT40 bond
   pair is only enforced when both peers declare one, so a narrow
   station still hears a bonded AP on the shared primary). A
   per-link SNR model decides per-frame whether to drop based on
   the sender's rate code and the receiver's link SNR. Links that
   touch a **physical radio** (the phys bridge, which registers
   with the `physical` hello flag) are exempt from this model:
   real-world RF is the channel, so the hub never adds simulated
   loss or rewrites their RSSI. Inter-hub TCP **bridges** are
   multi-channel trunks and are likewise never channel-filtered;
   the downstream hub filters per-receiver.
7. Each QEMU vwifi-virt device injects the frame into the guest's
   RX path.

### RX path (VMs → host)

1. QEMU vwifi-virt sends an 802.11 frame to the hub.
2. The hub fans it out (including to the relay).
3. The relay reads the framed message and writes it to
   `/dev/<radio>`.
4. The module parses the wire protocol (accepting both v1 28-byte
   and v2 40-byte headers) and calls `ieee80211_rx_irqsafe()`.
5. mac80211 delivers the frame to hostapd / wpa_supplicant / etc.

### TX status

The medium has no over-the-air ACK. The driver reports
`IEEE80211_TX_STAT_ACK` for frames successfully enqueued for the
relay. If the TX queue is full despite backpressure (relay died,
or a producer raced past the high watermark), the driver reports
the frame with `STAT_ACK` cleared so mac80211's retry counter and
rate-control logic see a real failure rather than a fake success.

### Own-frame filtering

The module compares each RX frame's `tx_mac` against its own MAC
and drops frames it originally sent, preventing echo when the hub
fans frames back to the relay.

## Troubleshooting

### `No such device` opening `/dev/vwifi-ctl`

The module isn't loaded:

```bash
sudo insmod host/vwifi_host.ko
ls -la /dev/vwifi-ctl
```

### `No such file or directory` opening a radio's `/dev/<name>`

The radio hasn't been created (or was destroyed). Create it:

```bash
sudo ./build/vwifi-ctl create --dev vwifi-lab --ifname vwm-lab
```

### `Device or resource busy` destroying a radio

A relay still holds that radio's char device open. Stop the relay first,
then `vwifi-ctl destroy`.

### `Device or resource busy` from the relay

Only one relay daemon can be attached to a given radio at a time. Check for
stale processes:

```bash
ps aux | grep vwifi-host-relay
```

### No `wlanX` interface after creating a radio

Check `dmesg`:

```bash
dmesg | grep vwifi_host
```

You should see something like:

```
vwifi_host v1.0: created radio 'vwifi-lab' id=0 phy=phy0 MAC=02:03:7f:.. chardev /dev/vwifi-lab
```

(Note: just loading the module logs only `control device /dev/vwifi-ctl
ready` — no radio is created until you ask for one, unless you loaded with
`default_radio=1`.) If mac80211 registration fails, ensure `mac80211` is
loaded:

```bash
sudo modprobe mac80211
```

### Frames not flowing

1. Verify the hub is running and that both the relay and at least
   one QEMU peer are connected (check the hub's stderr).
2. Verify the relay logged `bridging`.
3. `dmesg | grep "radio started"` to confirm mac80211 started TX.
4. If you suspect channel filtering is dropping frames, query
   `LIST_PEERS` on the control socket — each node's current
   channel is learned from its most recent TX.
5. Use monitor mode on the host to see raw frames:

   ```bash
   sudo iw dev wlan0 set type monitor
   sudo ip link set wlan0 up
   sudo tcpdump -i wlan0 -c 10
   ```

### Unloading

```bash
# Stop the relay first
sudo killall vwifi-host-relay

# Bring down the interface
sudo ip link set wlan0 down

# Remove the module
sudo rmmod vwifi_host
```

## Limitations

- No regulatory domain enforcement (virtual medium, no real RF)
- The relay daemon must be running for frames to flow
- HE (802.11ax) is advertised on 5 GHz STATION mode only; the
  driver's TX path emits VHT rate codes for HE frames and the
  hub's rate table covers HE80 NSS 1/2 MCS 0–11 but not HE160 /
  HE80+80
- `RX_INCLUDES_FCS` is set in hw flags; frames from the medium
  should include FCS or mac80211 will complain (the QEMU device
  appends a 4-byte FCS placeholder)
