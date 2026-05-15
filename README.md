# vwifi — Virtual WiFi Medium for QEMU

A virtual 802.11 medium for QEMU guests, plus a kernel module that lets
the host join the medium as a regular mac80211 radio. The hub is
channel-aware and models per-link SNR / frame-error rates, so the
simulated medium behaves like radio rather than a hub.

## Components

| File / Binary | Description |
|---|---|
| `vwifi.h` | Shared wire-protocol header (kernel + userspace) |
| `vwifi_medium.c` → `vwifi-medium` | Userspace medium hub; fans frames out to all connected peers with channel-aware filtering and per-link SNR |
| `vwifi_host.c` → `vwifi_host.ko` | Kernel module — mac80211 radio + char device at `/dev/vwifi` |
| `vwifi_host_relay.c` → `vwifi-host-relay` | Bridges `/dev/vwifi` ↔ hub Unix socket |
| `vwifi_phys_bridge.c` → `vwifi-phys-bridge` | Bridges a real WiFi interface in monitor mode into the medium |
| `tests/harness.py` | Userspace regression harness |
| `medium-controller/` | Web UI + Python helpers for the hub's control socket |

## Architecture

```
 ┌──────────────────────── Host Machine ────────────────────────┐
 │                                                              │
 │  hostapd / wpa_supplicant / iw / NetworkManager              │
 │         │                                                    │
 │     mac80211  (wlanX)                                        │
 │         │                                                    │
 │   vwifi_host.ko        ← kernel module                       │
 │         │  /dev/vwifi                                        │
 │   vwifi-host-relay     ← userspace daemon                    │
 │         │                                                    │
 └─────────┼────────────────────────────────────────────────────┘
           │  unix socket
   vwifi-medium             ← shared medium hub
           │
    ┌──────┼──────┬────────────────┐
    │      │      │                │
  QEMU   QEMU   QEMU         vwifi-phys-bridge
  VM-A   VM-B   VM-C        (optional: real radio
                             on a monitor-mode iface)
```

The host's `wlanX` interface is a full mac80211 radio — it supports
hostapd (AP mode), wpa_supplicant (STA mode), monitor mode, mesh, and
ad-hoc. Frames flow through the hub to all connected QEMU guests and
back.

## Building

### Userspace binaries

No kernel headers required:

```bash
make userspace
```

This builds three binaries:

```
vwifi-medium        # the medium hub
vwifi-host-relay    # chardev <-> hub bridge daemon
vwifi-phys-bridge   # real-radio <-> hub bridge
```

### Kernel module

Requires kernel headers for your running kernel:

```bash
# Debian/Ubuntu
sudo apt install linux-headers-$(uname -r)

# Build
make                               # produces vwifi_host.ko

# Build against a different kernel
make KDIR=/path/to/kernel/source
```

### Tests

```bash
make test     # runs python3 tests/harness.py against ./vwifi-medium
```

See [`tests/README.md`](tests/README.md) for what the harness covers
(it doesn't cover kernel-side fixes that need a real module + VM).

## Usage

### Step 1: Start the hub

```bash
# Local medium only
./vwifi-medium /tmp/vwifi.sock

# Medium with runtime control socket (recommended)
./vwifi-medium /tmp/vwifi.sock -c /tmp/vwifi.ctl

# Medium that also listens for incoming TCP bridge connections
./vwifi-medium /tmp/vwifi.sock -c /tmp/vwifi.ctl -t 5550

# Medium with a startup config file
./vwifi-medium /tmp/vwifi.sock -c /tmp/vwifi.ctl -C medium.cfg
```

Full options:

```
Usage: ./vwifi-medium <unix-socket-path> [options]

  -t <port>        TCP listen port for incoming bridge connections
  -u <host:port>   Connect to upstream hub (repeatable, max 16)
  -c <path>        Control socket path (for runtime commands)
  -C <path>        Initial config file (commands run at startup)
  -h               Show this help
```

The data socket is created mode 0666 so any user can connect QEMU
clients. The control socket is created mode 0600 because its
commands are unauthenticated and include `SAVE_CONFIG`.

### Step 2: Load the kernel module

```bash
sudo insmod vwifi_host.ko
# or with a custom MAC address:
sudo insmod vwifi_host.ko macaddr=00:03:7F:CC:DD:02
```

This creates:
- A new `wlanX` interface visible in `iw dev`
- A char device at `/dev/vwifi`

### Step 3: Start the relay daemon

```bash
sudo ./vwifi-host-relay /tmp/vwifi.sock
```

The relay bridges `/dev/vwifi` to the hub's Unix socket. Full options:

```
Usage: ./vwifi-host-relay <hub-socket-path> [chardev-path]

  hub-socket-path  Path to vwifi-medium Unix socket
  chardev-path     Path to kernel module char device
                   (default: /dev/vwifi)
```

It logs to stderr:

```
relay: chardev /dev/vwifi opened (fd=3)
relay: connected to hub /tmp/vwifi.sock (fd=4)
relay: bridging /dev/vwifi ↔ /tmp/vwifi.sock
```

### Step 4: Start QEMU VMs

```bash
qemu-system-x86_64 -machine q35 -m 512 \
  -drive file=vm.qcow2,format=qcow2 \
  -chardev socket,id=medium,path=/tmp/vwifi.sock,server=off \
  -device vwifi-virt,chardev=medium \
  -nographic
```

### Step 5: Use the host WiFi interface

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

## Bridging in a real radio (optional)

`vwifi-phys-bridge` connects a physical WiFi interface in monitor
mode to the medium, so an external device can associate with a
virtual AP running inside a QEMU VM.

```bash
sudo ./vwifi-phys-bridge /tmp/vwifi.sock wlx90de801c625f -c 6 -v
```

Options:

```
Usage: sudo ./vwifi-phys-bridge <hub-socket-path> <interface> -c <channel> [options]

  -c <channel>     Channel number (1-14, 36, 40, ...) or freq in MHz
                   Values <= 200 are channel numbers, > 200 are MHz
  -w <bandwidth>   Channel width (default: HT20):
                   HT20, HT40+, HT40-, VHT80, VHT160, VHT80+80
  -s <center2_mhz> Secondary 80MHz center freq (VHT80+80 only)
  -n <node_id>     Node ID for hub registration (default: phys-<ifname>)
  -v               Verbose logging
  -h               Show this help
```

Put the interface in monitor mode first:

```bash
sudo iw dev wlx90de801c625f set type monitor
sudo ip link set wlx90de801c625f up
sudo iw dev wlx90de801c625f set channel 6
```

## Runtime control

When the hub is started with `-c <path>`, a Unix socket at that
path accepts text commands. Connect with `socat` or `nc`:

```bash
# One-shot:
echo LIST_PEERS | socat - UNIX-CONNECT:/tmp/vwifi.ctl

# Interactive:
socat READLINE UNIX-CONNECT:/tmp/vwifi.ctl
```

Some useful commands (send `HELP` for the full list):

```
LIST_PEERS                                       # show all nodes + state
SET_POS <node-id> <x> <y> [<z>]                  # position a node in metres
SET_TXPOWER <node-id> <dBm>                      # set node TX power
SET_SNR <mac-a> <mac-b> <snr-db>                 # pin a per-link SNR override
CLEAR_SNR <mac-a> <mac-b>                        # release the override
STATS                                            # global counters
SAVE_CONFIG <path>                               # snapshot current config
LOAD_CONFIG <path>                               # replay commands from a file
QUIT                                             # close this connection
```

There's also a web UI in `medium-controller/` for the same control
socket — see `medium-controller/README.md`.

## Multi-host setup

The hub supports TCP bridging between hosts. The kernel module and
relay always connect to a local hub; remote hubs are linked via the
hub's own `-t`/`-u` options.

```
Host A:
  ./vwifi-medium /tmp/a.sock -t 5550
  sudo insmod vwifi_host.ko
  sudo ./vwifi-host-relay /tmp/a.sock

Host B:
  ./vwifi-medium /tmp/b.sock -u hostA:5550
  sudo insmod vwifi_host.ko macaddr=00:03:7F:CC:DD:02
  sudo ./vwifi-host-relay /tmp/b.sock
```

`wlanX` on both hosts and every QEMU VM on both hosts now share
the same medium.

## Module parameters

| Parameter | Default | Description |
|---|---|---|
| `macaddr` | `00:03:7F:CC:DD:01` | MAC address for the virtual radio |

The MAC must be unique on the medium.

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
   `/dev/vwifi`, and `write()`s it to the hub's Unix socket.
6. The hub looks up each peer's last known channel and fans the
   frame out only to peers whose channel matches (including HT40
   bond_freq and VHT80+ center_freq disambiguation). A per-link
   SNR model decides per-frame whether to drop based on the
   sender's rate code and the receiver's link SNR.
7. Each QEMU vwifi-virt device injects the frame into the guest's
   RX path.

### RX path (VMs → host)

1. QEMU vwifi-virt sends an 802.11 frame to the hub.
2. The hub fans it out (including to the relay).
3. The relay reads the framed message and writes it to
   `/dev/vwifi`.
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

### `No such device` opening `/dev/vwifi`

The module isn't loaded:

```bash
sudo insmod vwifi_host.ko
ls -la /dev/vwifi
```

### `Device or resource busy` from the relay

Only one relay daemon can be attached at a time. Check for stale
processes:

```bash
ps aux | grep vwifi-host-relay
```

### No `wlanX` interface after `insmod`

Check `dmesg`:

```bash
dmesg | grep vwifi_host
```

You should see something like:

```
vwifi_host v1.0: registered — MAC 00:03:7f:cc:dd:01, chardev /dev/vwifi
```

If mac80211 registration fails, ensure `mac80211` is loaded:

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

- Single radio per module load (no multi-radio yet)
- No regulatory domain enforcement (virtual medium, no real RF)
- The relay daemon must be running for frames to flow
- HE (802.11ax) is advertised on 5 GHz STATION mode only; the
  driver's TX path emits VHT rate codes for HE frames and the
  hub's rate table covers HE80 NSS 1/2 MCS 0–11 but not HE160 /
  HE80+80
- `RX_INCLUDES_FCS` is set in hw flags; frames from the medium
  should include FCS or mac80211 will complain (the QEMU device
  appends a 4-byte FCS placeholder)
