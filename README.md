# ath9k_medium_host — Host-Side Virtual Wireless Interface

A Linux kernel module and userspace relay daemon that puts the host machine on the same virtual wireless medium as QEMU guests running ath9k-virt devices.

## Architecture

```
 ┌──────────────────────── Host Machine ────────────────────────┐
 │                                                              │
 │  hostapd / wpa_supplicant / iw / NetworkManager              │
 │         │                                                    │
 │     mac80211  (wlanX)                                        │
 │         │                                                    │
 │   ath9k_medium_host.ko        ← kernel module                │
 │         │  /dev/ath9k_medium                                 │
 │   ath9k_host_relay            ← userspace daemon             │
 │         │                                                    │
 └─────────┼────────────────────────────────────────────────────┘
           │  unix socket
   ath9k_medium_hub              ← shared medium process
           │
    ┌──────┼──────┐
    │      │      │
  QEMU   QEMU   QEMU
  VM-A   VM-B   VM-C
```

The host's `wlanX` interface is a full mac80211 radio — it supports hostapd (AP mode), wpa_supplicant (STA mode), monitor mode, mesh, and ad-hoc. Frames transmitted on this interface flow through the hub to all connected QEMU guests, and vice versa.

## Files

| File | Description |
|------|-------------|
| `ath9k_medium.h` | Shared wire protocol header (kernel + userspace) |
| `ath9k_medium_host.c` | Kernel module — mac80211 radio + char device |
| `ath9k_host_relay.c` | Userspace daemon — bridges chardev ↔ hub socket |
| `Makefile` | Kernel module build system |

## Building

### Kernel Module

Requires kernel headers for your running kernel:

```bash
# Debian/Ubuntu
sudo apt install linux-headers-$(uname -r)

# Build
make

# The module is: ath9k_medium_host.ko
```

To build against a different kernel:

```bash
make KDIR=/path/to/kernel/source
```

### Relay Daemon

```bash
gcc -Wall -O2 -o ath9k_host_relay ath9k_host_relay.c
```

No dependencies beyond standard POSIX/libc.

## Usage

### Step 1: Start the Hub

```bash
./ath9k_medium_hub /tmp/ath9k.sock
```

### Step 2: Load the Kernel Module

```bash
sudo insmod ath9k_medium_host.ko
# or with a custom MAC address:
sudo insmod ath9k_medium_host.ko macaddr=00:03:7F:CC:DD:02
```

This creates:
- A new `wlanX` interface visible in `iw dev`
- A char device at `/dev/ath9k_medium`

### Step 3: Start the Relay Daemon

```bash
sudo ./ath9k_host_relay /tmp/ath9k.sock
```

The relay bridges `/dev/ath9k_medium` to the hub's unix socket. It logs to stderr:

```
relay: chardev /dev/ath9k_medium opened (fd=3)
relay: connected to hub /tmp/ath9k.sock (fd=4)
relay: bridging /dev/ath9k_medium ↔ /tmp/ath9k.sock
```

### Step 4: Start QEMU VMs

```bash
qemu-system-x86_64 -machine q35 -m 512 \
  -drive file=vm.qcow2,format=qcow2 \
  -chardev socket,id=medium,path=/tmp/ath9k.sock,server=off \
  -device ath9k-virt,chardev=medium \
  -nographic
```

### Step 5: Use the Host WiFi Interface

The host's `wlanX` is now on the same wireless medium as the QEMU guests.

**Host as AP, guest as STA:**
```bash
# On the host:
sudo ip link set wlan0 up
sudo hostapd /etc/hostapd/hostapd.conf

# Inside the VM:
sudo wpa_supplicant -i wlan0 -c /etc/wpa_supplicant.conf
```

**Host as STA, guest as AP:**
```bash
# Inside the VM (run hostapd to create AP)
# On the host:
sudo iw dev wlan0 scan
sudo wpa_supplicant -i wlan0 -c /etc/wpa_supplicant.conf
```

**Host as monitor:**
```bash
sudo iw dev wlan0 set type monitor
sudo ip link set wlan0 up
sudo tcpdump -i wlan0 -e -n
```

## Module Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `macaddr` | `00:03:7F:CC:DD:01` | MAC address for the virtual radio |

The MAC address should be unique on the medium. Use the Atheros OUI (`00:03:7F`) prefix for consistency with the QEMU devices.

## Multi-Host Setup

The hub already supports TCP bridging between hosts. The kernel module and relay don't need any changes — they always connect to a local hub.

```
Host A:
  ath9k_medium_hub /tmp/a.sock -t 5550
  insmod ath9k_medium_host.ko
  ./ath9k_host_relay /tmp/a.sock

Host B:
  ath9k_medium_hub /tmp/b.sock -u hostA:5550
  insmod ath9k_medium_host.ko macaddr=00:03:7F:CC:DD:02
  ./ath9k_host_relay /tmp/b.sock
```

Now `wlanX` on Host A, `wlanX` on Host B, and all QEMU VMs on both hosts share the same wireless medium.

## How It Works

### TX Path (host → VMs)

1. Application sends data through the `wlanX` interface
2. mac80211 builds an 802.11 frame and calls our `.tx` callback
3. The module wraps the frame in an `ath9k_medium_frame_hdr` with the wire protocol length prefix
4. The frame is queued in the module's TX skb queue
5. The relay daemon's `poll()` wakes up, `read()`s the frame from `/dev/ath9k_medium`
6. The relay `write()`s the frame to the hub's unix socket
7. The hub fans it out to all connected QEMU instances
8. Each QEMU ath9k-virt device injects the frame into the guest's RX DMA path

### RX Path (VMs → host)

1. QEMU ath9k-virt extracts an 802.11 frame from the guest's TX DMA
2. The frame is sent to the hub via unix socket
3. The hub fans it out, including to the relay daemon's connection
4. The relay `read()`s the frame from the hub socket
5. The relay `write()`s the frame to `/dev/ath9k_medium`
6. The module's chardev write handler parses the wire protocol
7. The module calls `ieee80211_rx_irqsafe()` to inject the frame into mac80211
8. mac80211 delivers the frame to hostapd/wpa_supplicant/application

### Auto-ACK

The module reports immediate TX success (`IEEE80211_TX_STAT_ACK`) for every transmitted frame. This matches the QEMU device's behavior and avoids retransmit storms on the virtual medium.

### Own-Frame Filtering

The module compares each received frame's `tx_mac` against its own MAC address and drops frames it originally transmitted. This prevents echo when the hub fans frames back to all clients.

## Troubleshooting

### "No such device" when opening /dev/ath9k_medium

The module isn't loaded:
```bash
sudo insmod ath9k_medium_host.ko
ls -la /dev/ath9k_medium
```

### "Device or resource busy" from relay

Only one relay daemon can connect at a time. Check for stale processes:
```bash
ps aux | grep ath9k_host_relay
```

### No wlanX interface after loading module

Check dmesg for errors:
```bash
dmesg | grep ath9k_medium_host
```

You should see:
```
ath9k_medium_host v1.0: registered — MAC 00:03:7f:cc:dd:01, chardev /dev/ath9k_medium
```

If mac80211 registration fails, ensure the `mac80211` module is loaded:
```bash
sudo modprobe mac80211
```

### Frames not flowing

1. Verify the hub is running and both the relay and QEMU are connected (check hub's stderr log)
2. Verify the relay shows "bridging" in its output
3. Check that the module is started: `dmesg | grep "radio started"`
4. Use monitor mode on the host to see raw frames:
   ```bash
   sudo iw dev wlan0 set type monitor
   sudo ip link set wlan0 up
   sudo tcpdump -i wlan0 -c 10
   ```

### Unloading

```bash
# Stop the relay first
kill $(pidof ath9k_host_relay)

# Bring down the interface
sudo ip link set wlan0 down

# Remove the module
sudo rmmod ath9k_medium_host
```

## Limitations

- Single radio per module load (no multi-radio support yet)
- No regulatory domain enforcement (virtual medium, no real RF)
- No rate control feedback from the medium (all TX reported as ACK'd)
- The relay daemon must be running for frames to flow
- FCS handling: `RX_INCLUDES_FCS` is set in hw flags; frames from the medium should include FCS or mac80211 will complain (the QEMU device includes a 4-byte FCS placeholder)
