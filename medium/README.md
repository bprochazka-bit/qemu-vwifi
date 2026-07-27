# medium — the shared air

`vwifi-medium` is the hub every radio attaches to. It is not a dumb
fan-out: it is channel-aware, models per-link SNR and frame error from
node positions and TX power, tracks per-channel airtime, and can be
driven live over a control socket while a lab is running.

Everything else in this repository is a peer on this medium. The hub
does not care whether a peer is a QEMU device, the host's kernel module,
a real radio bridged in over the air, or a Python test.

## What's here

| Path | What it is |
|---|---|
| `src/vwifi_medium.c` | The hub. Builds to `build/vwifi-medium` |
| `tools/vwifi_phys_bridge.c` | Bridges a real WiFi interface in monitor mode into the medium |
| `tools/vwifi_linkbench.c` | Throughput / loss benchmark against a running hub |
| `controller/` | Web UI and Python helpers for the control socket |
| `tests/harness.py` | Regression harness — spawns a hub on temp sockets and drives it |
| `legacy/` | The superseded `ath9k_medium` hubs, kept for reference |

## Building

```bash
make            # from the repository root; binaries land in build/
```

No kernel headers needed. The hub links `-lm` for the propagation model.

## Tests

```bash
make test-medium        # from the repository root
```

The harness spawns hubs on temporary sockets and exercises channel
filtering (including HT40 bonding and VHT/HE center frequencies), the
SNR and frame-error model, socket permissions, the control protocol,
peer churn, and malformed-input handling. See
[`tests/README.md`](tests/README.md) for what it does and does not
cover — kernel-side behaviour needs a real module and a VM.

## Running the hub

```bash
# Local medium only
./build/vwifi-medium /tmp/vwifi.sock

# Medium with runtime control socket (recommended)
./build/vwifi-medium /tmp/vwifi.sock -c /tmp/vwifi.ctl

# Medium that also listens for incoming TCP bridge connections
./build/vwifi-medium /tmp/vwifi.sock -c /tmp/vwifi.ctl -t 5550

# Medium with a startup config file
./build/vwifi-medium /tmp/vwifi.sock -c /tmp/vwifi.ctl -C medium.cfg
```

Full options:

```
Usage: ./build/vwifi-medium <unix-socket-path> [options]

  -t <port>        TCP listen port for incoming bridge connections
  -u <host:port>   Connect to upstream hub (repeatable, max 16)
  -c <path>        Control socket path (for runtime commands)
  -C <path>        Initial config file (commands run at startup)
  -h               Show this help
```

The data socket is created mode 0666 so any user can connect QEMU
clients. The control socket is created mode 0600 because its
commands are unauthenticated and include `SAVE_CONFIG`.

## Bridging in a real radio

`vwifi-phys-bridge` connects a physical WiFi interface in monitor
mode to the medium, making the virtual APs/clients **observable** over
the air (and real traffic injectable into the sim).

```bash
sudo ./build/vwifi-phys-bridge /tmp/vwifi.sock wlx90de801c625f -c 6 -v
```

> **Association caveat.** Plain monitor mode lets a real device *see*
> the virtual APs but **not reliably *join* one**: an 802.11 STA needs
> its frames ACKed within SIFS (~10–16 µs), which only a real radio's
> MAC hardware can do — a software relay is ~1000× too slow. To let an
> unmodifiable real station (e.g. an Android phone) actually associate
> to a virtual OpenWRT AP using a single radio, see
> [`../docs/ar9271-phys-bridge-lab.md`](../docs/ar9271-phys-bridge-lab.md),
> which keeps the VM as the literal AP and adds a hardware-ACK hook on
> an AR9271 (open firmware).

Options:

```
Usage: sudo ./build/vwifi-phys-bridge <hub-socket-path> <interface> -c <channel> [options]

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
SURVEY [RESET]                                   # per-channel utilisation
STATS                                            # global counters
SAVE_CONFIG <path>                               # snapshot current config
LOAD_CONFIG <path>                               # replay commands from a file
QUIT                                             # close this connection
```

### Node mode and channel

`LIST_PEERS` reports each node's inferred interface **mode** and its
last-seen **channel** alongside the existing position/power/counter
fields:

```
  openwrt-a    online mode=AP  chan=6  band=2.4G width=HT20 cfreq=2437 macs=[52:54:00:7f:f4:5e] pos=(27.5,6.2,0.0) txpow=15.0 tx=25151 rx=7195 rx_drop=98 tx_drop=0
  client-a     online mode=STA chan=6  band=2.4G width=HT20 cfreq=2437 macs=[52:54:00:4e:ca:0c] pos=(36.6,14.5,0.0) txpow=15.0 tx=7293 rx=20378 rx_drop=15 tx_drop=0
```

* `mode` is one of `AP`, `STA`, `MESH`, `IBSS`, or `?`. Nothing on the
  wire carries the transmitter's cfg80211 iftype, so the hub **infers**
  it from the 802.11 frames each node sends (Beacons and Probe/Assoc
  Responses ⇒ AP; Probe/Assoc Requests and ToDS data ⇒ STA; 4-address
  data ⇒ MESH). A node that has not yet transmitted anything conclusive
  shows `?`.
* `chan`/`band`/`width`/`cfreq` come from the v2 channel header the node
  last transmitted with (`chan=-` if it has only ever sent v1 frames).
  The channel is remembered across disconnect, so an `offline` node
  still shows the channel it last used.

### Site survey (channel utilisation)

`SURVEY` reports per-channel airtime, the way `iw dev <if> survey dump`
does on real hardware. Every transmitted frame is charged to its primary
channel's airtime budget (estimated from frame size and rate code), so
you can see which channels are busy and roughly how congested they are:

```
$ echo SURVEY | socat - UNIX-CONNECT:/tmp/vwifi.ctl
OK survey window=42.10s chans=2
  chan=6  freq=2437MHz band=2.4G width=HT20  nodes=2 frames=32446 bytes=41203712 airtime=6801.4ms util=16.2%
  chan=36 freq=5180MHz band=5G   width=VHT80 nodes=1 frames=118   bytes=151040   airtime=39.2ms   util=0.1%
```

`util` is busy airtime as a percentage of the survey window. Send
`SURVEY RESET` to zero the counters and restart the window (e.g. to
measure utilisation over a specific test interval). Utilisation is a
relative indicator, not MAC-accurate airtime accounting — it uses a
coarse fixed per-frame PHY overhead and does not model contention or
retransmission backoff.

#### Troubleshooting `chan=-`

`chan=-` (and `SURVEY` showing a `chan=-` bucket) means the frames
arriving at the hub carry `channel_freq=0`: the medium reports the
channel faithfully, but the **sender never put one in the header**. The
hub only knows a node's channel because that node stamps it into the v2
frame header on every TX — it cannot invent it.

`DIAG` shows, per connected peer, the header version and the raw channel
fields of the most recent frame, which tells you where the zero comes
from:

```
$ echo DIAG | socat - UNIX-CONNECT:/tmp/vwifi.ctl
OK diag 2 peers
  peer=0 node=openwrt-a    bridge=0 hdrver=2 paylen=242 frames=196 chan_freq=0    chan_flags=0x0000 bond=0 cf1=0 cf2=0
  peer=1 node=client-a     bridge=0 hdrver=2 paylen=131 frames=24  chan_freq=0    chan_flags=0x0000 bond=0 cf1=0 cf2=0
```

* `hdrver < 2` — the sender is emitting a v1 (pre-channel) header. Rebuild
  and reload that component from current source.
* `hdrver=2` **and** `chan_freq=0` — the sender speaks v2 but isn't
  populating the channel. Fix the source:
  * **Kernel driver (`vwifi_host.ko` in a VM):** the channel is filled
    from `priv->channel_freq`, which mac80211 sets via the driver's
    `.config`/chanctx path. A current build defaults to channel 1 and
    tracks the configured channel; a `chan_freq=0` almost always means an
    **older `.ko` is loaded** — rebuild and `rmmod`/`insmod` it in the VM.
  * **`vwifi-phys-bridge`:** it stamps the channel from the captured
    frame's radiotap, falling back to its **required** `-c <channel>`
    option. Make sure it was launched with `-c` matching the radio's
    channel (and `-w <bandwidth>` for HT40/VHT).

There's also a web UI in `controller/` for the same control socket —
see [`controller/README.md`](controller/README.md).

## Multi-host setup

The hub supports TCP bridging between hosts. The kernel module and
relay always connect to a local hub; remote hubs are linked via the
hub's own `-t`/`-u` options.

```
Host A:
  ./build/vwifi-medium /tmp/a.sock -t 5550
  sudo insmod ../host/vwifi_host.ko
  sudo ./build/vwifi-host-relay /tmp/a.sock

Host B:
  ./build/vwifi-medium /tmp/b.sock -u hostA:5550
  sudo insmod ../host/vwifi_host.ko macaddr=00:03:7F:CC:DD:02
  sudo ./build/vwifi-host-relay /tmp/b.sock
```

`wlanX` on both hosts and every QEMU VM on both hosts now share
the same medium.
