# vwifi_phys_bridge.c — Implementation Prompt

**Task: Implement `vwifi_phys_bridge.c` — a daemon that bridges a physical WiFi interface (in monitor mode) to a virtual wireless medium hub.**

## What This Is

The `vwifi-medium` is a fan-out process that connects virtual WiFi devices (QEMU VMs) over Unix domain sockets using a length-prefixed wire protocol. Each peer sends/receives raw 802.11 frames wrapped in a 40-byte medium header.

The bridge daemon connects to this hub as a regular peer, but instead of a virtual radio, it uses a real physical WiFi interface in monitor mode. It:
- Receives frames from the hub and injects them over the air via the physical radio
- Captures frames from the physical radio and forwards them into the hub

This lets a real laptop connect to a virtual AP running inside a QEMU VM.

## File Location

Create `/home/user/qemu-vwifi/vwifi_phys_bridge.c`. It `#include`s `"vwifi.h"` which is in the same directory.

## Build Command

```
gcc -Wall -Wextra -O2 -o vwifi-phys-bridge vwifi_phys_bridge.c
```

No external dependencies beyond standard POSIX/libc.

## CLI

```
sudo ./vwifi-phys-bridge <hub-socket-path> <interface> -c <channel> [options]
  -c <channel>       Required. Channel number (1-14, 36, 40, ...) or freq in MHz.
                     Values ≤200 treated as channel numbers, >200 as MHz.
  -w <bandwidth>     Channel width (default: HT20):
                     HT20, HT40+, HT40-, VHT80, VHT160, VHT80+80
  -s <center2_mhz>   Secondary 80MHz center freq (only for VHT80+80)
  -n <node_id>       Node ID for hub registration (default: "phys-<ifname>")
  -v                 Verbose logging
  -h                 Help
```

Example: `sudo ./vwifi-phys-bridge /tmp/vwifi.sock wlx90de801c625f -c 6 -v`

## Wire Protocol (from vwifi.h)

Every message on the hub Unix socket is length-prefixed:

```
[uint32_t length (network byte order)]   <- total bytes following
[struct vwifi_frame_hdr]          <- 40 bytes
[uint8_t payload[...]]                   <- raw 802.11 frame
```

The 40-byte medium header:

```c
struct vwifi_frame_hdr {
    uint32_t    magic;              /* 0x46495756 "VWIF" */
    uint16_t    version;            /* 2 */
    uint16_t    frame_len;          /* Length of 802.11 frame following this header */
    uint8_t     tx_mac[6];          /* Transmitting device's MAC address */
    uint8_t     rate_code;          /* TX rate code (see rate table below) */
    int8_t      rssi;               /* RSSI in dBm (default -30) */
    uint32_t    tsf_lo;             /* TSF timestamp low word */
    uint32_t    tsf_hi;             /* TSF timestamp high word */
    uint32_t    flags;              /* byte 27 = TTL; rest reserved */
    uint16_t    channel_freq;       /* Center frequency in MHz (0 = broadcast) */
    uint16_t    channel_flags;      /* See flag constants below */
    uint16_t    channel_bond_freq;  /* HT40 secondary channel freq (0 = none) */
    uint16_t    center_freq1;       /* VHT/HE center freq of primary segment (0=N/A) */
    uint16_t    center_freq2;       /* VHT80+80 center freq of secondary segment (0=N/A) */
    uint8_t     _reserved[2];       /* Must be zero */
};
```

All fields are **little-endian** (native x86). Only the 4-byte length prefix is network (big-endian) order.

Key constants:

```c
#define VWIFI_MAGIC           0x46495756
#define VWIFI_VERSION         2
#define VWIFI_MAX_FRAME_SIZE  8192
#define VWIFI_HDR_SIZE        40  /* sizeof(vwifi_frame_hdr) */
#define VWIFI_DEFAULT_RSSI    (-30)
#define VWIFI_DEFAULT_RATE    0x0B  /* 6 Mbps OFDM */

/* Channel flags */
#define VWIFI_CHAN_FLAG_2GHZ         0x0001
#define VWIFI_CHAN_FLAG_5GHZ         0x0002
#define VWIFI_CHAN_FLAG_HT20         0x0004
#define VWIFI_CHAN_FLAG_HT40PLUS     0x0008
#define VWIFI_CHAN_FLAG_HT40MINUS    0x0010
#define VWIFI_CHAN_FLAG_VHT80        0x0020
#define VWIFI_CHAN_FLAG_VHT160       0x0040
#define VWIFI_CHAN_FLAG_VHT80_80     0x0080
```

## Hub Connection

Connect via `socket(AF_UNIX, SOCK_STREAM, 0)` + `connect()` to the hub socket path.

Immediately after connecting, send a **hello message** to register a node identity:

```
[uint32_t length (network order)]  <- len of payload below
[uint32_t 0x52495756]              <- HELLO_MAGIC "VWIR"
[node_id string, null-terminated]
```

The hub absorbs this hello (doesn't forward it) and binds the peer to a named node. If `-n` not given, use `"phys-<ifname>"` as the node_id.

Reading from the hub is **stream-based** -- you must do your own framing: read the 4-byte length prefix, then read that many bytes of payload. Use a receive buffer with partial-message reassembly (the hub may send multiple messages in one `read()`, or split one across multiple reads).

## Physical Interface (AF_PACKET raw socket)

```c
int raw_fd = socket(AF_PACKET, SOCK_RAW, htons(ETH_P_ALL));
struct sockaddr_ll sll = {
    .sll_family = AF_PACKET,
    .sll_protocol = htons(ETH_P_ALL),
    .sll_ifindex = if_nametoindex(ifname),
};
bind(raw_fd, (struct sockaddr *)&sll, sizeof(sll));
```

Requires the interface to be in **monitor mode**. Validate at startup by reading `/sys/class/net/<ifname>/type` -- value should be `803` (ARPHRD_IEEE80211_RADIOTAP).

Reads return: `[radiotap header][802.11 frame]`
Writes expect: `[radiotap header][802.11 frame]`

## Radiotap Handling

### For injection (hub->physical): build a minimal radiotap header

Simplest and most compatible: **8-byte empty radiotap header** (present=0). Drivers choose their own rate for injection. This avoids compatibility issues across different drivers.

```c
static const uint8_t inject_radiotap[8] = {
    0x00,                   /* version */
    0x00,                   /* pad */
    0x08, 0x00,             /* length = 8 (LE) */
    0x00, 0x00, 0x00, 0x00  /* present = 0 (no fields) */
};
```

### For capture (physical->hub): parse the radiotap header

Radiotap is variable-length. The structure starts with:

```
offset 0: u8  version (always 0)
offset 1: u8  pad
offset 2: u16 length (LE) -- total radiotap header length
offset 4: u32 present (LE) -- bitmask of which fields follow
```

The `length` field tells you where the 802.11 frame starts: `frame = buf + radiotap_len`.

To extract rate, RSSI, channel from radiotap, walk the present bitmask. Each bit corresponds to a field in order, with specific sizes and alignment requirements:

| Bit | Field | Size | Alignment |
|-----|-------|------|-----------|
| 0 | TSFT | 8 | 8 |
| 1 | FLAGS | 1 | 1 |
| 2 | RATE | 1 | 1 |
| 3 | CHANNEL | 2+2 | 2 |
| 4 | FHSS | 2 | 2 |
| 5 | DBM_ANTSIGNAL | 1 | 1 |
| 6 | DBM_ANTNOISE | 1 | 1 |

If bit 31 is set in present, another u32 present word follows (extension).

**Parsing approach**: iterate bits 0-30 of present. For each set bit, align the cursor per the field's alignment requirement, then read/skip the field. Extract values for the bits we care about (RATE=bit2, CHANNEL=bit3, DBM_ANTSIGNAL=bit5, FLAGS=bit1). FLAGS bit 4 (0x10) = FCS present at end of frame.

If FLAGS indicates FCS present, strip the last 4 bytes from the 802.11 frame before forwarding to the hub.

**Fallback**: if radiotap parsing fails or fields are missing, use configured defaults (bridge's channel_freq, rate=0x0B, rssi=-30).

### Rate code mapping table

The medium protocol uses legacy rate codes. Radiotap uses 500kbps units. Bidirectional mapping:

```c
/* radiotap 500kbps <-> legacy rate_code */
static const struct { uint8_t rt_rate; uint8_t ath_code; } rate_map[] = {
    {  2, 0x1B },  /*  1   Mbps CCK */
    {  4, 0x1A },  /*  2   Mbps CCK */
    { 11, 0x19 },  /*  5.5 Mbps CCK */
    { 22, 0x18 },  /* 11   Mbps CCK */
    { 12, 0x0B },  /*  6   Mbps OFDM */
    { 18, 0x0F },  /*  9   Mbps OFDM */
    { 24, 0x0A },  /* 12   Mbps OFDM */
    { 36, 0x0E },  /* 18   Mbps OFDM */
    { 48, 0x09 },  /* 24   Mbps OFDM */
    { 72, 0x0D },  /* 36   Mbps OFDM */
    { 96, 0x08 },  /* 48   Mbps OFDM */
    {108, 0x0C },  /* 54   Mbps OFDM */
};
```

Write two helper functions: `rt_rate_to_ath(uint8_t rt) -> uint8_t` and `ath_rate_to_rt(uint8_t ath) -> uint8_t`, with 0x0B/12 as defaults for unknown codes.

## Channel/Bandwidth Configuration

### channel_to_freq() function

Map channel numbers to MHz:

```
2.4 GHz: ch 1=2412, 2=2417, ..., 13=2472, 14=2484  (formula: 2407 + ch*5, except ch14=2484)
5 GHz:   ch 36=5180, 40=5200, 44=5220, ..., 165=5825  (formula: 5000 + ch*5)
```

If the `-c` value is >200, treat it as MHz directly. Otherwise treat as channel number and convert.

### Bandwidth configuration

Based on `-w` flag, compute the channel fields for the medium header AND the frequency acceptance range for the channel filter:

| -w | channel_flags | channel_bond_freq | center_freq1 | center_freq2 | Acceptance range |
|----|--------------|-------------------|--------------|-------------|-----------------|
| HT20 (default) | band + HT20 | 0 | 0 | 0 | primary only |
| HT40+ | band + HT40PLUS | primary+20 | 0 | 0 | primary, primary+20 |
| HT40- | band + HT40MINUS | primary-20 | 0 | 0 | primary-20, primary |
| VHT80 | band + VHT80 | 0 | center of 80MHz block | 0 | 4 channels in block |
| VHT160 | band + VHT160 | 0 | center of 160MHz block | 0 | 8 channels in block |
| VHT80+80 | band + VHT80_80 | 0 | center1 | from -s flag | 4+4 channels |

Band flag: `VWIFI_CHAN_FLAG_2GHZ` if freq < 5000, else `VWIFI_CHAN_FLAG_5GHZ`.

For VHT80: the 80MHz blocks in 5GHz are: {36,40,44,48}, {52,56,60,64}, {100,104,108,112}, {116,120,124,128}, {132,136,140,144}, {149,153,157,161}. center_freq1 is the center of whichever block contains the primary channel. Store `freq_lo` and `freq_hi` (lowest and highest 20MHz channel center freq in the block) for filter matching.

For VHT160: pairs of adjacent 80MHz blocks.

## Hub-to-Physical Path

1. Reassemble a complete length-prefixed message from the hub stream socket
2. Validate: payload_len >= 28 (v1 min header), magic == 0x46495756
3. Extract `channel_freq` from offset 28-29 of payload (little-endian u16). If payload is <30 bytes (v1), treat as channel_freq=0.
4. **Channel filter**: accept if `channel_freq == 0` OR `channel_freq` falls within `[freq_lo, freq_hi]` of the bridge's configured bandwidth range
5. Extract the 802.11 frame: `payload + 40` for v2 headers (or `payload + 28` for v1, but v1 is unlikely)
6. Record echo hash before injection (see echo suppression below)
7. Build injection buffer: `inject_radiotap[8] + 802.11_frame`
8. Write to AF_PACKET socket

## Physical-to-Hub Path

1. Read from AF_PACKET socket -> `[radiotap][802.11 frame]`
2. Validate radiotap: version==0, length >= 8, length < total_read
3. Parse radiotap to extract: rate (bit 2), channel freq (bit 3), signal dBm (bit 5), flags (bit 1)
4. The 802.11 frame starts at `buf + radiotap_length`
5. If radiotap FLAGS bit 4 (FCS_INCLUDED) is set, strip last 4 bytes from the 802.11 frame
6. Minimum frame size check: need at least 10 bytes (FC + dur + addr1)
7. **Echo check**: compute hash, check ring, drop if echo (see below)
8. Extract tx_mac from 802.11 header: Address 2 (transmitter) at offset 10-15 of the 802.11 frame. For control frames that lack addr2, skip forwarding.
9. Build `vwifi_frame_hdr`:
   - magic = `0x46495756`
   - version = 2
   - frame_len = length of 802.11 frame
   - tx_mac = addr2 from 802.11 header
   - rate_code = converted from radiotap rate (or default 0x0B)
   - rssi = from radiotap signal (or default -30)
   - tsf_lo/tsf_hi = 0 (or from radiotap TSFT if available)
   - flags = 0
   - channel_freq = from radiotap channel, fallback to bridge's configured freq
   - channel_flags = bridge's configured flags
   - channel_bond_freq, center_freq1, center_freq2 = bridge's configured values
10. Send: `[uint32_t htonl(40 + frame_len)][header][frame]`

## Echo Suppression

When we inject a frame via monitor mode, the radio captures it back. We must suppress these echoes.

**FNV-1a hash ring buffer:**

```c
#define ECHO_RING_SIZE 256
#define ECHO_EXPIRE_MS 100

struct echo_entry {
    uint32_t hash;
    uint16_t frame_len;
    uint64_t timestamp_ms;  /* from clock_gettime(CLOCK_MONOTONIC) */
};

static struct echo_entry echo_ring[ECHO_RING_SIZE];
static int echo_ring_head = 0;
```

**FNV-1a hash**:
```c
static uint32_t fnv1a(const uint8_t *data, size_t len) {
    uint32_t h = 0x811C9DC5;
    for (size_t i = 0; i < len; i++) {
        h ^= data[i];
        h *= 0x01000193;
    }
    return h;
}
```

**On inject** (hub->phys, step 6): compute `fnv1a(802.11_frame, frame_len)`, store `{hash, frame_len, now_ms}` in ring at `echo_ring_head`, advance head.

**On capture** (phys->hub, step 7): compute hash of captured 802.11 frame. Scan the ring for any entry where `hash == captured_hash && frame_len == captured_len && (now - entry.timestamp) < ECHO_EXPIRE_MS`. If found, zero out that entry and drop the frame.

## Main Loop

```c
struct pollfd pfds[2];
pfds[0].fd = raw_fd;       pfds[0].events = POLLIN;
pfds[1].fd = hub_fd;       pfds[1].events = POLLIN;

while (g_running) {
    int n = poll(pfds, 2, 1000);
    if (n < 0) { if (errno == EINTR) continue; break; }
    if (pfds[1].revents & POLLIN) handle_hub_data();   /* hub->phys */
    if (pfds[0].revents & POLLIN) handle_phys_data();   /* phys->hub */
    if ((pfds[0].revents | pfds[1].revents) & (POLLERR | POLLHUP)) break;
}
```

Signal handling: `SIGINT`/`SIGTERM` -> set `g_running = 0`. `SIGPIPE` -> `SIG_IGN`.

On shutdown, print stats:
```
bridge: shutting down -- hub->phys: %u frames, phys->hub: %u frames, echoes suppressed: %u
```

## Hub Stream Reassembly

The hub socket is a stream, so you need a receive buffer for partial reads:

```c
static uint8_t hub_rxbuf[4 + 40 + 8192];
static size_t hub_rxlen = 0;

static void handle_hub_data(void) {
    ssize_t n = read(hub_fd, hub_rxbuf + hub_rxlen, sizeof(hub_rxbuf) - hub_rxlen);
    if (n <= 0) { /* disconnect */ g_running = 0; return; }
    hub_rxlen += n;

    while (hub_rxlen >= 4) {
        uint32_t payload_len = ntohl(*(uint32_t *)hub_rxbuf);
        if (payload_len > 40 + 8192) { hub_rxlen = 0; break; } /* corrupt */
        if (hub_rxlen < 4 + payload_len) break; /* need more data */

        process_hub_message(hub_rxbuf + 4, payload_len);

        size_t consumed = 4 + payload_len;
        hub_rxlen -= consumed;
        if (hub_rxlen > 0) memmove(hub_rxbuf, hub_rxbuf + consumed, hub_rxlen);
    }
}
```

## Error Handling

- **Monitor mode check at startup**: read `/sys/class/net/<ifname>/type`, must be `803`. If not, print:
  ```
  Error: %s is not in monitor mode (type=%d, expected 803)
  Fix: sudo ip link set %s down && sudo iw dev %s set type monitor && sudo ip link set %s up
  ```
- **Interface exists check**: `if_nametoindex()` returns 0 if interface doesn't exist
- **Hub connect failure**: print path and suggest checking if hub is running
- **Oversized frames**: silently drop (>8192 bytes)
- **Malformed radiotap**: drop, increment counter, log in verbose mode
- **Hub disconnect**: log and exit cleanly

## Style

- Single-file C, no external deps
- Follow the patterns in `vwifi_host_relay.c` for code style (same project)
- Use `fprintf(stderr, "bridge: ...")` for all logging
- Verbose logging (`-v` flag) for per-frame messages; always log startup/shutdown/errors
- `static` for all file-scope functions and variables
