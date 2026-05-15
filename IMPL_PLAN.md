# vwifi-phys-bridge.c — Implementation Steps

Incremental build plan. Each step adds to `vwifi-phys-bridge.c` and should
compile cleanly before moving to the next.

## Step 1: Scaffolding + CLI parsing

- Includes: `<stdio.h>`, `<stdlib.h>`, `<string.h>`, `<unistd.h>`, `<errno.h>`,
  `<signal.h>`, `<poll.h>`, `<getopt.h>`, `<stdint.h>`, `<time.h>`,
  `<net/if.h>`, `<sys/socket.h>`, `<sys/un.h>`, `<arpa/inet.h>`,
  `<linux/if_packet.h>`, `<linux/if_ether.h>`, `"vwifi.h"`
- Global config struct holding: hub socket path, interface name, channel,
  bandwidth mode string, center2 MHz, node ID, verbose flag
- Bandwidth channel state struct: `channel_freq`, `channel_flags`,
  `channel_bond_freq`, `center_freq1`, `center_freq2`, `freq_lo`, `freq_hi`
- `static volatile sig_atomic_t g_running = 1;` + signal handler
- Stats counters: `hub_to_phys`, `phys_to_hub`, `echoes_suppressed`
- `usage()` function
- `main()` with `getopt` loop parsing `-c`, `-w`, `-s`, `-n`, `-v`, `-h`
- Validate required args (hub path, interface, `-c`), print config, return 0
- Stub `// TODO` comments for where later steps plug in

**Compile test**: `gcc -Wall -Wextra -O2 -o vwifi-phys-bridge vwifi-phys-bridge.c`

## Step 2: Channel/frequency helpers

- `channel_to_freq(int ch)` — maps channel number to MHz
  (2.4 GHz: 2407+ch*5, ch14=2484; 5 GHz: 5000+ch*5)
- `compute_channel_config()` — based on `-c` and `-w` flags, fills the
  channel state struct: primary freq, band flag, channel_flags,
  channel_bond_freq, center_freq1/2, freq_lo/freq_hi
- VHT80 block table: `{36,40,44,48}`, `{52,56,60,64}`, etc.
- Call from `main()` after arg parsing; print config in verbose mode

## Step 3: Rate mapping helpers

- Rate map table: radiotap 500kbps units <-> legacy rate codes (12 entries)
- `rt_rate_to_ath(uint8_t rt)` — lookup, default 0x0B
- `ath_rate_to_rt(uint8_t ath)` — lookup, default 12

## Step 4: Echo suppression

- `ECHO_RING_SIZE 256`, `ECHO_EXPIRE_MS 100`
- `struct echo_entry { uint32_t hash; uint16_t frame_len; uint64_t timestamp_ms; }`
- `echo_ring[ECHO_RING_SIZE]`, `echo_ring_head`
- `fnv1a(data, len)` — FNV-1a 32-bit hash
- `now_ms()` — `clock_gettime(CLOCK_MONOTONIC)` in milliseconds
- `echo_record(frame, len)` — hash + store in ring
- `echo_check(frame, len)` — hash + scan ring, return 1 if echo found

## Step 5: Hub connection

- `connect_hub(path)` — `AF_UNIX SOCK_STREAM` connect
- `send_hello(fd, node_id)` — build and send hello message:
  `[uint32_t len][uint32_t 0x52495756][node_id\0]`
- `write_all(fd, buf, len)` — write with EINTR retry
- Wire into `main()`: connect, send hello, print status

## Step 6: Physical interface setup

- `check_monitor_mode(ifname)` — read `/sys/class/net/<ifname>/type`,
  verify value is 803
- `open_raw_socket(ifname)` — `AF_PACKET SOCK_RAW htons(ETH_P_ALL)`,
  bind to interface index
- Wire into `main()`: validate monitor mode, open socket, print status

## Step 7: Radiotap parsing

- Struct for parsed radiotap results: rate, channel_freq, signal_dbm,
  flags, tsft, has_* booleans
- `parse_radiotap(buf, len, result)` — walk present bitmask bits 0-6
  with proper alignment; handle extension bitmasks (bit 31)
- Extract: TSFT (bit 0), FLAGS (bit 1), RATE (bit 2),
  CHANNEL (bit 3, freq+flags), DBM_ANTSIGNAL (bit 5)

## Step 8: Hub-to-physical path

- `process_hub_message(payload, payload_len)`:
  1. Validate: len >= 28, magic == 0x46495756
  2. Extract channel_freq from header
  3. Channel filter: accept if freq==0 or within [freq_lo, freq_hi]
  4. Extract 802.11 frame (payload + 40 for v2)
  5. Record echo hash
  6. Build inject buffer: 8-byte empty radiotap + frame
  7. Write to raw socket
  8. Increment `hub_to_phys` counter
- Static inject radiotap header (8 bytes, present=0)

## Step 9: Physical-to-hub path

- `handle_phys_data()`:
  1. Read from raw socket -> [radiotap][802.11]
  2. Validate radiotap: version==0, length >= 8
  3. Parse radiotap for rate, channel, signal, flags
  4. Strip FCS if FLAGS bit 4 set
  5. Min frame size check (10 bytes)
  6. Echo check — drop if echo
  7. Extract tx_mac (addr2 at offset 10); skip control frames without addr2
  8. Build `vwifi_frame_hdr` with all fields
  9. Send: `[htonl(40+frame_len)][header][frame]`
  10. Increment `phys_to_hub` counter

## Step 10: Hub stream reassembly + main loop

- `hub_rxbuf[]` and `hub_rxlen` for stream reassembly
- `handle_hub_data()`:
  1. Read into buffer
  2. Loop: extract length prefix, check completeness, call `process_hub_message()`,
     memmove remainder
- Main `poll()` loop in `main()`:
  - `pfds[0]` = raw_fd, `pfds[1]` = hub_fd
  - Handle POLLIN on both, POLLERR/POLLHUP -> break
- Shutdown: print stats line
