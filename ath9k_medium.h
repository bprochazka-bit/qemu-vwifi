/*
 * Virtual Atheros AR9285 – Virtual Wireless Medium Protocol
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Defines the wire protocol for exchanging 802.11 frames between
 * QEMU instances via a chardev backend (typically a Unix domain socket
 * connected to a hub process).
 *
 * Phase 3+: Virtual medium transport with channel-aware filtering.
 *
 * Architecture:
 *   QEMU-A  <--chardev socket-->  ath9k_medium_hub  <--socket-->  QEMU-B
 *
 * Each QEMU instance connects its ath9k-virt device to the hub via
 *   -chardev socket,id=medium,path=/tmp/ath9k.sock,server=off
 *   -device ath9k-virt,chardev=medium
 *
 * The hub is a channel-aware fan-out process: frames received from one
 * client are forwarded only to clients on the same channel (or
 * channel 0 = broadcast/unknown which goes everywhere for backward
 * compat).
 *
 * Wire protocol (stream socket, length-prefixed):
 *   [uint32_t length (network byte order)]  -- total bytes following
 *   [struct ath9k_medium_frame_hdr]         -- medium header
 *   [uint8_t payload[...]]                  -- raw 802.11 frame
 *
 * The medium header carries metadata needed by the receiving device
 * to populate the RX descriptor status fields (RSSI, rate, etc.).
 *
 * Protocol version history:
 *   v1: Original protocol, no channel info
 *   v2: Added channel_freq, channel_flags, channel_bond_freq fields;
 *       hub performs channel-aware filtering
 */

#ifndef ATH9K_MEDIUM_H
#define ATH9K_MEDIUM_H

#include <stdint.h>

/* ================================================================
 *  Wire protocol constants
 * ================================================================ */

/* Protocol magic / version for future extensibility */
#define ATH9K_MEDIUM_MAGIC              0x41394B57  /* "A9KW" */
#define ATH9K_MEDIUM_VERSION            2

/* Maximum 802.11 frame size we'll transport.
 * Standard max is 2346 + headers, but AMSDU can be up to ~7935.
 * Use 8192 for comfortable headroom. */
#define ATH9K_MEDIUM_MAX_FRAME_SIZE     8192

/* Total max message size on the wire: header + max frame */
#define ATH9K_MEDIUM_MAX_MSG_SIZE       \
    (sizeof(struct ath9k_medium_frame_hdr) + ATH9K_MEDIUM_MAX_FRAME_SIZE)

/* Short aliases used by the driver and relay. The kernel side and the
 * relay daemon predate the _SIZE-suffixed names; provide aliases so
 * every component compiles against a single canonical header. */
#define ATH9K_MEDIUM_MAX_FRAME          ATH9K_MEDIUM_MAX_FRAME_SIZE
#define ATH9K_MEDIUM_MAX_MSG            ATH9K_MEDIUM_MAX_MSG_SIZE

/* Misc-device name exposed by the kernel module, as in
 *   /dev/ath9k_medium
 * The relay opens this path; the driver uses it as miscdevice .name. */
#define ATH9K_MEDIUM_CHRDEV_NAME        "ath9k_medium"

/* ================================================================
 *  Frame header – prepended to every 802.11 frame on the wire
 *
 *  V2 layout (40 bytes):
 *    Offset  Size  Field
 *    0       4     magic
 *    4       2     version
 *    6       2     frame_len
 *    8       6     tx_mac
 *    14      1     rate_code
 *    15      1     rssi (signed)
 *    16      4     tsf_lo
 *    20      4     tsf_hi
 *    24      4     flags            (byte 27 = TTL, used by hub)
 *    28      2     channel_freq     (MHz, e.g. 2412 for ch1)
 *    30      2     channel_flags    (see CHAN_FLAG_* below)
 *    32      2     channel_bond_freq (MHz, secondary channel for HT40, 0=none)
 *    34      2     center_freq1     (VHT/HE center freq of primary segment, 0=N/A)
 *    36      2     center_freq2     (VHT80+80 center freq of secondary segment, 0=N/A)
 *    38      2     _reserved        (must be zero)
 *
 *  Backward compatibility:
 *    - v1 clients send 28-byte headers with no channel info.
 *      The hub treats these as channel_freq=0 (broadcast to all).
 *    - v2 clients send 40-byte headers.
 *    - The hub checks the version field to distinguish.
 * ================================================================ */
struct ath9k_medium_frame_hdr {
    uint32_t    magic;              /* ATH9K_MEDIUM_MAGIC */
    uint16_t    version;            /* ATH9K_MEDIUM_VERSION */
    uint16_t    frame_len;          /* Length of 802.11 frame following this header */
    uint8_t     tx_mac[6];          /* Transmitting device's MAC address */
    uint8_t     rate_code;          /* TX rate code (OFDM/CCK rate from ath9k_dma.h) */
    int8_t      rssi;               /* Simulated RSSI (dBm, default -30) */
    uint32_t    tsf_lo;             /* TSF timestamp low word at TX time */
    uint32_t    tsf_hi;             /* TSF timestamp high word at TX time */
    uint32_t    flags;              /* byte 27 = TTL (hub use); rest reserved */
    /* --- v2 channel fields (12 bytes) --- */
    uint16_t    channel_freq;       /* Center frequency in MHz (0 = unknown/all) */
    uint16_t    channel_flags;      /* Channel flags (see below) */
    uint16_t    channel_bond_freq;  /* HT40 secondary channel freq (0 = none) */
    /* --- v2+ extended channel fields (using formerly reserved bytes) --- */
    uint16_t    center_freq1;       /* VHT/HE: center freq of primary segment (MHz), 0=N/A */
    uint16_t    center_freq2;       /* VHT80+80: center freq of secondary 80MHz segment, 0=N/A */
    uint8_t     _reserved[2];       /* Must be zero */
};

/* V1 header size (for backward compatibility detection) */
#define ATH9K_MEDIUM_HDR_SIZE_V1    28

/* V2 header size */
#define ATH9K_MEDIUM_HDR_SIZE       sizeof(struct ath9k_medium_frame_hdr)

/* Minimum header we'll accept (v1 compat) */
#define ATH9K_MEDIUM_HDR_SIZE_MIN   ATH9K_MEDIUM_HDR_SIZE_V1

/* ================================================================
 *  Channel flags
 * ================================================================ */
#define ATH9K_CHAN_FLAG_2GHZ         0x0001  /* 2.4 GHz band */
#define ATH9K_CHAN_FLAG_5GHZ         0x0002  /* 5 GHz band */
#define ATH9K_CHAN_FLAG_HT20         0x0004  /* HT20 */
#define ATH9K_CHAN_FLAG_HT40PLUS     0x0008  /* HT40+ (secondary above) */
#define ATH9K_CHAN_FLAG_HT40MINUS    0x0010  /* HT40- (secondary below) */
#define ATH9K_CHAN_FLAG_VHT80        0x0020  /* VHT 80 MHz */
#define ATH9K_CHAN_FLAG_VHT160       0x0040  /* VHT 160 MHz */
#define ATH9K_CHAN_FLAG_VHT80_80     0x0080  /* VHT 80+80 MHz */
#define ATH9K_CHAN_FLAG_HE20         0x0100  /* HE20 */
#define ATH9K_CHAN_FLAG_HE40         0x0200  /* HE40 */
#define ATH9K_CHAN_FLAG_HE80         0x0400  /* HE80 */
#define ATH9K_CHAN_FLAG_HE160        0x0800  /* HE160 */
#define ATH9K_CHAN_FLAG_HE80_80      0x1000  /* HE 80+80 */

/* Convenience masks used by the hub's wide-channel filter logic. */
#define ATH9K_CHAN_FLAG_WIDE80_MASK  \
    (ATH9K_CHAN_FLAG_VHT80 | ATH9K_CHAN_FLAG_HE80)
#define ATH9K_CHAN_FLAG_WIDE160_MASK \
    (ATH9K_CHAN_FLAG_VHT160 | ATH9K_CHAN_FLAG_HE160)
#define ATH9K_CHAN_FLAG_WIDE80_80_MASK \
    (ATH9K_CHAN_FLAG_VHT80_80 | ATH9K_CHAN_FLAG_HE80_80)
#define ATH9K_CHAN_FLAG_NEEDS_CENTER1 \
    (ATH9K_CHAN_FLAG_WIDE80_MASK | ATH9K_CHAN_FLAG_WIDE160_MASK | \
     ATH9K_CHAN_FLAG_WIDE80_80_MASK)
#define ATH9K_CHAN_FLAG_NEEDS_CENTER2 \
    ATH9K_CHAN_FLAG_WIDE80_80_MASK

/* ================================================================
 *  Rate-code namespace (one byte, internal to the medium)
 *
 *    0x00..0x1F   Legacy OFDM (802.11a/g) and CCK (802.11b)
 *    0x80..0x87   HT20  NSS=1  MCS 0..7
 *    0x88..0x8F   HT20  NSS=2  MCS 0..7    (i.e. MCS 8..15)
 *    0x90..0x97   HT40  NSS=1  MCS 0..7
 *    0x98..0x9F   HT40  NSS=2  MCS 0..7    (i.e. MCS 8..15)
 *    0xA0..0xA9   VHT80 NSS=1  MCS 0..9
 *    0xB0..0xB9   VHT80 NSS=2  MCS 0..9
 *    0xC0..0xC9   VHT160 NSS=1 MCS 0..9
 *    0xD0..0xD9   VHT160 NSS=2 MCS 0..9
 *    0xE0..0xEB   HE-SU 80  NSS=1 MCS 0..11
 *    0xF0..0xFB   HE-SU 80  NSS=2 MCS 0..11
 *
 *  The driver translates mac80211's tx_info->control.rates[0] (HT/VHT
 *  MCS hint with NSS in the upper nibble of .idx) into one of these
 *  codes; the hub uses the code to look up min_snr for the FER model.
 *  HE rate codes are reachable via direct medium injection today --
 *  mac80211 doesn't surface a clean per-frame HE MCS hint to drivers
 *  in tx_info, so HE traffic from a real VM lands as VHT codes after
 *  rate-control falls back through its MCS hierarchy.
 * ================================================================ */
#define ATH9K_RC_LEGACY_BASE    0x00
#define ATH9K_RC_HT20_NSS1_BASE 0x80
#define ATH9K_RC_HT20_NSS2_BASE 0x88
#define ATH9K_RC_HT40_NSS1_BASE 0x90
#define ATH9K_RC_HT40_NSS2_BASE 0x98
#define ATH9K_RC_VHT80_NSS1_BASE  0xA0
#define ATH9K_RC_VHT80_NSS2_BASE  0xB0
#define ATH9K_RC_VHT160_NSS1_BASE 0xC0
#define ATH9K_RC_VHT160_NSS2_BASE 0xD0
#define ATH9K_RC_HE80_NSS1_BASE   0xE0
#define ATH9K_RC_HE80_NSS2_BASE   0xF0

/* ================================================================
 *  Channel bonding virtual channel
 *
 *  When a device uses HT40 (channel bonding), the hub creates a
 *  "virtual bonded channel" by OR-ing the primary and secondary
 *  frequencies into a synthetic channel ID. Only other devices
 *  using the exact same bonded pair will receive the frame.
 *
 *  For filtering purposes, the hub uses:
 *    - If channel_bond_freq != 0: match on (channel_freq, channel_bond_freq)
 *    - If channel_bond_freq == 0: match on channel_freq only
 *    - If channel_freq == 0: broadcast to all (v1 compat / unknown)
 * ================================================================ */

/* ================================================================
 *  Default medium parameters
 * ================================================================ */

/* Default RSSI for frames received over the medium.
 * -30 dBm = excellent signal, appropriate for "same machine" VMs. */
#define ATH9K_MEDIUM_DEFAULT_RSSI       (-30)

/* Default TX rate code (6 Mbps OFDM) */
#define ATH9K_MEDIUM_DEFAULT_RATE       0x0B

/* ================================================================
 *  2.4 GHz channel table (for convenience)
 * ================================================================ */
#define ATH9K_CHAN_FREQ_1            2412
#define ATH9K_CHAN_FREQ_2            2417
#define ATH9K_CHAN_FREQ_3            2422
#define ATH9K_CHAN_FREQ_4            2427
#define ATH9K_CHAN_FREQ_5            2432
#define ATH9K_CHAN_FREQ_6            2437
#define ATH9K_CHAN_FREQ_7            2442
#define ATH9K_CHAN_FREQ_8            2447
#define ATH9K_CHAN_FREQ_9            2452
#define ATH9K_CHAN_FREQ_10           2457
#define ATH9K_CHAN_FREQ_11           2462
#define ATH9K_CHAN_FREQ_12           2467
#define ATH9K_CHAN_FREQ_13           2472
#define ATH9K_CHAN_FREQ_14           2484

/* ================================================================
 *  Receive buffer for reassembling length-prefixed messages
 *  from the stream socket.
 * ================================================================ */

/* Buffer size: 4 (length prefix) + max message */
#define ATH9K_MEDIUM_RXBUF_SIZE         \
    (4 + ATH9K_MEDIUM_HDR_SIZE + ATH9K_MEDIUM_MAX_FRAME_SIZE)

#endif /* ATH9K_MEDIUM_H */
