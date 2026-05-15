/*
 * ath9k Virtual Wireless Medium Hub
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * A fan-out hub for the ath9k-virt virtual wireless medium with
 * per-link SNR simulation and a runtime control channel.
 *
 * Identity model:
 *   Each QEMU peer is bound to a "node" identified by a stable
 *   string node_id (e.g. "ap1", "sta2").  Node identity is
 *   independent of MAC addresses and survives guest reboots,
 *   MAC randomization, and even VM shutdown/restart.
 *
 *   Node IDs come from:  1) QEMU hello message (node_id= property),
 *   2) pre-created via control channel, 3) auto "node0","node1",...
 *
 * Hello protocol (sent by QEMU right after connecting):
 *   [uint32_t len (net order)] [uint32_t 0x41394B52] [node_id\0]
 *   Hub absorbs the hello and binds peer to that node.
 *
 * Features:
 *   - Up to 256 local QEMU clients via Unix domain socket
 *   - TCP listen port for incoming bridge connections from remote hubs
 *   - Outbound TCP upstream connections to remote hubs (-u host:port)
 *   - TTL-based loop prevention for multi-hub topologies
 *   - Uses poll() instead of select() -- no FD_SETSIZE limit
 *   - Automatic reconnection to upstreams every 3 seconds
 *   - Per-link SNR with rate-dependent frame error simulation
 *   - Log-distance path loss model with position-based SNR
 *   - Runtime control channel for adjusting parameters live
 *   - Persistent node identity across reboots/MAC changes
 *
 * Usage:
 *   # Simple local hub (same as before):
 *   ./ath9k_medium_hub /tmp/ath9k-medium.sock
 *
 *   # Hub with control socket:
 *   ./ath9k_medium_hub /tmp/ath9k.sock -c /tmp/ath9k.ctl
 *
 *   # Hub with initial config + control socket:
 *   ./ath9k_medium_hub /tmp/ath9k.sock -c /tmp/ath9k.ctl -C medium.cfg
 *
 *   # Hub with TCP bridge + control:
 *   ./ath9k_medium_hub /tmp/ath9k.sock -t 5550 -c /tmp/ath9k.ctl
 *
 * Control channel (connect with socat/nc/python to the -c socket):
 *   echo "LIST_PEERS" | socat - UNIX-CONNECT:/tmp/ath9k.ctl
 *   echo "SET_SNR 02:00:00:00:00:00 02:00:00:00:01:00 25" | socat - UNIX-CONNECT:/tmp/ath9k.ctl
 *
 * Build:
 *   gcc -Wall -O2 -o ath9k_medium_hub ath9k_medium_hub_scalable.c -lm
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include <unistd.h>
#include <signal.h>
#include <errno.h>
#include <fcntl.h>
#include <poll.h>
#include <netdb.h>
#include <math.h>
#include <ctype.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <stdint.h>
#include <inttypes.h>
#include <stdarg.h>
#include <stddef.h>
#include <time.h>
#include <stdbool.h>

#include "ath9k_medium.h"

/* -------------------------------------------------------------------
 *  Configuration constants
 * ------------------------------------------------------------------- */
#define MAX_PEERS           256
#define MAX_UPSTREAMS       16
#define MAX_CTL_CLIENTS     8
#define MAX_LINK_OVERRIDES  256

/* Wire-protocol sizes derived from the canonical header definition. */
#define MAX_MSG_SIZE        (ATH9K_MEDIUM_MAX_MSG_SIZE)
#define RECV_BUF_SIZE       (4 + MAX_MSG_SIZE)
#define CTL_BUF_SIZE        4096
#define CTL_RESP_SIZE       4096

#define DEFAULT_TTL         8
#define UPSTREAM_RETRY_SEC  3

#define MAX_NODES           256
#define MAX_MACS_PER_NODE   16
#define NODE_ID_LEN         32
#define HELLO_MAGIC         0x41394B52  /* "A9KR" -- registration hello */

/*
 * Medium header field offsets (bytes from start of payload, after the
 * 4-byte length prefix) -- derived from the canonical struct layout in
 * ath9k_medium.h so they can never drift.
 *
 * TTL is the high byte of the v1 `flags` u32 (offset 27 = 24 + 3),
 * which is unused by the driver and repurposed by the hub for loop
 * prevention in multi-hub topologies.
 *
 * The minimum header size we'll accept is the v1 layout (28 bytes);
 * the v2 channel fields are an optional extension.
 */
#define HDR_OFF_TX_MAC          offsetof(struct ath9k_medium_frame_hdr, tx_mac)
#define HDR_OFF_RATE_CODE       offsetof(struct ath9k_medium_frame_hdr, rate_code)
#define HDR_OFF_RSSI            offsetof(struct ath9k_medium_frame_hdr, rssi)
#define HDR_OFF_FLAGS           offsetof(struct ath9k_medium_frame_hdr, flags)
#define HDR_OFF_CHAN_FREQ       offsetof(struct ath9k_medium_frame_hdr, channel_freq)
#define HDR_OFF_CHAN_BOND_FREQ  offsetof(struct ath9k_medium_frame_hdr, channel_bond_freq)
#define TTL_BYTE_OFFSET         (HDR_OFF_FLAGS + 3)
#define MIN_HDR_SIZE            ATH9K_MEDIUM_HDR_SIZE_MIN

/* -------------------------------------------------------------------
 *  Rate table: legacy + HT + VHT
 *
 *  Rate code → data rate in Mbps, and minimum SNR (dB) required for
 *  <10% frame error rate at 1500-byte frames.  Legacy thresholds are
 *  derived from the same curves wmediumd uses (IEEE 802.11a BER); HT
 *  and VHT thresholds follow standard 802.11n/ac link-budget tables.
 *
 *  Rate-code namespace (single byte, internal to this medium):
 *    0x00..0x1F  legacy: ath9k PLCP codes (OFDM 802.11a/g, CCK 802.11b)
 *    0x80..0x87  HT20  NSS=1  MCS 0..7
 *    0x88..0x8F  HT20  NSS=2  MCS 0..7  (i.e. MCS 8..15)
 *    0x90..0x97  HT40  NSS=1  MCS 0..7
 *    0x98..0x9F  HT40  NSS=2  MCS 0..7  (i.e. MCS 8..15)
 *    0xA0..0xA9  VHT80 NSS=1  MCS 0..9
 *    0xB0..0xB9  VHT80 NSS=2  MCS 0..9
 *
 *  The driver currently only emits legacy codes; HT/VHT entries here
 *  let the physics layer respond correctly once the driver is taught
 *  to encode MCS rates from tx_info->control.rates[0]. Until then,
 *  unknown codes fall through to the 6 Mbps default in
 *  get_frame_error_prob() and behave exactly as before.
 * ------------------------------------------------------------------- */
struct rate_info {
    uint8_t     code;
    double      mbps;
    double      min_snr;    /* dB: below this, loss > 10% */
};

/*
 * SNR thresholds for ~10% FER at 1500-byte frames.
 * Sources: wmediumd per_packet_delivery(), 802.11a/n/ac simulation data,
 * standard MCS link-budget tables.
 */
static const struct rate_info rate_table[] = {
    /* ---- Legacy OFDM rates (802.11a/g) ---- */
    { 0x0B,   6.0,   4.0 },    /* 6 Mbps   - BPSK 1/2 */
    { 0x0F,   9.0,   5.0 },    /* 9 Mbps   - BPSK 3/4 */
    { 0x0A,  12.0,   7.0 },    /* 12 Mbps  - QPSK 1/2 */
    { 0x0E,  18.0,  10.0 },    /* 18 Mbps  - QPSK 3/4 */
    { 0x09,  24.0,  14.0 },    /* 24 Mbps  - 16-QAM 1/2 */
    { 0x0D,  36.0,  18.0 },    /* 36 Mbps  - 16-QAM 3/4 */
    { 0x08,  48.0,  22.0 },    /* 48 Mbps  - 64-QAM 2/3 */
    { 0x0C,  54.0,  25.0 },    /* 54 Mbps  - 64-QAM 3/4 */
    /* ---- Legacy CCK rates (802.11b) ---- */
    { 0x1B,   1.0,   2.0 },    /* 1 Mbps   - DBPSK */
    { 0x1A,   2.0,   4.0 },    /* 2 Mbps   - DQPSK */
    { 0x19,   5.5,   6.0 },    /* 5.5 Mbps - CCK */
    { 0x18,  11.0,   9.0 },    /* 11 Mbps  - CCK */

    /* ---- 802.11n HT20, single spatial stream (MCS 0..7) ---- */
    { 0x80,   6.5,   5.0 },    /* HT20 NSS=1 MCS0  - BPSK 1/2 */
    { 0x81,  13.0,   7.0 },    /* HT20 NSS=1 MCS1  - QPSK 1/2 */
    { 0x82,  19.5,  10.0 },    /* HT20 NSS=1 MCS2  - QPSK 3/4 */
    { 0x83,  26.0,  13.0 },    /* HT20 NSS=1 MCS3  - 16QAM 1/2 */
    { 0x84,  39.0,  17.0 },    /* HT20 NSS=1 MCS4  - 16QAM 3/4 */
    { 0x85,  52.0,  21.0 },    /* HT20 NSS=1 MCS5  - 64QAM 2/3 */
    { 0x86,  58.5,  23.0 },    /* HT20 NSS=1 MCS6  - 64QAM 3/4 */
    { 0x87,  65.0,  25.0 },    /* HT20 NSS=1 MCS7  - 64QAM 5/6 */

    /* ---- 802.11n HT20, two spatial streams (MCS 8..15) ---- */
    /* SU-MIMO needs ~3 dB more SNR than the equivalent NSS=1 MCS. */
    { 0x88,  13.0,   8.0 },    /* HT20 NSS=2 MCS8  */
    { 0x89,  26.0,  10.0 },    /* HT20 NSS=2 MCS9  */
    { 0x8A,  39.0,  13.0 },    /* HT20 NSS=2 MCS10 */
    { 0x8B,  52.0,  16.0 },    /* HT20 NSS=2 MCS11 */
    { 0x8C,  78.0,  20.0 },    /* HT20 NSS=2 MCS12 */
    { 0x8D, 104.0,  24.0 },    /* HT20 NSS=2 MCS13 */
    { 0x8E, 117.0,  26.0 },    /* HT20 NSS=2 MCS14 */
    { 0x8F, 130.0,  28.0 },    /* HT20 NSS=2 MCS15 */

    /* ---- 802.11n HT40, single spatial stream ---- */
    /* Same modulation as HT20 → same SNR threshold; double the rate. */
    { 0x90,  13.5,   5.0 },    /* HT40 NSS=1 MCS0  */
    { 0x91,  27.0,   7.0 },    /* HT40 NSS=1 MCS1  */
    { 0x92,  40.5,  10.0 },    /* HT40 NSS=1 MCS2  */
    { 0x93,  54.0,  13.0 },    /* HT40 NSS=1 MCS3  */
    { 0x94,  81.0,  17.0 },    /* HT40 NSS=1 MCS4  */
    { 0x95, 108.0,  21.0 },    /* HT40 NSS=1 MCS5  */
    { 0x96, 121.5,  23.0 },    /* HT40 NSS=1 MCS6  */
    { 0x97, 135.0,  25.0 },    /* HT40 NSS=1 MCS7  */

    /* ---- 802.11n HT40, two spatial streams ---- */
    { 0x98,  27.0,   8.0 },
    { 0x99,  54.0,  10.0 },
    { 0x9A,  81.0,  13.0 },
    { 0x9B, 108.0,  16.0 },
    { 0x9C, 162.0,  20.0 },
    { 0x9D, 216.0,  24.0 },
    { 0x9E, 243.0,  26.0 },
    { 0x9F, 270.0,  28.0 },

    /* ---- 802.11ac VHT 80 MHz, single spatial stream (MCS 0..9) ---- */
    /* MCS 8/9 add 256-QAM modulation → much higher SNR floors. */
    { 0xA0,  32.5,   5.0 },    /* VHT80 NSS=1 MCS0  */
    { 0xA1,  65.0,   7.0 },    /* VHT80 NSS=1 MCS1  */
    { 0xA2,  97.5,  10.0 },    /* VHT80 NSS=1 MCS2  */
    { 0xA3, 130.0,  13.0 },    /* VHT80 NSS=1 MCS3  */
    { 0xA4, 195.0,  17.0 },    /* VHT80 NSS=1 MCS4  */
    { 0xA5, 260.0,  21.0 },    /* VHT80 NSS=1 MCS5  */
    { 0xA6, 292.5,  23.0 },    /* VHT80 NSS=1 MCS6  */
    { 0xA7, 325.0,  25.0 },    /* VHT80 NSS=1 MCS7  */
    { 0xA8, 390.0,  28.0 },    /* VHT80 NSS=1 MCS8  - 256QAM 3/4 */
    { 0xA9, 433.3,  30.0 },    /* VHT80 NSS=1 MCS9  - 256QAM 5/6 */

    /* ---- 802.11ac VHT 80 MHz, two spatial streams ---- */
    { 0xB0,  65.0,   8.0 },
    { 0xB1, 130.0,  10.0 },
    { 0xB2, 195.0,  13.0 },
    { 0xB3, 260.0,  16.0 },
    { 0xB4, 390.0,  20.0 },
    { 0xB5, 520.0,  24.0 },
    { 0xB6, 585.0,  26.0 },
    { 0xB7, 650.0,  28.0 },
    { 0xB8, 780.0,  31.0 },
    { 0xB9, 866.7,  33.0 },

    /* ---- 802.11ac VHT 160 MHz, single spatial stream ----
     * Same modulation/coding as VHT80 (same min_snr); double the rate.
     * Wide channels also incur ~3 dB more noise from the larger band,
     * but for our use case (low-error simulated medium) we keep
     * thresholds aligned with VHT80 so rate-control behavior is the
     * dominant effect. */
    { 0xC0,  65.0,   5.0 },    /* VHT160 NSS=1 MCS0  */
    { 0xC1, 130.0,   7.0 },
    { 0xC2, 195.0,  10.0 },
    { 0xC3, 260.0,  13.0 },
    { 0xC4, 390.0,  17.0 },
    { 0xC5, 520.0,  21.0 },
    { 0xC6, 585.0,  23.0 },
    { 0xC7, 650.0,  25.0 },
    { 0xC8, 780.0,  28.0 },
    { 0xC9, 866.7,  30.0 },

    /* ---- 802.11ac VHT 160 MHz, two spatial streams ---- */
    { 0xD0, 130.0,   8.0 },
    { 0xD1, 260.0,  10.0 },
    { 0xD2, 390.0,  13.0 },
    { 0xD3, 520.0,  16.0 },
    { 0xD4, 780.0,  20.0 },
    { 0xD5, 1040.0, 24.0 },
    { 0xD6, 1170.0, 26.0 },
    { 0xD7, 1300.0, 28.0 },
    { 0xD8, 1560.0, 31.0 },
    { 0xD9, 1733.3, 33.0 },

    /* ---- 802.11ax HE-SU 80 MHz, single spatial stream (MCS 0..11) ----
     * MCS 10/11 add 1024-QAM, which needs substantially higher SNR
     * than VHT's 256-QAM ceiling. Thresholds for 0..9 mirror VHT80
     * (HE uses the same modulation/coding for that range); 10/11
     * follow published 1024-QAM tables (~32/35 dB). */
    { 0xE0,  36.0,   5.0 },    /* HE80 NSS=1 MCS0  */
    { 0xE1,  72.1,   7.0 },
    { 0xE2, 108.1,  10.0 },
    { 0xE3, 144.1,  13.0 },
    { 0xE4, 216.2,  17.0 },
    { 0xE5, 288.2,  21.0 },
    { 0xE6, 324.3,  23.0 },
    { 0xE7, 360.3,  25.0 },
    { 0xE8, 432.4,  28.0 },
    { 0xE9, 480.4,  30.0 },
    { 0xEA, 540.4,  32.0 },    /* HE80 NSS=1 MCS10 - 1024QAM 3/4 */
    { 0xEB, 600.5,  35.0 },    /* HE80 NSS=1 MCS11 - 1024QAM 5/6 */

    /* ---- 802.11ax HE-SU 80 MHz, two spatial streams ---- */
    { 0xF0,  72.1,   8.0 },
    { 0xF1, 144.1,  10.0 },
    { 0xF2, 216.2,  13.0 },
    { 0xF3, 288.2,  16.0 },
    { 0xF4, 432.4,  20.0 },
    { 0xF5, 576.5,  24.0 },
    { 0xF6, 648.5,  26.0 },
    { 0xF7, 720.6,  28.0 },
    { 0xF8, 864.7,  31.0 },
    { 0xF9, 960.8,  33.0 },
    { 0xFA, 1080.9, 35.0 },
    { 0xFB, 1201.0, 38.0 },
};
#define NUM_RATES   (int)(sizeof(rate_table) / sizeof(rate_table[0]))

/*
 * Frame error probability as a function of SNR margin (dB above threshold).
 * Approximation: FER ≈ 1.0 when margin < -3dB, drops roughly exponentially
 * toward 0 as margin increases.
 *
 * This is a simplified model.  For a given rate with min_snr threshold T
 * and an actual link SNR of S:
 *   margin = S - T
 *   if margin < -3:  FER = 1.0  (always drop)
 *   if margin > 10:  FER = 0.0  (never drop)
 *   otherwise:       FER = exp(-0.5 * margin)  (smooth transition)
 *
 * This gives roughly:
 *   margin -3 → FER ≈ 1.0
 *   margin  0 → FER ≈ 0.60
 *   margin  2 → FER ≈ 0.37
 *   margin  5 → FER ≈ 0.08
 *   margin  8 → FER ≈ 0.018
 *   margin 10 → FER ≈ 0.007
 */
static double get_frame_error_prob(uint8_t rate_code, double snr_db)
{
    int i;
    double min_snr = 4.0;  /* default: assume 6 Mbps if unknown rate */

    for (i = 0; i < NUM_RATES; i++) {
        if (rate_table[i].code == rate_code) {
            min_snr = rate_table[i].min_snr;
            break;
        }
    }

    double margin = snr_db - min_snr;

    if (margin > 10.0)
        return 0.0;
    if (margin < -3.0)
        return 1.0;

    return exp(-0.5 * margin);
}

/* -------------------------------------------------------------------
 *  Propagation model
 * ------------------------------------------------------------------- */
enum model_type {
    MODEL_NONE = 0,         /* no physics — pure fan-out (default) */
    MODEL_SNR_TABLE,        /* per-link manual SNR only (no positions) */
    MODEL_LOG_DISTANCE,     /* log-distance path loss from positions */
    MODEL_FREE_SPACE,       /* free-space path loss */
};

static struct {
    enum model_type type;
    double  path_loss_exp;      /* n for log-distance, default 3.0 */
    double  ref_distance;       /* d0 in meters, default 1.0 */
    double  ref_loss_db;        /* PL(d0) in dB, default 40.0 (at 2.4GHz, 1m) */
    double  noise_floor_dbm;    /* default -95.0 */
} model = {
    .type           = MODEL_NONE,
    .path_loss_exp  = 3.0,
    .ref_distance   = 1.0,
    .ref_loss_db    = 40.0,     /* free-space at 2.4GHz, 1m */
    .noise_floor_dbm = -95.0,
};

static void mac_to_str(const uint8_t *mac, char *buf, size_t buflen)
{
    snprintf(buf, buflen, "%02x:%02x:%02x:%02x:%02x:%02x",
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}

static int parse_mac(const char *str, uint8_t *mac)
{
    unsigned int m[6];
    if (sscanf(str, "%x:%x:%x:%x:%x:%x",
               &m[0], &m[1], &m[2], &m[3], &m[4], &m[5]) != 6)
        return -1;
    for (int i = 0; i < 6; i++) {
        if (m[i] > 255) return -1;
        mac[i] = (uint8_t)m[i];
    }
    return 0;
}

/* -------------------------------------------------------------------
 *  Node table -- persistent physical identity
 *
 *  Nodes persist across peer disconnects.  A node's physical state
 *  (position, tx_power) is retained when its peer disconnects, and
 *  reattached when it reconnects with the same node_id.
 * ------------------------------------------------------------------- */
struct node_phys {
    char        node_id[NODE_ID_LEN];
    uint8_t     macs[MAX_MACS_PER_NODE][6];
    int         num_macs;
    double      pos_x, pos_y, pos_z;
    bool        pos_set;
    double      tx_power_dbm;
    uint64_t    tx_frames;
    uint64_t    rx_frames;
    uint64_t    rx_dropped;
    int         peer_idx;       /* bound peer, or -1 if offline */
    bool        active;
    bool        auto_named;     /* true = auto "node%d" id (recycled on disconnect) */
};

static struct node_phys nodes[MAX_NODES];
static int num_nodes = 0;
static int next_auto_id = 0;

static struct node_phys *find_node(const char *nid, bool create)
{
    for (int i = 0; i < num_nodes; i++)
        if (nodes[i].active && strcmp(nodes[i].node_id, nid) == 0)
            return &nodes[i];
    if (!create) return NULL;

    /* Reuse an inactive slot before extending the array, so a churning
     * peer doesn't permanently consume node-table capacity. */
    struct node_phys *nd = NULL;
    for (int i = 0; i < num_nodes; i++) {
        if (!nodes[i].active) { nd = &nodes[i]; break; }
    }
    if (!nd) {
        if (num_nodes >= MAX_NODES) return NULL;
        nd = &nodes[num_nodes++];
    }

    memset(nd, 0, sizeof(*nd));
    snprintf(nd->node_id, NODE_ID_LEN, "%s", nid);
    nd->tx_power_dbm = 15.0;
    nd->peer_idx = -1;
    nd->active = true;
    nd->auto_named = false;
    return nd;
}

static struct node_phys *find_node_by_mac(const uint8_t *mac)
{
    for (int i = 0; i < num_nodes; i++) {
        if (!nodes[i].active) continue;
        for (int m = 0; m < nodes[i].num_macs; m++)
            if (memcmp(nodes[i].macs[m], mac, 6) == 0)
                return &nodes[i];
    }
    return NULL;
}

static void node_add_mac(struct node_phys *nd, const uint8_t *mac)
{
    for (int m = 0; m < nd->num_macs; m++)
        if (memcmp(nd->macs[m], mac, 6) == 0) return;
    if (nd->num_macs >= MAX_MACS_PER_NODE) return;
    memcpy(nd->macs[nd->num_macs], mac, 6);
    nd->num_macs++;
    char ms[20]; mac_to_str(mac, ms, sizeof(ms));
    fprintf(stderr, "hub: node %s: learned MAC %s (%d total)\n",
            nd->node_id, ms, nd->num_macs);
}

static struct node_phys *resolve_node(const char *ident)
{
    struct node_phys *nd = find_node(ident, false);
    if (nd) return nd;
    uint8_t mac[6];
    if (parse_mac(ident, mac) == 0)
        return find_node_by_mac(mac);
    return NULL;
}

static int node_index(const struct node_phys *nd)
{
    return (int)(nd - nodes);
}

/* -------------------------------------------------------------------
 *  Per-link overrides (keyed by node_id pair -- survives MAC changes)
 * ------------------------------------------------------------------- */
struct link_override {
    char        node_a[NODE_ID_LEN];
    char        node_b[NODE_ID_LEN];
    double      snr_ab, snr_ba;
    double      fixed_loss_ab, fixed_loss_ba;
    bool        active;
};

static struct link_override link_overrides[MAX_LINK_OVERRIDES];
static int num_link_overrides = 0;

static struct link_override *find_link(const char *id_a, const char *id_b,
                                       bool create)
{
    for (int i = 0; i < num_link_overrides; i++) {
        if (!link_overrides[i].active) continue;
        if ((strcmp(link_overrides[i].node_a, id_a) == 0 &&
             strcmp(link_overrides[i].node_b, id_b) == 0) ||
            (strcmp(link_overrides[i].node_a, id_b) == 0 &&
             strcmp(link_overrides[i].node_b, id_a) == 0))
            return &link_overrides[i];
    }
    if (!create) return NULL;

    /* Reuse an inactive slot before extending the array, so repeated
     * SET_SNR/CLEAR_SNR cycles don't permanently exhaust capacity. */
    struct link_override *lk = NULL;
    for (int i = 0; i < num_link_overrides; i++) {
        if (!link_overrides[i].active) { lk = &link_overrides[i]; break; }
    }
    if (!lk) {
        if (num_link_overrides >= MAX_LINK_OVERRIDES) return NULL;
        lk = &link_overrides[num_link_overrides++];
    }

    snprintf(lk->node_a, NODE_ID_LEN, "%s", id_a);
    snprintf(lk->node_b, NODE_ID_LEN, "%s", id_b);
    lk->snr_ab = NAN;  lk->snr_ba = NAN;
    lk->fixed_loss_ab = NAN;  lk->fixed_loss_ba = NAN;
    lk->active = true;
    return lk;
}

/*
 * Get effective SNR for a TX->RX link (by node index).
 */
static double get_link_snr(int tx_ni, int rx_ni)
{
    struct node_phys *tx = &nodes[tx_ni], *rx = &nodes[rx_ni];
    struct link_override *lk = find_link(tx->node_id, rx->node_id, false);
    if (lk) {
        if (strcmp(lk->node_a, tx->node_id) == 0) {
            if (!isnan(lk->snr_ab)) return lk->snr_ab;
        } else {
            if (!isnan(lk->snr_ba)) return lk->snr_ba;
            if (!isnan(lk->snr_ab)) return lk->snr_ab;
        }
    }
    if ((model.type == MODEL_LOG_DISTANCE || model.type == MODEL_FREE_SPACE) &&
        tx->pos_set && rx->pos_set) {
        double dx = rx->pos_x - tx->pos_x, dy = rx->pos_y - tx->pos_y;
        double dz = rx->pos_z - tx->pos_z;
        double dist = sqrt(dx*dx + dy*dy + dz*dz);
        if (dist < 0.01) dist = 0.01;
        double pl;
        if (model.type == MODEL_FREE_SPACE)
            pl = 20.0 * log10(dist / model.ref_distance) + model.ref_loss_db;
        else
            pl = model.ref_loss_db + 10.0 * model.path_loss_exp
                 * log10(dist / model.ref_distance);
        return tx->tx_power_dbm - pl - model.noise_floor_dbm;
    }
    return NAN;
}

static double get_fixed_loss(int tx_ni, int rx_ni)
{
    struct node_phys *tx = &nodes[tx_ni], *rx = &nodes[rx_ni];
    struct link_override *lk = find_link(tx->node_id, rx->node_id, false);
    if (!lk) return NAN;
    if (strcmp(lk->node_a, tx->node_id) == 0) {
        if (!isnan(lk->fixed_loss_ab)) return lk->fixed_loss_ab;
    } else {
        if (!isnan(lk->fixed_loss_ba)) return lk->fixed_loss_ba;
        if (!isnan(lk->fixed_loss_ab)) return lk->fixed_loss_ab;
    }
    return NAN;
}

/*
 * Compute the RSSI (dBm) that the receiver should report, given the link SNR.
 */
static int8_t snr_to_rssi(double snr_db)
{
    double rssi = model.noise_floor_dbm + snr_db;
    if (rssi < -127.0) rssi = -127.0;
    if (rssi > 0.0) rssi = 0.0;
    return (int8_t)rssi;
}

/* Simple xorshift64* PRNG for fast, reproducible random numbers */
static uint64_t rng_state = 0x12345678ABCDEF01ULL;

static double rand_uniform(void)
{
    rng_state ^= rng_state >> 12;
    rng_state ^= rng_state << 25;
    rng_state ^= rng_state >> 27;
    uint64_t val = rng_state * 0x2545F4914F6CDD1DULL;
    return (double)(val >> 11) / (double)(1ULL << 53);
}

/* -------------------------------------------------------------------
 *  Data peer structures
 * ------------------------------------------------------------------- */
static volatile int running = 1;

struct peer {
    int         fd;
    uint8_t     buf[RECV_BUF_SIZE];
    uint32_t    buf_used;
    int         is_bridge;
    char        label[64];
    uint64_t    tx_dropped;     /* frames dropped due to socket backpressure */
    /* Channel state learned from peer's most recent TX (v2 header).
     * Both 0 means "unknown" -- such peers receive everything until
     * they transmit and reveal their channel. */
    uint16_t    channel_freq;
    uint16_t    channel_bond_freq;
};

static struct peer peers[MAX_PEERS];
static int peer_node[MAX_PEERS]; /* index into nodes[], or -1 */
static int num_peers = 0;

static int unix_listen_fd = -1;
static int tcp_listen_fd = -1;
static char *socket_path = NULL;

struct upstream_addr {
    char host[256];
    char port[16];
};
static struct upstream_addr upstreams[MAX_UPSTREAMS];
static int num_upstreams = 0;

struct upstream_state {
    int     peer_idx;
    time_t  last_attempt;
};
static struct upstream_state upstream_state[MAX_UPSTREAMS];

/* -------------------------------------------------------------------
 *  Control channel state
 * ------------------------------------------------------------------- */
static int ctl_listen_fd = -1;
static char *ctl_socket_path = NULL;

/* Per-client outbound buffer: lets ctl_respond keep writing across
 * EAGAIN without dropping the response. Sized for the worst-case
 * LIST_PEERS dump (256 nodes * ~120 chars). Excess is dropped (with
 * a warning) -- the responses were always best-effort. */
#define CTL_OUTBUF_SIZE     (64 * 1024)

struct ctl_client {
    int         fd;
    char        buf[CTL_BUF_SIZE];
    uint32_t    buf_used;
    char        outbuf[CTL_OUTBUF_SIZE];
    uint32_t    outbuf_used;
    bool        active;
};

static struct ctl_client ctl_clients[MAX_CTL_CLIENTS];

/* Look up ctl_client index by fd, or -1 if fd isn't a tracked client
 * (e.g. STDERR_FILENO from the startup config-load path). */
static int ctl_client_by_fd(int fd)
{
    for (int i = 0; i < MAX_CTL_CLIENTS; i++) {
        if (ctl_clients[i].active && ctl_clients[i].fd == fd)
            return i;
    }
    return -1;
}

/* Global stats */
static uint64_t total_frames_forwarded = 0;
static uint64_t total_frames_dropped = 0;       /* physics-model drops */
static uint64_t total_frames_tx_dropped = 0;    /* hub->peer socket-backpressure drops */

/* -------------------------------------------------------------------
 *  Utility
 * ------------------------------------------------------------------- */
static void set_nonblock(int fd)
{
    int flags = fcntl(fd, F_GETFL, 0);
    if (flags >= 0) {
        fcntl(fd, F_SETFL, flags | O_NONBLOCK);
    }
}

static void set_tcp_nodelay(int fd)
{
    int val = 1;
    setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &val, sizeof(val));
}

/* Trim leading/trailing whitespace in-place, return pointer to content */
static char *trim(char *s)
{
    while (*s && isspace((unsigned char)*s)) s++;
    char *end = s + strlen(s);
    while (end > s && isspace((unsigned char)end[-1])) end--;
    *end = '\0';
    return s;
}

/* -------------------------------------------------------------------
 *  Peer management
 * ------------------------------------------------------------------- */
static int add_peer(int fd, int is_bridge, const char *label)
{
    if (num_peers >= MAX_PEERS) {
        fprintf(stderr, "hub: max peers reached\n");
        close(fd); return -1;
    }
    set_nonblock(fd);
    /* Enlarge the send buffer so short bursts don't immediately
     * overflow and trigger backpressure drops. */
    {
        int sndbuf = 1024 * 1024;
        setsockopt(fd, SOL_SOCKET, SO_SNDBUF, &sndbuf, sizeof(sndbuf));
    }
    int idx = num_peers;
    peers[idx].fd = fd;
    peers[idx].buf_used = 0;
    peers[idx].is_bridge = is_bridge;
    peers[idx].tx_dropped = 0;
    peers[idx].channel_freq = 0;
    peers[idx].channel_bond_freq = 0;
    snprintf(peers[idx].label, sizeof(peers[idx].label), "%s", label);
    peer_node[idx] = -1;
    fprintf(stderr, "hub: peer %d connected: %s (fd=%d, %s)\n",
            idx, label, fd, is_bridge ? "bridge" : "local");
    num_peers++;
    return idx;
}

static void bind_peer_to_node(int pidx, const char *nid)
{
    struct node_phys *nd = find_node(nid, true);
    if (!nd) { fprintf(stderr, "hub: node table full\n"); return; }

    /* If this peer was already bound to an auto-named node and is now
     * being moved to a different node (typically because a HELLO
     * announced its real id after auto_bind_peer ran), retire the
     * auto-named placeholder so the node table doesn't accumulate
     * orphaned "nodeN" entries under reconnect/hello churn. User-named
     * source nodes are left intact (they may have configured state). */
    int prev_ni = peer_node[pidx];
    int new_ni = node_index(nd);
    if (prev_ni >= 0 && prev_ni != new_ni) {
        struct node_phys *prev = &nodes[prev_ni];
        prev->peer_idx = -1;
        if (prev->auto_named) {
            prev->active = false;
            prev->num_macs = 0;
        }
    }

    if (nd->peer_idx >= 0 && nd->peer_idx != pidx) {
        fprintf(stderr, "hub: node %s: unbinding old peer %d\n", nid, nd->peer_idx);
        peer_node[nd->peer_idx] = -1;
    }
    nd->peer_idx = pidx;
    peer_node[pidx] = new_ni;
    fprintf(stderr, "hub: peer %d -> node '%s'\n", pidx, nid);
}

static void auto_bind_peer(int pidx)
{
    if (peer_node[pidx] >= 0) return;
    char id[NODE_ID_LEN];
    snprintf(id, NODE_ID_LEN, "node%d", next_auto_id++);
    bind_peer_to_node(pidx, id);
    int ni = peer_node[pidx];
    if (ni >= 0)
        nodes[ni].auto_named = true;
}

static void remove_peer(int idx)
{
    int i;

    fprintf(stderr, "hub: peer %d disconnected: %s (fd=%d)\n",
            idx, peers[idx].label, peers[idx].fd);
    if (peers[idx].fd >= 0)
        close(peers[idx].fd);

    /* Detach from node. Auto-named nodes are recycled on disconnect so
     * the table doesn't grow without bound under reconnect churn;
     * user-named nodes persist with their physical state intact. */
    int ni = peer_node[idx];
    if (ni >= 0) {
        nodes[ni].peer_idx = -1;
        if (nodes[ni].auto_named) {
            nodes[ni].active = false;
            nodes[ni].num_macs = 0;
        }
    }
    peer_node[idx] = -1;

    /* Update upstream_state references before the swap */
    for (i = 0; i < num_upstreams; i++) {
        if (upstream_state[i].peer_idx == idx) {
            upstream_state[i].peer_idx = -1;
        } else if (upstream_state[i].peer_idx == num_peers - 1) {
            upstream_state[i].peer_idx = idx;
        }
    }

    /* Swap with last -- also move the node->peer back-reference and
     * the peer_node[] entry so they stay in sync after the swap. */
    if (idx < num_peers - 1) {
        int last = num_peers - 1;
        peers[idx] = peers[last];
        peer_node[idx] = peer_node[last];
        if (peer_node[idx] >= 0)
            nodes[peer_node[idx]].peer_idx = idx;
    }
    num_peers--;
}

static void learn_peer_mac(int pidx, const uint8_t *payload)
{
    const uint8_t *mac = payload + HDR_OFF_TX_MAC;
    static const uint8_t zero[6] = {0};
    if (memcmp(mac, zero, 6) == 0) return;
    auto_bind_peer(pidx);
    int ni = peer_node[pidx];
    if (ni < 0) return;
    /* Migrate MAC from another node if needed */
    struct node_phys *prev = find_node_by_mac(mac);
    if (prev && prev != &nodes[ni]) {
        for (int m = 0; m < prev->num_macs; m++) {
            if (memcmp(prev->macs[m], mac, 6) == 0) {
                prev->num_macs--;
                if (m < prev->num_macs)
                    memcpy(prev->macs[m], prev->macs[prev->num_macs], 6);
                break;
            }
        }
    }
    node_add_mac(&nodes[ni], mac);
}

/* Hello message handling */
static void process_hello(int pidx, const uint8_t *payload, uint32_t len)
{
    if (len < 5) return;
    const char *id = (const char *)(payload + 4);
    size_t maxl = len - 4, slen = strnlen(id, maxl);
    if (slen == 0 || slen >= NODE_ID_LEN) return;
    char nid[NODE_ID_LEN];
    memcpy(nid, id, slen); nid[slen] = '\0';
    fprintf(stderr, "hub: peer %d: hello node_id='%s'\n", pidx, nid);
    bind_peer_to_node(pidx, nid);
}

/* -------------------------------------------------------------------
 *  Non-blocking framed send to a peer.
 *
 *  The peer's socket is non-blocking. When the peer's kernel send
 *  buffer is full at frame boundary we drop the frame rather than
 *  spin -- a slow or hung peer must not stall the whole hub. This
 *  matches the lossy-medium semantics of the simulated radio.
 *
 *  In the rare case where the buffer fills *mid-frame* (after we've
 *  already written some bytes), continuing without finishing the
 *  frame would corrupt the receiver's stream framing. We treat the
 *  peer as broken and let the caller remove it.
 *
 *  Returns:
 *      1  -- whole message written
 *      0  -- dropped at frame boundary (peer slow, but still healthy)
 *     -1  -- peer is broken (partial-write stall or fatal error);
 *            caller must remove this peer.
 * ------------------------------------------------------------------- */
static int peer_send_frame(int idx, const uint8_t *buf, uint32_t len)
{
    uint32_t offset = 0;
    while (offset < len) {
        ssize_t sent = write(peers[idx].fd, buf + offset, len - offset);
        if (sent > 0) {
            offset += (uint32_t)sent;
            continue;
        }
        if (sent < 0 && errno == EINTR)
            continue;
        if (sent < 0 && errno == EAGAIN) {
            if (offset == 0) {
                peers[idx].tx_dropped++;
                total_frames_tx_dropped++;
                return 0;
            }
            return -1;
        }
        return -1;
    }
    return 1;
}

/* -------------------------------------------------------------------
 *  Channel-match policy.
 *
 *  Per the v2 wire protocol (ath9k_medium.h):
 *    - channel_freq == 0 means "unknown / broadcast" and matches anything
 *    - otherwise the primary frequency must match
 *    - if either side has a bond_freq != 0, the bond_freq must also match
 *      (so HT40+ ch6 and HT40- ch6 don't collide with each other)
 *
 *  This is symmetric in the sender/receiver pair, which is what we want
 *  for the simulated medium.
 * ------------------------------------------------------------------- */
static bool channels_match(uint16_t a_freq, uint16_t a_bond,
                           uint16_t b_freq, uint16_t b_bond)
{
    if (a_freq == 0 || b_freq == 0)
        return true;
    if (a_freq != b_freq)
        return false;
    if ((a_bond != 0 || b_bond != 0) && a_bond != b_bond)
        return false;
    return true;
}

/* -------------------------------------------------------------------
 *  Frame forwarding with TTL + physics
 *
 *  forward_message is invoked sequentially from the single-threaded
 *  main loop, so the scratch buffer can be allocated once and reused
 *  -- this keeps an 8 KB+ array off the stack (matters for restricted
 *  stack environments) and avoids per-frame allocation overhead.
 * ------------------------------------------------------------------- */
static uint8_t *fwd_scratch = NULL;     /* 4 + MAX_MSG_SIZE bytes */

static void forward_message(int sender_idx, const uint8_t *msg,
                            uint32_t total_len)
{
    uint8_t *tmp = fwd_scratch;
    uint32_t payload_len;
    int sender_is_bridge;
    uint8_t ttl;
    int forward_to_bridges;
    int i;
    int dead[MAX_PEERS];
    int n_dead = 0;
    uint16_t msg_chan_freq = 0;
    uint16_t msg_chan_bond = 0;

    if (total_len < 4 || total_len > (4 + MAX_MSG_SIZE))
        return;
    payload_len = total_len - 4;

    /* Check for hello message BEFORE min-size filter (hellos are smaller) */
    if (payload_len >= 4) {
        uint32_t magic;
        memcpy(&magic, msg + 4, 4);
        if (magic == HELLO_MAGIC) {
            process_hello(sender_idx, msg + 4, payload_len);
            return;
        }
    }

    if (payload_len < MIN_HDR_SIZE)
        return;

    sender_is_bridge = peers[sender_idx].is_bridge;

    /* Work on a copy so we can modify TTL/RSSI without corrupting source */
    memcpy(tmp, msg, total_len);

    /* Learn MAC from this frame */
    if (!sender_is_bridge)
        learn_peer_mac(sender_idx, tmp + 4);

    int sender_ni = peer_node[sender_idx];
    if (sender_ni >= 0) nodes[sender_ni].tx_frames++;

    /* Extract rate code from the medium header */
    uint8_t rate_code = tmp[4 + HDR_OFF_RATE_CODE];

    /* Extract channel info if the sender uses the v2 header. v1 senders
     * are treated as channel=0 (broadcast) for backward compatibility. */
    if (payload_len >= ATH9K_MEDIUM_HDR_SIZE) {
        memcpy(&msg_chan_freq, tmp + 4 + HDR_OFF_CHAN_FREQ, 2);
        memcpy(&msg_chan_bond, tmp + 4 + HDR_OFF_CHAN_BOND_FREQ, 2);
        /* Cache the sender's current channel so symmetric filtering
         * can match it on subsequent frames sent toward this peer. */
        peers[sender_idx].channel_freq = msg_chan_freq;
        peers[sender_idx].channel_bond_freq = msg_chan_bond;
    }

    /* --- TTL handling (unchanged logic) --- */
    ttl = tmp[4 + TTL_BYTE_OFFSET];

    if (sender_is_bridge) {
        if (ttl == 0) {
            forward_to_bridges = 0;
        } else {
            ttl--;
            tmp[4 + TTL_BYTE_OFFSET] = ttl;
            forward_to_bridges = (ttl > 0);
        }
    } else {
        forward_to_bridges = 1;
        tmp[4 + TTL_BYTE_OFFSET] = DEFAULT_TTL;
    }

    /* --- Forward to each peer with physics filtering --- */
    for (i = 0; i < num_peers; i++) {
        if (i == sender_idx)
            continue;

        if (peers[i].fd < 0)
            continue;

        if (peers[i].is_bridge && !forward_to_bridges)
            continue;

        /* Channel-aware filtering. If we know both sender and receiver
         * channels and they don't match (with HT40 bond awareness),
         * the receiver wouldn't hear this transmission on a real
         * radio, so skip it. */
        if (!channels_match(msg_chan_freq, msg_chan_bond,
                            peers[i].channel_freq,
                            peers[i].channel_bond_freq))
            continue;

        int rx_ni = peer_node[i];

        /* Physics filtering for local peers with bound nodes */
        if (!peers[i].is_bridge && model.type != MODEL_NONE &&
            sender_ni >= 0 && rx_ni >= 0)
        {
            double fixed = get_fixed_loss(sender_ni, rx_ni);
            if (!isnan(fixed)) {
                if (rand_uniform() < fixed) {
                    nodes[rx_ni].rx_dropped++;
                    total_frames_dropped++;
                    continue;
                }
            } else {
                double snr = get_link_snr(sender_ni, rx_ni);
                if (!isnan(snr)) {
                    double fer = get_frame_error_prob(rate_code, snr);
                    if (rand_uniform() < fer) {
                        nodes[rx_ni].rx_dropped++;
                        total_frames_dropped++;
                        continue;
                    }
                    tmp[4 + HDR_OFF_RSSI] = (uint8_t)snr_to_rssi(snr);
                }
            }
        }

        if (rx_ni >= 0) nodes[rx_ni].rx_frames++;
        total_frames_forwarded++;

        int rc;
        if (!peers[i].is_bridge) {
            uint8_t saved_ttl = tmp[4 + TTL_BYTE_OFFSET];
            tmp[4 + TTL_BYTE_OFFSET] = 0;

            rc = peer_send_frame(i, tmp, total_len);

            tmp[4 + TTL_BYTE_OFFSET] = saved_ttl;

            /* Restore original RSSI for next peer (may differ per-link) */
            tmp[4 + HDR_OFF_RSSI] = msg[4 + HDR_OFF_RSSI];
        } else {
            rc = peer_send_frame(i, tmp, total_len);
        }

        if (rc < 0) {
            /* Peer is broken (partial-write stall or fatal error).
             * Close fd now and queue for removal after the loop so
             * we don't disturb iteration. */
            fprintf(stderr, "hub: peer %d (%s) broken on send -- removing\n",
                    i, peers[i].label);
            close(peers[i].fd);
            peers[i].fd = -1;
            dead[n_dead++] = i;
        }
    }

    /* Remove broken peers in reverse order (remove_peer swaps with last). */
    while (n_dead > 0) {
        remove_peer(dead[--n_dead]);
    }
}

/* -------------------------------------------------------------------
 *  Peer data handling
 * ------------------------------------------------------------------- */
static void handle_peer_data(int idx)
{
    struct peer *p = &peers[idx];
    ssize_t n;
    uint32_t net_len, msg_len, consumed;

    n = read(p->fd, p->buf + p->buf_used,
             RECV_BUF_SIZE - p->buf_used);
    if (n <= 0) {
        if (n < 0 && (errno == EAGAIN || errno == EINTR))
            return;
        remove_peer(idx);
        return;
    }
    p->buf_used += (uint32_t)n;

    while (p->buf_used >= 4) {
        memcpy(&net_len, p->buf, 4);
        msg_len = ntohl(net_len);

        if (msg_len > MAX_MSG_SIZE) {
            fprintf(stderr, "hub: peer %d (%s) oversized msg (%u), "
                    "disconnecting\n", idx, p->label, msg_len);
            remove_peer(idx);
            return;
        }

        if (p->buf_used < 4 + msg_len)
            break;

        forward_message(idx, p->buf, 4 + msg_len);

        consumed = 4 + msg_len;
        if (consumed < p->buf_used)
            memmove(p->buf, p->buf + consumed, p->buf_used - consumed);
        p->buf_used -= consumed;
    }
}

/* -------------------------------------------------------------------
 *  TCP upstream connection + auto-reconnect
 * ------------------------------------------------------------------- */
/*
 * Bounded-time connect for upstream hubs.
 *
 * The previous implementation used a blocking connect(), which stalls
 * the entire single-threaded hub for the kernel's full TCP connect
 * timeout (~75 s on Linux) every time an upstream is unreachable.
 * Switch to a non-blocking connect with a short poll wait so a dead
 * upstream costs at most CONNECT_TIMEOUT_MS rather than freezing
 * fanout for everyone.
 */
#define UPSTREAM_CONNECT_TIMEOUT_MS  500

static int connect_upstream(int upstream_idx)
{
    struct addrinfo hints, *res, *rp;
    int fd = -1;
    char label[80];
    int pidx;

    memset(&hints, 0, sizeof(hints));
    hints.ai_family = AF_UNSPEC;
    hints.ai_socktype = SOCK_STREAM;

    if (getaddrinfo(upstreams[upstream_idx].host,
                    upstreams[upstream_idx].port,
                    &hints, &res) != 0)
        return -1;

    for (rp = res; rp != NULL; rp = rp->ai_next) {
        fd = socket(rp->ai_family, rp->ai_socktype, rp->ai_protocol);
        if (fd < 0) continue;

        set_nonblock(fd);
        int rc = connect(fd, rp->ai_addr, rp->ai_addrlen);
        if (rc == 0)
            break;  /* connected immediately */

        if (errno != EINPROGRESS) {
            close(fd);
            fd = -1;
            continue;
        }

        /* Wait up to UPSTREAM_CONNECT_TIMEOUT_MS for the connection
         * to complete (or fail). */
        struct pollfd p = { .fd = fd, .events = POLLOUT };
        int pr = poll(&p, 1, UPSTREAM_CONNECT_TIMEOUT_MS);
        if (pr <= 0) {
            /* Timeout or poll error: abandon this address. */
            close(fd);
            fd = -1;
            continue;
        }

        /* Check actual connect result via SO_ERROR. */
        int soerr = 0;
        socklen_t solen = sizeof(soerr);
        if (getsockopt(fd, SOL_SOCKET, SO_ERROR, &soerr, &solen) < 0
            || soerr != 0) {
            close(fd);
            fd = -1;
            continue;
        }
        break;  /* connected */
    }
    freeaddrinfo(res);

    if (fd < 0) return -1;

    set_tcp_nodelay(fd);
    snprintf(label, sizeof(label), "upstream[%s:%s]",
             upstreams[upstream_idx].host,
             upstreams[upstream_idx].port);

    pidx = add_peer(fd, 1, label);
    if (pidx < 0) return -1;
    upstream_state[upstream_idx].peer_idx = pidx;
    return 0;
}

static void try_reconnect_upstreams(void)
{
    int i;
    time_t now = time(NULL);

    for (i = 0; i < num_upstreams; i++) {
        if (upstream_state[i].peer_idx >= 0)
            continue;
        if (now - upstream_state[i].last_attempt < UPSTREAM_RETRY_SEC)
            continue;
        upstream_state[i].last_attempt = now;

        if (connect_upstream(i) == 0) {
            fprintf(stderr, "hub: reconnected to upstream %s:%s\n",
                    upstreams[i].host, upstreams[i].port);
        }
    }
}

/* -------------------------------------------------------------------
 *  Control channel — command processing
 * ------------------------------------------------------------------- */

/* Write a response string to a control client fd */
/*
 * Try to drain a ctl_client's outbuf. Returns true if the buffer is
 * now empty (POLLOUT can be unregistered), false if bytes remain.
 */
static bool ctl_drain_outbuf(int idx)
{
    struct ctl_client *c = &ctl_clients[idx];
    while (c->outbuf_used > 0) {
        ssize_t wr = write(c->fd, c->outbuf, c->outbuf_used);
        if (wr > 0) {
            uint32_t left = c->outbuf_used - (uint32_t)wr;
            if (left > 0)
                memmove(c->outbuf, c->outbuf + wr, left);
            c->outbuf_used = left;
            continue;
        }
        if (wr < 0 && (errno == EAGAIN || errno == EINTR))
            return false;
        /* Other write errors: client is dead. Drop the buffered
         * output and let the main loop notice the disconnect. */
        c->outbuf_used = 0;
        return true;
    }
    return true;
}

/*
 * Send formatted text to a control client.
 *
 * If `fd` matches an active ctl_client, the response is written
 * directly when possible and buffered into the client's outbuf when
 * the kernel buffer is full. The main loop arms POLLOUT to drain.
 * If `fd` doesn't match a tracked client (e.g. STDERR_FILENO during
 * startup config load), we just do a best-effort blocking-style write.
 */
static void ctl_respond(int fd, const char *fmt, ...)
{
    char buf[CTL_RESP_SIZE];
    va_list ap;
    va_start(ap, fmt);
    int len = vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    if (len <= 0)
        return;

    int idx = ctl_client_by_fd(fd);
    if (idx < 0) {
        /* Untracked fd (stderr, ad-hoc): legacy best-effort write. */
        ssize_t wr = write(fd, buf, (size_t)len);
        (void)wr;
        return;
    }

    struct ctl_client *c = &ctl_clients[idx];
    size_t off = 0;
    size_t need = (size_t)len;

    /* If the outbuf is empty, try a direct write first to avoid the
     * memcpy on the fast path. */
    if (c->outbuf_used == 0) {
        ssize_t wr = write(fd, buf, need);
        if (wr > 0) {
            off = (size_t)wr;
            if (off >= need)
                return;
        } else if (wr < 0 && errno != EAGAIN && errno != EINTR) {
            /* Hard write error: drop. The next read will see the
             * disconnect and clean up. */
            return;
        }
    }

    /* Buffer the remainder. If it would overflow, log and truncate. */
    size_t leftover = need - off;
    if (c->outbuf_used + leftover > sizeof(c->outbuf)) {
        size_t fit = sizeof(c->outbuf) - c->outbuf_used;
        fprintf(stderr,
                "hub: ctl client %d outbuf overflow (dropping %zu bytes)\n",
                idx, leftover - fit);
        leftover = fit;
    }
    memcpy(c->outbuf + c->outbuf_used, buf + off, leftover);
    c->outbuf_used += (uint32_t)leftover;
}

/*
 * Process a single control command line.
 * Returns 0 normally, -1 to close the control connection.
 */
static int ctl_process_command(int fd, char *line)
{
    char *cmd = trim(line);
    if (cmd[0] == '\0' || cmd[0] == '#')
        return 0;   /* blank or comment */

    /* ---- LIST_PEERS ---- */
    if (strncasecmp(cmd, "LIST_PEERS", 10) == 0) {
        /* Sized for MAX_MACS_PER_NODE (16) entries of 17 chars + 1
         * separator each, plus the "(none)" placeholder, with comfortable
         * headroom so output never silently truncates. */
        char maclist[512];
        ctl_respond(fd, "OK %d nodes (%d peers online)\n",
                    num_nodes, num_peers);
        for (int ni = 0; ni < num_nodes; ni++) {
            struct node_phys *nd = &nodes[ni];
            if (!nd->active) continue;

            maclist[0] = '\0';
            size_t mused = 0;
            for (int m = 0; m < nd->num_macs; m++) {
                char ms[20];
                mac_to_str(nd->macs[m], ms, sizeof(ms));
                int w = snprintf(maclist + mused, sizeof(maclist) - mused,
                                 "%s%s", (m > 0) ? "," : "", ms);
                if (w < 0 || (size_t)w >= sizeof(maclist) - mused) break;
                mused += (size_t)w;
            }
            if (nd->num_macs == 0)
                snprintf(maclist, sizeof(maclist), "(none)");

            const char *st = (nd->peer_idx >= 0) ? "online" : "offline";
            uint64_t txd = (nd->peer_idx >= 0)
                ? peers[nd->peer_idx].tx_dropped : 0;

            if (nd->pos_set)
                ctl_respond(fd, "  %-12s %s macs=[%s] pos=(%.1f,%.1f,%.1f) "
                    "txpow=%.1f tx=%" PRIu64 " rx=%" PRIu64
                    " rx_drop=%" PRIu64 " tx_drop=%" PRIu64 "\n",
                    nd->node_id, st, maclist, nd->pos_x, nd->pos_y, nd->pos_z,
                    nd->tx_power_dbm, nd->tx_frames, nd->rx_frames,
                    nd->rx_dropped, txd);
            else
                ctl_respond(fd, "  %-12s %s macs=[%s] pos=none txpow=%.1f "
                    "tx=%" PRIu64 " rx=%" PRIu64
                    " rx_drop=%" PRIu64 " tx_drop=%" PRIu64 "\n",
                    nd->node_id, st, maclist, nd->tx_power_dbm,
                    nd->tx_frames, nd->rx_frames,
                    nd->rx_dropped, txd);
        }
        return 0;
    }

    /* ---- SET_POS <node_or_mac> <x> <y> [<z>] ---- */
    if (strncasecmp(cmd, "SET_POS ", 8) == 0) {
        char ident[64]; double x, y, z = 0.0;
        int n = sscanf(cmd + 8, "%63s %lf %lf %lf", ident, &x, &y, &z);
        if (n < 3) { ctl_respond(fd, "ERR usage: SET_POS <node> <x> <y> [z]\n"); return 0; }
        struct node_phys *nd = resolve_node(ident);
        if (!nd) nd = find_node(ident, true); /* pre-create */
        if (!nd) { ctl_respond(fd, "ERR node table full\n"); return 0; }
        nd->pos_x = x; nd->pos_y = y; nd->pos_z = z; nd->pos_set = true;
        ctl_respond(fd, "OK %s pos=(%.2f,%.2f,%.2f)\n", nd->node_id, x, y, z);
        return 0;
    }

    if (strncasecmp(cmd, "GET_POS ", 8) == 0) {
        char ident[64];
        if (sscanf(cmd + 8, "%63s", ident) != 1) { ctl_respond(fd, "ERR usage\n"); return 0; }
        struct node_phys *nd = resolve_node(ident);
        if (!nd) { ctl_respond(fd, "ERR not found: %s\n", ident); return 0; }
        if (nd->pos_set) ctl_respond(fd, "OK %.2f %.2f %.2f\n", nd->pos_x, nd->pos_y, nd->pos_z);
        else ctl_respond(fd, "OK none\n");
        return 0;
    }

    if (strncasecmp(cmd, "SET_TXPOWER ", 12) == 0) {
        char ident[64]; double pw;
        if (sscanf(cmd + 12, "%63s %lf", ident, &pw) != 2) {
            ctl_respond(fd, "ERR usage: SET_TXPOWER <node> <dBm>\n"); return 0;
        }
        struct node_phys *nd = resolve_node(ident);
        if (!nd) nd = find_node(ident, true);
        if (!nd) { ctl_respond(fd, "ERR node table full\n"); return 0; }
        nd->tx_power_dbm = pw;
        ctl_respond(fd, "OK %s txpower=%.1f\n", nd->node_id, pw);
        return 0;
    }

    if (strncasecmp(cmd, "GET_TXPOWER ", 12) == 0) {
        char ident[64];
        if (sscanf(cmd + 12, "%63s", ident) != 1) { ctl_respond(fd, "ERR usage\n"); return 0; }
        struct node_phys *nd = resolve_node(ident);
        if (!nd) { ctl_respond(fd, "ERR not found\n"); return 0; }
        ctl_respond(fd, "OK %.1f\n", nd->tx_power_dbm);
        return 0;
    }

    if (strncasecmp(cmd, "SET_SNR ", 8) == 0) {
        char ia[64], ib[64]; double sab, sba;
        int n = sscanf(cmd + 8, "%63s %63s %lf %lf", ia, ib, &sab, &sba);
        if (n < 3) { ctl_respond(fd, "ERR usage: SET_SNR <a> <b> <dB> [dB_ba]\n"); return 0; }
        struct node_phys *na = resolve_node(ia); if (!na) na = find_node(ia, true);
        struct node_phys *nb = resolve_node(ib); if (!nb) nb = find_node(ib, true);
        if (!na || !nb) { ctl_respond(fd, "ERR resolve failed\n"); return 0; }
        struct link_override *lk = find_link(na->node_id, nb->node_id, true);
        if (!lk) { ctl_respond(fd, "ERR link table full\n"); return 0; }
        if (strcmp(lk->node_a, na->node_id) == 0) {
            lk->snr_ab = sab; lk->snr_ba = (n >= 4) ? sba : NAN;
        } else {
            lk->snr_ba = sab; lk->snr_ab = (n >= 4) ? sba : NAN;
        }
        if (model.type == MODEL_NONE) model.type = MODEL_SNR_TABLE;
        ctl_respond(fd, "OK snr %s<->%s = %.1f dB\n", na->node_id, nb->node_id, sab);
        return 0;
    }

    if (strncasecmp(cmd, "CLEAR_SNR ", 10) == 0) {
        char ia[64], ib[64];
        if (sscanf(cmd + 10, "%63s %63s", ia, ib) != 2) { ctl_respond(fd, "ERR usage\n"); return 0; }
        struct node_phys *na = resolve_node(ia), *nb = resolve_node(ib);
        if (na && nb) {
            struct link_override *lk = find_link(na->node_id, nb->node_id, false);
            if (lk) { lk->snr_ab = NAN; lk->snr_ba = NAN;
                if (isnan(lk->fixed_loss_ab) && isnan(lk->fixed_loss_ba)) lk->active = false; }
        }
        ctl_respond(fd, "OK cleared snr %s<->%s\n", ia, ib);
        return 0;
    }

    if (strncasecmp(cmd, "SET_LOSS ", 9) == 0) {
        char ia[64], ib[64]; double lab, lba;
        int n = sscanf(cmd + 9, "%63s %63s %lf %lf", ia, ib, &lab, &lba);
        if (n < 3) { ctl_respond(fd, "ERR usage: SET_LOSS <a> <b> <p> [p_ba]\n"); return 0; }
        if (lab < 0 || lab > 1 || (n >= 4 && (lba < 0 || lba > 1))) {
            ctl_respond(fd, "ERR probability 0.0-1.0\n"); return 0; }
        struct node_phys *na = resolve_node(ia); if (!na) na = find_node(ia, true);
        struct node_phys *nb = resolve_node(ib); if (!nb) nb = find_node(ib, true);
        if (!na || !nb) { ctl_respond(fd, "ERR resolve failed\n"); return 0; }
        struct link_override *lk = find_link(na->node_id, nb->node_id, true);
        if (!lk) { ctl_respond(fd, "ERR link table full\n"); return 0; }
        if (strcmp(lk->node_a, na->node_id) == 0) {
            lk->fixed_loss_ab = lab; lk->fixed_loss_ba = (n >= 4) ? lba : NAN;
        } else {
            lk->fixed_loss_ba = lab; lk->fixed_loss_ab = (n >= 4) ? lba : NAN;
        }
        if (model.type == MODEL_NONE) model.type = MODEL_SNR_TABLE;
        ctl_respond(fd, "OK loss %s->%s=%.3f\n", na->node_id, nb->node_id, lab);
        return 0;
    }

    if (strncasecmp(cmd, "CLEAR_LOSS ", 11) == 0) {
        char ia[64], ib[64];
        if (sscanf(cmd + 11, "%63s %63s", ia, ib) != 2) { ctl_respond(fd, "ERR usage\n"); return 0; }
        struct node_phys *na = resolve_node(ia), *nb = resolve_node(ib);
        if (na && nb) {
            struct link_override *lk = find_link(na->node_id, nb->node_id, false);
            if (lk) { lk->fixed_loss_ab = NAN; lk->fixed_loss_ba = NAN;
                if (isnan(lk->snr_ab) && isnan(lk->snr_ba)) lk->active = false; }
        }
        ctl_respond(fd, "OK cleared loss %s<->%s\n", ia, ib);
        return 0;
    }

    /* ---- SET_MODEL <none|snr_table|log_distance|free_space> ---- */
    if (strncasecmp(cmd, "SET_MODEL ", 10) == 0) {
        char mname[32];
        if (sscanf(cmd + 10, "%31s", mname) != 1) {
            ctl_respond(fd, "ERR usage: SET_MODEL "
                        "<none|snr_table|log_distance|free_space>\n");
            return 0;
        }
        if (strcasecmp(mname, "none") == 0)
            model.type = MODEL_NONE;
        else if (strcasecmp(mname, "snr_table") == 0)
            model.type = MODEL_SNR_TABLE;
        else if (strcasecmp(mname, "log_distance") == 0)
            model.type = MODEL_LOG_DISTANCE;
        else if (strcasecmp(mname, "free_space") == 0)
            model.type = MODEL_FREE_SPACE;
        else {
            ctl_respond(fd, "ERR unknown model: %s\n", mname);
            return 0;
        }
        ctl_respond(fd, "OK model = %s\n", mname);
        return 0;
    }

    /* ---- SET_MODEL_PARAM <key> <value> ---- */
    if (strncasecmp(cmd, "SET_MODEL_PARAM ", 16) == 0) {
        char key[32];
        double val;
        if (sscanf(cmd + 16, "%31s %lf", key, &val) != 2) {
            ctl_respond(fd, "ERR usage: SET_MODEL_PARAM <key> <value>\n");
            return 0;
        }
        if (strcasecmp(key, "path_loss_exp") == 0)
            model.path_loss_exp = val;
        else if (strcasecmp(key, "ref_distance") == 0)
            model.ref_distance = val;
        else if (strcasecmp(key, "ref_loss_db") == 0)
            model.ref_loss_db = val;
        else if (strcasecmp(key, "noise_floor") == 0)
            model.noise_floor_dbm = val;
        else {
            ctl_respond(fd, "ERR unknown param: %s "
                        "(path_loss_exp|ref_distance|ref_loss_db|noise_floor)\n",
                        key);
            return 0;
        }
        ctl_respond(fd, "OK %s = %.2f\n", key, val);
        return 0;
    }

    /* ---- GET_MODEL ---- */
    if (strncasecmp(cmd, "GET_MODEL", 9) == 0) {
        const char *names[] = { "none", "snr_table",
                                "log_distance", "free_space" };
        ctl_respond(fd, "OK model=%s path_loss_exp=%.2f ref_distance=%.2f "
                    "ref_loss_db=%.2f noise_floor=%.2f\n",
                    names[model.type], model.path_loss_exp,
                    model.ref_distance, model.ref_loss_db,
                    model.noise_floor_dbm);
        return 0;
    }

    /* ---- SET_NOISE_FLOOR <dBm> ---- */
    if (strncasecmp(cmd, "SET_NOISE_FLOOR ", 16) == 0) {
        double nf;
        if (sscanf(cmd + 16, "%lf", &nf) != 1) {
            ctl_respond(fd, "ERR usage: SET_NOISE_FLOOR <dBm>\n");
            return 0;
        }
        model.noise_floor_dbm = nf;
        ctl_respond(fd, "OK noise_floor = %.1f dBm\n", nf);
        return 0;
    }

    if (strncasecmp(cmd, "DUMP_LINKS", 10) == 0) {
        ctl_respond(fd, "OK link overrides (%d)\n", num_link_overrides);
        for (int k = 0; k < num_link_overrides; k++) {
            struct link_override *lk = &link_overrides[k];
            if (!lk->active) continue;
            ctl_respond(fd, "  %s <-> %s", lk->node_a, lk->node_b);
            if (!isnan(lk->snr_ab)) ctl_respond(fd, " snr_ab=%.1f", lk->snr_ab);
            if (!isnan(lk->snr_ba)) ctl_respond(fd, " snr_ba=%.1f", lk->snr_ba);
            if (!isnan(lk->fixed_loss_ab)) ctl_respond(fd, " loss_ab=%.3f", lk->fixed_loss_ab);
            if (!isnan(lk->fixed_loss_ba)) ctl_respond(fd, " loss_ba=%.3f", lk->fixed_loss_ba);
            ctl_respond(fd, "\n");
        }
        if (model.type == MODEL_LOG_DISTANCE || model.type == MODEL_FREE_SPACE) {
            ctl_respond(fd, "  --- computed ---\n");
            for (int a = 0; a < num_nodes; a++) {
                if (!nodes[a].active || !nodes[a].pos_set) continue;
                for (int b = a+1; b < num_nodes; b++) {
                    if (!nodes[b].active || !nodes[b].pos_set) continue;
                    double dx = nodes[b].pos_x - nodes[a].pos_x;
                    double dy = nodes[b].pos_y - nodes[a].pos_y;
                    double dz = nodes[b].pos_z - nodes[a].pos_z;
                    double dist = sqrt(dx*dx + dy*dy + dz*dz);
                    double sab = get_link_snr(a, b), sba = get_link_snr(b, a);
                    ctl_respond(fd, "  %s <-> %s dist=%.1fm snr=%.1f/%.1f\n",
                                nodes[a].node_id, nodes[b].node_id, dist, sab, sba);
                }
            }
        }
        return 0;
    }

    /* ---- STATS ---- */
    if (strncasecmp(cmd, "STATS", 5) == 0) {
        ctl_respond(fd, "OK fwd=%" PRIu64 " drop=%" PRIu64
                    " tx_drop=%" PRIu64
                    " peers=%d nodes=%d links=%d\n",
                    total_frames_forwarded, total_frames_dropped,
                    total_frames_tx_dropped,
                    num_peers, num_nodes, num_link_overrides);
        return 0;
    }

    /* ---- LOAD_CONFIG <path> ---- */
    if (strncasecmp(cmd, "LOAD_CONFIG ", 12) == 0) {
        /* Cap recursion so a config file that LOAD_CONFIGs itself (or
         * mutually recursive configs) can't blow the stack. The hub is
         * single-threaded, so a static counter is sufficient. */
        #define MAX_LOAD_CONFIG_DEPTH 4
        static int load_depth = 0;

        char path[256];
        if (sscanf(cmd + 12, "%255s", path) != 1) {
            ctl_respond(fd, "ERR usage: LOAD_CONFIG <path>\n");
            return 0;
        }
        if (load_depth >= MAX_LOAD_CONFIG_DEPTH) {
            ctl_respond(fd, "ERR LOAD_CONFIG too deeply nested (max %d)\n",
                        MAX_LOAD_CONFIG_DEPTH);
            return 0;
        }
        FILE *f = fopen(path, "r");
        if (!f) {
            ctl_respond(fd, "ERR cannot open %s: %s\n",
                        path, strerror(errno));
            return 0;
        }
        load_depth++;
        char cfgline[512];
        int count = 0;
        int rc = 0;
        while (fgets(cfgline, sizeof(cfgline), f)) {
            char *cl = trim(cfgline);
            if (cl[0] == '\0' || cl[0] == '#') continue;
            /* If a nested command is QUIT (returns -1), stop reading
             * the file -- the fd has been closed and any further
             * ctl_respond would be a write to a dead socket. Propagate
             * the -1 up so handle_ctl_data also stops processing. */
            if (ctl_process_command(fd, cl) < 0) {
                rc = -1;
                break;
            }
            count++;
        }
        load_depth--;
        fclose(f);
        if (rc < 0)
            return -1;
        ctl_respond(fd, "OK loaded %d commands from %s\n", count, path);
        return 0;
    }

    /* ---- SAVE_CONFIG <path> ---- */
    if (strncasecmp(cmd, "SAVE_CONFIG ", 12) == 0) {
        char path[256];
        if (sscanf(cmd + 12, "%255s", path) != 1) {
            ctl_respond(fd, "ERR usage: SAVE_CONFIG <path>\n");
            return 0;
        }
        FILE *f = fopen(path, "w");
        if (!f) {
            ctl_respond(fd, "ERR cannot write %s: %s\n",
                        path, strerror(errno));
            return 0;
        }
        /* Model */
        const char *names[] = { "none", "snr_table",
                                "log_distance", "free_space" };
        fprintf(f, "SET_MODEL %s\n", names[model.type]);
        fprintf(f, "SET_MODEL_PARAM path_loss_exp %.4f\n",
                model.path_loss_exp);
        fprintf(f, "SET_MODEL_PARAM ref_distance %.4f\n",
                model.ref_distance);
        fprintf(f, "SET_MODEL_PARAM ref_loss_db %.4f\n",
                model.ref_loss_db);
        fprintf(f, "SET_NOISE_FLOOR %.2f\n", model.noise_floor_dbm);
        fprintf(f, "\n");
        /* Positions and TX powers */
        for (int i = 0; i < num_nodes; i++) {
            struct node_phys *nd = &nodes[i];
            if (!nd->active) continue;
            if (nd->pos_set)
                fprintf(f, "SET_POS %s %.4f %.4f %.4f\n", nd->node_id,
                        nd->pos_x, nd->pos_y, nd->pos_z);
            fprintf(f, "SET_TXPOWER %s %.2f\n", nd->node_id,
                    nd->tx_power_dbm);
        }
        fprintf(f, "\n");
        /* Link overrides */
        for (int k = 0; k < num_link_overrides; k++) {
            struct link_override *lk = &link_overrides[k];
            if (!lk->active) continue;
            if (!isnan(lk->snr_ab)) {
                if (!isnan(lk->snr_ba))
                    fprintf(f, "SET_SNR %s %s %.2f %.2f\n",
                            lk->node_a, lk->node_b, lk->snr_ab, lk->snr_ba);
                else
                    fprintf(f, "SET_SNR %s %s %.2f\n",
                            lk->node_a, lk->node_b, lk->snr_ab);
            }
            if (!isnan(lk->fixed_loss_ab)) {
                if (!isnan(lk->fixed_loss_ba))
                    fprintf(f, "SET_LOSS %s %s %.4f %.4f\n",
                            lk->node_a, lk->node_b,
                            lk->fixed_loss_ab, lk->fixed_loss_ba);
                else
                    fprintf(f, "SET_LOSS %s %s %.4f\n",
                            lk->node_a, lk->node_b, lk->fixed_loss_ab);
            }
        }
        fclose(f);
        ctl_respond(fd, "OK saved to %s\n", path);
        return 0;
    }

    /* ---- HELP ---- */
    if (strncasecmp(cmd, "HELP", 4) == 0) {
        ctl_respond(fd,
            "OK commands:\n"
            "  LIST_PEERS                          Show connected peers\n"
            "  SET_POS <mac> <x> <y> [<z>]         Set node position (m)\n"
            "  GET_POS <mac>                        Query position\n"
            "  SET_TXPOWER <mac> <dBm>              Set TX power\n"
            "  GET_TXPOWER <mac>                    Query TX power\n"
            "  SET_SNR <a> <b> <dB> [<dB_ba>]       Per-link SNR override\n"
            "  CLEAR_SNR <a> <b>                    Remove SNR override\n"
            "  SET_LOSS <a> <b> <p> [<p_ba>]        Fixed loss probability\n"
            "  CLEAR_LOSS <a> <b>                   Remove loss override\n"
            "  SET_MODEL <none|snr_table|log_distance|free_space>\n"
            "  SET_MODEL_PARAM <key> <value>        Model parameters\n"
            "  GET_MODEL                            Show model + params\n"
            "  SET_NOISE_FLOOR <dBm>                Set noise floor\n"
            "  DUMP_LINKS                           Show all link info\n"
            "  STATS                                Global counters\n"
            "  LOAD_CONFIG <path>                   Run commands from file\n"
            "  SAVE_CONFIG <path>                   Dump state to file\n"
            "  HELP                                 This help\n"
            "  QUIT                                 Close control connection\n"
        );
        return 0;
    }

    /* ---- QUIT ---- */
    if (strncasecmp(cmd, "QUIT", 4) == 0) {
        ctl_respond(fd, "OK bye\n");
        return -1;
    }

    ctl_respond(fd, "ERR unknown command: %.40s (try HELP)\n", cmd);
    return 0;
}

/* -------------------------------------------------------------------
 *  Control channel I/O
 * ------------------------------------------------------------------- */
static void handle_ctl_data(int ctl_idx)
{
    struct ctl_client *c = &ctl_clients[ctl_idx];
    ssize_t n = read(c->fd, c->buf + c->buf_used,
                     CTL_BUF_SIZE - c->buf_used - 1);
    if (n <= 0) {
        if (n < 0 && (errno == EAGAIN || errno == EINTR))
            return;
        /* Disconnect */
        close(c->fd);
        c->active = false;
        fprintf(stderr, "hub: ctl client %d disconnected\n", ctl_idx);
        return;
    }
    c->buf_used += (uint32_t)n;
    c->buf[c->buf_used] = '\0';

    /* Process complete lines */
    char *start = c->buf;
    char *nl;
    while ((nl = strchr(start, '\n')) != NULL) {
        *nl = '\0';
        if (ctl_process_command(c->fd, start) < 0) {
            close(c->fd);
            c->active = false;
            return;
        }
        start = nl + 1;
    }

    /* Shift unprocessed remainder */
    uint32_t remaining = (uint32_t)(c->buf + c->buf_used - start);
    if (remaining > 0 && start != c->buf)
        memmove(c->buf, start, remaining);
    c->buf_used = remaining;

    /* Overflow protection */
    if (c->buf_used >= CTL_BUF_SIZE - 1) {
        ctl_respond(c->fd, "ERR line too long\n");
        c->buf_used = 0;
    }
}

/* -------------------------------------------------------------------
 *  Socket setup
 * ------------------------------------------------------------------- */
static int setup_tcp_listen(const char *port_str)
{
    struct addrinfo hints, *res;
    int fd, val = 1;

    memset(&hints, 0, sizeof(hints));
    hints.ai_family = AF_INET6;
    hints.ai_socktype = SOCK_STREAM;
    hints.ai_flags = AI_PASSIVE | AI_V4MAPPED;

    if (getaddrinfo(NULL, port_str, &hints, &res) != 0) {
        hints.ai_family = AF_INET;
        if (getaddrinfo(NULL, port_str, &hints, &res) != 0) {
            fprintf(stderr, "hub: cannot resolve TCP port %s\n", port_str);
            return -1;
        }
    }

    fd = socket(res->ai_family, res->ai_socktype, res->ai_protocol);
    if (fd < 0) {
        freeaddrinfo(res);
        return -1;
    }

    setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &val, sizeof(val));
    if (res->ai_family == AF_INET6) {
        val = 0;
        setsockopt(fd, IPPROTO_IPV6, IPV6_V6ONLY, &val, sizeof(val));
    }

    if (bind(fd, res->ai_addr, res->ai_addrlen) < 0) {
        perror("hub: TCP bind");
        close(fd);
        freeaddrinfo(res);
        return -1;
    }
    freeaddrinfo(res);

    if (listen(fd, 16) < 0) {
        perror("hub: TCP listen");
        close(fd);
        return -1;
    }

    set_nonblock(fd);
    return fd;
}

/*
 * Bind a listening Unix socket at `path` with file mode `mode`.
 *
 * The umask is temporarily restricted around bind() so the socket file
 * is created with the requested permissions atomically -- without this,
 * an attacker could race a connect() through the bind→chmod window.
 * A follow-up chmod() handles the case where bind() ignored the umask.
 */
static int setup_unix_listen(const char *path, mode_t mode)
{
    struct sockaddr_un addr;
    int fd, rc;
    mode_t old_umask;

    unlink(path);

    fd = socket(AF_UNIX, SOCK_STREAM, 0);
    if (fd < 0) {
        perror("hub: Unix socket");
        return -1;
    }

    memset(&addr, 0, sizeof(addr));
    addr.sun_family = AF_UNIX;
    strncpy(addr.sun_path, path, sizeof(addr.sun_path) - 1);

    old_umask = umask((mode_t)(~mode & 0777));
    rc = bind(fd, (struct sockaddr *)&addr, sizeof(addr));
    umask(old_umask);
    if (rc < 0) {
        perror("hub: Unix bind");
        close(fd);
        return -1;
    }

    if (chmod(path, mode) < 0) {
        /* Non-fatal: log but keep going. The umask above already
         * restricted creation perms in the typical case. */
        fprintf(stderr, "hub: chmod %s 0%o: %s\n",
                path, (unsigned)mode, strerror(errno));
    }

    if (listen(fd, 16) < 0) {
        perror("hub: Unix listen");
        close(fd);
        return -1;
    }

    set_nonblock(fd);
    return fd;
}

/* -------------------------------------------------------------------
 *  Cleanup and signal handling
 * ------------------------------------------------------------------- */
static void cleanup(void)
{
    int i;
    if (unix_listen_fd >= 0) close(unix_listen_fd);
    if (tcp_listen_fd >= 0) close(tcp_listen_fd);
    if (ctl_listen_fd >= 0) close(ctl_listen_fd);
    for (i = 0; i < num_peers; i++) {
        if (peers[i].fd >= 0) close(peers[i].fd);
    }
    for (i = 0; i < MAX_CTL_CLIENTS; i++) {
        if (ctl_clients[i].active) close(ctl_clients[i].fd);
    }
    if (socket_path) unlink(socket_path);
    if (ctl_socket_path) unlink(ctl_socket_path);
    free(fwd_scratch);
    fwd_scratch = NULL;
}

static void sighandler(int sig)
{
    (void)sig;
    running = 0;
}

/* -------------------------------------------------------------------
 *  Usage
 * ------------------------------------------------------------------- */
static void usage(const char *prog)
{
    fprintf(stderr,
        "Usage: %s <unix-socket-path> [options]\n"
        "\n"
        "Options:\n"
        "  -t <port>        TCP listen port for incoming bridge connections\n"
        "  -u <host:port>   Connect to upstream hub (repeatable, max %d)\n"
        "  -c <path>        Control socket path (for runtime commands)\n"
        "  -C <path>        Initial config file (commands run at startup)\n"
        "  -h               Show this help\n"
        "\n"
        "Examples:\n"
        "  # Local-only hub:\n"
        "  %s /tmp/ath9k.sock\n"
        "\n"
        "  # Hub with control channel:\n"
        "  %s /tmp/ath9k.sock -c /tmp/ath9k.ctl\n"
        "\n"
        "  # Hub with startup config + control:\n"
        "  %s /tmp/ath9k.sock -c /tmp/ath9k.ctl -C medium.cfg\n"
        "\n"
        "  # Runtime control:\n"
        "  echo 'SET_SNR 02:00:00:00:00:00 02:00:00:00:01:00 25' "
            "| socat - UNIX-CONNECT:/tmp/ath9k.ctl\n"
        "\n"
        "  # Interactive control:\n"
        "  socat READLINE UNIX-CONNECT:/tmp/ath9k.ctl\n"
        "\n",
        prog, MAX_UPSTREAMS, prog, prog, prog);
}

/* -------------------------------------------------------------------
 *  Main loop
 * ------------------------------------------------------------------- */
int main(int argc, char **argv)
{
    const char *tcp_port = NULL;
    const char *config_path = NULL;
    struct pollfd *pfds = NULL;
    int nfds, i, opt;
    int tcp_listen_pfd, ctl_listen_pfd, peer_base, ctl_base;

    if (argc < 2 || argv[1][0] == '-') {
        usage(argv[0]);
        return 1;
    }
    socket_path = argv[1];

    /* Init control clients */
    for (i = 0; i < MAX_CTL_CLIENTS; i++)
        ctl_clients[i].active = false;

    /* Parse options (appear after the positional socket path) */
    optind = 2;
    while ((opt = getopt(argc, argv, "t:u:c:C:h")) != -1) {
        switch (opt) {
        case 't':
            tcp_port = optarg;
            break;
        case 'u':
            if (num_upstreams >= MAX_UPSTREAMS) {
                fprintf(stderr, "hub: too many -u (max %d)\n",
                        MAX_UPSTREAMS);
                return 1;
            }
            {
                char *colon = strrchr(optarg, ':');
                size_t hlen;
                if (!colon || colon == optarg) {
                    fprintf(stderr, "hub: bad -u format '%s' "
                            "(need host:port)\n", optarg);
                    return 1;
                }
                hlen = (size_t)(colon - optarg);
                if (hlen >= sizeof(upstreams[0].host))
                    hlen = sizeof(upstreams[0].host) - 1;
                memcpy(upstreams[num_upstreams].host, optarg, hlen);
                upstreams[num_upstreams].host[hlen] = '\0';
                snprintf(upstreams[num_upstreams].port,
                         sizeof(upstreams[0].port), "%s", colon + 1);
                num_upstreams++;
            }
            break;
        case 'c':
            ctl_socket_path = optarg;
            break;
        case 'C':
            config_path = optarg;
            break;
        case 'h':
            usage(argv[0]);
            return 0;
        default:
            usage(argv[0]);
            return 1;
        }
    }

    /* Seed the PRNG */
    rng_state = (uint64_t)time(NULL) ^ ((uint64_t)getpid() << 32);

    /* Allocate the per-frame forwarding scratch buffer once -- keeps
     * an 8 KB+ array off forward_message's stack frame. */
    fwd_scratch = malloc(4 + MAX_MSG_SIZE);
    if (!fwd_scratch) {
        perror("malloc fwd_scratch");
        return 1;
    }

    signal(SIGINT, sighandler);
    signal(SIGTERM, sighandler);
    signal(SIGPIPE, SIG_IGN);

    /* Setup listen sockets.
     *
     * The data socket is world-accessible (0666) so unprivileged QEMU
     * processes from any user can connect. The control socket is owner-
     * only (0600) because its commands are unauthenticated and include
     * SAVE_CONFIG which would otherwise be a privesc primitive. */
    unix_listen_fd = setup_unix_listen(socket_path, 0666);
    if (unix_listen_fd < 0)
        return 1;
    fprintf(stderr, "hub: Unix socket %s\n", socket_path);

    if (tcp_port) {
        tcp_listen_fd = setup_tcp_listen(tcp_port);
        if (tcp_listen_fd < 0) {
            cleanup();
            return 1;
        }
        fprintf(stderr, "hub: TCP port %s\n", tcp_port);
    }

    if (ctl_socket_path) {
        ctl_listen_fd = setup_unix_listen(ctl_socket_path, 0600);
        if (ctl_listen_fd < 0) {
            cleanup();
            return 1;
        }
        fprintf(stderr, "hub: control socket %s (mode 0600)\n",
                ctl_socket_path);
    }

    /* Initialize upstream state */
    for (i = 0; i < MAX_UPSTREAMS; i++) {
        upstream_state[i].peer_idx = -1;
        upstream_state[i].last_attempt = 0;
    }

    /* Connect to upstreams */
    for (i = 0; i < num_upstreams; i++) {
        fprintf(stderr, "hub: connecting upstream %s:%s...\n",
                upstreams[i].host, upstreams[i].port);
        upstream_state[i].last_attempt = time(NULL);
        if (connect_upstream(i) != 0) {
            fprintf(stderr, "hub: upstream %s:%s unavailable (will retry)\n",
                    upstreams[i].host, upstreams[i].port);
        }
    }

    /* Load initial config if specified */
    if (config_path) {
        FILE *f = fopen(config_path, "r");
        if (!f) {
            fprintf(stderr, "hub: cannot open config %s: %s\n",
                    config_path, strerror(errno));
        } else {
            char cfgline[512];
            int count = 0;
            fprintf(stderr, "hub: loading config from %s\n", config_path);
            while (fgets(cfgline, sizeof(cfgline), f)) {
                char *cl = trim(cfgline);
                if (cl[0] == '\0' || cl[0] == '#') continue;
                /* Process command, responses go to stderr (fd 2) */
                ctl_process_command(STDERR_FILENO, cl);
                count++;
            }
            fclose(f);
            fprintf(stderr, "hub: loaded %d config commands\n", count);
        }
    }

    fprintf(stderr, "hub: ready (max %d peers, TTL %d, %d upstreams, "
            "%d nodes preloaded)\n",
            MAX_PEERS, DEFAULT_TTL, num_upstreams, num_nodes);

    /* Allocate poll array: unix_listen + tcp_listen + ctl_listen
     *                      + MAX_PEERS + MAX_CTL_CLIENTS */
    pfds = calloc(3 + MAX_PEERS + MAX_CTL_CLIENTS, sizeof(struct pollfd));
    if (!pfds) {
        perror("calloc");
        cleanup();
        return 1;
    }

    while (running) {
        nfds = 0;

        /* Unix data listen socket */
        pfds[nfds].fd = unix_listen_fd;
        pfds[nfds].events = POLLIN;
        pfds[nfds].revents = 0;
        nfds++;

        /* TCP listen socket */
        tcp_listen_pfd = -1;
        if (tcp_listen_fd >= 0) {
            tcp_listen_pfd = nfds;
            pfds[nfds].fd = tcp_listen_fd;
            pfds[nfds].events = POLLIN;
            pfds[nfds].revents = 0;
            nfds++;
        }

        /* Control listen socket */
        ctl_listen_pfd = -1;
        if (ctl_listen_fd >= 0) {
            ctl_listen_pfd = nfds;
            pfds[nfds].fd = ctl_listen_fd;
            pfds[nfds].events = POLLIN;
            pfds[nfds].revents = 0;
            nfds++;
        }

        /* Data peers */
        peer_base = nfds;
        for (i = 0; i < num_peers; i++) {
            pfds[nfds].fd = peers[i].fd;
            pfds[nfds].events = POLLIN;
            pfds[nfds].revents = 0;
            nfds++;
        }

        /* Control clients (POLLIN always; POLLOUT when there's
         * buffered response output that didn't fit in the kernel
         * send buffer earlier). */
        ctl_base = nfds;
        for (i = 0; i < MAX_CTL_CLIENTS; i++) {
            if (ctl_clients[i].active) {
                pfds[nfds].fd = ctl_clients[i].fd;
                pfds[nfds].events = POLLIN;
                if (ctl_clients[i].outbuf_used > 0)
                    pfds[nfds].events |= POLLOUT;
                pfds[nfds].revents = 0;
                nfds++;
            }
        }

        if (poll(pfds, (nfds_t)nfds, 1000) < 0) {
            if (errno == EINTR)
                continue;
            perror("poll");
            break;
        }

        /* Reconnect dropped upstreams */
        if (num_upstreams > 0)
            try_reconnect_upstreams();

        /* Accept Unix data clients */
        if (pfds[0].revents & POLLIN) {
            int cfd = accept(unix_listen_fd, NULL, NULL);
            if (cfd >= 0)
                add_peer(cfd, 0, "local-qemu");
        }

        /* Accept TCP bridge clients */
        if (tcp_listen_pfd >= 0 &&
            (pfds[tcp_listen_pfd].revents & POLLIN))
        {
            struct sockaddr_storage sa;
            socklen_t salen = sizeof(sa);
            int cfd = accept(tcp_listen_fd,
                             (struct sockaddr *)&sa, &salen);
            if (cfd >= 0) {
                char lbl[64], hbuf[48], sbuf[8];
                if (getnameinfo((struct sockaddr *)&sa, salen,
                                hbuf, sizeof(hbuf), sbuf, sizeof(sbuf),
                                NI_NUMERICHOST | NI_NUMERICSERV) == 0)
                    snprintf(lbl, sizeof(lbl), "tcp[%.40s:%s]", hbuf, sbuf);
                else
                    snprintf(lbl, sizeof(lbl), "tcp-in[?]");
                set_tcp_nodelay(cfd);
                add_peer(cfd, 1, lbl);
            }
        }

        /* Accept control clients */
        if (ctl_listen_pfd >= 0 &&
            (pfds[ctl_listen_pfd].revents & POLLIN))
        {
            int cfd = accept(ctl_listen_fd, NULL, NULL);
            if (cfd >= 0) {
                set_nonblock(cfd);
                /* Find a free slot */
                bool placed = false;
                for (i = 0; i < MAX_CTL_CLIENTS; i++) {
                    if (!ctl_clients[i].active) {
                        ctl_clients[i].fd = cfd;
                        ctl_clients[i].buf_used = 0;
                        ctl_clients[i].outbuf_used = 0;
                        ctl_clients[i].active = true;
                        placed = true;
                        fprintf(stderr,
                                "hub: ctl client %d connected (fd=%d)\n",
                                i, cfd);
                        break;
                    }
                }
                if (!placed) {
                    const char *msg = "ERR max control clients\n";
                    ssize_t wr = write(cfd, msg, strlen(msg));
                    (void)wr;
                    close(cfd);
                }
            }
        }

        /*
         * Handle data peer I/O (backwards iteration for safe removal).
         *
         * forward_message can call remove_peer for *other* indices
         * (peers that fail a fanout write) before the dispatch loop
         * gets to them. remove_peer swaps with last, so a previously-
         * polled fd at index N may have been replaced by a different
         * peer with no events to handle. Two protections:
         *
         *   1. i may now be >= num_peers (slot was discarded).
         *   2. peers[i].fd may not match pfds[pfd_idx].fd (slot was
         *      swapped to a different peer whose events live elsewhere
         *      in pfds, or whose events arrive in the next poll
         *      iteration).
         *
         * In either case, skip dispatching -- spurious read() on the
         * wrong peer would corrupt its rx state or misattribute
         * counters.
         */
        for (i = num_peers - 1; i >= 0; i--) {
            int pfd_idx = peer_base + i;
            if (pfd_idx >= nfds)
                continue;
            if (pfds[pfd_idx].fd != peers[i].fd)
                continue;   /* slot swapped after a remove_peer */
            if (pfds[pfd_idx].revents & (POLLIN | POLLHUP | POLLERR))
                handle_peer_data(i);
        }

        /* Handle control client I/O */
        {
            int pfd_idx = ctl_base;
            for (i = 0; i < MAX_CTL_CLIENTS; i++) {
                if (!ctl_clients[i].active) continue;
                if (pfd_idx < nfds) {
                    short rev = pfds[pfd_idx].revents;
                    /* Drain pending response bytes first so a client
                     * blocked on POLLOUT doesn't get a fresh response
                     * appended ahead of the queued one. */
                    if ((rev & POLLOUT) && ctl_clients[i].active)
                        ctl_drain_outbuf(i);
                    if ((rev & (POLLIN | POLLHUP | POLLERR))
                        && ctl_clients[i].active)
                        handle_ctl_data(i);
                }
                pfd_idx++;
            }
        }
    }

    fprintf(stderr, "hub: shutting down (%d peers, %" PRIu64
            " forwarded, %" PRIu64 " phys-dropped, %" PRIu64
            " tx-dropped)\n",
            num_peers, total_frames_forwarded,
            total_frames_dropped, total_frames_tx_dropped);
    free(pfds);
    cleanup();
    return 0;
}
