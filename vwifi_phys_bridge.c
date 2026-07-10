/*
 * vwifi-phys-bridge — Bridge a physical WiFi interface to a vwifi-medium hub
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Connects to a vwifi-medium hub as a regular peer, but instead of a
 * virtual radio it uses a real physical WiFi interface in monitor mode.
 *
 * Hub -> bridge: receives 802.11 frames, injects them over the air
 * Physical -> bridge: captures OTA frames, forwards them into the hub
 *
 * This lets a real device (e.g. a laptop) associate with a virtual AP
 * running inside a QEMU VM.
 *
 * Build:
 *   make vwifi-phys-bridge
 *
 * Usage:
 *   sudo ./vwifi-phys-bridge <hub-socket-path> <interface> -c <channel> [opts]
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <signal.h>
#include <poll.h>
#include <getopt.h>
#include <stdint.h>
#include <time.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <arpa/inet.h>
#include <linux/if_packet.h>
#include <linux/if_ether.h>

#include "vwifi.h"

/* ================================================================
 *  Global state
 * ================================================================ */

static volatile sig_atomic_t g_running = 1;

/* File descriptors */
static int hub_fd = -1;
static int raw_fd = -1;

/* Our interface MAC (== the AP BSSID when bridging a single AP). Used by
 * the phys->hub relevance filter so a busy channel's ambient traffic
 * isn't dumped wholesale into the medium (which floods the hub and can
 * crash the USB radio firmware). */
static uint8_t own_mac[6];
static int forward_all;   /* -A: disable the relevance filter (sniff mode) */

/* -b <bssid>: address the relevance filter matches on, instead of the
 * interface MAC. Needed for the two-radio setup where a dedicated ACK
 * radio holds MAC == BSSID and a *separate* radio runs this bridge for
 * capture/injection (its MAC is not the BSSID). */
static uint8_t filter_bssid[6];
static int have_filter_bssid;

/* -m <mask>: relevance-filter address mask (1=care, 0=don't-care), mirroring
 * the hardware bssidmask. Lets one bridge forward a whole prefix-aligned set
 * of lab BSSIDs (e.g. 02:11:22:33:44:00..FF with mask ff:ff:ff:ff:ff:00)
 * instead of a single exact address. Defaults to all-ones == exact match. */
static uint8_t filter_mask[6] = { 0xff, 0xff, 0xff, 0xff, 0xff, 0xff };

/* -r <n>: inject each critical downlink frame N times (default 1). Injected
 * frames are NO_ACK single-shot with no hardware retransmit (the injector can
 * never hear "its" ACK -- the frame's source is the VM BSSID, not this radio),
 * so on a congested/co-located channel a single collision drops a handshake
 * frame with only slow ~1s EAPOL recovery. Sending 2-3 copies (retry bit set
 * on copies 2+) trades a little airtime for a much better 4-way / assoc
 * completion rate. Only association-critical frames are duplicated: management
 * and the unprotected EAPOL handshake. Beacons/probe-responses (periodic) and
 * encrypted user data are always injected once -- the latter because upper
 * layers already retransmit and injected copies are not reliably de-duplicated
 * by the receiver, so they would surface as application-level dupes (ping
 * "DUP!"). */
static int inject_copies = 1;

/* -R <mbps>: fixed injected-downlink rate, in radiotap 500 kbps units.
 * 0 (default) = "auto": reproduce the rate the VM chose (the medium header's
 * rate_code), floored at 6 Mbps. Without a rate in the radiotap header the
 * mt76 monitor path transmits at the band's lowest basic rate (~1 Mbps in
 * 2.4 GHz), which caps downlink throughput hard. Note there is no rate
 * fallback on NO_ACK injection, so a fixed rate too high for the link SNR
 * just raises loss -- sweep to find the desense wall (24-36 Mbps is typical). */
static int inject_rate_rt;   /* 0 = auto/echo medium rate_code */

/* Injection rate mode (mt76/ath9k_htc ignore the requested rate and always
 * transmit at ~1 Mbps; a Realtek rtl88xxau honors these):
 *   inject_mcs >= 0  : -M, inject HT MCS <n> (TX_FLAGS + MCS radiotap)
 *   inject_rateless  : -R 0, inject with NO rate field so the driver picks
 *                      (rtl88xxau w/ rtw_monitor_disable_1m=1 => HT-MCS7/VHT-MCS9)
 *   otherwise        : legacy RATE (inject_rate_rt or the echoed medium rate) */
static int inject_mcs = -1;
static int inject_rateless;

/* Raise the interface MTU this high at startup so a full-size downlink
 * frame (radiotap + 802.11 header + 1500-byte payload, ~1540 bytes) can
 * be injected without EMSGSIZE ("Message too long"). */
#define INJECT_MTU  2400

/* CLI configuration */
static const char *hub_path;
static const char *ifname;
static int         channel_num;       /* raw -c value */
static const char *bw_str = "HT20";  /* -w value */
static int         center2_mhz;      /* -s value (VHT80+80 only) */
static char        node_id[64];      /* -n value or "phys-<ifname>" */
static int         verbose;

/* Channel/bandwidth state (filled by compute_channel_config) */
static struct {
    uint16_t channel_freq;      /* primary channel center freq MHz */
    uint16_t channel_flags;
    uint16_t channel_bond_freq; /* HT40 secondary freq, else 0 */
    uint16_t center_freq1;      /* VHT/HE primary segment center */
    uint16_t center_freq2;      /* VHT80+80 secondary segment center */
    uint16_t freq_lo;           /* lowest 20MHz channel in BW range */
    uint16_t freq_hi;           /* highest 20MHz channel in BW range */
} chan_cfg;

/* Statistics. Cumulative since start; the periodic reporter (-S) prints
 * per-interval deltas so you can see live fps/throughput and, crucially,
 * whether injects are being dropped by the driver's monitor TX queue
 * (EAGAIN/ENOBUFS) -- an otherwise-silent downlink loss. */
static uint64_t stats_hub_to_phys;      /* frames injected OK (per source frame) */
static uint64_t stats_phys_to_hub;      /* frames forwarded to the hub */
static uint64_t stats_echoes;           /* own injects suppressed on capture */
static uint64_t stats_hp_bytes;         /* bytes injected (hub->phys, on-air) */
static uint64_t stats_ph_bytes;         /* bytes forwarded (phys->hub) */
static uint64_t stats_inject_writes;    /* write() calls incl. -r copies */
static uint64_t stats_drop_enobufs;     /* injects dropped: driver TX queue full */
static uint64_t stats_drop_eagain;      /* injects dropped: socket would block */
static uint64_t stats_drop_err;         /* injects dropped: other write() error */

static int stats_interval;              /* -S <sec>: periodic report (0 = off) */
static volatile sig_atomic_t stats_dump_now;  /* set by SIGUSR1 */

/* ================================================================
 *  Signal handler
 * ================================================================ */

static void sig_handler(int sig)
{
    (void)sig;
    g_running = 0;
}

static void sigusr1_handler(int sig)
{
    (void)sig;
    stats_dump_now = 1;
}

/* Print a one-line throughput report: per-interval deltas plus cumulative
 * totals. Called on the -S timer and on SIGUSR1. `elapsed_ms` is the wall
 * time since the previous report (0 => print cumulative only, no rates). */
static void stats_report(double elapsed_ms)
{
    static uint64_t p_hp, p_ph, p_hpb, p_phb, p_deno, p_deag, p_derr, p_echo;

    uint64_t d_hp   = stats_hub_to_phys   - p_hp;
    uint64_t d_ph   = stats_phys_to_hub   - p_ph;
    uint64_t d_hpb  = stats_hp_bytes      - p_hpb;
    uint64_t d_phb  = stats_ph_bytes      - p_phb;
    uint64_t d_deno = stats_drop_enobufs  - p_deno;
    uint64_t d_deag = stats_drop_eagain   - p_deag;
    uint64_t d_derr = stats_drop_err      - p_derr;
    uint64_t d_echo = stats_echoes        - p_echo;

    if (elapsed_ms > 0) {
        double s = elapsed_ms / 1000.0;
        fprintf(stderr,
            "bridge: stats %.1fs | hub->phys %.0f fps %.2f Mbps "
            "drop(enobufs=%llu eagain=%llu err=%llu) | "
            "phys->hub %.0f fps %.2f Mbps | echoes %llu\n",
            s,
            d_hp / s, (d_hpb * 8.0) / (s * 1e6),
            (unsigned long long)d_deno, (unsigned long long)d_deag,
            (unsigned long long)d_derr,
            d_ph / s, (d_phb * 8.0) / (s * 1e6),
            (unsigned long long)d_echo);
    } else {
        fprintf(stderr,
            "bridge: stats (cumulative) | hub->phys %llu frames %llu bytes "
            "drop(enobufs=%llu eagain=%llu err=%llu) | phys->hub %llu frames "
            "%llu bytes | echoes %llu\n",
            (unsigned long long)stats_hub_to_phys,
            (unsigned long long)stats_hp_bytes,
            (unsigned long long)stats_drop_enobufs,
            (unsigned long long)stats_drop_eagain,
            (unsigned long long)stats_drop_err,
            (unsigned long long)stats_phys_to_hub,
            (unsigned long long)stats_ph_bytes,
            (unsigned long long)stats_echoes);
    }

    p_hp = stats_hub_to_phys; p_ph = stats_phys_to_hub;
    p_hpb = stats_hp_bytes;   p_phb = stats_ph_bytes;
    p_deno = stats_drop_enobufs; p_deag = stats_drop_eagain;
    p_derr = stats_drop_err;  p_echo = stats_echoes;
}

/* ================================================================
 *  Step 2: channel/frequency helpers
 * ================================================================ */

/*
 * Convert a WiFi channel number to center frequency in MHz.
 * Returns 0 on invalid channel.
 */
static uint16_t channel_to_freq(int ch)
{
    /* 2.4 GHz band */
    if (ch >= 1 && ch <= 13)
        return 2407 + ch * 5;
    if (ch == 14)
        return 2484;

    /* 5 GHz band */
    if (ch >= 36 && ch <= 165 && (ch % 4) == 0)
        return 5000 + ch * 5;
    /* Odd 5 GHz channels (e.g. 169) */
    if (ch > 14 && ch <= 200)
        return 5000 + ch * 5;

    return 0;
}

/*
 * VHT 80 MHz channel blocks in 5 GHz.
 * Each row: {lowest_ch, highest_ch, center_freq_mhz}
 */
static const struct {
    uint16_t lo_freq;   /* lowest 20MHz channel center */
    uint16_t hi_freq;   /* highest 20MHz channel center */
    uint16_t center;    /* 80MHz block center freq */
} vht80_blocks[] = {
    { 5180, 5240, 5210 },  /* ch 36-48  */
    { 5260, 5320, 5290 },  /* ch 52-64  */
    { 5500, 5560, 5530 },  /* ch 100-112 */
    { 5580, 5640, 5610 },  /* ch 116-128 */
    { 5660, 5720, 5690 },  /* ch 132-144 */
    { 5745, 5805, 5775 },  /* ch 149-161 */
};
#define NUM_VHT80_BLOCKS (sizeof(vht80_blocks) / sizeof(vht80_blocks[0]))

/*
 * Find the VHT80 block index that contains freq.
 * Returns -1 if not found.
 */
static int find_vht80_block(uint16_t freq)
{
    for (size_t i = 0; i < NUM_VHT80_BLOCKS; i++) {
        if (freq >= vht80_blocks[i].lo_freq &&
            freq <= vht80_blocks[i].hi_freq)
            return (int)i;
    }
    return -1;
}

/*
 * Compute channel configuration from CLI args.
 * Fills chan_cfg with freq, flags, bond freq, center freqs, and
 * the acceptance range [freq_lo, freq_hi].
 * Returns 0 on success, -1 on error.
 */
static int compute_channel_config(void)
{
    uint16_t freq;
    uint16_t band_flag;

    /* Resolve channel to frequency */
    if (channel_num > 200) {
        freq = (uint16_t)channel_num;
    } else {
        freq = channel_to_freq(channel_num);
        if (freq == 0) {
            fprintf(stderr, "bridge: unknown channel number %d\n",
                    channel_num);
            return -1;
        }
    }

    chan_cfg.channel_freq = freq;
    band_flag = (freq < 5000) ? VWIFI_CHAN_FLAG_2GHZ : VWIFI_CHAN_FLAG_5GHZ;

    /* Default: single 20 MHz channel */
    chan_cfg.freq_lo = freq;
    chan_cfg.freq_hi = freq;
    chan_cfg.channel_bond_freq = 0;
    chan_cfg.center_freq1 = 0;
    chan_cfg.center_freq2 = 0;

    if (strcmp(bw_str, "HT20") == 0) {
        chan_cfg.channel_flags = band_flag | VWIFI_CHAN_FLAG_HT20;

    } else if (strcmp(bw_str, "HT40+") == 0) {
        chan_cfg.channel_flags = band_flag | VWIFI_CHAN_FLAG_HT40PLUS;
        chan_cfg.channel_bond_freq = freq + 20;
        chan_cfg.freq_hi = freq + 20;

    } else if (strcmp(bw_str, "HT40-") == 0) {
        chan_cfg.channel_flags = band_flag | VWIFI_CHAN_FLAG_HT40MINUS;
        chan_cfg.channel_bond_freq = freq - 20;
        chan_cfg.freq_lo = freq - 20;

    } else if (strcmp(bw_str, "VHT80") == 0) {
        int blk = find_vht80_block(freq);
        if (blk < 0) {
            fprintf(stderr, "bridge: freq %u MHz not in any 80MHz block\n",
                    freq);
            return -1;
        }
        chan_cfg.channel_flags = band_flag | VWIFI_CHAN_FLAG_VHT80;
        chan_cfg.center_freq1 = vht80_blocks[blk].center;
        chan_cfg.freq_lo = vht80_blocks[blk].lo_freq;
        chan_cfg.freq_hi = vht80_blocks[blk].hi_freq;

    } else if (strcmp(bw_str, "VHT160") == 0) {
        int blk = find_vht80_block(freq);
        if (blk < 0 || blk + 1 >= (int)NUM_VHT80_BLOCKS) {
            fprintf(stderr, "bridge: freq %u MHz not in a valid 160MHz block\n",
                    freq);
            return -1;
        }
        /* 160 MHz = two adjacent 80 MHz blocks */
        int blk2 = blk + 1;
        /* Verify the blocks are truly adjacent (no gap) */
        if (vht80_blocks[blk2].lo_freq - vht80_blocks[blk].hi_freq > 20) {
            fprintf(stderr,
                "bridge: freq %u MHz: adjacent 80MHz blocks are not contiguous\n",
                freq);
            return -1;
        }
        chan_cfg.channel_flags = band_flag | VWIFI_CHAN_FLAG_VHT160;
        chan_cfg.center_freq1 =
            (vht80_blocks[blk].center + vht80_blocks[blk2].center) / 2;
        chan_cfg.freq_lo = vht80_blocks[blk].lo_freq;
        chan_cfg.freq_hi = vht80_blocks[blk2].hi_freq;

    } else if (strcmp(bw_str, "VHT80+80") == 0) {
        int blk1 = find_vht80_block(freq);
        if (blk1 < 0) {
            fprintf(stderr, "bridge: freq %u MHz not in any 80MHz block\n",
                    freq);
            return -1;
        }
        if (center2_mhz == 0) {
            fprintf(stderr,
                "bridge: VHT80+80 requires -s <center2_mhz>\n");
            return -1;
        }
        /* Find the second 80 MHz block by its center freq */
        int blk2 = -1;
        for (size_t i = 0; i < NUM_VHT80_BLOCKS; i++) {
            if (vht80_blocks[i].center == (uint16_t)center2_mhz) {
                blk2 = (int)i;
                break;
            }
        }
        if (blk2 < 0) {
            fprintf(stderr,
                "bridge: -s %d does not match any 80MHz block center\n",
                center2_mhz);
            return -1;
        }
        chan_cfg.channel_flags = band_flag | VWIFI_CHAN_FLAG_VHT80_80;
        chan_cfg.center_freq1 = vht80_blocks[blk1].center;
        chan_cfg.center_freq2 = (uint16_t)center2_mhz;
        /* Acceptance range covers both blocks */
        chan_cfg.freq_lo = vht80_blocks[blk1].lo_freq < vht80_blocks[blk2].lo_freq
                        ? vht80_blocks[blk1].lo_freq : vht80_blocks[blk2].lo_freq;
        chan_cfg.freq_hi = vht80_blocks[blk1].hi_freq > vht80_blocks[blk2].hi_freq
                        ? vht80_blocks[blk1].hi_freq : vht80_blocks[blk2].hi_freq;

    } else {
        fprintf(stderr, "bridge: unknown bandwidth mode: %s\n", bw_str);
        fprintf(stderr, "bridge: valid: HT20, HT40+, HT40-, VHT80, VHT160, VHT80+80\n");
        return -1;
    }

    return 0;
}

/* ================================================================
 *  Step 3: rate mapping helpers
 * ================================================================ */

/* Radiotap uses 500 kbps units; the vwifi medium protocol uses the
 * legacy PLCP rate codes from vwifi.h. This table maps between the
 * two in both directions: the receive path (radiotap -> medium) via
 * rt_rate_to_code(), and the injection path (medium -> radiotap) via
 * code_to_rt_rate(), which lets the downlink honor the rate the VM
 * actually chose instead of the mt76 monitor default (~1 Mbps). */
static const struct {
    uint8_t rt_rate;        /* radiotap 500kbps units */
    uint8_t code;           /* medium PLCP rate code */
} rate_map[] = {
    {   2, 0x1B },  /*  1   Mbps CCK  */
    {   4, 0x1A },  /*  2   Mbps CCK  */
    {  11, 0x19 },  /*  5.5 Mbps CCK  */
    {  22, 0x18 },  /* 11   Mbps CCK  */
    {  12, 0x0B },  /*  6   Mbps OFDM */
    {  18, 0x0F },  /*  9   Mbps OFDM */
    {  24, 0x0A },  /* 12   Mbps OFDM */
    {  36, 0x0E },  /* 18   Mbps OFDM */
    {  48, 0x09 },  /* 24   Mbps OFDM */
    {  72, 0x0D },  /* 36   Mbps OFDM */
    {  96, 0x08 },  /* 48   Mbps OFDM */
    { 108, 0x0C },  /* 54   Mbps OFDM */
};
#define NUM_RATES (sizeof(rate_map) / sizeof(rate_map[0]))

/* Convert a radiotap rate (500 kbps units) to a vwifi medium rate code. */
static uint8_t rt_rate_to_code(uint8_t rt)
{
    for (size_t i = 0; i < NUM_RATES; i++) {
        if (rate_map[i].rt_rate == rt)
            return rate_map[i].code;
    }
    return VWIFI_DEFAULT_RATE;  /* 6 Mbps OFDM */
}

/* Convert a vwifi medium rate code to a radiotap rate (500 kbps units).
 * Used on the inject path to reproduce the VM's chosen TX rate over the
 * air. Falls back to 6 Mbps OFDM (rt 12) for an unknown/zero code so the
 * downlink never drops to the ~1 Mbps monitor default. */
#define INJECT_RATE_FLOOR_RT 12   /* 6 Mbps OFDM */
static uint8_t code_to_rt_rate(uint8_t code)
{
    for (size_t i = 0; i < NUM_RATES; i++) {
        if (rate_map[i].code == code)
            return rate_map[i].rt_rate;
    }
    return INJECT_RATE_FLOOR_RT;
}

/* ================================================================
 *  Step 4: echo suppression
 *
 *  When we inject a frame via monitor mode, the radio captures it
 *  back. We use an FNV-1a hash ring buffer to detect and suppress
 *  these echoes.
 * ================================================================ */

#define ECHO_RING_SIZE  256
#define ECHO_EXPIRE_MS  100

struct echo_entry {
    uint32_t hash;
    uint16_t frame_len;
    uint64_t timestamp_ms;
};

static struct echo_entry echo_ring[ECHO_RING_SIZE];
static int echo_ring_head;

static uint32_t fnv1a(const uint8_t *data, size_t len)
{
    uint32_t h = 0x811C9DC5;
    for (size_t i = 0; i < len; i++) {
        h ^= data[i];
        h *= 0x01000193;
    }
    return h;
}

static uint64_t now_ms(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000 + (uint64_t)ts.tv_nsec / 1000000;
}

/* True if (addr & mask) == (key & mask) -- the same match the hardware
 * bssidmask uses. Lets the relevance filter accept a prefix-aligned set. */
static int addr_in_range(const uint8_t *addr, const uint8_t *key,
                         const uint8_t *mask)
{
    for (int i = 0; i < 6; i++)
        if ((addr[i] & mask[i]) != (key[i] & mask[i]))
            return 0;
    return 1;
}

/*
 * Record a frame hash before injection (hub->phys path).
 * Called just before writing to the raw socket so we can
 * recognise the echo when it comes back.
 */
static void echo_record(const uint8_t *frame, size_t len)
{
    struct echo_entry *e = &echo_ring[echo_ring_head];
    e->hash = fnv1a(frame, len);
    e->frame_len = (uint16_t)len;
    e->timestamp_ms = now_ms();
    echo_ring_head = (echo_ring_head + 1) % ECHO_RING_SIZE;
}

/*
 * Check if a captured frame is an echo of something we injected.
 * Returns 1 (echo, should drop) or 0 (genuine capture, forward).
 */
static int echo_check(const uint8_t *frame, size_t len)
{
    uint32_t h = fnv1a(frame, len);
    uint64_t now = now_ms();

    for (int i = 0; i < ECHO_RING_SIZE; i++) {
        struct echo_entry *e = &echo_ring[i];
        if (e->hash == h &&
            e->frame_len == (uint16_t)len &&
            (now - e->timestamp_ms) < ECHO_EXPIRE_MS) {
            /* Match — zero out entry and report echo */
            memset(e, 0, sizeof(*e));
            return 1;
        }
    }
    return 0;
}

/* ================================================================
 *  Step 5: hub connection + hello
 * ================================================================ */

#define HELLO_MAGIC 0x52495756  /* "VWIR" -- vwifi registration */

/*
 * Write all bytes to fd, retrying on EINTR.
 * Returns 0 on success, -1 on error.
 */
static int write_all(int fd, const void *buf, size_t len)
{
    const uint8_t *p = buf;
    size_t done = 0;
    while (done < len) {
        ssize_t n = write(fd, p + done, len - done);
        if (n < 0) {
            if (errno == EINTR)
                continue;
            return -1;
        }
        done += n;
    }
    return 0;
}

/*
 * Connect to the hub's Unix domain socket.
 * Returns fd on success, -1 on failure.
 */
static int connect_hub(const char *path)
{
    int fd;
    struct sockaddr_un addr;

    fd = socket(AF_UNIX, SOCK_STREAM, 0);
    if (fd < 0) {
        perror("bridge: socket(AF_UNIX)");
        return -1;
    }

    memset(&addr, 0, sizeof(addr));
    addr.sun_family = AF_UNIX;
    snprintf(addr.sun_path, sizeof(addr.sun_path), "%s", path);

    if (connect(fd, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        fprintf(stderr, "bridge: connect(%s): %s\n", path, strerror(errno));
        close(fd);
        return -1;
    }

    return fd;
}

/*
 * Send the hello/registration message to the hub.
 * Format: [uint32_t len (net order)][uint32_t HELLO_MAGIC][node_id\0][flags]
 *
 * The trailing flags byte announces this peer as a physical radio so the
 * hub exempts its links from the simulated propagation model -- real RF
 * is the only thing that should drop or attenuate these frames.
 */
static int send_hello(int fd, const char *id)
{
    size_t id_len = strlen(id) + 1;  /* include null terminator */
    uint32_t payload_len = 4 + id_len + 1;  /* + flags byte */
    uint32_t len_be = htonl(payload_len);
    uint32_t magic = HELLO_MAGIC;

    uint8_t buf[4 + 4 + 64 + 1];
    if (4 + payload_len > sizeof(buf)) {
        fprintf(stderr, "bridge: node_id too long\n");
        return -1;
    }

    memcpy(buf, &len_be, 4);
    memcpy(buf + 4, &magic, 4);
    memcpy(buf + 8, id, id_len);
    buf[8 + id_len] = VWIFI_HELLO_FLAG_PHYSICAL;

    if (write_all(fd, buf, 4 + payload_len) < 0) {
        fprintf(stderr, "bridge: send_hello: %s\n", strerror(errno));
        return -1;
    }

    return 0;
}

/* ================================================================
 *  Step 6: physical interface setup (AF_PACKET raw socket)
 * ================================================================ */

#define ARPHRD_IEEE80211_RADIOTAP 803

/*
 * Verify the interface is in monitor mode by reading its type
 * from sysfs.  Returns 0 if OK, -1 on error.
 */
static int check_monitor_mode(const char *iface)
{
    char path[256];
    FILE *fp;
    int type = -1;

    snprintf(path, sizeof(path), "/sys/class/net/%s/type", iface);
    fp = fopen(path, "r");
    if (!fp) {
        fprintf(stderr, "bridge: cannot read %s: %s\n",
                path, strerror(errno));
        fprintf(stderr, "bridge: does interface %s exist?\n", iface);
        return -1;
    }
    if (fscanf(fp, "%d", &type) != 1)
        type = -1;
    fclose(fp);

    if (type != ARPHRD_IEEE80211_RADIOTAP) {
        fprintf(stderr,
            "Error: %s is not in monitor mode (type=%d, expected %d)\n"
            "Fix: sudo ip link set %s down && "
            "sudo iw dev %s set type monitor && "
            "sudo ip link set %s up\n",
            iface, type, ARPHRD_IEEE80211_RADIOTAP,
            iface, iface, iface);
        return -1;
    }

    return 0;
}

/*
 * Open an AF_PACKET raw socket bound to the given interface.
 * Returns fd on success, -1 on failure.
 */
static int open_raw_socket(const char *iface)
{
    int fd;
    unsigned int ifindex;
    struct sockaddr_ll sll;

    ifindex = if_nametoindex(iface);
    if (ifindex == 0) {
        fprintf(stderr, "bridge: interface %s not found: %s\n",
                iface, strerror(errno));
        return -1;
    }

    fd = socket(AF_PACKET, SOCK_RAW, htons(ETH_P_ALL));
    if (fd < 0) {
        perror("bridge: socket(AF_PACKET)");
        return -1;
    }

    memset(&sll, 0, sizeof(sll));
    sll.sll_family   = AF_PACKET;
    sll.sll_protocol = htons(ETH_P_ALL);
    sll.sll_ifindex  = (int)ifindex;

    if (bind(fd, (struct sockaddr *)&sll, sizeof(sll)) < 0) {
        perror("bridge: bind(AF_PACKET)");
        close(fd);
        return -1;
    }

    /* Read our own MAC (== the AP BSSID) for the relevance filter. */
    {
        struct ifreq ifr;
        memset(&ifr, 0, sizeof(ifr));
        strncpy(ifr.ifr_name, iface, IFNAMSIZ - 1);
        if (ioctl(fd, SIOCGIFHWADDR, &ifr) == 0) {
            memcpy(own_mac, ifr.ifr_hwaddr.sa_data, 6);
            const uint8_t *k = have_filter_bssid ? filter_bssid : own_mac;
            fprintf(stderr, "bridge: interface MAC %02x:%02x:%02x:%02x:%02x:%02x;"
                    " relevance filter %s (match %02x:%02x:%02x:%02x:%02x:%02x"
                    " mask %02x:%02x:%02x:%02x:%02x:%02x)\n",
                    own_mac[0], own_mac[1], own_mac[2],
                    own_mac[3], own_mac[4], own_mac[5],
                    forward_all ? "OFF (-A)" : "on",
                    k[0], k[1], k[2], k[3], k[4], k[5],
                    filter_mask[0], filter_mask[1], filter_mask[2],
                    filter_mask[3], filter_mask[4], filter_mask[5]);
        } else {
            fprintf(stderr, "bridge: SIOCGIFHWADDR: %s "
                    "(relevance filter disabled)\n", strerror(errno));
            forward_all = 1;
        }
    }

    /* Raise the MTU so full-size downlink frames can be injected. Drivers
     * cap this differently (ath9k_htc rejects 2400), so probe a descending
     * list and keep the highest the driver accepts. Best effort: if none
     * take, warn -- the operator can clamp MSS on the AP instead. */
    {
        static const int mtu_try[] = { INJECT_MTU, 2304, 2048, 1800, 1600 };
        int set_mtu = 0;
        for (size_t i = 0; i < sizeof(mtu_try) / sizeof(mtu_try[0]); i++) {
            struct ifreq ifr;
            memset(&ifr, 0, sizeof(ifr));
            strncpy(ifr.ifr_name, iface, IFNAMSIZ - 1);
            ifr.ifr_mtu = mtu_try[i];
            if (ioctl(fd, SIOCSIFMTU, &ifr) == 0) {
                set_mtu = mtu_try[i];
                break;
            }
        }
        if (set_mtu)
            fprintf(stderr, "bridge: %s MTU set to %d\n", iface, set_mtu);
        else
            fprintf(stderr, "bridge: could not raise %s MTU (driver cap): %s "
                    "-- large downlink frames will fail; clamp MSS on the AP "
                    "(e.g. lower br-lan MTU to 1400)\n",
                    iface, strerror(errno));
    }

    return fd;
}

/* Set when the capture interface flaps (e.g. an mt76 USB reset produces
 * ENETDOWN on read). The main loop reopens the raw socket instead of exiting,
 * so a momentary radio reset is a self-healing blip rather than a full outage
 * that needs a manual restart. */
static int raw_needs_reopen;

/* Close and reopen the raw capture socket, retrying with backoff until the
 * interface comes back up (or shutdown is requested). Updates the global
 * raw_fd; the main loop rebinds its pollfd to it each iteration. */
static void reopen_raw_socket(void)
{
    if (raw_fd >= 0) {
        close(raw_fd);
        raw_fd = -1;
    }
    fprintf(stderr, "bridge: capture interface %s down -- reopening...\n", ifname);
    int backoff_ms = 200;
    while (g_running) {
        int fd = open_raw_socket(ifname);
        if (fd >= 0) {
            raw_fd = fd;
            fprintf(stderr, "bridge: %s recovered\n", ifname);
            return;
        }
        struct timespec ts = { backoff_ms / 1000,
                               (long)(backoff_ms % 1000) * 1000000L };
        nanosleep(&ts, NULL);
        if (backoff_ms < 2000)
            backoff_ms *= 2;
    }
}

/* ================================================================
 *  Step 7: radiotap parsing
 *
 *  Parse the variable-length radiotap header prepended by the kernel
 *  to every monitor-mode capture.  We extract rate, channel, signal,
 *  flags, and optionally TSFT.
 * ================================================================ */

/* Radiotap present-word bit numbers */
#define RT_TSFT             0
#define RT_FLAGS            1
#define RT_RATE             2
#define RT_CHANNEL          3
#define RT_FHSS             4
#define RT_DBM_ANTSIGNAL    5
#define RT_DBM_ANTNOISE     6
#define RT_EXT              31

/* Radiotap FLAGS bits */
#define RT_FLAGS_FCS        0x10  /* FCS included at end of frame */

struct radiotap_info {
    uint8_t  rate;          /* 500 kbps units */
    uint16_t chan_freq;     /* MHz */
    uint16_t chan_flags;    /* radiotap channel flags (not vwifi flags) */
    int8_t   signal_dbm;
    uint64_t tsft;
    uint8_t  flags;         /* radiotap FLAGS field */
    int      has_rate;
    int      has_channel;
    int      has_signal;
    int      has_tsft;
    int      has_flags;
};

/* Field sizes for radiotap bits 0-6 (size, alignment) */
static const struct { uint8_t size; uint8_t align; } rt_fields[] = {
    [RT_TSFT]          = { 8, 8 },  /* u64 */
    [RT_FLAGS]         = { 1, 1 },  /* u8  */
    [RT_RATE]          = { 1, 1 },  /* u8  */
    [RT_CHANNEL]       = { 4, 2 },  /* u16 freq + u16 flags */
    [RT_FHSS]          = { 2, 2 },  /* u8 hop_set + u8 hop_pattern */
    [RT_DBM_ANTSIGNAL] = { 1, 1 },  /* s8  */
    [RT_DBM_ANTNOISE]  = { 1, 1 },  /* s8  */
};
#define RT_NUM_KNOWN_FIELDS 7

/*
 * Align cursor to required boundary (relative to radiotap header start).
 */
static inline size_t rt_align(size_t cursor, uint8_t alignment)
{
    return (cursor + alignment - 1) & ~((size_t)alignment - 1);
}

/*
 * Parse a radiotap header.
 * buf points to the start of the radiotap header, len is total capture length.
 * Returns radiotap header length on success, 0 on failure.
 */
static size_t parse_radiotap(const uint8_t *buf, size_t len,
                             struct radiotap_info *info)
{
    uint16_t rt_len;
    uint32_t present;
    size_t cursor;

    memset(info, 0, sizeof(*info));

    /* Minimum radiotap header: 8 bytes */
    if (len < 8)
        return 0;

    /* Version must be 0 */
    if (buf[0] != 0)
        return 0;

    /* Total radiotap length (LE) */
    memcpy(&rt_len, buf + 2, 2);
    if (rt_len < 8 || rt_len > len)
        return 0;

    /* First present word */
    memcpy(&present, buf + 4, 4);

    /* Skip past all present words (bit 31 = extension) */
    cursor = 8;
    {
        uint32_t p = present;
        while (p & (1u << RT_EXT)) {
            if (cursor + 4 > rt_len)
                return 0;
            memcpy(&p, buf + cursor, 4);
            cursor += 4;
        }
    }

    /* Walk bits 0-6 of the first present word, skipping/reading fields */
    for (int bit = 0; bit < RT_NUM_KNOWN_FIELDS && cursor < rt_len; bit++) {
        if (!(present & (1u << bit)))
            continue;

        /* Align cursor */
        cursor = rt_align(cursor, rt_fields[bit].align);
        if (cursor + rt_fields[bit].size > rt_len)
            break;

        switch (bit) {
        case RT_TSFT:
            memcpy(&info->tsft, buf + cursor, 8);
            info->has_tsft = 1;
            break;
        case RT_FLAGS:
            info->flags = buf[cursor];
            info->has_flags = 1;
            break;
        case RT_RATE:
            info->rate = buf[cursor];
            info->has_rate = 1;
            break;
        case RT_CHANNEL:
            memcpy(&info->chan_freq, buf + cursor, 2);
            memcpy(&info->chan_flags, buf + cursor + 2, 2);
            info->has_channel = 1;
            break;
        case RT_DBM_ANTSIGNAL:
            info->signal_dbm = (int8_t)buf[cursor];
            info->has_signal = 1;
            break;
        default:
            break;
        }

        cursor += rt_fields[bit].size;
    }

    return rt_len;
}

/* ================================================================
 *  Step 8: hub-to-physical path (process_hub_message)
 *
 *  Receives a complete wire-protocol message from the hub, validates
 *  it, applies the channel filter, and injects the 802.11 frame
 *  over the air via the monitor-mode interface.
 * ================================================================ */

/*
 * Radiotap header for injection.
 *
 * We MUST advertise IEEE80211_RADIOTAP_TX_FLAGS with the NO_ACK bit set.
 * Without it, mac80211 treats an injected frame as an ordinary transmit that
 * expects a hardware ACK, and when the injecting radio does not hear one it
 * retransmits the frame up to the driver's retry limit (~15x). For a virtual
 * AP whose real ACKs are produced by a *different* logical entity (the phone
 * ACKs the downlink in the air; the AR9271 ACKs the uplink), those hardware
 * retransmits are pure self-inflicted airtime: each downlink EAPOL/data frame
 * hits the air ~15 times with the retry flag set, saturating the SIFS window
 * that the station's uplink ACK needs and stalling the 4-way handshake.
 *
 * NO_ACK => inject exactly once, do not wait for an ACK, do not retransmit.
 * The medium is the retransmit authority; the air is a single-shot relay.
 *
 * The rate is set per the injection mode (see inject_mcs/inject_rateless):
 * legacy RATE (bit 2), HT MCS (bit 19), or rate-less (no rate field, driver
 * picks). All three keep TX_FLAGS=NO_ACK. build_inject_rtap() writes the right
 * header for the mode and returns its length.
 *
 *   present  = (1<<2)|(1<<15)=0x8004 legacy | (1<<15)|(1<<19)=0x88000 HT |
 *              (1<<15)=0x8000 rate-less
 *   tx_flags = 0x0008 (IEEE80211_RADIOTAP_F_TX_NOACK)
 */
#define RADIOTAP_TX_FLAG_NOACK 0x0008
#define INJECT_MAX_RTAP        13   /* largest header (HT MCS variant) */

/* Build the injection radiotap header into buf; return its byte length. */
static int build_inject_rtap(uint8_t *buf, uint8_t medium_rate_code)
{
    if (inject_mcs >= 0) {                 /* HT MCS: TX_FLAGS(15) + MCS(19) */
        static const uint8_t h[13] = {
            0x00, 0x00, 0x0d, 0x00,        /* version, pad, len=13 */
            0x00, 0x80, 0x08, 0x00,        /* present = TX_FLAGS | MCS */
            0x08, 0x00,                    /* [8]  tx_flags = NO_ACK */
            0x02, 0x00, 0x00               /* [10] mcs: known=idx, flags, index */
        };
        memcpy(buf, h, 13);
        buf[12] = (uint8_t)inject_mcs;
        return 13;
    }
    if (inject_rateless) {                 /* no rate field: driver picks */
        static const uint8_t h[10] = {
            0x00, 0x00, 0x0a, 0x00,        /* version, pad, len=10 */
            0x00, 0x80, 0x00, 0x00,        /* present = TX_FLAGS only */
            0x08, 0x00                     /* [8] tx_flags = NO_ACK */
        };
        memcpy(buf, h, 10);
        return 10;
    }
    static const uint8_t h[12] = {         /* legacy RATE(2) + TX_FLAGS(15) */
        0x00, 0x00, 0x0c, 0x00,            /* version, pad, len=12 */
        0x04, 0x80, 0x00, 0x00,            /* present = RATE | TX_FLAGS */
        0x00, 0x00,                        /* [8] rate (patched), [9] pad */
        0x08, 0x00                         /* [10] tx_flags = NO_ACK */
    };
    memcpy(buf, h, 12);
    buf[8] = inject_rate_rt ? (uint8_t)inject_rate_rt
                            : code_to_rt_rate(medium_rate_code);
    return 12;
}

/*
 * Process a single hub message (payload after the 4-byte length prefix).
 * Validates, filters by channel, and injects the frame.
 */
static void process_hub_message(const uint8_t *payload, uint32_t payload_len)
{
    const struct vwifi_frame_hdr *hdr;
    const uint8_t *frame;
    uint16_t frame_len;
    uint16_t msg_chan_freq;
    uint8_t inject_buf[INJECT_MAX_RTAP + VWIFI_MAX_FRAME_SIZE];
    int rtap_len;
    ssize_t n;

    /* Need at least a v1 header */
    if (payload_len < VWIFI_HDR_SIZE_V1) {
        if (verbose)
            fprintf(stderr, "bridge: hub->phys: short message (%u bytes)\n",
                    payload_len);
        return;
    }

    hdr = (const struct vwifi_frame_hdr *)payload;

    /* Validate magic */
    if (hdr->magic != VWIFI_MAGIC) {
        if (verbose)
            fprintf(stderr, "bridge: hub->phys: bad magic 0x%08x\n",
                    hdr->magic);
        return;
    }

    /* Extract channel frequency (v2 has it at offset 28; v1 = treat as 0) */
    if (payload_len >= VWIFI_HDR_SIZE)
        msg_chan_freq = hdr->channel_freq;
    else
        msg_chan_freq = 0;

    /* Channel filter: accept if freq==0 (broadcast) or in our range */
    if (msg_chan_freq != 0 &&
        (msg_chan_freq < chan_cfg.freq_lo || msg_chan_freq > chan_cfg.freq_hi)) {
        if (verbose)
            fprintf(stderr,
                "bridge: hub->phys: filtered freq=%u (range %u-%u)\n",
                msg_chan_freq, chan_cfg.freq_lo, chan_cfg.freq_hi);
        return;
    }

    /* Locate the 802.11 frame */
    frame_len = hdr->frame_len;
    if (payload_len >= VWIFI_HDR_SIZE)
        frame = payload + VWIFI_HDR_SIZE;
    else
        frame = payload + VWIFI_HDR_SIZE_V1;

    /* Sanity check frame length */
    if (frame_len == 0 || frame_len > VWIFI_MAX_FRAME_SIZE) {
        if (verbose)
            fprintf(stderr, "bridge: hub->phys: bad frame_len=%u\n",
                    frame_len);
        return;
    }

    /* Record echo hash before injection */
    echo_record(frame, frame_len);

    /* Build injection buffer: radiotap header (per injection mode) + frame. */
    rtap_len = build_inject_rtap(inject_buf, hdr->rate_code);
    memcpy(inject_buf + rtap_len, frame, frame_len);

    /* Redundant injection is only for the one-shot, association-critical
     * frames that have no L2 retransmit and whose loss stalls the connection:
     * management (auth/assoc/deauth/disassoc) and the unprotected EAPOL
     * handshake (data frames sent before the key is installed). Everything
     * else is injected once:
     *   - beacons / probe responses: periodic, one loss is harmless;
     *   - encrypted user data (Protected bit set): duplicating it delivers
     *     duplicates all the way to the application (ping "DUP!", etc.) --
     *     the receiver's dup filter does not reliably drop injected copies --
     *     and TCP/ARP/DHCP already retransmit on their own. */
    int copies = inject_copies;
    {
        int ftype       = (frame[0] >> 2) & 0x3;   /* 0=mgmt,1=ctrl,2=data */
        int fsub        = (frame[0] >> 4) & 0xf;
        int protected_f = frame[1] & 0x40;         /* FC Protected bit */
        if (ftype == 0 && (fsub == 8 || fsub == 5))   /* beacon / probe-resp */
            copies = 1;
        else if (ftype == 2 && protected_f)           /* encrypted user data */
            copies = 1;
    }

    int any_ok = 0;
    for (int c = 0; c < copies; c++) {
        /* Mark copies 2+ as retransmissions so the receiver's duplicate
         * filter drops them when an earlier copy already arrived. */
        if (c == 1)
            inject_buf[rtap_len + 1] |= 0x08;  /* FC retry bit */

        stats_inject_writes++;
        n = write(raw_fd, inject_buf, rtap_len + frame_len);
        if (n < 0) {
            /* ENOBUFS/EAGAIN = the driver's monitor TX queue is full: the
             * frame is dropped on the floor with no L2 retransmit. Count it
             * (this is the silent downlink loss that caps throughput on a
             * fast offered load) rather than swallowing it. */
            if (errno == ENOBUFS) {
                stats_drop_enobufs++;
                continue;  /* a later copy may still get a queue slot */
            }
            if (errno == EAGAIN) {
                stats_drop_eagain++;
                continue;
            }
            stats_drop_err++;
            if (errno == EMSGSIZE)
                fprintf(stderr, "bridge: inject write: Message too long "
                        "(frame=%u, need MTU >= %d; raise it or clamp MSS)\n",
                        frame_len, rtap_len + frame_len);
            else
                fprintf(stderr, "bridge: inject write: %s\n", strerror(errno));
            break;  /* EMSGSIZE/hard errors fail every copy identically */
        }
        any_ok = 1;
        stats_hp_bytes += frame_len;
    }

    if (any_ok)
        stats_hub_to_phys++;
    if (verbose)
        fprintf(stderr, "bridge: hub->phys: injected %u bytes x%d (ch=%u)\n",
                frame_len, copies, msg_chan_freq);
}

/* ================================================================
 *  Step 9: physical-to-hub path (handle_phys_data)
 *
 *  Reads a captured frame from the AF_PACKET socket, parses the
 *  radiotap header, strips FCS if present, checks for echo, extracts
 *  the transmitter MAC, builds the medium header, and sends to the hub.
 * ================================================================ */

/*
 * 802.11 frame type/subtype helpers.
 * Frame Control is the first 2 bytes (LE).  Bits 3:2 = type.
 * Type 1 = Control frames — some lack Address 2.
 */
#define IEEE80211_FC_TYPE_MASK  0x000C
#define IEEE80211_FC_TYPE_CTL   0x0004

/* Control subtypes that have Address 2 (offset 10): RTS, PS-Poll, CF-End, BAR, BA */
static int ctl_frame_has_addr2(uint16_t fc)
{
    uint8_t subtype = (fc >> 4) & 0x0F;
    /* RTS=11, PS-Poll=10, CF-End=14, CF-End+Ack=15, BAR=8, BA=9 */
    switch (subtype) {
    case 8: case 9: case 10: case 11: case 14: case 15:
        return 1;
    default:
        return 0;
    }
}

static void handle_phys_data(void)
{
    uint8_t buf[8 + VWIFI_MAX_FRAME_SIZE + 64];
    ssize_t nread;
    struct radiotap_info rt_info;
    size_t rt_len;
    const uint8_t *frame;
    size_t frame_len;
    uint16_t fc;
    struct vwifi_frame_hdr hdr;
    uint32_t msg_len;
    uint32_t len_be;
    uint8_t sendbuf[4 + sizeof(hdr) + VWIFI_MAX_FRAME_SIZE];

    nread = read(raw_fd, buf, sizeof(buf));
    if (nread <= 0) {
        if (nread < 0 && (errno == EAGAIN || errno == EINTR))
            return;
        /* ENETDOWN and friends: the capture radio flapped (mt76 USB reset).
         * Recover by reopening the socket rather than exiting. */
        fprintf(stderr, "bridge: raw socket read error: %s -- recovering\n",
                strerror(errno));
        raw_needs_reopen = 1;
        return;
    }

    /* Parse radiotap header */
    rt_len = parse_radiotap(buf, (size_t)nread, &rt_info);
    if (rt_len == 0) {
        if (verbose)
            fprintf(stderr, "bridge: phys->hub: bad radiotap header\n");
        return;
    }

    /* 802.11 frame starts after radiotap */
    frame = buf + rt_len;
    frame_len = (size_t)nread - rt_len;

    /* Strip FCS if radiotap FLAGS says it's included */
    if (rt_info.has_flags && (rt_info.flags & RT_FLAGS_FCS)) {
        if (frame_len < 4)
            return;
        frame_len -= 4;
    }

    /* Minimum frame: FC (2) + Duration (2) + Addr1 (6) = 10 bytes */
    if (frame_len < 10)
        return;

    /* Oversized frame check */
    if (frame_len > VWIFI_MAX_FRAME_SIZE)
        return;

    /* Relevance filter: on a busy channel the radio hears tens of thousands
     * of unrelated frames; forwarding them all floods the hub, loads the
     * bridge, and can crash the USB firmware. Keep only frames that belong to
     * our lab BSSID set. Disable with -A to forward everything.
     *
     *  - Control frames (ACK/RTS/CTS/BlockAck): never forwarded. They are
     *    PHY-local and meaningless in the medium.
     *  - Unicast: Addr1 (RA, offset 4) must be in our BSSID range.
     *  - Group-addressed: forward probe requests (discovery) and any frame
     *    whose BSSID (Addr3, offset 16) is in our range; drop foreign beacons
     *    and neighbours' broadcasts -- the bulk of the ambient flood. A
     *    client's broadcast uplink (DHCP/ARP) is toDS with RA == BSSID, so it
     *    is unicast-at-L2 and already covered by the RA test above. */
    if (!forward_all) {
        int ftype = (frame[0] >> 2) & 0x3;   /* 0=mgmt,1=ctrl,2=data */
        int fsub  = (frame[0] >> 4) & 0xf;

        if (ftype == 1)                      /* control frame */
            return;

        const uint8_t *ra = frame + 4;
        const uint8_t *key = have_filter_bssid ? filter_bssid : own_mac;
        int is_group = (ra[0] & 0x01);

        if (!is_group) {
            if (!addr_in_range(ra, key, filter_mask))
                return;
        } else {
            int is_probe_req = (ftype == 0 && fsub == 4);
            const uint8_t *bssid = frame + 16;   /* Addr3 (mgmt: == BSSID) */
            int have_bssid = (frame_len >= 24);
            if (!is_probe_req &&
                !(have_bssid && addr_in_range(bssid, key, filter_mask)))
                return;
        }
    }

    /* Echo check — drop frames we injected */
    if (echo_check(frame, frame_len)) {
        stats_echoes++;
        if (verbose)
            fprintf(stderr, "bridge: phys->hub: echo suppressed\n");
        return;
    }

    /* Extract frame control to determine frame type */
    memcpy(&fc, frame, 2);

    /* Control frames: some lack Address 2 — skip those */
    if ((fc & IEEE80211_FC_TYPE_MASK) == IEEE80211_FC_TYPE_CTL) {
        if (!ctl_frame_has_addr2(fc))
            return;
    }

    /* Need at least 16 bytes to have Address 2 (offset 10, 6 bytes) */
    if (frame_len < 16)
        return;

    /* Build medium header */
    memset(&hdr, 0, sizeof(hdr));
    hdr.magic   = VWIFI_MAGIC;
    hdr.version = VWIFI_VERSION;
    hdr.frame_len = (uint16_t)frame_len;

    /* tx_mac = Address 2 (transmitter) at offset 10 */
    memcpy(hdr.tx_mac, frame + 10, 6);

    /* Rate */
    hdr.rate_code = rt_info.has_rate
                  ? rt_rate_to_code(rt_info.rate)
                  : VWIFI_DEFAULT_RATE;

    /* RSSI */
    hdr.rssi = rt_info.has_signal
             ? rt_info.signal_dbm
             : VWIFI_DEFAULT_RSSI;

    /* TSF */
    if (rt_info.has_tsft) {
        hdr.tsf_lo = (uint32_t)(rt_info.tsft & 0xFFFFFFFF);
        hdr.tsf_hi = (uint32_t)(rt_info.tsft >> 32);
    }

    /* Channel info: prefer radiotap channel, fall back to our config */
    hdr.channel_freq = rt_info.has_channel
                     ? rt_info.chan_freq
                     : chan_cfg.channel_freq;
    hdr.channel_flags     = chan_cfg.channel_flags;
    hdr.channel_bond_freq = chan_cfg.channel_bond_freq;
    hdr.center_freq1      = chan_cfg.center_freq1;
    hdr.center_freq2      = chan_cfg.center_freq2;

    /* flags: TTL = 0, rest reserved */
    hdr.flags = 0;

    /* Send: [uint32_t len (net order)][header][frame] */
    msg_len = (uint32_t)(sizeof(hdr) + frame_len);
    len_be = htonl(msg_len);

    memcpy(sendbuf, &len_be, 4);
    memcpy(sendbuf + 4, &hdr, sizeof(hdr));
    memcpy(sendbuf + 4 + sizeof(hdr), frame, frame_len);

    if (write_all(hub_fd, sendbuf, 4 + msg_len) < 0) {
        if (errno == EPIPE || errno == ECONNRESET) {
            fprintf(stderr, "bridge: phys->hub: hub disconnected\n");
            g_running = 0;
            return;
        }
        /* Backpressure — drop frame, don't exit */
        return;
    }

    stats_phys_to_hub++;
    stats_ph_bytes += frame_len;
    if (verbose)
        fprintf(stderr,
            "bridge: phys->hub: forwarded %zu bytes "
            "(rate=0x%02x rssi=%d ch=%u)\n",
            frame_len, hdr.rate_code, hdr.rssi, hdr.channel_freq);
}

/* ================================================================
 *  Step 10: hub stream reassembly + main loop
 *
 *  The hub socket is a stream, so we need our own framing: read into
 *  a buffer, extract complete length-prefixed messages, and process
 *  each one.
 * ================================================================ */

static uint8_t hub_rxbuf[4 + VWIFI_HDR_SIZE + VWIFI_MAX_FRAME_SIZE];
static size_t  hub_rxlen;

static void handle_hub_data(void)
{
    ssize_t n;

    n = read(hub_fd, hub_rxbuf + hub_rxlen, sizeof(hub_rxbuf) - hub_rxlen);
    if (n <= 0) {
        if (n < 0 && (errno == EAGAIN || errno == EINTR))
            return;
        fprintf(stderr, "bridge: hub disconnected\n");
        g_running = 0;
        return;
    }
    hub_rxlen += (size_t)n;

    /* Process complete messages */
    while (hub_rxlen >= 4) {
        uint32_t payload_len = ntohl(*(uint32_t *)hub_rxbuf);

        /* Sanity check */
        if (payload_len > VWIFI_HDR_SIZE + VWIFI_MAX_FRAME_SIZE) {
            fprintf(stderr, "bridge: hub stream corrupt (len=%u), resetting\n",
                    payload_len);
            hub_rxlen = 0;
            break;
        }

        /* Need more data? */
        if (hub_rxlen < 4 + payload_len)
            break;

        process_hub_message(hub_rxbuf + 4, payload_len);

        /* Consume this message */
        size_t consumed = 4 + payload_len;
        hub_rxlen -= consumed;
        if (hub_rxlen > 0)
            memmove(hub_rxbuf, hub_rxbuf + consumed, hub_rxlen);
    }
}

/* ================================================================
 *  Usage / help
 * ================================================================ */

static void usage(const char *prog)
{
    fprintf(stderr,
        "Usage: %s <hub-socket-path> <interface> -c <channel> [options]\n"
        "\n"
        "  -c <channel>     Channel number (1-14, 36, 40, ...) or freq in MHz\n"
        "                   Values <= 200 are channel numbers, > 200 are MHz\n"
        "  -w <bandwidth>   Channel width (default: HT20):\n"
        "                   HT20, HT40+, HT40-, VHT80, VHT160, VHT80+80\n"
        "  -s <center2_mhz> Secondary 80MHz center freq (VHT80+80 only)\n"
        "  -n <node_id>     Node ID for hub registration (default: phys-<ifname>)\n"
        "  -b <bssid>       Relevance-filter on this BSSID instead of the\n"
        "                   interface MAC (two-radio setup: dedicated ACK radio\n"
        "                   holds MAC=BSSID, this radio does capture/inject)\n"
        "  -m <mask>        Relevance-filter address mask (1=care, 0=don't-care,\n"
        "                   default ff:ff:ff:ff:ff:ff). Forward a prefix-aligned\n"
        "                   BSSID set, e.g. -b 02:11:22:33:44:00 -m ff:ff:ff:ff:ff:00\n"
        "                   forwards 02:11:22:33:44:00..FF\n"
        "  -r <n>           Inject each critical downlink frame N times (default\n"
        "                   1; try 2-3 on a lossy/congested channel). Only\n"
        "                   management + EAPOL are duplicated; beacons and\n"
        "                   encrypted data are always injected once.\n"
        "  -R <mbps>        Legacy over-air downlink rate: 1,2,5.5,6,9,11,12,18,\n"
        "                   24,36,48,54, or 0=auto (default; echo the VM's rate).\n"
        "                   NOTE: mt76/ath9k_htc IGNORE this and always inject at\n"
        "                   ~1 Mbps -- use -M/-L on a Realtek rtl88xxau instead.\n"
        "  -M <mcs>         Inject at HT MCS <0..31> (TX_FLAGS+MCS radiotap). On a\n"
        "                   Realtek rtl88xxau this gives real rates (MCS7=65Mbps);\n"
        "                   pick a moderate MCS (e.g. 4) for robustness. Overrides -R.\n"
        "  -L               Inject rate-less (no rate field; driver picks). On a\n"
        "                   rtl88xxau w/ rtw_monitor_disable_1m=1 => HT-MCS7/VHT-MCS9.\n"
        "  -S <sec>         Print a periodic throughput report every <sec>s:\n"
        "                   per-direction fps/Mbps and inject drop counters\n"
        "                   (enobufs/eagain = silent downlink loss). SIGUSR1\n"
        "                   dumps on demand. 0 = off (default).\n"
        "  -A               Forward ALL captured frames (disable the relevance\n"
        "                   filter; only for sniffing -- floods the hub on a\n"
        "                   busy channel)\n"
        "  -v               Verbose logging\n"
        "  -h               This help\n"
        "\n"
        "Example:\n"
        "  sudo %s /tmp/vwifi.sock wlx90de801c625f -c 6 -v\n",
        prog, prog);
}

/* ================================================================
 *  main
 * ================================================================ */

int main(int argc, char *argv[])
{
    int opt;

    if (argc < 3) {
        usage(argv[0]);
        return 1;
    }

    hub_path = argv[1];
    ifname   = argv[2];

    /* Reset getopt to start scanning from argv[3] */
    optind = 3;

    while ((opt = getopt(argc, argv, "c:w:s:n:b:m:r:R:M:S:LAvh")) != -1) {
        switch (opt) {
        case 'b': {
            unsigned int m[6];
            if (sscanf(optarg, "%x:%x:%x:%x:%x:%x",
                       &m[0], &m[1], &m[2], &m[3], &m[4], &m[5]) != 6) {
                fprintf(stderr, "bridge: bad -b BSSID: %s\n", optarg);
                return 1;
            }
            for (int i = 0; i < 6; i++)
                filter_bssid[i] = (uint8_t)m[i];
            have_filter_bssid = 1;
            break;
        }
        case 'm': {
            unsigned int m[6];
            if (sscanf(optarg, "%x:%x:%x:%x:%x:%x",
                       &m[0], &m[1], &m[2], &m[3], &m[4], &m[5]) != 6) {
                fprintf(stderr, "bridge: bad -m mask: %s\n", optarg);
                return 1;
            }
            for (int i = 0; i < 6; i++)
                filter_mask[i] = (uint8_t)m[i];
            break;
        }
        case 'c':
            channel_num = atoi(optarg);
            if (channel_num <= 0) {
                fprintf(stderr, "bridge: invalid channel: %s\n", optarg);
                return 1;
            }
            break;
        case 'w':
            bw_str = optarg;
            break;
        case 's':
            center2_mhz = atoi(optarg);
            break;
        case 'n':
            snprintf(node_id, sizeof(node_id), "%s", optarg);
            break;
        case 'r':
            inject_copies = atoi(optarg);
            if (inject_copies < 1 || inject_copies > 8) {
                fprintf(stderr, "bridge: -r must be 1..8: %s\n", optarg);
                return 1;
            }
            break;
        case 'R': {
            /* Legacy rate in Mbps -> radiotap 500 kbps units. 0 = auto. */
            int rt = (int)(atof(optarg) * 2 + 0.5);
            int ok = (rt == 0);
            for (size_t i = 0; i < NUM_RATES && !ok; i++)
                if (rate_map[i].rt_rate == rt)
                    ok = 1;
            if (!ok) {
                fprintf(stderr, "bridge: -R must be a legacy rate in Mbps "
                        "(1,2,5.5,6,9,11,12,18,24,36,48,54) or 0=auto: %s\n",
                        optarg);
                return 1;
            }
            inject_rate_rt = rt;
            break;
        }
        case 'M':
            inject_mcs = atoi(optarg);
            if (inject_mcs < 0 || inject_mcs > 31) {
                fprintf(stderr, "bridge: -M must be an HT MCS 0..31: %s\n",
                        optarg);
                return 1;
            }
            break;
        case 'L':
            inject_rateless = 1;
            break;
        case 'S':
            stats_interval = atoi(optarg);
            if (stats_interval < 0) {
                fprintf(stderr, "bridge: -S must be >= 0: %s\n", optarg);
                return 1;
            }
            break;
        case 'A':
            forward_all = 1;
            break;
        case 'v':
            verbose = 1;
            break;
        case 'h':
            usage(argv[0]);
            return 0;
        default:
            usage(argv[0]);
            return 1;
        }
    }

    if (channel_num == 0) {
        fprintf(stderr, "bridge: -c <channel> is required\n");
        usage(argv[0]);
        return 1;
    }

    /* Default node ID */
    if (node_id[0] == '\0')
        snprintf(node_id, sizeof(node_id), "phys-%s", ifname);

    /* Install signal handlers */
    signal(SIGINT,  sig_handler);
    signal(SIGTERM, sig_handler);
    signal(SIGPIPE, SIG_IGN);
    signal(SIGUSR1, sigusr1_handler);   /* dump stats on demand */

    fprintf(stderr, "bridge: hub=%s iface=%s channel=%d bw=%s node=%s\n",
            hub_path, ifname, channel_num, bw_str, node_id);
    if (inject_mcs >= 0)
        fprintf(stderr, "bridge: inject HT MCS%d copies=%d\n",
                inject_mcs, inject_copies);
    else if (inject_rateless)
        fprintf(stderr, "bridge: inject rate-less (driver default) copies=%d\n",
                inject_copies);
    else if (inject_rate_rt)
        fprintf(stderr, "bridge: inject rate=%.1f Mbps (fixed legacy) copies=%d\n",
                inject_rate_rt / 2.0, inject_copies);
    else
        fprintf(stderr, "bridge: inject rate=auto (echo VM legacy rate) "
                "copies=%d\n", inject_copies);

    /* Compute channel/bandwidth configuration */
    if (compute_channel_config() < 0)
        return 1;

    if (verbose) {
        fprintf(stderr,
            "bridge: freq=%u flags=0x%04x bond=%u cf1=%u cf2=%u "
            "range=[%u-%u]\n",
            chan_cfg.channel_freq, chan_cfg.channel_flags,
            chan_cfg.channel_bond_freq, chan_cfg.center_freq1,
            chan_cfg.center_freq2, chan_cfg.freq_lo, chan_cfg.freq_hi);
    }
    /* Connect to the hub */
    hub_fd = connect_hub(hub_path);
    if (hub_fd < 0) {
        fprintf(stderr, "bridge: is vwifi-medium running at %s?\n",
                hub_path);
        return 1;
    }
    if (send_hello(hub_fd, node_id) < 0) {
        close(hub_fd);
        return 1;
    }
    /* Increase socket send buffer to handle burst captures */
    {
        int sndbuf = 1024 * 1024;
        setsockopt(hub_fd, SOL_SOCKET, SO_SNDBUF, &sndbuf, sizeof(sndbuf));
    }
    fprintf(stderr, "bridge: connected to hub %s (fd=%d, node=%s)\n",
            hub_path, hub_fd, node_id);
    /* Validate monitor mode and open raw capture/inject socket */
    if (check_monitor_mode(ifname) < 0)
        goto out;
    raw_fd = open_raw_socket(ifname);
    if (raw_fd < 0)
        goto out;
    fprintf(stderr, "bridge: %s raw socket opened (fd=%d)\n", ifname, raw_fd);
    fprintf(stderr, "bridge: running — bridging %s <-> %s\n", ifname, hub_path);

    /* Main poll loop */
    {
        struct pollfd pfds[2];
        struct timespec last_report;
        pfds[1].fd = hub_fd;   pfds[1].events = POLLIN;
        clock_gettime(CLOCK_MONOTONIC, &last_report);

        while (g_running) {
            /* Periodic (-S) or on-demand (SIGUSR1) throughput report. */
            if (stats_interval || stats_dump_now) {
                struct timespec now;
                clock_gettime(CLOCK_MONOTONIC, &now);
                double ms = (now.tv_sec - last_report.tv_sec) * 1000.0
                          + (now.tv_nsec - last_report.tv_nsec) / 1e6;
                if (stats_dump_now) {
                    stats_report(ms);
                    stats_dump_now = 0;
                    last_report = now;
                } else if (ms >= stats_interval * 1000.0) {
                    stats_report(ms);
                    last_report = now;
                }
            }

            /* A capture-interface flap (mt76 USB reset) is recoverable: reopen
             * the raw socket rather than tearing the whole bridge down. */
            if (raw_needs_reopen) {
                reopen_raw_socket();
                raw_needs_reopen = 0;
                if (!g_running)
                    break;
            }

            pfds[0].fd = raw_fd;   pfds[0].events = POLLIN;
            pfds[0].revents = 0;
            pfds[1].revents = 0;

            int nready = poll(pfds, 2, 1000);
            if (nready < 0) {
                if (errno == EINTR)
                    continue;
                perror("bridge: poll");
                break;
            }

            /* Hub -> physical */
            if (pfds[1].revents & POLLIN)
                handle_hub_data();

            /* Physical -> hub */
            if (pfds[0].revents & POLLIN)
                handle_phys_data();

            /* Capture interface error (flap): reopen on the next iteration. */
            if (pfds[0].revents & (POLLERR | POLLHUP | POLLNVAL))
                raw_needs_reopen = 1;

            /* Hub disconnect is fatal */
            if (pfds[1].revents & (POLLERR | POLLHUP)) {
                fprintf(stderr, "bridge: hub connection lost\n");
                break;
            }
        }
    }

    /* Normal shutdown (reached after main loop exits) */
    fprintf(stderr, "bridge: shutting down\n");
    stats_report(0);

out:
    if (hub_fd >= 0) close(hub_fd);
    if (raw_fd >= 0) close(raw_fd);
    return 0;
}
