/*
 * ath9k_phys_bridge — Bridge a physical WiFi interface to a virtual medium hub
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Connects to an ath9k_medium_hub as a regular peer, but instead of a
 * virtual radio it uses a real physical WiFi interface in monitor mode.
 *
 * Hub -> bridge: receives 802.11 frames, injects them over the air
 * Physical -> bridge: captures OTA frames, forwards them into the hub
 *
 * This lets a real device (e.g. a laptop) associate with a virtual AP
 * running inside a QEMU VM.
 *
 * Build:
 *   gcc -Wall -Wextra -O2 -o ath9k_phys_bridge ath9k_phys_bridge.c
 *
 * Usage:
 *   sudo ./ath9k_phys_bridge <hub-socket-path> <interface> -c <channel> [opts]
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
#include <sys/socket.h>
#include <sys/un.h>
#include <arpa/inet.h>
#include <linux/if_packet.h>
#include <linux/if_ether.h>

#include "ath9k_medium.h"

/* ================================================================
 *  Global state
 * ================================================================ */

static volatile sig_atomic_t g_running = 1;

/* File descriptors */
static int hub_fd = -1;
static int raw_fd = -1;

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

/* Statistics */
static unsigned int stats_hub_to_phys;
static unsigned int stats_phys_to_hub;
static unsigned int stats_echoes;

/* ================================================================
 *  Signal handler
 * ================================================================ */

static void sig_handler(int sig)
{
    (void)sig;
    g_running = 0;
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
    band_flag = (freq < 5000) ? ATH9K_CHAN_FLAG_2GHZ : ATH9K_CHAN_FLAG_5GHZ;

    /* Default: single 20 MHz channel */
    chan_cfg.freq_lo = freq;
    chan_cfg.freq_hi = freq;
    chan_cfg.channel_bond_freq = 0;
    chan_cfg.center_freq1 = 0;
    chan_cfg.center_freq2 = 0;

    if (strcmp(bw_str, "HT20") == 0) {
        chan_cfg.channel_flags = band_flag | ATH9K_CHAN_FLAG_HT20;

    } else if (strcmp(bw_str, "HT40+") == 0) {
        chan_cfg.channel_flags = band_flag | ATH9K_CHAN_FLAG_HT40PLUS;
        chan_cfg.channel_bond_freq = freq + 20;
        chan_cfg.freq_hi = freq + 20;

    } else if (strcmp(bw_str, "HT40-") == 0) {
        chan_cfg.channel_flags = band_flag | ATH9K_CHAN_FLAG_HT40MINUS;
        chan_cfg.channel_bond_freq = freq - 20;
        chan_cfg.freq_lo = freq - 20;

    } else if (strcmp(bw_str, "VHT80") == 0) {
        int blk = find_vht80_block(freq);
        if (blk < 0) {
            fprintf(stderr, "bridge: freq %u MHz not in any 80MHz block\n",
                    freq);
            return -1;
        }
        chan_cfg.channel_flags = band_flag | ATH9K_CHAN_FLAG_VHT80;
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
        chan_cfg.channel_flags = band_flag | ATH9K_CHAN_FLAG_VHT160;
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
        chan_cfg.channel_flags = band_flag | ATH9K_CHAN_FLAG_VHT80_80;
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

/* Radiotap uses 500 kbps units; medium protocol uses ath9k rate codes */
static const struct {
    uint8_t rt_rate;    /* radiotap 500kbps */
    uint8_t ath_code;   /* ath9k rate code */
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

/* Convert radiotap rate (500kbps units) to ath9k rate code */
static uint8_t rt_rate_to_ath(uint8_t rt)
{
    for (size_t i = 0; i < NUM_RATES; i++) {
        if (rate_map[i].rt_rate == rt)
            return rate_map[i].ath_code;
    }
    return ATH9K_MEDIUM_DEFAULT_RATE;  /* 6 Mbps OFDM */
}

/* Convert ath9k rate code to radiotap rate (500kbps units) */
static uint8_t ath_rate_to_rt(uint8_t ath)
{
    for (size_t i = 0; i < NUM_RATES; i++) {
        if (rate_map[i].ath_code == ath)
            return rate_map[i].rt_rate;
    }
    return 12;  /* 6 Mbps OFDM in 500kbps units */
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

#define HELLO_MAGIC 0x41394B52  /* "A9KR" */

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
 * Format: [uint32_t len (net order)][uint32_t HELLO_MAGIC][node_id\0]
 */
static int send_hello(int fd, const char *id)
{
    size_t id_len = strlen(id) + 1;  /* include null terminator */
    uint32_t payload_len = 4 + id_len;
    uint32_t len_be = htonl(payload_len);
    uint32_t magic = HELLO_MAGIC;

    uint8_t buf[4 + 4 + 64];
    if (4 + payload_len > sizeof(buf)) {
        fprintf(stderr, "bridge: node_id too long\n");
        return -1;
    }

    memcpy(buf, &len_be, 4);
    memcpy(buf + 4, &magic, 4);
    memcpy(buf + 8, id, id_len);

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

    return fd;
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
    uint16_t chan_flags;    /* radiotap channel flags (not ath9k flags) */
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

/* Minimal radiotap header for injection: 8 bytes, no fields */
static const uint8_t inject_radiotap[8] = {
    0x00,                   /* version */
    0x00,                   /* pad */
    0x08, 0x00,             /* length = 8 (LE) */
    0x00, 0x00, 0x00, 0x00  /* present = 0 */
};

/*
 * Process a single hub message (payload after the 4-byte length prefix).
 * Validates, filters by channel, and injects the frame.
 */
static void process_hub_message(const uint8_t *payload, uint32_t payload_len)
{
    const struct ath9k_medium_frame_hdr *hdr;
    const uint8_t *frame;
    uint16_t frame_len;
    uint16_t msg_chan_freq;
    uint8_t inject_buf[8 + ATH9K_MEDIUM_MAX_FRAME_SIZE];
    ssize_t n;

    /* Need at least a v1 header */
    if (payload_len < ATH9K_MEDIUM_HDR_SIZE_V1) {
        if (verbose)
            fprintf(stderr, "bridge: hub->phys: short message (%u bytes)\n",
                    payload_len);
        return;
    }

    hdr = (const struct ath9k_medium_frame_hdr *)payload;

    /* Validate magic */
    if (hdr->magic != ATH9K_MEDIUM_MAGIC) {
        if (verbose)
            fprintf(stderr, "bridge: hub->phys: bad magic 0x%08x\n",
                    hdr->magic);
        return;
    }

    /* Extract channel frequency (v2 has it at offset 28; v1 = treat as 0) */
    if (payload_len >= ATH9K_MEDIUM_HDR_SIZE)
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
    if (payload_len >= ATH9K_MEDIUM_HDR_SIZE)
        frame = payload + ATH9K_MEDIUM_HDR_SIZE;
    else
        frame = payload + ATH9K_MEDIUM_HDR_SIZE_V1;

    /* Sanity check frame length */
    if (frame_len == 0 || frame_len > ATH9K_MEDIUM_MAX_FRAME_SIZE) {
        if (verbose)
            fprintf(stderr, "bridge: hub->phys: bad frame_len=%u\n",
                    frame_len);
        return;
    }

    /*
     * Strip the 4-byte FCS placeholder before injection.
     * Frames on the virtual medium always include a 4-byte FCS (the QEMU
     * ath9k-virt device and kernel module both append one).  The physical
     * radio adds its own real FCS during injection, so we must remove the
     * placeholder to avoid corrupting the frame (extra 4 bytes would cause
     * CCMP MIC failure for encrypted frames).
     */
    if (frame_len > 4)
        frame_len -= 4;

    /* Record echo hash before injection (hash the frame without FCS) */
    echo_record(frame, frame_len);

    /* Build injection buffer: radiotap header + 802.11 frame */
    memcpy(inject_buf, inject_radiotap, 8);
    memcpy(inject_buf + 8, frame, frame_len);

    n = write(raw_fd, inject_buf, 8 + frame_len);
    if (n < 0) {
        if (errno != EAGAIN && errno != ENOBUFS)
            fprintf(stderr, "bridge: inject write: %s\n", strerror(errno));
        return;
    }

    stats_hub_to_phys++;
    if (verbose)
        fprintf(stderr, "bridge: hub->phys: injected %u bytes (ch=%u)\n",
                frame_len, msg_chan_freq);
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
    uint8_t buf[8 + ATH9K_MEDIUM_MAX_FRAME_SIZE + 64];
    ssize_t nread;
    struct radiotap_info rt_info;
    size_t rt_len;
    const uint8_t *frame;
    size_t frame_len;
    uint16_t fc;
    struct ath9k_medium_frame_hdr hdr;
    uint32_t msg_len;
    uint32_t len_be;
    uint8_t sendbuf[4 + sizeof(hdr) + ATH9K_MEDIUM_MAX_FRAME_SIZE + 4]; /* +4 for FCS */

    nread = read(raw_fd, buf, sizeof(buf));
    if (nread <= 0) {
        if (nread < 0 && (errno == EAGAIN || errno == EINTR))
            return;
        fprintf(stderr, "bridge: raw socket read error: %s\n",
                strerror(errno));
        g_running = 0;
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
    if (frame_len > ATH9K_MEDIUM_MAX_FRAME_SIZE)
        return;

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
    hdr.magic   = ATH9K_MEDIUM_MAGIC;
    hdr.version = ATH9K_MEDIUM_VERSION;
    hdr.frame_len = (uint16_t)frame_len;

    /* tx_mac = Address 2 (transmitter) at offset 10 */
    memcpy(hdr.tx_mac, frame + 10, 6);

    /* Rate */
    hdr.rate_code = rt_info.has_rate
                  ? rt_rate_to_ath(rt_info.rate)
                  : ATH9K_MEDIUM_DEFAULT_RATE;

    /* RSSI */
    hdr.rssi = rt_info.has_signal
             ? rt_info.signal_dbm
             : ATH9K_MEDIUM_DEFAULT_RSSI;

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

    /*
     * Append a 4-byte FCS placeholder.  Frames on the virtual medium
     * always include FCS (the QEMU ath9k-virt device and kernel module
     * both have RX_INCLUDES_FCS set, so mac80211 expects FCS at the end).
     * We already stripped the real FCS from the capture above; now add
     * 4 zero bytes as a placeholder.
     */
    size_t wire_frame_len = frame_len + 4;  /* frame + FCS placeholder */
    hdr.frame_len = (uint16_t)wire_frame_len;

    /* Send: [uint32_t len (net order)][header][frame][fcs_placeholder] */
    msg_len = (uint32_t)(sizeof(hdr) + wire_frame_len);
    len_be = htonl(msg_len);

    memcpy(sendbuf, &len_be, 4);
    memcpy(sendbuf + 4, &hdr, sizeof(hdr));
    memcpy(sendbuf + 4 + sizeof(hdr), frame, frame_len);
    memset(sendbuf + 4 + sizeof(hdr) + frame_len, 0, 4);  /* FCS placeholder */

    if (write_all(hub_fd, sendbuf, 4 + msg_len) < 0) {
        if (errno == EPIPE || errno == ECONNRESET) {
            fprintf(stderr, "bridge: phys->hub: hub disconnected\n");
            g_running = 0;
            return;
        }
        /* Non-fatal write failure (backpressure) -- drop frame */
        if (verbose)
            fprintf(stderr, "bridge: phys->hub: write failed (dropped): %s\n",
                    strerror(errno));
        return;
    }

    stats_phys_to_hub++;
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

static uint8_t hub_rxbuf[4 + ATH9K_MEDIUM_HDR_SIZE + ATH9K_MEDIUM_MAX_FRAME_SIZE];
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
        if (payload_len > ATH9K_MEDIUM_HDR_SIZE + ATH9K_MEDIUM_MAX_FRAME_SIZE) {
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
        "  -v               Verbose logging\n"
        "  -h               This help\n"
        "\n"
        "Example:\n"
        "  sudo %s /tmp/ath9k.sock wlx90de801c625f -c 6 -v\n",
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

    while ((opt = getopt(argc, argv, "c:w:s:n:vh")) != -1) {
        switch (opt) {
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

    fprintf(stderr, "bridge: hub=%s iface=%s channel=%d bw=%s node=%s\n",
            hub_path, ifname, channel_num, bw_str, node_id);

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
        fprintf(stderr, "bridge: is ath9k_medium_hub running at %s?\n",
                hub_path);
        return 1;
    }
    if (send_hello(hub_fd, node_id) < 0) {
        close(hub_fd);
        return 1;
    }
    /* Increase socket send buffer to absorb bursts from physical captures */
    {
        int sndbuf = 1024 * 1024;  /* 1 MB */
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
        pfds[0].fd = raw_fd;   pfds[0].events = POLLIN;
        pfds[1].fd = hub_fd;   pfds[1].events = POLLIN;

        while (g_running) {
            pfds[0].revents = 0;
            pfds[1].revents = 0;

            int nready = poll(pfds, 2, 1000);
            if (nready < 0) {
                if (errno == EINTR)
                    continue;
                perror("bridge: poll");
                break;
            }
            if (nready == 0)
                continue;

            /* Hub -> physical */
            if (pfds[1].revents & POLLIN)
                handle_hub_data();

            /* Physical -> hub */
            if (pfds[0].revents & POLLIN)
                handle_phys_data();

            /* Hub disconnect is fatal */
            if (pfds[1].revents & (POLLERR | POLLHUP)) {
                fprintf(stderr, "bridge: hub connection lost\n");
                break;
            }
            /* Raw socket errors are typically transient (interface flap) */
            if (pfds[0].revents & POLLERR) {
                if (verbose)
                    fprintf(stderr, "bridge: raw socket error (transient)\n");
            }
        }
    }

    /* Normal shutdown (reached after main loop exits) */
    fprintf(stderr,
        "bridge: shutting down -- hub->phys: %u frames, phys->hub: %u frames, "
        "echoes suppressed: %u\n",
        stats_hub_to_phys, stats_phys_to_hub, stats_echoes);

out:
    if (hub_fd >= 0) close(hub_fd);
    if (raw_fd >= 0) close(raw_fd);
    return 0;
}
