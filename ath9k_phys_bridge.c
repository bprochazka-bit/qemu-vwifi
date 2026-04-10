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
 *  TODO
 * ================================================================ */

/* ================================================================
 *  Step 5: hub connection + hello
 *  TODO
 * ================================================================ */

/* ================================================================
 *  Step 6: physical interface setup
 *  TODO
 * ================================================================ */

/* ================================================================
 *  Step 7: radiotap parsing
 *  TODO
 * ================================================================ */

/* ================================================================
 *  Step 8: hub-to-physical path (process_hub_message)
 *  TODO
 * ================================================================ */

/* ================================================================
 *  Step 9: physical-to-hub path (handle_phys_data)
 *  TODO
 * ================================================================ */

/* ================================================================
 *  Step 10: hub stream reassembly + main loop
 *  TODO
 * ================================================================ */

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
    /* TODO step 5: connect_hub(), send_hello() */
    /* TODO step 6: check_monitor_mode(), open_raw_socket() */
    /* TODO step 10: main poll loop */

    /* Shutdown */
    fprintf(stderr,
        "bridge: shutting down -- hub->phys: %u frames, phys->hub: %u frames, "
        "echoes suppressed: %u\n",
        stats_hub_to_phys, stats_phys_to_hub, stats_echoes);

    if (hub_fd >= 0) close(hub_fd);
    if (raw_fd >= 0) close(raw_fd);
    return 0;
}
