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
 *  Step 2: channel_to_freq(), compute_channel_config()
 *  TODO
 * ================================================================ */

/* ================================================================
 *  Step 3: rate mapping helpers
 *  TODO
 * ================================================================ */

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

    /* TODO step 2: compute_channel_config() */
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
