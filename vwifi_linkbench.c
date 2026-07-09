// SPDX-License-Identifier: GPL-2.0
/*
 * vwifi-linkbench — an "iperf" for the vwifi-phys-bridge PHY relay.
 *
 * The phys-bridge moves frames between the vwifi medium and real air on a
 * monitor-mode radio: it injects downlink frames (write() to an AF_PACKET
 * raw socket) and captures uplink frames (recv() from the same). When
 * end-to-end throughput is low it is hard to tell *which* leg is the
 * bottleneck — the air rate, the driver's monitor TX queue, upper-layer
 * TCP, or air loss. This tool measures the two legs in isolation, with no
 * hub, no VM, no TCP, no association in the path:
 *
 *   inject  — blast well-formed 802.11 frames out a monitor iface as fast as
 *             the driver accepts them (or at a target pps), and report the
 *             sustained fps / Mbps and how many writes the driver dropped
 *             (ENOBUFS = monitor TX queue full). This is the raw ceiling of
 *             the downlink inject path.
 *
 *   capture — count frames arriving on a monitor iface and report fps / Mbps.
 *             Frames this tool injects carry a magic + sequence number, so a
 *             capture on a *second* radio also reports air loss and reordering
 *             versus what the injector accepted.
 *
 * Typical uses
 *   # 1) Inject ceiling of the bridge's own radio (tear the lab down first):
 *   sudo ./vwifi-linkbench inject wlx00c0cab57e6f -R 6 -s 1500 -t 10
 *
 *   # 2) Full air path + loss, two monitor radios on the same channel:
 *   sudo ./vwifi-linkbench capture wlxAAAA -t 12 &      # sink on radio A
 *   sudo ./vwifi-linkbench inject  wlxBBBB -R 6 -t 10   # source on radio B
 *
 *   # 3) Does raising the rate lift the ceiling? sweep -R and watch accepted Mbps.
 *
 * It shares the injection frame format (radiotap RATE + NO_ACK) with
 * vwifi-phys-bridge so the numbers are directly comparable.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <signal.h>
#include <stdint.h>
#include <time.h>
#include <fcntl.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <linux/if_packet.h>
#include <linux/if_ether.h>
#include <net/if_arp.h>

/* Payload signature so a capture can pick our frames out of ambient air
 * traffic and compute loss/reorder. Layout at the start of the 802.11 data
 * payload: magic[4] "VWLB", then u32 little-endian sequence number. */
static const uint8_t LB_MAGIC[4] = { 'V', 'W', 'L', 'B' };
#define LB_HDR_802_11   24          /* data frame: FC..seqctl, no QoS/HT */
#define LB_PAYLOAD_OFF  LB_HDR_802_11
#define LB_MIN_SIZE     (LB_HDR_802_11 + 8)   /* header + magic + seq */
#define LB_MAX_SIZE     2304

static volatile sig_atomic_t g_stop;
static void on_sig(int s) { (void)s; g_stop = 1; }

/* radiotap rate (500 kbps units) for common legacy rates */
static int mbps_to_rt(double mbps)
{
    int rt = (int)(mbps * 2 + 0.5);
    switch (rt) {
    case 2: case 4: case 11: case 22:            /* 1/2/5.5/11 CCK   */
    case 12: case 18: case 24: case 36:          /* 6/9/12/18 OFDM   */
    case 48: case 72: case 96: case 108:         /* 24/36/48/54 OFDM */
        return rt;
    default:
        return -1;
    }
}

static double now_ms(void)
{
    struct timespec t;
    clock_gettime(CLOCK_MONOTONIC, &t);
    return t.tv_sec * 1000.0 + t.tv_nsec / 1e6;
}

/* Open an AF_PACKET raw socket bound to a monitor-mode interface. */
static int open_mon(const char *iface, int for_tx)
{
    int fd, type = -1;
    unsigned int ifindex;
    struct sockaddr_ll sll;
    char path[128];
    FILE *fp;

    /* Confirm monitor mode (ARPHRD_IEEE80211_RADIOTAP). */
    snprintf(path, sizeof(path), "/sys/class/net/%s/type", iface);
    if ((fp = fopen(path, "r"))) {
        if (fscanf(fp, "%d", &type) != 1) type = -1;
        fclose(fp);
    }
    if (type != ARPHRD_IEEE80211_RADIOTAP) {
        fprintf(stderr,
            "linkbench: %s is not in monitor mode (type=%d, want %d)\n"
            "  sudo ip link set %s down && sudo iw dev %s set type monitor && "
            "sudo ip link set %s up\n",
            iface, type, ARPHRD_IEEE80211_RADIOTAP, iface, iface, iface);
        return -1;
    }

    ifindex = if_nametoindex(iface);
    if (!ifindex) {
        fprintf(stderr, "linkbench: %s not found: %s\n", iface, strerror(errno));
        return -1;
    }
    fd = socket(AF_PACKET, SOCK_RAW, htons(ETH_P_ALL));
    if (fd < 0) { perror("linkbench: socket"); return -1; }

    memset(&sll, 0, sizeof(sll));
    sll.sll_family  = AF_PACKET;
    sll.sll_protocol = htons(ETH_P_ALL);
    sll.sll_ifindex = (int)ifindex;
    if (bind(fd, (struct sockaddr *)&sll, sizeof(sll)) < 0) {
        perror("linkbench: bind"); close(fd); return -1;
    }

    if (for_tx) {
        /* Raise MTU so full frames inject without EMSGSIZE. Best effort. */
        static const int tries[] = { LB_MAX_SIZE, 2048, 1800, 1600 };
        for (size_t i = 0; i < sizeof(tries)/sizeof(tries[0]); i++) {
            struct ifreq ifr;
            memset(&ifr, 0, sizeof(ifr));
            strncpy(ifr.ifr_name, iface, IFNAMSIZ - 1);
            ifr.ifr_mtu = tries[i];
            if (ioctl(fd, SIOCSIFMTU, &ifr) == 0) break;
        }
    }
    return fd;
}

/* ---- inject mode ------------------------------------------------------- */

static int do_inject(const char *iface, double mbps, int size, double secs,
                     long pps, int nonblock, int qdisc_bypass, int sndbuf,
                     int verbose)
{
    int rt = mbps_to_rt(mbps);
    if (rt < 0) {
        fprintf(stderr, "linkbench: bad -R %.1f (use 1,2,5.5,6,9,11,12,18,24,"
                "36,48,54)\n", mbps);
        return 1;
    }
    if (size < LB_MIN_SIZE) size = LB_MIN_SIZE;
    if (size > LB_MAX_SIZE) size = LB_MAX_SIZE;

    int fd = open_mon(iface, 1);
    if (fd < 0) return 1;
    if (nonblock)
        fcntl(fd, F_SETFL, fcntl(fd, F_GETFL, 0) | O_NONBLOCK);

    /* -Q: hand frames straight to the driver's ndo_start_xmit, skipping the
     * netdev qdisc. If the ~10 ms/frame injection pacing lives in the qdisc
     * layer (fq_codel/AQL waking per timer tick), this bypasses it. */
#ifndef PACKET_QDISC_BYPASS
#define PACKET_QDISC_BYPASS 20
#endif
    if (qdisc_bypass) {
        int one = 1;
        if (setsockopt(fd, SOL_PACKET, PACKET_QDISC_BYPASS,
                       &one, sizeof(one)) < 0)
            perror("linkbench: PACKET_QDISC_BYPASS (continuing without)");
    }
    /* -B: enlarge the socket send buffer so more frames can queue while the
     * driver drains TX completions — tests whether the block is TX-status
     * batching rather than a hard per-frame rate. */
    if (sndbuf > 0 &&
        setsockopt(fd, SOL_SOCKET, SO_SNDBUF, &sndbuf, sizeof(sndbuf)) < 0)
        perror("linkbench: SO_SNDBUF (continuing)");

    /* Build radiotap (RATE + NO_ACK) + 802.11 data header once; only the
     * sequence number in the payload changes per frame. */
    uint8_t buf[12 + LB_MAX_SIZE];
    uint8_t rtap[12] = {
        0x00, 0x00, 0x0c, 0x00,          /* version, pad, len=12 */
        0x04, 0x80, 0x00, 0x00,          /* present = RATE(2) | TX_FLAGS(15) */
        (uint8_t)rt, 0x00,               /* rate, pad */
        0x08, 0x00                       /* tx_flags = NO_ACK */
    };
    memcpy(buf, rtap, 12);
    uint8_t *f = buf + 12;               /* 802.11 frame */
    memset(f, 0, size);
    f[0] = 0x08; f[1] = 0x02;            /* data, FromDS (AP->STA) */
    /* addr1 = DA (dummy STA), addr2 = BSSID/SA, addr3 = SA */
    static const uint8_t da[6]   = { 0xde, 0xad, 0xbe, 0xef, 0x00, 0x01 };
    static const uint8_t bssid[6]= { 0x02, 0x11, 0x22, 0x33, 0x44, 0x01 };
    memcpy(f + 4,  da,    6);
    memcpy(f + 10, bssid, 6);
    memcpy(f + 16, bssid, 6);
    memcpy(f + LB_PAYLOAD_OFF, LB_MAGIC, 4);
    int total = 12 + size;

    signal(SIGINT, on_sig);
    signal(SIGTERM, on_sig);

    fprintf(stderr, "linkbench: inject %s: rate=%.1f Mbps size=%dB dur=%.0fs "
            "%s%s%s%s\n", iface, mbps, size, secs,
            pps ? "paced " : "max-rate ", nonblock ? "nonblock" : "blocking",
            qdisc_bypass ? " qdisc-bypass" : "",
            sndbuf ? " sndbuf-set" : "");
    if (pps) fprintf(stderr, "linkbench: target %ld pps\n", pps);

    uint64_t offered = 0, ok = 0, enobufs = 0, eagain = 0, err = 0;
    uint32_t seq = 0;
    double t0 = now_ms(), tend = t0 + secs * 1000.0, tnext = t0;
    double interval_ms = pps ? 1000.0 / (double)pps : 0.0;
    double last_log = t0;
    uint64_t last_ok = 0;

    while (!g_stop && now_ms() < tend) {
        if (interval_ms) {
            double t = now_ms();
            if (t < tnext) {
                struct timespec ts = { 0, (long)((tnext - t) * 1e6) };
                if (ts.tv_nsec > 0) nanosleep(&ts, NULL);
            }
            tnext += interval_ms;
        }
        memcpy(f + LB_PAYLOAD_OFF + 4, &seq, 4);   /* LE seq */
        offered++;
        ssize_t n = write(fd, buf, total);
        if (n < 0) {
            if (errno == ENOBUFS)      enobufs++;
            else if (errno == EAGAIN)  eagain++;
            else { err++; if (err <= 3) perror("linkbench: write"); }
        } else {
            ok++; seq++;
        }
        /* 1 Hz progress line */
        double t = now_ms();
        if (verbose && t - last_log >= 1000.0) {
            fprintf(stderr, "  t=%.0fs ok=%llu (%.0f fps) drop(enobufs=%llu "
                    "eagain=%llu)\n", (t - t0) / 1000.0,
                    (unsigned long long)ok, (ok - last_ok) * 1000.0 / (t - last_log),
                    (unsigned long long)enobufs, (unsigned long long)eagain);
            last_log = t; last_ok = ok;
        }
    }
    double dur = (now_ms() - t0) / 1000.0;
    close(fd);

    double fps = ok / dur;
    double mbps_air = ok * (double)size * 8.0 / (dur * 1e6);
    fprintf(stderr,
        "\nlinkbench: INJECT RESULT (%.2fs)\n"
        "  offered   : %llu frames\n"
        "  accepted  : %llu frames  (%.0f fps, %.2f Mbps on-air @ %dB)\n"
        "  dropped   : enobufs=%llu eagain=%llu err=%llu  (%.1f%% of offered)\n",
        dur,
        (unsigned long long)offered,
        (unsigned long long)ok, fps, mbps_air, size,
        (unsigned long long)enobufs, (unsigned long long)eagain,
        (unsigned long long)err,
        offered ? 100.0 * (offered - ok) / offered : 0.0);
    /* A real over-the-air TX tops out in the low thousands of fps even for
     * tiny frames; sustaining tens of thousands with a blocking socket means
     * write() never hit the hardware — the frames are being discarded at the
     * driver because the interface is not actually on a channel / not really
     * transmitting. This is the classic "forgot to set the channel" setup bug. */
    if (fps > 50000.0)
        fprintf(stderr,
        "  WARNING: %.0f fps / %.0f Mbps is impossible over the air — these\n"
        "        frames are NOT reaching the radio. The interface is up but not\n"
        "        transmitting (no channel set, or not really in monitor mode).\n"
        "        Fix it first:  sudo ./scripts/mon-setup.sh %s <channel>\n",
        fps, mbps_air, iface);
    else if (enobufs > ok / 20)
        fprintf(stderr,
        "  NOTE: heavy ENOBUFS => the monitor TX queue is the ceiling; the\n"
        "        driver is dropping injects, not the air. This is the silent\n"
        "        downlink loss the bridge sees under load.\n");
    else
        fprintf(stderr,
        "  (sustained %.0f fps = %.1f ms/frame; if far below the air rate,\n"
        "        the driver's monitor TX-completion rate is the ceiling.)\n",
        fps, fps > 0 ? 1000.0 / fps : 0.0);
    return 0;
}

/* ---- capture mode ------------------------------------------------------ */

/* Find our magic in a captured buffer (radiotap + 802.11). Returns 1 and
 * sets *seq if found. We scan a small window rather than parse every
 * radiotap/header variant — robust and cheap for a bench. */
static int find_magic(const uint8_t *buf, int len, uint32_t *seq)
{
    if (len < 12) return 0;
    int rtlen = buf[2] | (buf[3] << 8);          /* radiotap total length */
    if (rtlen < 8 || rtlen > len) return 0;
    const uint8_t *p = buf + rtlen;
    int rem = len - rtlen;
    /* magic sits at payload offset (24) but headers can carry QoS/HT; scan a
     * bounded window from the 802.11 header start. */
    int maxoff = rem - 8;
    if (maxoff > 40) maxoff = 40;
    for (int off = 20; off <= maxoff; off++) {
        if (memcmp(p + off, LB_MAGIC, 4) == 0) {
            memcpy(seq, p + off + 4, 4);
            return 1;
        }
    }
    return 0;
}

/* Extract the legacy on-air rate (radiotap RATE, 500 kbps units) from a
 * captured frame, or -1 if absent (HT/VHT/HE frames carry MCS instead of a
 * legacy rate). Walks the present bitmap and the fields preceding bit 2
 * (RATE), honoring radiotap alignment. This tells us the rate the frame
 * *actually* went out at — i.e. whether the injected -R was honored. */
static int rt_rate_500k(const uint8_t *buf, int len)
{
    if (len < 8) return -1;
    int rtlen = buf[2] | (buf[3] << 8);
    if (rtlen < 8 || rtlen > len) return -1;

    uint32_t present;
    memcpy(&present, buf + 4, 4);

    /* Skip any chained extended present words (bit 31). Field data starts
     * after the last present word. */
    int off = 8;
    for (uint32_t p = present; p & (1u << 31); ) {
        if (off + 4 > rtlen) return -1;
        memcpy(&p, buf + off, 4);
        off += 4;
    }

    int cur = off;                                  /* alignment is vs buf[0] */
    if (present & (1u << 0)) { cur = (cur + 7) & ~7; cur += 8; }  /* TSFT   */
    if (present & (1u << 1)) { cur += 1; }                        /* Flags  */
    if (present & (1u << 2)) {                                    /* Rate   */
        if (cur + 1 > rtlen) return -1;
        return buf[cur];
    }
    return -1;   /* no legacy RATE field present */
}

static int do_capture(const char *iface, double secs, int verbose)
{
    int fd = open_mon(iface, 0);
    if (fd < 0) return 1;
    signal(SIGINT, on_sig);
    signal(SIGTERM, on_sig);

    fprintf(stderr, "linkbench: capture %s: dur=%.0fs (counting VWLB frames + "
            "all frames)\n", iface, secs);

    uint8_t buf[4096];
    uint64_t all = 0, all_bytes = 0, bench = 0, bench_bytes = 0;
    int have_seq = 0;
    uint32_t seq_min = 0, seq_max = 0;
    uint64_t rate_hist[256] = { 0 };   /* VWLB frames by on-air rate (500k) */
    uint64_t rate_ht = 0;              /* VWLB frames with no legacy rate (HT/VHT) */
    double t0 = now_ms(), tend = t0 + secs * 1000.0;
    double last_log = t0;
    uint64_t last_bench = 0;

    /* Non-blocking-ish: use a short recv timeout so we can honor the deadline. */
    struct timeval tv = { 0, 200000 };
    setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    while (!g_stop && now_ms() < tend) {
        ssize_t n = recv(fd, buf, sizeof(buf), 0);
        if (n < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR)
                continue;
            perror("linkbench: recv");
            break;
        }
        all++; all_bytes += n;
        uint32_t seq;
        if (find_magic(buf, (int)n, &seq)) {
            bench++; bench_bytes += n;
            if (!have_seq) { seq_min = seq_max = seq; have_seq = 1; }
            else { if (seq < seq_min) seq_min = seq; if (seq > seq_max) seq_max = seq; }
            int r = rt_rate_500k(buf, (int)n);
            if (r >= 0) rate_hist[r & 0xff]++; else rate_ht++;
        }
        double t = now_ms();
        if (verbose && t - last_log >= 1000.0) {
            fprintf(stderr, "  t=%.0fs bench=%llu (%.0f fps) all=%llu\n",
                    (t - t0) / 1000.0, (unsigned long long)bench,
                    (bench - last_bench) * 1000.0 / (t - last_log),
                    (unsigned long long)all);
            last_log = t; last_bench = bench;
        }
    }
    double dur = (now_ms() - t0) / 1000.0;
    close(fd);

    fprintf(stderr,
        "\nlinkbench: CAPTURE RESULT (%.2fs)\n"
        "  all frames  : %llu  (%.0f fps, %.2f Mbps captured)\n"
        "  VWLB frames : %llu  (%.0f fps, %.2f Mbps)\n",
        dur,
        (unsigned long long)all, all / dur, all_bytes * 8.0 / (dur * 1e6),
        (unsigned long long)bench, bench / dur, bench_bytes * 8.0 / (dur * 1e6));
    if (have_seq) {
        uint64_t span = (uint64_t)(seq_max - seq_min) + 1;
        double loss = span ? 100.0 * (span - bench) / span : 0.0;
        fprintf(stderr,
        "  seq span    : %llu..%llu (%llu sent by injector in window)\n"
        "  air loss    : %.1f%%  (received %llu of %llu)\n",
        (unsigned long long)seq_min, (unsigned long long)seq_max,
        (unsigned long long)span, loss,
        (unsigned long long)bench, (unsigned long long)span);

        /* On-air rate the injected frames ACTUALLY went out at — settles
         * whether the radio honored the injected -R or fell back to a basic
         * rate. */
        fprintf(stderr, "  on-air rate :");
        int shown = 0;
        for (int r = 0; r < 256; r++)
            if (rate_hist[r]) {
                fprintf(stderr, " %g Mbps=%llu", r / 2.0,
                        (unsigned long long)rate_hist[r]);
                shown++;
            }
        if (rate_ht) fprintf(stderr, " HT/VHT(no-legacy-rate)=%llu",
                             (unsigned long long)rate_ht);
        if (!shown && !rate_ht) fprintf(stderr, " (radiotap had no rate field)");
        fprintf(stderr, "\n");
    } else {
        fprintf(stderr, "  (no VWLB frames seen — is an injector running on "
                "the same channel?)\n");
    }
    return 0;
}

static void usage(const char *p)
{
    fprintf(stderr,
        "Usage:\n"
        "  %s inject  <iface> [-R mbps] [-s bytes[,bytes...]] [-t secs]\n"
        "             [-p pps] [-N] [-Q] [-B bytes] [-v]\n"
        "  %s capture <iface> [-t secs] [-v]\n"
        "\n"
        "inject  — blast VWLB frames out a monitor iface; report accepted\n"
        "          fps/Mbps and driver drops (ENOBUFS = TX queue is the cap).\n"
        "capture — count frames on a monitor iface; with an injector on a\n"
        "          second radio, also report air loss from the VWLB seq stream.\n"
        "\n"
        "Set the interface up FIRST (monitor mode + channel), both radios on the\n"
        "same channel:  sudo ./scripts/mon-setup.sh <iface> <channel>\n"
        "\n"
        "  -R <mbps>  inject rate: 1,2,5.5,6,9,11,12,18,24,36,48,54 (default 6)\n"
        "  -s <bytes> 802.11 frame size incl header (default 1500, max %d).\n"
        "             A comma list (e.g. 200,700,1500) sweeps each size in turn:\n"
        "             flat fps across sizes = fps-bound; rising fps = byte-bound.\n"
        "  -t <secs>  duration, per size (default 5)\n"
        "  -p <pps>   inject: pace to this many frames/s (default 0 = max rate)\n"
        "  -N         inject: non-blocking socket (surface ENOBUFS at max offer\n"
        "             instead of letting write() apply backpressure)\n"
        "  -Q         inject: PACKET_QDISC_BYPASS (skip the netdev qdisc — test\n"
        "             whether the per-frame pacing lives there)\n"
        "  -B <bytes> inject: SO_SNDBUF size (test TX-completion batching)\n"
        "  -v         per-second progress\n",
        p, p, LB_MAX_SIZE);
}

int main(int argc, char *argv[])
{
    if (argc < 3) { usage(argv[0]); return 1; }
    const char *mode  = argv[1];
    const char *iface = argv[2];

    double mbps = 6.0, secs = 5.0;
    int nonblock = 0, verbose = 0, qdisc_bypass = 0, sndbuf = 0;
    long pps = 0;
    const char *size_list = "1500";   /* -s accepts a comma list to sweep */

    for (int i = 3; i < argc; i++) {
        if (!strcmp(argv[i], "-R") && i + 1 < argc)      mbps = atof(argv[++i]);
        else if (!strcmp(argv[i], "-s") && i + 1 < argc) size_list = argv[++i];
        else if (!strcmp(argv[i], "-t") && i + 1 < argc) secs = atof(argv[++i]);
        else if (!strcmp(argv[i], "-p") && i + 1 < argc) pps  = atol(argv[++i]);
        else if (!strcmp(argv[i], "-B") && i + 1 < argc) sndbuf = atoi(argv[++i]);
        else if (!strcmp(argv[i], "-N"))                 nonblock = 1;
        else if (!strcmp(argv[i], "-Q"))                 qdisc_bypass = 1;
        else if (!strcmp(argv[i], "-v"))                 verbose = 1;
        else { fprintf(stderr, "linkbench: unknown arg: %s\n", argv[i]);
               usage(argv[0]); return 1; }
    }
    if (secs <= 0) secs = 5.0;

    if (!strcmp(mode, "inject")) {
        /* -s may be a comma list (e.g. 200,700,1500): run each in turn so a
         * single command shows whether the ceiling is fps-bound (fps ~flat
         * across sizes) or byte-bound (fps rises as size falls). */
        int rc = 0;
        char list[256];
        snprintf(list, sizeof(list), "%s", size_list);
        for (char *tok = strtok(list, ","); tok; tok = strtok(NULL, ",")) {
            int size = atoi(tok);
            rc |= do_inject(iface, mbps, size, secs, pps, nonblock,
                            qdisc_bypass, sndbuf, verbose);
        }
        return rc;
    }
    if (!strcmp(mode, "capture"))
        return do_capture(iface, secs, verbose);

    fprintf(stderr, "linkbench: mode must be 'inject' or 'capture'\n");
    usage(argv[0]);
    return 1;
}
