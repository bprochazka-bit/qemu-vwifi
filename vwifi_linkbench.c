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
                     int mcs, int noack, int verbose)
{
    int rt = 0;
    int rateless = 0;
    if (mcs >= 0) {
        if (mcs > 31) {
            fprintf(stderr, "linkbench: bad -M %d (HT MCS 0..31)\n", mcs);
            return 1;
        }
    } else if (mbps == 0) {
        /* -R 0: inject with NO rate/MCS radiotap field at all — let the driver
         * pick. On mt76/ath9k_htc that means the 1 Mbps floor; on the Realtek
         * rtl88xxau with rtw_monitor_disable_1m=1 a rate-less inject defaults
         * to HT-MCS7/VHT-MCS9, which is the whole point of this mode. */
        rateless = 1;
    } else {
        rt = mbps_to_rt(mbps);
        if (rt < 0) {
            fprintf(stderr, "linkbench: bad -R %.1f (use 0=driver-default, or "
                    "1,2,5.5,6,9,11,12,18,24,36,48,54)\n", mbps);
            return 1;
        }
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

    /* Build the radiotap header once; only the payload sequence number
     * changes per frame. Two variants:
     *   legacy — RATE (bit 2) + TX_FLAGS (bit 15), rate in 500 kbps units.
     *   HT MCS — TX_FLAGS (bit 15) + MCS (bit 19). mt76 has historically
     *            ignored the legacy RATE on monitor injection while still
     *            honoring MCS, so this is the lever that may actually raise
     *            the on-air rate above ~1 Mbps. MCS i @ 20 MHz long-GI:
     *            0→6.5, 1→13, ... 7→65 Mbps. */
    /* tx_flags low byte: NO_ACK (0x08) normally; -A clears it to test whether
     * the NO_ACK flag is what makes the driver drop to the 1 Mbps basic rate.
     * Without NO_ACK the injecting radio expects an ACK it can't hear and will
     * retransmit — capture shows that as a >1x duplication factor. */
    uint8_t txf = noack ? 0x08 : 0x00;
    uint8_t buf[16 + LB_MAX_SIZE];
    int rtlen;
    if (rateless) {
        uint8_t rtap[10] = {
            0x00, 0x00, 0x0a, 0x00,      /* version, pad, len=10 */
            0x00, 0x80, 0x00, 0x00,      /* present = TX_FLAGS(15) only */
            txf,  0x00                   /* [8] tx_flags (no rate/MCS field) */
        };
        memcpy(buf, rtap, sizeof rtap);
        rtlen = sizeof rtap;
    } else if (mcs >= 0) {
        uint8_t rtap[13] = {
            0x00, 0x00, 0x0d, 0x00,      /* version, pad, len=13 */
            0x00, 0x80, 0x08, 0x00,      /* present = TX_FLAGS(15) | MCS(19) */
            txf,  0x00,                  /* [8]  tx_flags */
            0x02, 0x00, (uint8_t)mcs     /* [10] mcs: known=idx, flags=0, index */
        };
        memcpy(buf, rtap, sizeof rtap);
        rtlen = sizeof rtap;
    } else {
        uint8_t rtap[12] = {
            0x00, 0x00, 0x0c, 0x00,      /* version, pad, len=12 */
            0x04, 0x80, 0x00, 0x00,      /* present = RATE(2) | TX_FLAGS(15) */
            (uint8_t)rt, 0x00,           /* rate, pad */
            txf,  0x00                   /* tx_flags */
        };
        memcpy(buf, rtap, sizeof rtap);
        rtlen = sizeof rtap;
    }
    uint8_t *f = buf + rtlen;             /* 802.11 frame */
    memset(f, 0, size);
    f[0] = 0x08; f[1] = 0x02;            /* data, FromDS (AP->STA) */
    /* addr1 = DA (dummy STA), addr2 = BSSID/SA, addr3 = SA */
    static const uint8_t da[6]   = { 0xde, 0xad, 0xbe, 0xef, 0x00, 0x01 };
    static const uint8_t bssid[6]= { 0x02, 0x11, 0x22, 0x33, 0x44, 0x01 };
    memcpy(f + 4,  da,    6);
    memcpy(f + 10, bssid, 6);
    memcpy(f + 16, bssid, 6);
    memcpy(f + LB_PAYLOAD_OFF, LB_MAGIC, 4);
    int total = rtlen + size;

    signal(SIGINT, on_sig);
    signal(SIGTERM, on_sig);

    char ratestr[40];
    if (rateless)      snprintf(ratestr, sizeof ratestr, "rate-less (driver default)");
    else if (mcs >= 0) snprintf(ratestr, sizeof ratestr, "MCS%d (HT)", mcs);
    else               snprintf(ratestr, sizeof ratestr, "%.1f Mbps", mbps);
    fprintf(stderr, "linkbench: inject %s: rate=%s size=%dB dur=%.0fs "
            "%s%s%s%s%s\n", iface, ratestr, size, secs,
            pps ? "paced " : "max-rate ", nonblock ? "nonblock" : "blocking",
            qdisc_bypass ? " qdisc-bypass" : "",
            sndbuf ? " sndbuf-set" : "",
            noack ? " NO_ACK" : " ACK-expected");
    if (pps) fprintf(stderr, "linkbench: target %ld pps\n", pps);

    uint64_t offered = 0, ok = 0, enobufs = 0, eagain = 0, err = 0;
    /* Sequence numbers stay monotonic ACROSS a -s sweep (do_inject is called
     * once per size). Resetting per call would make a single capture see
     * seq restart at 0 and mis-compute loss. */
    static uint32_t seq = 0;
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

/* Decode the on-air rate of a captured frame from its RX radiotap. Fills:
 *   legacy_500k : legacy RATE in 500 kbps units, or -1 if absent
 *   ht_mcs      : HT MCS index (0-31), or -1
 *   vht_mcs     : VHT MCS index (0-9), or -1
 * A frame carries exactly one of these. This tells us the rate the frame
 * *actually* went out at — e.g. whether a Realtek rate-less inject came out
 * as HT-MCS7/VHT-MCS9 or fell back to 1 Mbps like mt76.
 *
 * Walks the present bitmap honoring each field's radiotap size/alignment
 * (alignment is measured from the start of the radiotap header). */
struct rt_rate { int legacy_500k, ht_mcs, vht_mcs; };

static void rt_parse(const uint8_t *buf, int len, struct rt_rate *out)
{
    out->legacy_500k = out->ht_mcs = out->vht_mcs = -1;
    if (len < 8) return;
    int rtlen = buf[2] | (buf[3] << 8);
    if (rtlen < 8 || rtlen > len) return;

    uint32_t present;
    memcpy(&present, buf + 4, 4);

    /* Field data starts after all chained extended-present words (bit 31). */
    int off = 8;
    for (uint32_t p = present; p & (1u << 31); ) {
        if (off + 4 > rtlen) return;
        memcpy(&p, buf + off, 4);
        off += 4;
    }

    /* {size, align} for radiotap bits 0..21 (the ones before/at VHT). */
    static const struct { uint8_t size, align; } F[] = {
        {8,8},{1,1},{1,1},{4,2},{2,2},{1,1},{1,1},{2,2},   /* 0..7  */
        {2,2},{2,2},{1,1},{1,1},{1,1},{1,1},{2,2},{2,2},   /* 8..15 */
        {1,1},{1,1},{8,4},{3,1},{8,4},{12,2}               /* 16..21 */
    };
    int cur = off;                                  /* absolute, vs buf[0] */
    for (int bit = 0; bit <= 21; bit++) {
        if (!(present & (1u << bit))) continue;
        cur = (cur + F[bit].align - 1) & ~(F[bit].align - 1);
        if (cur + F[bit].size > rtlen) return;      /* truncated */
        if (bit == 2)  out->legacy_500k = buf[cur];
        if (bit == 19) out->ht_mcs = buf[cur + 2];  /* known, flags, index */
        if (bit == 21) out->vht_mcs = (buf[cur + 4] >> 4) & 0x0f; /* user0 MCS */
        cur += F[bit].size;
    }
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
    uint64_t rate_hist[256] = { 0 };   /* VWLB frames by legacy rate (500k) */
    uint64_t ht_hist[32] = { 0 };      /* VWLB frames by HT MCS index */
    uint64_t vht_hist[16] = { 0 };     /* VWLB frames by VHT MCS index */
    uint64_t rate_unknown = 0;         /* VWLB frames with no decodable rate */
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
            struct rt_rate rr;
            rt_parse(buf, (int)n, &rr);
            if (rr.legacy_500k >= 0)   rate_hist[rr.legacy_500k & 0xff]++;
            else if (rr.vht_mcs >= 0)  vht_hist[rr.vht_mcs & 0x0f]++;
            else if (rr.ht_mcs >= 0)   ht_hist[rr.ht_mcs & 0x1f]++;
            else                       rate_unknown++;
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
        /* bench > span means each frame was received more than once: the
         * injecting radio retransmitted it (NO_ACK not honored) — report the
         * duplication factor. Otherwise report air loss. Clamp either way so
         * the unsigned subtraction never underflows. */
        fprintf(stderr,
        "  seq span    : %llu..%llu (%llu unique sent in window)\n",
        (unsigned long long)seq_min, (unsigned long long)seq_max,
        (unsigned long long)span);
        if (bench > span)
            fprintf(stderr,
        "  duplication : %.1fx  (received %llu for %llu unique — injector is\n"
        "                retransmitting; radio not honoring NO_ACK)\n",
            (double)bench / (double)span,
            (unsigned long long)bench, (unsigned long long)span);
        else
            fprintf(stderr,
        "  air loss    : %.1f%%  (received %llu of %llu)\n",
            100.0 * (span - bench) / span,
            (unsigned long long)bench, (unsigned long long)span);

        /* On-air rate the injected frames ACTUALLY went out at — settles
         * whether the radio honored the requested rate or fell back to a basic
         * rate. HT MCS0-7 @20MHz LGI = 6.5..65 Mbps; VHT-MCS9 @20MHz = 86.7. */
        fprintf(stderr, "  on-air rate :");
        int shown = 0;
        for (int r = 0; r < 256; r++)
            if (rate_hist[r]) {
                fprintf(stderr, " %g Mbps=%llu", r / 2.0,
                        (unsigned long long)rate_hist[r]);
                shown++;
            }
        for (int m = 0; m < 32; m++)
            if (ht_hist[m]) {
                fprintf(stderr, " HT-MCS%d=%llu", m,
                        (unsigned long long)ht_hist[m]);
                shown++;
            }
        for (int m = 0; m < 16; m++)
            if (vht_hist[m]) {
                fprintf(stderr, " VHT-MCS%d=%llu", m,
                        (unsigned long long)vht_hist[m]);
                shown++;
            }
        if (rate_unknown)
            fprintf(stderr, " unknown(no-rate-field)=%llu",
                    (unsigned long long)rate_unknown);
        if (!shown && !rate_unknown)
            fprintf(stderr, " (radiotap had no rate field)");
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
        "  %s inject  <iface> [-R mbps | -M mcs] [-s bytes[,bytes...]] [-t secs]\n"
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
        "  -R <mbps>  legacy inject rate: 1,2,5.5,6,9,11,12,18,24,36,48,54 (def 6),\n"
        "             or 0 = rate-less (no rate field; let the driver pick — a\n"
        "             Realtek rtl88xxau with rtw_monitor_disable_1m=1 then uses\n"
        "             HT-MCS7/VHT-MCS9 instead of the 1 Mbps floor).\n"
        "  -M <mcs>   HT MCS inject (0-31) instead of a legacy rate. mt76 may\n"
        "             honor MCS injection even when it ignores -R; MCS0=6.5..\n"
        "             MCS7=65 Mbps @20MHz. capture decodes the on-air MCS.\n"
        "  -s <bytes> 802.11 frame size incl header (default 1500, max %d).\n"
        "             A comma list (e.g. 200,700,1500) sweeps each size in turn:\n"
        "             flat fps across sizes = fps-bound; rising fps = byte-bound.\n"
        "  -t <secs>  duration, per size (default 5)\n"
        "  -p <pps>   inject: pace to this many frames/s (default 0 = max rate)\n"
        "  -N         inject: non-blocking socket (surface ENOBUFS at max offer\n"
        "             instead of letting write() apply backpressure)\n"
        "  -A         inject: DON'T set NO_ACK (expects an ACK, will retransmit)\n"
        "             — tests whether NO_ACK is what forces the 1 Mbps basic rate;\n"
        "             capture shows a >1x duplication factor from the retransmits\n"
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
    int nonblock = 0, verbose = 0, qdisc_bypass = 0, sndbuf = 0, mcs = -1;
    int noack = 1;
    long pps = 0;
    const char *size_list = "1500";   /* -s accepts a comma list to sweep */

    for (int i = 3; i < argc; i++) {
        if (!strcmp(argv[i], "-R") && i + 1 < argc)      mbps = atof(argv[++i]);
        else if (!strcmp(argv[i], "-M") && i + 1 < argc) mcs  = atoi(argv[++i]);
        else if (!strcmp(argv[i], "-s") && i + 1 < argc) size_list = argv[++i];
        else if (!strcmp(argv[i], "-t") && i + 1 < argc) secs = atof(argv[++i]);
        else if (!strcmp(argv[i], "-p") && i + 1 < argc) pps  = atol(argv[++i]);
        else if (!strcmp(argv[i], "-B") && i + 1 < argc) sndbuf = atoi(argv[++i]);
        else if (!strcmp(argv[i], "-N"))                 nonblock = 1;
        else if (!strcmp(argv[i], "-Q"))                 qdisc_bypass = 1;
        else if (!strcmp(argv[i], "-A"))                 noack = 0;
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
                            qdisc_bypass, sndbuf, mcs, noack, verbose);
        }
        return rc;
    }
    if (!strcmp(mode, "capture"))
        return do_capture(iface, secs, verbose);

    fprintf(stderr, "linkbench: mode must be 'inject' or 'capture'\n");
    usage(argv[0]);
    return 1;
}
