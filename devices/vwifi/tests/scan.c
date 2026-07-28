/*
 * vwifi-virt — scan state machine test
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Drives a real multi-channel scan against the mock backend:
 *
 *  1. Start a scan over channels 1, 6, 11 with a 100 ms dwell.
 *  2. Verify the device tuned to channel 1 and injected a probe request.
 *  3. Feed a beacon for "vwifi-test" on channel 1; verify a BSS_FOUND
 *     event is emitted with the right BSSID, SSID, RSSI and raw IEs.
 *  4. Feed the same beacon again; verify NO duplicate event (dedup).
 *  5. Advance the timer; verify hop to channel 6, another probe request.
 *  6. Feed a beacon for a second AP on channel 6; verify its BSS_FOUND.
 *  7. Advance through channel 11 and off the end; verify SCAN_COMPLETE
 *     fires and the pre-scan channel is restored.
 *  8. Verify a second scan re-reports both BSSes (per-scan dedup reset).
 *
 * Build:
 *   cc -Wall -Wextra -O2 -I../src -I. \
 *      ../src/vwifi_device.c mock_backend.c scan.c -o scan
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <arpa/inet.h>

#include "vwifi_abi.h"
#include "vwifi_device.h"
#include "mock_backend.h"
#include "vwifi.h"

#define CTRL_REQ_OFFSET     0x00000
#define CTRL_RSP_OFFSET     0x01000
#define TX_OFFSET           0x02000
#define RX_OFFSET           0x03000
#define REQ_PAYLOAD_OFFSET  0x10000
#define RSP_PAYLOAD_OFFSET  0x12000   /* per-slot, 1 KiB apart */
#define RSP_PAYLOAD_STRIDE  1024
#define RX_BUFFER_OFFSET    0x40000
#define RING_ENTRIES        16
#define RX_BUFSZ            4096

static uint64_t g_ram;
static uint8_t *g_ram_va;
static struct vwifi_dev *g_dev;
static struct mock_backend *g_mock;
static uint32_t g_req_slot;

static void wreg(uint32_t off, uint32_t val) { vwifi_reg_write(g_dev, off, val, 4); }

/* Pre-arm every ctrl-rsp slot with its own payload buffer, so async
 * events (BSS_FOUND, SCAN_COMPLETE) have somewhere to land. */
static void arm_all_rsp_slots(void)
{
    for (uint32_t i = 0; i < RING_ENTRIES; i++) {
        struct vwifi_ctrl_rsp_desc *r =
            (struct vwifi_ctrl_rsp_desc *)(g_ram_va + CTRL_RSP_OFFSET
                                           + i * sizeof(*r));
        memset(r, 0, sizeof(*r));
        r->payload_addr = g_ram + RSP_PAYLOAD_OFFSET + (uint64_t)i * RSP_PAYLOAD_STRIDE;
        r->payload_len  = RSP_PAYLOAD_STRIDE;
        r->flags        = VWIFI_DESC_F_OWN;
    }
}

/* Submit a control request; returns the status the device wrote. */
static int32_t ctrl_send(uint16_t opcode, const void *payload, uint32_t plen)
{
    uint32_t slot = g_req_slot % RING_ENTRIES;

    if (payload && plen) {
        memcpy(g_ram_va + REQ_PAYLOAD_OFFSET, payload, plen);
    }

    struct vwifi_ctrl_req_desc *req =
        (struct vwifi_ctrl_req_desc *)(g_ram_va + CTRL_REQ_OFFSET
                                       + slot * sizeof(*req));
    memset(req, 0, sizeof(*req));
    req->opcode       = opcode;
    req->req_id       = 0x2000 + g_req_slot;
    req->payload_addr = g_ram + REQ_PAYLOAD_OFFSET;
    req->payload_len  = plen;
    req->flags        = VWIFI_DESC_F_OWN;

    g_req_slot++;
    wreg(VWIFI_REG_CTRL_REQ_RING_DOORBELL, g_req_slot);

    /* The response for this request lands in whichever rsp slot the
     * device's producer index was on. Scan the ring for our req_id. */
    for (uint32_t i = 0; i < RING_ENTRIES; i++) {
        struct vwifi_ctrl_rsp_desc *r =
            (struct vwifi_ctrl_rsp_desc *)(g_ram_va + CTRL_RSP_OFFSET
                                           + i * sizeof(*r));
        if (!(r->flags & VWIFI_DESC_F_OWN) &&
            !(r->flags & VWIFI_RSP_F_EVENT) &&
            r->req_id == 0x2000 + (g_req_slot - 1)) {
            return r->status;
        }
    }
    return -999;   /* response not found */
}

/* Walk the ctrl-rsp ring collecting async events of a given code. */
static int collect_events(uint16_t event_code,
                          struct vwifi_ctrl_rsp_desc **out, int max_out)
{
    int n = 0;
    for (uint32_t i = 0; i < RING_ENTRIES && n < max_out; i++) {
        struct vwifi_ctrl_rsp_desc *r =
            (struct vwifi_ctrl_rsp_desc *)(g_ram_va + CTRL_RSP_OFFSET
                                           + i * sizeof(*r));
        if (!(r->flags & VWIFI_DESC_F_OWN) &&
            (r->flags & VWIFI_RSP_F_EVENT) &&
            r->opcode == event_code) {
            out[n++] = r;
        }
    }
    return n;
}

/* Consume all events so the next phase of the test starts clean. */
static void drain_rsp_ring(void)
{
    arm_all_rsp_slots();
}

/* Build a beacon frame for a given AP. Returns total length. */
static uint16_t build_beacon(uint8_t *buf, const uint8_t bssid[6],
                             const char *ssid, uint64_t tsf,
                             uint16_t beacon_int, uint16_t caps)
{
    uint16_t len = 0;
    uint8_t ssid_len = (uint8_t)strlen(ssid);

    memset(buf, 0, 128);
    buf[0] = 0x80;                        /* mgmt, subtype 8 = beacon */
    buf[1] = 0x00;
    memset(buf + 4, 0xFF, 6);             /* addr1 = broadcast */
    memcpy(buf + 10, bssid, 6);           /* addr2 = AP */
    memcpy(buf + 16, bssid, 6);           /* addr3 = BSSID */
    len = 24;

    /* Fixed params: timestamp(8) beacon interval(2) capability(2). */
    for (int i = 0; i < 8; i++) buf[len + i] = (uint8_t)(tsf >> (8 * i));
    len += 8;
    buf[len++] = (uint8_t)(beacon_int & 0xFF);
    buf[len++] = (uint8_t)(beacon_int >> 8);
    buf[len++] = (uint8_t)(caps & 0xFF);
    buf[len++] = (uint8_t)(caps >> 8);

    /* SSID IE. */
    buf[len++] = 0;             /* EID_SSID */
    buf[len++] = ssid_len;
    memcpy(buf + len, ssid, ssid_len);
    len += ssid_len;

    /* Supported rates IE (id 1) — gives us a second IE to check that
     * the raw IE blob is captured whole, not just the SSID. */
    buf[len++] = 1;
    buf[len++] = 4;
    buf[len++] = 0x82; buf[len++] = 0x84;
    buf[len++] = 0x8B; buf[len++] = 0x96;

    return len;
}

/* Push an 802.11 frame in through the medium RX path. */
static void medium_rx(const uint8_t *frame, uint16_t frame_len,
                      uint16_t freq, int8_t rssi, const uint8_t tx_mac[6])
{
    uint8_t msg[4 + sizeof(struct vwifi_frame_hdr) + 512];
    struct vwifi_frame_hdr hdr;
    uint32_t plen = sizeof(hdr) + frame_len;
    uint32_t len_be = htonl(plen);
    uint32_t off = 0;

    memset(&hdr, 0, sizeof(hdr));
    hdr.magic        = VWIFI_MAGIC;
    hdr.version      = VWIFI_VERSION;
    hdr.frame_len    = frame_len;
    memcpy(hdr.tx_mac, tx_mac, 6);
    hdr.rate_code    = 0x0B;
    hdr.rssi         = rssi;
    hdr.channel_freq = freq;

    memcpy(msg + off, &len_be, 4);        off += 4;
    memcpy(msg + off, &hdr, sizeof(hdr));  off += sizeof(hdr);
    memcpy(msg + off, frame, frame_len);   off += frame_len;

    vwifi_medium_rx_bytes(g_dev, msg, off);
}

/* Count probe requests among the medium-tx events. */
static int count_probe_reqs(void)
{
    int n = 0;
    for (size_t i = 0; i < mock_backend_event_count(g_mock); i++) {
        const struct mock_event *e = &mock_backend_events(g_mock)[i];
        if (e->kind != MOCK_EV_MEDIUM_TX) continue;
        if (e->medium_len < 4 + 40 + 1) continue;
        const uint8_t *frame = e->medium_buf + 4 + 40;
        uint8_t type    = (frame[0] >> 2) & 0x3;
        uint8_t subtype = (frame[0] >> 4) & 0xF;
        if (type == 0 && subtype == 4) n++;
    }
    return n;
}

/* The channel a medium-tx went out on, from its medium header. */
static uint16_t last_tx_freq(void)
{
    uint16_t freq = 0;
    for (size_t i = 0; i < mock_backend_event_count(g_mock); i++) {
        const struct mock_event *e = &mock_backend_events(g_mock)[i];
        if (e->kind != MOCK_EV_MEDIUM_TX) continue;
        if (e->medium_len < 4 + 40) continue;
        /* channel_freq at medium hdr offset 28. */
        const uint8_t *h = e->medium_buf + 4;
        freq = (uint16_t)h[28] | ((uint16_t)h[29] << 8);
    }
    return freq;
}

int main(void)
{
    g_mock = mock_backend_new(4 * 1024 * 1024, 512);
    mock_backend_set_quiet(g_mock, true);
    g_ram = mock_backend_ram_base(g_mock);
    g_ram_va = mock_backend_ram(g_mock);

    uint8_t sta_mac[6] = { 0x00, 0x03, 0x7F, 0xCC, 0xDD, 0x10 };
    g_dev = malloc(vwifi_dev_sizeof());
    vwifi_dev_init(g_dev, mock_backend_ops_get(), g_mock, "scan-test",
                   false, sta_mac);
    vwifi_medium_link_event(g_dev, true);

    /* Program + enable rings. */
    struct { uint32_t lo, hi, sz; uint64_t base; } r[] = {
        { VWIFI_REG_CTRL_REQ_RING_BASE_LO, VWIFI_REG_CTRL_REQ_RING_BASE_HI,
          VWIFI_REG_CTRL_REQ_RING_SIZE, g_ram + CTRL_REQ_OFFSET },
        { VWIFI_REG_CTRL_RSP_RING_BASE_LO, VWIFI_REG_CTRL_RSP_RING_BASE_HI,
          VWIFI_REG_CTRL_RSP_RING_SIZE, g_ram + CTRL_RSP_OFFSET },
        { VWIFI_REG_TX_RING_BASE_LO, VWIFI_REG_TX_RING_BASE_HI,
          VWIFI_REG_TX_RING_SIZE, g_ram + TX_OFFSET },
        { VWIFI_REG_RX_RING_BASE_LO, VWIFI_REG_RX_RING_BASE_HI,
          VWIFI_REG_RX_RING_SIZE, g_ram + RX_OFFSET },
    };
    for (int i = 0; i < 4; i++) {
        wreg(r[i].lo, (uint32_t)(r[i].base & 0xFFFFFFFF));
        wreg(r[i].hi, (uint32_t)(r[i].base >> 32));
        wreg(r[i].sz, RING_ENTRIES);
    }
    for (int i = 0; i < RING_ENTRIES; i++) {
        struct vwifi_rx_desc *d =
            (struct vwifi_rx_desc *)(g_ram_va + RX_OFFSET + i * sizeof(*d));
        memset(d, 0, sizeof(*d));
        d->frame_addr = g_ram + RX_BUFFER_OFFSET + (uint64_t)i * RX_BUFSZ;
        d->buffer_len = RX_BUFSZ;
        d->flags      = VWIFI_DESC_F_OWN;
    }
    arm_all_rsp_slots();
    wreg(VWIFI_REG_CTRL, VWIFI_CTRL_ENABLE | VWIFI_CTRL_IRQ_ENABLE);

    /* Park on channel 11 so we can prove the scan restores it. */
    struct vwifi_channel home = { .primary_freq = 2462 };
    assert(ctrl_send(VWIFI_OP_SET_CHANNEL, &home, sizeof(home)) == 0);
    drain_rsp_ring();

    /* ---- 1. Start a scan over channels 1, 6, 11 ---- */
    struct vwifi_scan_req sr = { 0 };
    sr.channel_mask_24 = (1u << 1) | (1u << 6) | (1u << 11);
    sr.dwell_ms        = 100;
    sr.num_ssids       = 0;   /* broadcast scan */

    mock_backend_clear_events(g_mock);
    assert(ctrl_send(VWIFI_OP_SCAN, &sr, sizeof(sr)) == 0);
    printf("  scan request accepted: PASS\n");

    /* ---- 2. Should be dwelling on channel 1 (2412) with a probe out ---- */
    assert(last_tx_freq() == 2412);
    assert(count_probe_reqs() == 1);
    assert(mock_backend_timer_armed(g_mock));
    printf("  tuned ch1 + probe request injected + timer armed: PASS\n");

    drain_rsp_ring();

    /* ---- 3. Feed a beacon on channel 1 ---- */
    uint8_t ap1[6] = { 0xAA, 0xBB, 0xCC, 0x00, 0x00, 0x01 };
    uint8_t beacon[128];
    uint16_t blen = build_beacon(beacon, ap1, "vwifi-test",
                                 0x0123456789ABCDEFULL, 100, 0x0431);
    medium_rx(beacon, blen, 2412, -55, ap1);

    struct vwifi_ctrl_rsp_desc *evs[8];
    int n = collect_events(VWIFI_EV_BSS_FOUND, evs, 8);
    assert(n == 1);

    /* Decode the BSS entry payload. */
    {
        uint32_t slot = (uint32_t)((evs[0]->payload_addr - g_ram
                                    - RSP_PAYLOAD_OFFSET) / RSP_PAYLOAD_STRIDE);
        struct vwifi_bss_entry *e = (struct vwifi_bss_entry *)
            (g_ram_va + RSP_PAYLOAD_OFFSET + (uint64_t)slot * RSP_PAYLOAD_STRIDE);

        assert(memcmp(e->bssid, ap1, 6) == 0);
        assert(e->ssid_len == strlen("vwifi-test"));
        assert(strcmp((const char *)e->ssid, "vwifi-test") == 0);
        assert(e->rssi == -55);
        assert(e->channel_freq == 2412);
        assert(e->beacon_period_tu == 100);
        assert((e->capability_info & 0xFFFF) == 0x0431);
        assert(e->tsf == 0x0123456789ABCDEFULL);
        /* ie_len now carries the WHOLE frame (WDI wants the raw beacon,
         * not just the IE tail). 24 hdr + 12 fixed + 12 SSID IE +
         * 6 rates IE = 54. */
        assert(e->ie_len == blen);
        assert(e->capability_info & VWIFI_BSS_F_BEACON);
        const uint8_t *f = (const uint8_t *)e + sizeof(*e);
        assert(f[0] == 0x80);                        /* beacon frame control */
        assert(memcmp(f + 10, ap1, 6) == 0);         /* addr2 = AP */
        const uint8_t *ies = f + 36;                 /* IEs start at 36 */
        assert(ies[0] == 0 && ies[1] == 10);         /* SSID IE header */
        assert(ies[12] == 1 && ies[13] == 4);        /* rates IE header */
    }
    printf("  BSS_FOUND with correct bssid/ssid/rssi/tsf/caps/IEs: PASS\n");

    /* ---- 4. Same beacon again -> no duplicate event ---- */
    drain_rsp_ring();
    medium_rx(beacon, blen, 2412, -55, ap1);
    n = collect_events(VWIFI_EV_BSS_FOUND, evs, 8);
    assert(n == 0);
    printf("  duplicate beacon suppressed within one scan: PASS\n");

    /* ---- 5. Hop to channel 6 ---- */
    drain_rsp_ring();
    mock_backend_clear_events(g_mock);
    assert(mock_backend_advance_to_timer(g_mock));
    vwifi_timer_expired(g_dev);

    assert(last_tx_freq() == 2437);     /* channel 6 */
    assert(count_probe_reqs() == 1);
    printf("  hopped to ch6 + probe request: PASS\n");

    /* ---- 6. Beacon from a second AP on channel 6 ---- */
    uint8_t ap2[6] = { 0xAA, 0xBB, 0xCC, 0x00, 0x00, 0x02 };
    uint16_t blen2 = build_beacon(beacon, ap2, "other-net",
                                  0xFEEDFACEULL, 200, 0x0011);
    medium_rx(beacon, blen2, 2437, -70, ap2);
    n = collect_events(VWIFI_EV_BSS_FOUND, evs, 8);
    assert(n == 1);
    {
        uint32_t slot = (uint32_t)((evs[0]->payload_addr - g_ram
                                    - RSP_PAYLOAD_OFFSET) / RSP_PAYLOAD_STRIDE);
        struct vwifi_bss_entry *e = (struct vwifi_bss_entry *)
            (g_ram_va + RSP_PAYLOAD_OFFSET + (uint64_t)slot * RSP_PAYLOAD_STRIDE);
        assert(memcmp(e->bssid, ap2, 6) == 0);
        assert(strcmp((const char *)e->ssid, "other-net") == 0);
        assert(e->rssi == -70);
    }
    printf("  second BSS discovered on ch6: PASS\n");

    /* ---- 7. Hop to ch11, then off the end -> SCAN_COMPLETE ---- */
    drain_rsp_ring();
    assert(mock_backend_advance_to_timer(g_mock));
    vwifi_timer_expired(g_dev);        /* now on channel 11 */
    assert(last_tx_freq() == 2462);

    drain_rsp_ring();
    assert(mock_backend_advance_to_timer(g_mock));
    vwifi_timer_expired(g_dev);        /* past the last channel */

    n = collect_events(VWIFI_EV_SCAN_COMPLETE, evs, 8);
    assert(n == 1);
    {
        uint32_t slot = (uint32_t)((evs[0]->payload_addr - g_ram
                                    - RSP_PAYLOAD_OFFSET) / RSP_PAYLOAD_STRIDE);
        int32_t *st = (int32_t *)(g_ram_va + RSP_PAYLOAD_OFFSET
                                  + (uint64_t)slot * RSP_PAYLOAD_STRIDE);
        assert(*st == 0);
    }
    assert(!mock_backend_timer_armed(g_mock));
    printf("  SCAN_COMPLETE emitted with status 0: PASS\n");

    /* ---- 8. Second scan re-reports both BSSes ---- */
    drain_rsp_ring();
    assert(ctrl_send(VWIFI_OP_SCAN, &sr, sizeof(sr)) == 0);

    /* Re-feed both beacons on their channels during this scan. */
    blen = build_beacon(beacon, ap1, "vwifi-test",
                        0x0123456789ABCDEFULL, 100, 0x0431);
    medium_rx(beacon, blen, 2412, -55, ap1);
    mock_backend_advance_to_timer(g_mock);
    vwifi_timer_expired(g_dev);
    blen2 = build_beacon(beacon, ap2, "other-net", 0xFEEDFACEULL, 200, 0x0011);
    medium_rx(beacon, blen2, 2437, -70, ap2);

    n = collect_events(VWIFI_EV_BSS_FOUND, evs, 8);
    assert(n == 2);
    printf("  second scan re-reports both BSSes: PASS\n");

    /* Abort mid-scan and confirm the channel is restored to 2462. */
    assert(ctrl_send(VWIFI_OP_SCAN_ABORT, NULL, 0) == 0);
    assert(!mock_backend_timer_armed(g_mock));
    printf("  scan abort restores idle state: PASS\n");

    /* ---- 9. Wildcard SSID entry reports named networks ----
     *
     * cfg80211 puts a single zero-length SSID in almost every scan it
     * issues -- `iw dev wlan0 scan` and NetworkManager both do -- and a
     * zero-length SSID is 802.11's wildcard, not a literal empty name.
     * Comparing it literally matches only hidden APs and drops every
     * named network, which presents as an empty scan on a busy medium.
     * Steps 1-8 all use num_ssids = 0, so only this case covers it. */
    {
        uint8_t req[sizeof(struct vwifi_scan_req) + 34] = { 0 };
        struct vwifi_scan_req *sw = (struct vwifi_scan_req *)req;

        sw->channel_mask_24 = (1u << 1);
        sw->dwell_ms        = 100;
        sw->num_ssids       = 1;
        req[sizeof(*sw)]    = 0;          /* SSID length 0 = wildcard */

        drain_rsp_ring();
        assert(ctrl_send(VWIFI_OP_SCAN, req, sizeof(req)) == 0);

        blen = build_beacon(beacon, ap1, "vwifi-test",
                            0x0123456789ABCDEFULL, 100, 0x0431);
        medium_rx(beacon, blen, 2412, -55, ap1);

        n = collect_events(VWIFI_EV_BSS_FOUND, evs, 8);
        assert(n == 1);
        assert(ctrl_send(VWIFI_OP_SCAN_ABORT, NULL, 0) == 0);
        printf("  wildcard SSID entry still reports named BSS: PASS\n");
    }

    /* ---- 10. ...but a directed scan still filters ---- */
    {
        uint8_t req[sizeof(struct vwifi_scan_req) + 34] = { 0 };
        struct vwifi_scan_req *sd = (struct vwifi_scan_req *)req;
        const char *want = "not-on-the-air";

        sd->channel_mask_24 = (1u << 1);
        sd->dwell_ms        = 100;
        sd->num_ssids       = 1;
        req[sizeof(*sd)]    = (uint8_t)strlen(want);
        memcpy(req + sizeof(*sd) + 1, want, strlen(want));

        drain_rsp_ring();
        assert(ctrl_send(VWIFI_OP_SCAN, req, sizeof(req)) == 0);

        blen = build_beacon(beacon, ap1, "vwifi-test",
                            0x0123456789ABCDEFULL, 100, 0x0431);
        medium_rx(beacon, blen, 2412, -55, ap1);

        n = collect_events(VWIFI_EV_BSS_FOUND, evs, 8);
        assert(n == 0);
        assert(ctrl_send(VWIFI_OP_SCAN_ABORT, NULL, 0) == 0);
        printf("  directed scan for an absent SSID reports nothing: PASS\n");
    }

    printf("scan: PASS\n");
    free(g_dev);
    mock_backend_free(g_mock);
    return 0;
}
