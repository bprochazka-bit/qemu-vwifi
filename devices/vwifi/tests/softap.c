/*
 * vwifi-virt — SoftAP + 5 GHz test (Phase 5)
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 *  1. START_AP -> a beacon goes out immediately, with correct SSID,
 *     capability, DS param channel, and the driver's RSN IE.
 *  2. Beacons repeat on the beacon interval (in Time Units = 1024us,
 *     not 1000us — a 2.4% error that's easy to miss).
 *  3. A wildcard probe request gets a probe response.
 *  4. A directed probe for our SSID gets a response.
 *  5. A directed probe for someone else's SSID does NOT.
 *  6. Auth request -> auth response, station enters the table.
 *  7. Assoc request -> assoc response with an AID, ASSOC_RESULT event.
 *  8. Assoc without prior auth is rejected.
 *  9. STOP_AP deauths the associated station and halts beacons.
 * 10. 5 GHz: a scan over channel_mask_5 visits 5 GHz frequencies.
 *
 * Build:
 *   cc -Wall -Wextra -O2 -I../src -I. ../src/vwifi_device.c \
 *      ../src/vwifi_crypto.c mock_backend.c softap.c -o softap
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
#define RSP_PAYLOAD_OFFSET  0x12000
#define RSP_PAYLOAD_STRIDE  1024
#define RX_BUFFER_OFFSET    0x40000
#define RING_ENTRIES        16
#define RX_BUFSZ            4096
#define MEDIUM_HDR_LEN      40

static uint64_t g_ram;
static uint8_t *g_ram_va;
static struct vwifi_dev *g_dev;
static struct mock_backend *g_mock;
static uint32_t g_req_slot;

static const uint8_t AP_MAC[6]  = { 0x00,0x03,0x7F,0xAA,0xBB,0xCC };
static const uint8_t STA1[6]    = { 0x11,0x22,0x33,0x44,0x55,0x66 };

static void wreg(uint32_t off, uint32_t val) { vwifi_reg_write(g_dev, off, val, 4); }
static uint16_t le16(const uint8_t *p) { return (uint16_t)p[0] | ((uint16_t)p[1] << 8); }
static void put16(uint8_t *p, uint16_t v) { p[0] = v & 0xFF; p[1] = v >> 8; }

static void arm_all_rsp_slots(void)
{
    for (uint32_t i = 0; i < RING_ENTRIES; i++) {
        struct vwifi_ctrl_rsp_desc *r =
            (struct vwifi_ctrl_rsp_desc *)(g_ram_va + CTRL_RSP_OFFSET + i * sizeof(*r));
        memset(r, 0, sizeof(*r));
        r->payload_addr = g_ram + RSP_PAYLOAD_OFFSET + (uint64_t)i * RSP_PAYLOAD_STRIDE;
        r->payload_len  = RSP_PAYLOAD_STRIDE;
        r->flags        = VWIFI_DESC_F_OWN;
    }
}

static int32_t ctrl_send(uint16_t opcode, const void *payload, uint32_t plen)
{
    uint32_t slot = g_req_slot % RING_ENTRIES;
    if (payload && plen) memcpy(g_ram_va + REQ_PAYLOAD_OFFSET, payload, plen);

    struct vwifi_ctrl_req_desc *req =
        (struct vwifi_ctrl_req_desc *)(g_ram_va + CTRL_REQ_OFFSET + slot * sizeof(*req));
    memset(req, 0, sizeof(*req));
    req->opcode = opcode;
    req->req_id = 0x5000 + g_req_slot;
    req->payload_addr = g_ram + REQ_PAYLOAD_OFFSET;
    req->payload_len = plen;
    req->flags = VWIFI_DESC_F_OWN;

    g_req_slot++;
    wreg(VWIFI_REG_CTRL_REQ_RING_DOORBELL, g_req_slot);

    for (uint32_t i = 0; i < RING_ENTRIES; i++) {
        struct vwifi_ctrl_rsp_desc *r =
            (struct vwifi_ctrl_rsp_desc *)(g_ram_va + CTRL_RSP_OFFSET + i * sizeof(*r));
        if (!(r->flags & VWIFI_DESC_F_OWN) && !(r->flags & VWIFI_RSP_F_EVENT) &&
            r->req_id == 0x5000 + (g_req_slot - 1)) return r->status;
    }
    return -999;
}

static struct vwifi_ctrl_rsp_desc *find_event(uint16_t code)
{
    for (uint32_t i = 0; i < RING_ENTRIES; i++) {
        struct vwifi_ctrl_rsp_desc *r =
            (struct vwifi_ctrl_rsp_desc *)(g_ram_va + CTRL_RSP_OFFSET + i * sizeof(*r));
        if (!(r->flags & VWIFI_DESC_F_OWN) && (r->flags & VWIFI_RSP_F_EVENT) &&
            r->opcode == code) return r;
    }
    return NULL;
}

static void *event_payload(const struct vwifi_ctrl_rsp_desc *e)
{
    uint32_t slot = (uint32_t)((e->payload_addr - g_ram - RSP_PAYLOAD_OFFSET)
                               / RSP_PAYLOAD_STRIDE);
    return g_ram_va + RSP_PAYLOAD_OFFSET + (uint64_t)slot * RSP_PAYLOAD_STRIDE;
}

static void medium_rx(const uint8_t *frame, uint16_t frame_len,
                      uint16_t freq, const uint8_t tx_mac[6])
{
    uint8_t msg[4 + MEDIUM_HDR_LEN + 512];
    struct vwifi_frame_hdr hdr;
    uint32_t plen = sizeof(hdr) + frame_len;
    uint32_t len_be = htonl(plen);
    uint32_t off = 0;

    memset(&hdr, 0, sizeof(hdr));
    hdr.magic = VWIFI_MAGIC;
    hdr.version = VWIFI_VERSION;
    hdr.frame_len = frame_len;
    memcpy(hdr.tx_mac, tx_mac, 6);
    hdr.rate_code = 0x0B;
    hdr.rssi = -45;
    hdr.channel_freq = freq;

    memcpy(msg + off, &len_be, 4);        off += 4;
    memcpy(msg + off, &hdr, sizeof(hdr)); off += sizeof(hdr);
    memcpy(msg + off, frame, frame_len);  off += frame_len;
    vwifi_medium_rx_bytes(g_dev, msg, off);
}

/* Find the last TX frame of a given mgmt subtype; NULL if none. */
static const uint8_t *last_tx_of_subtype(uint8_t subtype, uint16_t *len_out)
{
    const struct mock_event *found = NULL;
    for (size_t i = 0; i < mock_backend_event_count(g_mock); i++) {
        const struct mock_event *e = &mock_backend_events(g_mock)[i];
        if (e->kind != MOCK_EV_MEDIUM_TX) continue;
        if (e->medium_len < 4 + MEDIUM_HDR_LEN + 24) continue;
        const uint8_t *f = e->medium_buf + 4 + MEDIUM_HDR_LEN;
        if (((f[0] >> 2) & 0x3) != 0) continue;
        if (((f[0] >> 4) & 0xF) != subtype) continue;
        found = e;
    }
    if (!found) { if (len_out) *len_out = 0; return NULL; }
    if (len_out) *len_out = (uint16_t)(found->medium_len - 4 - MEDIUM_HDR_LEN);
    return found->medium_buf + 4 + MEDIUM_HDR_LEN;
}

static int count_tx_of_subtype(uint8_t subtype)
{
    int n = 0;
    for (size_t i = 0; i < mock_backend_event_count(g_mock); i++) {
        const struct mock_event *e = &mock_backend_events(g_mock)[i];
        if (e->kind != MOCK_EV_MEDIUM_TX) continue;
        if (e->medium_len < 4 + MEDIUM_HDR_LEN + 24) continue;
        const uint8_t *f = e->medium_buf + 4 + MEDIUM_HDR_LEN;
        if (((f[0] >> 2) & 0x3) != 0) continue;
        if (((f[0] >> 4) & 0xF) == subtype) n++;
    }
    return n;
}

/* Walk IEs from a beacon/probe-resp body looking for one element. */
static const uint8_t *find_ie(const uint8_t *f, uint16_t len, uint8_t eid,
                              uint8_t *ie_len_out)
{
    uint16_t off = 36;   /* 24 hdr + 12 fixed */
    while (off + 2 <= len) {
        uint8_t id = f[off], l = f[off + 1];
        if (off + 2 + l > len) return NULL;
        if (id == eid) { if (ie_len_out) *ie_len_out = l; return f + off + 2; }
        off += 2 + l;
    }
    return NULL;
}

/* Build a probe request with an optional SSID. */
static uint16_t build_probe_req(uint8_t *buf, const uint8_t *src,
                                const char *ssid)
{
    uint16_t len = 0;
    uint8_t sl = ssid ? (uint8_t)strlen(ssid) : 0;
    memset(buf, 0, 64);
    buf[0] = (4 << 4);                 /* mgmt, probe request */
    memset(buf + 4, 0xFF, 6);          /* addr1 broadcast */
    memcpy(buf + 10, src, 6);
    memset(buf + 16, 0xFF, 6);
    len = 24;
    buf[len++] = 0;                    /* SSID IE */
    buf[len++] = sl;
    if (sl) { memcpy(buf + len, ssid, sl); len += sl; }
    return len;
}

static uint16_t build_auth_req(uint8_t *buf, const uint8_t *src)
{
    memset(buf, 0, 64);
    buf[0] = (11 << 4);
    memcpy(buf + 4, AP_MAC, 6);
    memcpy(buf + 10, src, 6);
    memcpy(buf + 16, AP_MAC, 6);
    put16(buf + 24, 0);   /* Open */
    put16(buf + 26, 1);   /* seq 1 */
    put16(buf + 28, 0);
    return 30;
}

static uint16_t build_assoc_req(uint8_t *buf, const uint8_t *src,
                                const char *ssid)
{
    uint16_t len;
    uint8_t sl = (uint8_t)strlen(ssid);
    memset(buf, 0, 64);
    buf[0] = (0 << 4);    /* assoc request */
    memcpy(buf + 4, AP_MAC, 6);
    memcpy(buf + 10, src, 6);
    memcpy(buf + 16, AP_MAC, 6);
    put16(buf + 24, 0x0431);   /* capability */
    put16(buf + 26, 10);       /* listen interval */
    len = 28;
    buf[len++] = 0; buf[len++] = sl;
    memcpy(buf + len, ssid, sl); len += sl;
    return len;
}

int main(void)
{
    g_mock = mock_backend_new(4 * 1024 * 1024, 512);
    mock_backend_set_quiet(g_mock, true);
    g_ram = mock_backend_ram_base(g_mock);
    g_ram_va = mock_backend_ram(g_mock);

    g_dev = malloc(vwifi_dev_sizeof());
    vwifi_dev_init(g_dev, mock_backend_ops_get(), g_mock, "ap-test", false, AP_MAC);
    vwifi_medium_link_event(g_dev, true);

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
        d->flags = VWIFI_DESC_F_OWN;
    }
    arm_all_rsp_slots();
    wreg(VWIFI_REG_CTRL, VWIFI_CTRL_ENABLE | VWIFI_CTRL_IRQ_ENABLE);

    /* Caps must now advertise SoftAP and 5 GHz. */
    {
        struct vwifi_caps caps;
        uint8_t save[sizeof(caps)];
        assert(ctrl_send(VWIFI_OP_GET_CAPS, NULL, 0) == 0);
        memcpy(save, g_ram_va + RSP_PAYLOAD_OFFSET, sizeof(caps));
        memcpy(&caps, save, sizeof(caps));
        assert(caps.caps & VWIFI_CAP_SOFTAP);
        assert(caps.supported_channels_5 != 0);
        printf("  caps advertise SoftAP + 5 GHz: PASS\n");
    }
    arm_all_rsp_slots();

    /* SoftAP mode. */
    struct vwifi_op_mode om = { .mode = VWIFI_MODE_SOFTAP };
    assert(ctrl_send(VWIFI_OP_SET_OP_MODE, &om, sizeof(om)) == 0);
    arm_all_rsp_slots();

    /* ---- 1. START_AP -> immediate beacon ---- */
    uint8_t cfgbuf[sizeof(struct vwifi_ap_config) + 8];
    struct vwifi_ap_config *cfg = (struct vwifi_ap_config *)cfgbuf;
    memset(cfgbuf, 0, sizeof(cfgbuf));
    cfg->ssid_len = (uint16_t)strlen("vwifi-ap");
    memcpy(cfg->ssid, "vwifi-ap", cfg->ssid_len);
    cfg->channel_freq       = 2437;   /* channel 6 */
    cfg->beacon_interval_tu = 100;
    cfg->dtim_period        = 1;
    cfg->ie_len             = 6;
    {   /* a pretend RSN element so the AP advertises privacy */
        uint8_t *rsn = cfgbuf + sizeof(*cfg);
        rsn[0] = 48; rsn[1] = 4; rsn[2] = 0x01; rsn[3] = 0x00;
        rsn[4] = 0x00; rsn[5] = 0x0F;
    }

    mock_backend_clear_events(g_mock);
    assert(ctrl_send(VWIFI_OP_START_AP, cfgbuf,
                     sizeof(*cfg) + cfg->ie_len) == 0);

    {
        uint16_t len;
        const uint8_t *b = last_tx_of_subtype(8 /* beacon */, &len);
        assert(b != NULL);
        assert(memcmp(b + 10, AP_MAC, 6) == 0);      /* addr2 = us */
        assert(memcmp(b + 16, AP_MAC, 6) == 0);      /* addr3 = BSSID */
        uint16_t caps = le16(b + 34);
        assert(caps & 0x0001);                        /* ESS */
        assert(caps & 0x0010);                        /* Privacy (we gave RSN) */
        assert(le16(b + 32) == 100);                  /* beacon interval */

        uint8_t sl = 0;
        const uint8_t *ssid = find_ie(b, len, 0, &sl);
        assert(ssid && sl == 8 && memcmp(ssid, "vwifi-ap", 8) == 0);

        uint8_t dsl = 0;
        const uint8_t *ds = find_ie(b, len, 3, &dsl);
        assert(ds && dsl == 1 && ds[0] == 6);         /* DS param: channel 6 */

        uint8_t rl = 0;
        const uint8_t *rsn = find_ie(b, len, 48, &rl);
        assert(rsn && rl == 4);                        /* driver's RSN IE */
    }
    printf("  START_AP emits beacon w/ SSID, DS param, RSN IE, privacy: PASS\n");

    /* ---- 2. Beacons repeat on the interval, drift-free ----
     *
     * A Time Unit is 1024us, not 1000us. Over 10 beacons at 100 TU the
     * difference between getting that right and wrong is 24ms — small
     * per beacon, obvious in aggregate. This also catches drift: the
     * host timer only has millisecond resolution, so a naive
     * "schedule from now" would creep. */
    {
        uint64_t t0 = mock_backend_now_us(g_mock);
        int beacons = 0;

        for (int i = 0; i < 10; i++) {
            mock_backend_clear_events(g_mock);
            assert(mock_backend_advance_to_timer(g_mock));
            vwifi_timer_expired(g_dev);
            beacons += count_tx_of_subtype(8);
        }
        uint64_t elapsed = mock_backend_now_us(g_mock) - t0;

        assert(beacons == 10);
        /* Expect 10 x 102400us = 1024000us, within one timer tick of
         * slack for the ms-granularity rounding. Crucially this is NOT
         * 1000000us, which is what TU=1000 would produce. */
        assert(elapsed >= 1024000 - 2000 && elapsed <= 1024000 + 2000);
        assert(elapsed > 1010000);   /* would fail if TU were 1000us */
    }
    printf("  beacons repeat at 100 TU (1024us each), drift-free over 10: PASS\n");

    /* ---- 3. Wildcard probe -> response ---- */
    uint8_t pr[64];
    uint16_t prlen = build_probe_req(pr, STA1, NULL);
    mock_backend_clear_events(g_mock);
    medium_rx(pr, prlen, 2437, STA1);
    {
        uint16_t len;
        const uint8_t *resp = last_tx_of_subtype(5 /* probe resp */, &len);
        assert(resp != NULL);
        assert(memcmp(resp + 4, STA1, 6) == 0);   /* addressed to the prober */
        uint8_t sl = 0;
        const uint8_t *ssid = find_ie(resp, len, 0, &sl);
        assert(ssid && sl == 8 && memcmp(ssid, "vwifi-ap", 8) == 0);
    }
    printf("  wildcard probe request -> probe response: PASS\n");

    /* ---- 4. Directed probe for our SSID -> response ---- */
    prlen = build_probe_req(pr, STA1, "vwifi-ap");
    mock_backend_clear_events(g_mock);
    medium_rx(pr, prlen, 2437, STA1);
    assert(count_tx_of_subtype(5) == 1);
    printf("  directed probe for our SSID -> response: PASS\n");

    /* ---- 5. Directed probe for another SSID -> silence ---- */
    prlen = build_probe_req(pr, STA1, "someone-else");
    mock_backend_clear_events(g_mock);
    medium_rx(pr, prlen, 2437, STA1);
    assert(count_tx_of_subtype(5) == 0);
    printf("  directed probe for a different SSID ignored: PASS\n");

    /* ---- 6. Auth request -> auth response ---- */
    uint8_t ar[64];
    uint16_t arlen = build_auth_req(ar, STA1);
    mock_backend_clear_events(g_mock);
    medium_rx(ar, arlen, 2437, STA1);
    {
        uint16_t len;
        const uint8_t *resp = last_tx_of_subtype(11, &len);
        assert(resp != NULL);
        assert(memcmp(resp + 4, STA1, 6) == 0);
        assert(le16(resp + 24) == 0);    /* Open System */
        assert(le16(resp + 26) == 2);    /* seq 2 */
        assert(le16(resp + 28) == 0);    /* success */
    }
    printf("  auth request -> auth response (open, seq 2, success): PASS\n");

    /* ---- 7. Assoc request -> assoc response + ASSOC_RESULT ---- */
    arm_all_rsp_slots();
    uint8_t asr[64];
    uint16_t asrlen = build_assoc_req(asr, STA1, "vwifi-ap");
    mock_backend_clear_events(g_mock);
    medium_rx(asr, asrlen, 2437, STA1);
    {
        uint16_t len;
        const uint8_t *resp = last_tx_of_subtype(1, &len);
        assert(resp != NULL);
        assert(le16(resp + 26) == 0);                  /* status success */
        uint16_t aid = le16(resp + 28);
        assert((aid & 0xC000) == 0xC000);              /* top bits set */
        assert((aid & 0x3FFF) == 1);                   /* first AID */

        struct vwifi_ctrl_rsp_desc *e = find_event(VWIFI_EV_ASSOC_RESULT);
        assert(e != NULL);
        struct vwifi_assoc_result *res = event_payload(e);
        assert(memcmp(res->bssid, STA1, 6) == 0);      /* the peer that joined */
        assert(res->aid == 1);
    }
    printf("  assoc request -> assoc response aid=1 + ASSOC_RESULT: PASS\n");

    /* ---- 8. Assoc without auth is rejected ---- */
    uint8_t STA2[6] = { 0x99,0x88,0x77,0x66,0x55,0x44 };
    asrlen = build_assoc_req(asr, STA2, "vwifi-ap");
    mock_backend_clear_events(g_mock);
    medium_rx(asr, asrlen, 2437, STA2);
    {
        uint16_t len;
        const uint8_t *resp = last_tx_of_subtype(1, &len);
        assert(resp != NULL);
        assert(le16(resp + 26) != 0);   /* rejected */
    }
    printf("  assoc without prior auth rejected: PASS\n");

    /* ---- 9. STOP_AP deauths stations and halts beacons ---- */
    mock_backend_clear_events(g_mock);
    assert(ctrl_send(VWIFI_OP_STOP_AP, NULL, 0) == 0);
    {
        uint16_t len;
        const uint8_t *d = last_tx_of_subtype(12 /* deauth */, &len);
        assert(d != NULL);
        assert(memcmp(d + 4, STA1, 6) == 0);   /* the associated station */
    }
    /* No beacon timer should remain armed. */
    mock_backend_clear_events(g_mock);
    if (mock_backend_advance_to_timer(g_mock)) {
        vwifi_timer_expired(g_dev);
    }
    assert(count_tx_of_subtype(8) == 0);
    printf("  STOP_AP deauths stations and stops beacons: PASS\n");

    /* ---- 10. 5 GHz scan ---- */
    arm_all_rsp_slots();
    om.mode = VWIFI_MODE_STA;
    assert(ctrl_send(VWIFI_OP_SET_OP_MODE, &om, sizeof(om)) == 0);
    arm_all_rsp_slots();
    {
        struct vwifi_scan_req sr = { 0 };
        sr.channel_mask_24 = 0;
        sr.channel_mask_5  = 0x1;   /* index 0 => channel 36 => 5180 MHz */
        sr.dwell_ms        = 50;
        mock_backend_clear_events(g_mock);
        assert(ctrl_send(VWIFI_OP_SCAN, &sr, sizeof(sr)) == 0);

        uint16_t len;
        const uint8_t *p = last_tx_of_subtype(4 /* probe req */, &len);
        assert(p != NULL);
        /* Confirm the probe went out on 5180 MHz: medium hdr offset 28. */
        const struct mock_event *ev = NULL;
        for (size_t i = 0; i < mock_backend_event_count(g_mock); i++) {
            const struct mock_event *e = &mock_backend_events(g_mock)[i];
            if (e->kind == MOCK_EV_MEDIUM_TX) ev = e;
        }
        assert(ev != NULL);
        const uint8_t *h = ev->medium_buf + 4;
        uint16_t freq = (uint16_t)h[28] | ((uint16_t)h[29] << 8);
        assert(freq == 5180);
    }
    printf("  5 GHz scan reaches channel 36 (5180 MHz): PASS\n");

    printf("softap: PASS\n");
    free(g_dev);
    mock_backend_free(g_mock);
    return 0;
}
