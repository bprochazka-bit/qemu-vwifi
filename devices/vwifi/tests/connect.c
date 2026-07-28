/*
 * vwifi-virt — connect + data path test
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Simulates an AP on the medium and drives a full association:
 *
 *  1. CONNECT -> device sends an Auth Request; verify its contents.
 *  2. Reply with an Auth Response -> device sends an Assoc Request;
 *     verify the SSID IE and the driver-supplied RSN IE are present.
 *  3. Reply with an Assoc Response -> verify ASSOC_RESULT carries the
 *     AID, status 0, and the AP's response IEs.
 *  4. TX: hand the device an 802.3 frame; verify the 802.11 data frame
 *     on the medium has correct ToDS, addresses, LLC/SNAP, ethertype.
 *  5. RX: feed an 802.11 downlink data frame; verify the driver gets
 *     a correct 802.3 frame back.
 *  6. Filtering: a data frame from a different BSSID is dropped.
 *  7. Deauth from the AP -> DISCONNECTED event, state resets.
 *  8. Timeout: a CONNECT with no AP reply fails cleanly.
 *
 * Build:
 *   cc -Wall -Wextra -O2 -I../src -I. \
 *      ../src/vwifi_device.c mock_backend.c connect.c -o connect
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
#define TX_FRAME_OFFSET     0x60000
#define RING_ENTRIES        16
#define RX_BUFSZ            4096

#define MEDIUM_HDR_LEN      40   /* vwifi_frame_hdr v2 */

static uint64_t g_ram;
static uint8_t *g_ram_va;
static struct vwifi_dev *g_dev;
static struct mock_backend *g_mock;
static uint32_t g_req_slot;
static uint32_t g_tx_slot;

static const uint8_t STA_MAC[6] = { 0x00, 0x03, 0x7F, 0xCC, 0xDD, 0x10 };
static const uint8_t AP_MAC[6]  = { 0xAA, 0xBB, 0xCC, 0x00, 0x00, 0x01 };
static const uint8_t PEER_MAC[6]= { 0x11, 0x22, 0x33, 0x44, 0x55, 0x66 };

static void wreg(uint32_t off, uint32_t val) { vwifi_reg_write(g_dev, off, val, 4); }

static void arm_all_rsp_slots(void)
{
    for (uint32_t i = 0; i < RING_ENTRIES; i++) {
        struct vwifi_ctrl_rsp_desc *r =
            (struct vwifi_ctrl_rsp_desc *)(g_ram_va + CTRL_RSP_OFFSET
                                           + i * sizeof(*r));
        memset(r, 0, sizeof(*r));
        r->payload_addr = g_ram + RSP_PAYLOAD_OFFSET
                        + (uint64_t)i * RSP_PAYLOAD_STRIDE;
        r->payload_len  = RSP_PAYLOAD_STRIDE;
        r->flags        = VWIFI_DESC_F_OWN;
    }
}

static int32_t ctrl_send(uint16_t opcode, const void *payload, uint32_t plen)
{
    uint32_t slot = g_req_slot % RING_ENTRIES;
    if (payload && plen) memcpy(g_ram_va + REQ_PAYLOAD_OFFSET, payload, plen);

    struct vwifi_ctrl_req_desc *req =
        (struct vwifi_ctrl_req_desc *)(g_ram_va + CTRL_REQ_OFFSET
                                       + slot * sizeof(*req));
    memset(req, 0, sizeof(*req));
    req->opcode       = opcode;
    req->req_id       = 0x3000 + g_req_slot;
    req->payload_addr = g_ram + REQ_PAYLOAD_OFFSET;
    req->payload_len  = plen;
    req->flags        = VWIFI_DESC_F_OWN;

    g_req_slot++;
    wreg(VWIFI_REG_CTRL_REQ_RING_DOORBELL, g_req_slot);

    for (uint32_t i = 0; i < RING_ENTRIES; i++) {
        struct vwifi_ctrl_rsp_desc *r =
            (struct vwifi_ctrl_rsp_desc *)(g_ram_va + CTRL_RSP_OFFSET
                                           + i * sizeof(*r));
        if (!(r->flags & VWIFI_DESC_F_OWN) && !(r->flags & VWIFI_RSP_F_EVENT) &&
            r->req_id == 0x3000 + (g_req_slot - 1)) {
            return r->status;
        }
    }
    return -999;
}

/* Find the first async event of a given code; NULL if none. */
static struct vwifi_ctrl_rsp_desc *find_event(uint16_t code)
{
    for (uint32_t i = 0; i < RING_ENTRIES; i++) {
        struct vwifi_ctrl_rsp_desc *r =
            (struct vwifi_ctrl_rsp_desc *)(g_ram_va + CTRL_RSP_OFFSET
                                           + i * sizeof(*r));
        if (!(r->flags & VWIFI_DESC_F_OWN) && (r->flags & VWIFI_RSP_F_EVENT) &&
            r->opcode == code) {
            return r;
        }
    }
    return NULL;
}

static void *event_payload(const struct vwifi_ctrl_rsp_desc *e)
{
    uint32_t slot = (uint32_t)((e->payload_addr - g_ram - RSP_PAYLOAD_OFFSET)
                               / RSP_PAYLOAD_STRIDE);
    return g_ram_va + RSP_PAYLOAD_OFFSET + (uint64_t)slot * RSP_PAYLOAD_STRIDE;
}

/* Push a raw 802.11 frame in through the medium RX path. */
static void medium_rx(const uint8_t *frame, uint16_t frame_len,
                      uint16_t freq, const uint8_t tx_mac[6])
{
    uint8_t msg[4 + MEDIUM_HDR_LEN + 1024];
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
    hdr.rssi         = -50;
    hdr.channel_freq = freq;

    memcpy(msg + off, &len_be, 4);         off += 4;
    memcpy(msg + off, &hdr, sizeof(hdr));   off += sizeof(hdr);
    memcpy(msg + off, frame, frame_len);    off += frame_len;
    vwifi_medium_rx_bytes(g_dev, msg, off);
}

/* Pull the last 802.11 frame the device sent to the medium. */
static const uint8_t *last_tx_frame(uint16_t *len_out)
{
    const struct mock_event *found = NULL;
    for (size_t i = 0; i < mock_backend_event_count(g_mock); i++) {
        const struct mock_event *e = &mock_backend_events(g_mock)[i];
        if (e->kind == MOCK_EV_MEDIUM_TX && e->medium_len > 4 + MEDIUM_HDR_LEN) {
            found = e;
        }
    }
    if (!found) { *len_out = 0; return NULL; }
    *len_out = (uint16_t)(found->medium_len - 4 - MEDIUM_HDR_LEN);
    return found->medium_buf + 4 + MEDIUM_HDR_LEN;
}

/* Channel the device stamped on the last frame it sent. */
static uint16_t last_tx_freq(void)
{
    const struct mock_event *found = NULL;
    for (size_t i = 0; i < mock_backend_event_count(g_mock); i++) {
        const struct mock_event *e = &mock_backend_events(g_mock)[i];
        if (e->kind == MOCK_EV_MEDIUM_TX && e->medium_len > 4 + MEDIUM_HDR_LEN) {
            found = e;
        }
    }
    if (!found) return 0;
    struct vwifi_frame_hdr h;
    memcpy(&h, found->medium_buf + 4, sizeof(h));
    return h.channel_freq;
}

/* Minimal beacon: mgmt header, fixed fields, one SSID element. */
static uint16_t build_beacon(uint8_t *buf, const uint8_t bssid[6],
                             const char *ssid)
{
    uint16_t len = 0;
    uint8_t slen = (uint8_t)strlen(ssid);

    memset(buf, 0, 64);
    buf[0] = (8 << 4);                  /* mgmt, beacon */
    memset(buf + 4, 0xFF, 6);           /* addr1 broadcast */
    memcpy(buf + 10, bssid, 6);         /* addr2 */
    memcpy(buf + 16, bssid, 6);         /* addr3 = BSSID */
    len = 24 + 12;                      /* header + tsf/interval/caps */
    buf[len++] = 0;                     /* SSID element */
    buf[len++] = slen;
    memcpy(buf + len, ssid, slen);
    len += slen;
    return len;
}

static uint16_t le16(const uint8_t *p) { return (uint16_t)p[0] | ((uint16_t)p[1] << 8); }
static void put16(uint8_t *p, uint16_t v) { p[0] = v & 0xFF; p[1] = v >> 8; }

/* Build an Auth Response from the AP. */
static uint16_t build_auth_resp(uint8_t *buf, uint16_t status)
{
    memset(buf, 0, 64);
    buf[0] = (11 << 4);              /* mgmt, subtype 11 = Auth */
    memcpy(buf + 4,  STA_MAC, 6);    /* addr1 = us */
    memcpy(buf + 10, AP_MAC, 6);     /* addr2 = AP */
    memcpy(buf + 16, AP_MAC, 6);     /* addr3 = BSSID */
    put16(buf + 24, 0);              /* algorithm = Open */
    put16(buf + 26, 2);              /* sequence 2 */
    put16(buf + 28, status);
    return 30;
}

/* Build an Assoc Response from the AP, with a couple of IEs. */
static uint16_t build_assoc_resp(uint8_t *buf, uint16_t status, uint16_t aid)
{
    uint16_t len;
    memset(buf, 0, 64);
    buf[0] = (1 << 4);               /* mgmt, subtype 1 = Assoc Resp */
    memcpy(buf + 4,  STA_MAC, 6);
    memcpy(buf + 10, AP_MAC, 6);
    memcpy(buf + 16, AP_MAC, 6);
    put16(buf + 24, 0x0431);         /* capability */
    put16(buf + 26, status);
    put16(buf + 28, aid | 0xC000);   /* AID with top bits set, as real APs do */
    len = 30;
    /* Supported rates IE. */
    buf[len++] = 1; buf[len++] = 4;
    buf[len++] = 0x82; buf[len++] = 0x84; buf[len++] = 0x8B; buf[len++] = 0x96;
    return len;
}

/* Build a Deauth from the AP. */
static uint16_t build_deauth(uint8_t *buf, uint16_t reason)
{
    memset(buf, 0, 32);
    buf[0] = (12 << 4);              /* mgmt, subtype 12 = Deauth */
    memcpy(buf + 4,  STA_MAC, 6);
    memcpy(buf + 10, AP_MAC, 6);
    memcpy(buf + 16, AP_MAC, 6);
    put16(buf + 24, reason);
    return 26;
}

/* Build a downlink 802.11 data frame from the AP to us. */
static uint16_t build_data_downlink(uint8_t *buf, const uint8_t bssid[6],
                                    const uint8_t *payload, uint16_t plen,
                                    uint16_t ethertype)
{
    uint16_t len = 0;
    memset(buf, 0, 32);
    buf[0] = 0x08;                   /* data, subtype 0 */
    buf[1] = 0x02;                   /* FromDS */
    memcpy(buf + 4,  STA_MAC, 6);    /* addr1 = DA (us) */
    memcpy(buf + 10, bssid, 6);      /* addr2 = BSSID */
    memcpy(buf + 16, PEER_MAC, 6);   /* addr3 = SA */
    len = 24;
    /* LLC/SNAP. */
    buf[len++] = 0xAA; buf[len++] = 0xAA; buf[len++] = 0x03;
    buf[len++] = 0x00; buf[len++] = 0x00; buf[len++] = 0x00;
    buf[len++] = (uint8_t)(ethertype >> 8);
    buf[len++] = (uint8_t)(ethertype & 0xFF);
    memcpy(buf + len, payload, plen);
    len += plen;
    return len;
}

int main(void)
{
    g_mock = mock_backend_new(4 * 1024 * 1024, 512);
    mock_backend_set_quiet(g_mock, true);
    g_ram = mock_backend_ram_base(g_mock);
    g_ram_va = mock_backend_ram(g_mock);

    g_dev = malloc(vwifi_dev_sizeof());
    vwifi_dev_init(g_dev, mock_backend_ops_get(), g_mock, "conn-test",
                   false, STA_MAC);
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
        d->flags      = VWIFI_DESC_F_OWN;
    }
    arm_all_rsp_slots();
    wreg(VWIFI_REG_CTRL, VWIFI_CTRL_ENABLE | VWIFI_CTRL_IRQ_ENABLE);

    /* STA mode. */
    struct vwifi_op_mode om = { .mode = VWIFI_MODE_STA };
    assert(ctrl_send(VWIFI_OP_SET_OP_MODE, &om, sizeof(om)) == 0);
    arm_all_rsp_slots();

    /* ---- 1. CONNECT -> Auth Request ---- */
    uint8_t connbuf[sizeof(struct vwifi_connect_req) + 32];
    struct vwifi_connect_req *cr = (struct vwifi_connect_req *)connbuf;
    memset(connbuf, 0, sizeof(connbuf));
    memcpy(cr->bssid, AP_MAC, 6);
    cr->ssid_len = (uint16_t)strlen("vwifi-test");
    memcpy(cr->ssid, "vwifi-test", cr->ssid_len);
    cr->channel_freq    = 2437;
    cr->auth_algo       = VWIFI_AUTH_OPEN;
    cr->cipher_pairwise = VWIFI_CIPHER_NONE;
    cr->akm_suite       = VWIFI_AKM_NONE;
    /* Pretend the driver wants an RSN IE included (id 48). */
    cr->assoc_ie_len = 6;
    uint8_t *rsn = connbuf + sizeof(*cr);
    rsn[0] = 48; rsn[1] = 4; rsn[2] = 0x01; rsn[3] = 0x00;
    rsn[4] = 0x00; rsn[5] = 0x0F;

    mock_backend_clear_events(g_mock);
    assert(ctrl_send(VWIFI_OP_CONNECT, connbuf,
                     sizeof(*cr) + cr->assoc_ie_len) == 0);

    {
        uint16_t len;
        const uint8_t *f = last_tx_frame(&len);
        assert(f != NULL);
        assert(((f[0] >> 2) & 0x3) == 0);        /* mgmt */
        assert(((f[0] >> 4) & 0xF) == 11);       /* Auth */
        assert(memcmp(f + 4,  AP_MAC, 6) == 0);  /* addr1 = AP */
        assert(memcmp(f + 10, STA_MAC, 6) == 0); /* addr2 = us */
        assert(le16(f + 24) == 0);               /* algo = Open */
        assert(le16(f + 26) == 1);               /* seq 1 */
    }
    printf("  CONNECT emits well-formed Auth Request: PASS\n");

    /* ---- 2. Auth Response -> Assoc Request ---- */
    uint8_t apbuf[128];
    uint16_t aplen = build_auth_resp(apbuf, 0);
    mock_backend_clear_events(g_mock);
    medium_rx(apbuf, aplen, 2437, AP_MAC);

    {
        uint16_t len;
        const uint8_t *f = last_tx_frame(&len);
        assert(f != NULL);
        assert(((f[0] >> 4) & 0xF) == 0);        /* Assoc Request */
        assert(memcmp(f + 4, AP_MAC, 6) == 0);
        /* Body: capability(2) listen(2), then IEs at offset 28. */
        const uint8_t *ie = f + 28;
        assert(ie[0] == 0);                      /* SSID IE */
        assert(ie[1] == strlen("vwifi-test"));
        assert(memcmp(ie + 2, "vwifi-test", 10) == 0);

        /*
         * Supported Rates must follow. hostapd rejects an Association
         * Request without this element with WLAN_STATUS_UNSPECIFIED_
         * FAILURE, which surfaces as an unexplained "status=1" and
         * nothing else -- worth a test that names the element.
         */
        const uint8_t *rates = ie + 2 + 10;
        assert(rates[0] == 1);                   /* EID Supported Rates */
        assert(rates[1] == 8);
        assert(rates[2] == 0x82);                /* 1 Mbit/s, basic */

        /* 2.4 GHz overflows into Extended Supported Rates. */
        const uint8_t *ext = rates + 2 + 8;
        assert(ext[0] == 50);
        assert(ext[1] == 4);
        assert(ext[2] == 0x30);                  /* 24 Mbit/s */

        /* Then the driver's own IEs. */
        const uint8_t *rsnie = ext + 2 + 4;
        assert(rsnie[0] == 48);
        assert(rsnie[1] == 4);
    }
    printf("  Assoc Request carries SSID + rates + driver RSN IEs: PASS\n");

    /* ---- 3. Assoc Response -> ASSOC_RESULT ---- */
    arm_all_rsp_slots();
    aplen = build_assoc_resp(apbuf, 0, 7);
    medium_rx(apbuf, aplen, 2437, AP_MAC);

    {
        struct vwifi_ctrl_rsp_desc *e = find_event(VWIFI_EV_ASSOC_RESULT);
        assert(e != NULL);
        struct vwifi_assoc_result *ar = event_payload(e);
        assert(memcmp(ar->bssid, AP_MAC, 6) == 0);
        assert(ar->status_code == 0);
        assert(ar->aid == 7);              /* top two bits masked off */
        assert(ar->ie_len == 6);           /* the rates IE */
        const uint8_t *ies = (const uint8_t *)ar + sizeof(*ar);
        assert(ies[0] == 1 && ies[1] == 4);
    }
    printf("  Assoc Response -> ASSOC_RESULT aid=7 + response IEs: PASS\n");

    /* ---- 4. TX: 802.3 in, 802.11 out ---- */
    uint8_t eth[64];
    memset(eth, 0, sizeof(eth));
    memcpy(eth,     PEER_MAC, 6);       /* dst */
    memcpy(eth + 6, STA_MAC, 6);        /* src */
    eth[12] = 0x08; eth[13] = 0x00;     /* ethertype IPv4 */
    for (int i = 0; i < 20; i++) eth[14 + i] = (uint8_t)(0xA0 + i);
    uint16_t eth_len = 14 + 20;

    memcpy(g_ram_va + TX_FRAME_OFFSET, eth, eth_len);
    struct vwifi_tx_desc *tx =
        (struct vwifi_tx_desc *)(g_ram_va + TX_OFFSET
                                 + (g_tx_slot % RING_ENTRIES) * sizeof(*tx));
    memset(tx, 0, sizeof(*tx));
    tx->frame_addr = g_ram + TX_FRAME_OFFSET;
    tx->frame_len  = eth_len;
    tx->flags      = VWIFI_DESC_F_OWN;
    g_tx_slot++;

    mock_backend_clear_events(g_mock);
    wreg(VWIFI_REG_TX_RING_DOORBELL, g_tx_slot);

    {
        uint16_t len;
        const uint8_t *f = last_tx_frame(&len);
        assert(f != NULL);
        assert(((f[0] >> 2) & 0x3) == 2);           /* data frame */
        assert(f[1] & 0x01);                        /* ToDS set */
        assert(!(f[1] & 0x02));                     /* FromDS clear */
        assert(memcmp(f + 4,  AP_MAC, 6) == 0);     /* addr1 = BSSID */
        assert(memcmp(f + 10, STA_MAC, 6) == 0);    /* addr2 = us */
        assert(memcmp(f + 16, PEER_MAC, 6) == 0);   /* addr3 = DA */
        /* LLC/SNAP at offset 24. */
        assert(f[24] == 0xAA && f[25] == 0xAA && f[26] == 0x03);
        assert(f[27] == 0 && f[28] == 0 && f[29] == 0);
        assert(f[30] == 0x08 && f[31] == 0x00);     /* ethertype preserved */
        /* Payload follows at 32. */
        assert(f[32] == 0xA0 && f[33] == 0xA1);
        assert(len == 24 + 8 + 20);
    }
    printf("  TX 802.3 -> 802.11 (ToDS, addrs, LLC/SNAP, payload): PASS\n");

    /* ---- 5. RX: 802.11 downlink in, 802.3 out ---- */
    uint8_t payload[20];
    for (int i = 0; i < 20; i++) payload[i] = (uint8_t)(0xB0 + i);
    uint8_t dl[128];
    uint16_t dl_len = build_data_downlink(dl, AP_MAC, payload, 20, 0x0800);

    uint32_t rx_tail_before = (uint32_t)vwifi_reg_read(g_dev,
                                                       VWIFI_REG_RX_RING_TAIL, 4);
    medium_rx(dl, dl_len, 2437, AP_MAC);
    uint32_t rx_tail_after = (uint32_t)vwifi_reg_read(g_dev,
                                                      VWIFI_REG_RX_RING_TAIL, 4);
    assert(rx_tail_after != rx_tail_before);

    {
        struct vwifi_rx_desc *rd =
            (struct vwifi_rx_desc *)(g_ram_va + RX_OFFSET
                                     + rx_tail_before * sizeof(*rd));
        assert(!(rd->flags & VWIFI_DESC_F_OWN));
        assert(!(rd->flags & VWIFI_RX_F_RAW));       /* STA mode => 802.3 */
        assert(rd->frame_len == 14 + 20);

        const uint8_t *out = g_ram_va + RX_BUFFER_OFFSET
                           + (uint64_t)rx_tail_before * RX_BUFSZ;
        assert(memcmp(out,     STA_MAC, 6) == 0);    /* dst = us */
        assert(memcmp(out + 6, PEER_MAC, 6) == 0);   /* src = peer */
        assert(out[12] == 0x08 && out[13] == 0x00);  /* ethertype */
        assert(out[14] == 0xB0 && out[15] == 0xB1);  /* payload */
    }
    printf("  RX 802.11 -> 802.3 (addrs, ethertype, payload): PASS\n");

    /* ---- 6. Frame from a different BSSID is filtered out ---- */
    uint8_t other_bssid[6] = { 0xDE, 0xAD, 0x00, 0x00, 0x00, 0x99 };
    dl_len = build_data_downlink(dl, other_bssid, payload, 20, 0x0800);
    rx_tail_before = (uint32_t)vwifi_reg_read(g_dev, VWIFI_REG_RX_RING_TAIL, 4);
    medium_rx(dl, dl_len, 2437, other_bssid);
    rx_tail_after = (uint32_t)vwifi_reg_read(g_dev, VWIFI_REG_RX_RING_TAIL, 4);
    assert(rx_tail_before == rx_tail_after);
    printf("  data frame from foreign BSSID filtered: PASS\n");

    /* ---- 7. Deauth from the AP ---- */
    arm_all_rsp_slots();
    aplen = build_deauth(apbuf, 7);
    medium_rx(apbuf, aplen, 2437, AP_MAC);

    {
        struct vwifi_ctrl_rsp_desc *e = find_event(VWIFI_EV_DISCONNECTED);
        assert(e != NULL);
        struct vwifi_disconnect_ev *de = event_payload(e);
        assert(de->reason_code == 7);
        assert(de->local == 0);          /* remotely initiated */
    }
    /* After deauth, TX should be dropped (not associated). */
    tx = (struct vwifi_tx_desc *)(g_ram_va + TX_OFFSET
                                  + (g_tx_slot % RING_ENTRIES) * sizeof(*tx));
    memset(tx, 0, sizeof(*tx));
    tx->frame_addr = g_ram + TX_FRAME_OFFSET;
    tx->frame_len  = eth_len;
    tx->flags      = VWIFI_DESC_F_OWN;
    g_tx_slot++;
    mock_backend_clear_events(g_mock);
    wreg(VWIFI_REG_TX_RING_DOORBELL, g_tx_slot);
    {
        uint16_t len;
        assert(last_tx_frame(&len) == NULL);   /* nothing went out */
    }
    printf("  Deauth -> DISCONNECTED + TX blocked when unassociated: PASS\n");

    /* ---- 8. Connect timeout when the AP never replies ---- */
    arm_all_rsp_slots();
    mock_backend_clear_events(g_mock);
    assert(ctrl_send(VWIFI_OP_CONNECT, connbuf,
                     sizeof(*cr) + cr->assoc_ie_len) == 0);
    assert(mock_backend_timer_armed(g_mock));

    assert(mock_backend_advance_to_timer(g_mock));
    vwifi_timer_expired(g_dev);

    {
        struct vwifi_ctrl_rsp_desc *e = find_event(VWIFI_EV_ASSOC_RESULT);
        assert(e != NULL);
        struct vwifi_assoc_result *ar = event_payload(e);
        assert(ar->status_code == 16);   /* auth sequence timeout */
    }
    printf("  no AP reply -> connect times out with status 16: PASS\n");

    /* ---- 9. A connect with no channel takes it from the BSS table ----
     *
     * wpa_supplicant omits the channel whenever it will let the driver
     * roam, and WDI's connect parameters have no channel field at all,
     * so "the driver always tells us" is not an assumption the device
     * can make. Falling back to the currently tuned channel puts the
     * exchange on whatever the last scan restored. */
    {
        static const uint8_t AP2[6] = { 0xAA, 0xBB, 0xCC, 0x00, 0x00, 0x02 };
        uint8_t beacon[64];
        uint16_t blen = build_beacon(beacon, AP2, "other-ap");
        uint8_t cbuf[sizeof(struct vwifi_connect_req)];
        struct vwifi_connect_req *c2 = (struct vwifi_connect_req *)cbuf;

        /* Park the radio on "unknown" first. A device tuned to 2437
         * from the earlier test would drop a 5220 MHz beacon in its own
         * RX filter, and a fallback to the current channel would then
         * be indistinguishable from the lookup being tested. */
        struct vwifi_channel any = { 0 };
        assert(ctrl_send(VWIFI_OP_SET_CHANNEL, &any, sizeof(any)) == 0);

        /* Heard outside any scan -- the BSS table is populated by the
         * RX path, not by scanning. */
        medium_rx(beacon, blen, 5220, AP2);

        memset(cbuf, 0, sizeof(cbuf));
        memcpy(c2->bssid, AP2, 6);
        c2->ssid_len = (uint16_t)strlen("other-ap");
        memcpy(c2->ssid, "other-ap", c2->ssid_len);
        c2->channel_freq = 0;              /* the case under test */
        c2->auth_algo    = VWIFI_AUTH_OPEN;

        arm_all_rsp_slots();
        mock_backend_clear_events(g_mock);
        assert(ctrl_send(VWIFI_OP_CONNECT, cbuf, sizeof(cbuf)) == 0);
        assert(last_tx_freq() == 5220);
        assert(ctrl_send(VWIFI_OP_DISCONNECT, NULL, 0) == 0);
        printf("  connect w/o channel uses the BSS table's: PASS\n");
    }

    printf("connect: PASS\n");
    free(g_dev);
    mock_backend_free(g_mock);
    return 0;
}
