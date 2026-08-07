/*
 * vwifi-virt — WPA2 / CCMP integration test
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Exercises the full WPA2 data path through the device:
 *
 *  1. Associate (open auth + assoc, as WPA2 does — the RSN handshake
 *     happens afterwards over EAPOL).
 *  2. Before keys: an EAPOL frame goes out UNENCRYPTED. This is the
 *     rule that makes the 4-way handshake possible at all.
 *  3. Before keys: a normal data frame also goes out unencrypted
 *     (nothing to encrypt with yet).
 *  4. Install the PTK via SET_KEY; verify KEY_INSTALLED.
 *  5. After keys: a data frame is CCMP-encrypted, and decrypting it
 *     with the same key recovers the exact original payload — this is
 *     what a real hostapd on the medium would do.
 *  6. After keys: EAPOL STILL goes out unencrypted (rekey frames).
 *  7. RX: an AP-encrypted frame is decrypted and delivered as 802.3.
 *  8. Replay: the same frame fed twice is rejected the second time.
 *  9. Forgery: a frame with a bad MIC is dropped.
 * 10. Group key: a broadcast frame under the GTK decrypts.
 * 11. Disconnect clears the keys.
 *
 * Build:
 *   cc -Wall -Wextra -O2 -I../src -I. ../src/vwifi_device.c \
 *      ../src/vwifi_crypto.c mock_backend.c wpa2.c -o wpa2
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <arpa/inet.h>

#include "vwifi_abi.h"
#include "vwifi_device.h"
#include "vwifi_crypto.h"
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
#define MEDIUM_HDR_LEN      40

static uint64_t g_ram;
static uint8_t *g_ram_va;
static struct vwifi_dev *g_dev;
static struct mock_backend *g_mock;
static uint32_t g_req_slot, g_tx_slot;

static const uint8_t STA_MAC[6]  = { 0x00,0x03,0x7F,0xCC,0xDD,0x10 };
static const uint8_t AP_MAC[6]   = { 0xAA,0xBB,0xCC,0x00,0x00,0x01 };
static const uint8_t PEER_MAC[6] = { 0x11,0x22,0x33,0x44,0x55,0x66 };
static const uint8_t BCAST[6]    = { 0xFF,0xFF,0xFF,0xFF,0xFF,0xFF };

/* The PTK/GTK a real 4-way handshake would derive. */
static const uint8_t PTK[16] = {
    0x11,0x22,0x33,0x44,0x55,0x66,0x77,0x88,
    0x99,0xAA,0xBB,0xCC,0xDD,0xEE,0xFF,0x00
};
static const uint8_t GTK[16] = {
    0xA0,0xA1,0xA2,0xA3,0xA4,0xA5,0xA6,0xA7,
    0xA8,0xA9,0xAA,0xAB,0xAC,0xAD,0xAE,0xAF
};

static void wreg(uint32_t off, uint32_t val) { vwifi_reg_write(g_dev, off, val, 4); }
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
    req->req_id = 0x4000 + g_req_slot;
    req->payload_addr = g_ram + REQ_PAYLOAD_OFFSET;
    req->payload_len = plen;
    req->flags = VWIFI_DESC_F_OWN;

    g_req_slot++;
    wreg(VWIFI_REG_CTRL_REQ_RING_DOORBELL, g_req_slot);

    for (uint32_t i = 0; i < RING_ENTRIES; i++) {
        struct vwifi_ctrl_rsp_desc *r =
            (struct vwifi_ctrl_rsp_desc *)(g_ram_va + CTRL_RSP_OFFSET + i * sizeof(*r));
        if (!(r->flags & VWIFI_DESC_F_OWN) && !(r->flags & VWIFI_RSP_F_EVENT) &&
            r->req_id == 0x4000 + (g_req_slot - 1)) return r->status;
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

static void medium_rx(const uint8_t *frame, uint16_t frame_len)
{
    uint8_t msg[4 + MEDIUM_HDR_LEN + 1024];
    struct vwifi_frame_hdr hdr;
    uint32_t plen = sizeof(hdr) + frame_len;
    uint32_t len_be = htonl(plen);
    uint32_t off = 0;

    memset(&hdr, 0, sizeof(hdr));
    hdr.magic = VWIFI_MAGIC;
    hdr.version = VWIFI_VERSION;
    hdr.frame_len = frame_len;
    memcpy(hdr.tx_mac, AP_MAC, 6);
    hdr.rate_code = 0x0B;
    hdr.rssi = -50;
    hdr.channel_freq = 2437;

    memcpy(msg + off, &len_be, 4);        off += 4;
    memcpy(msg + off, &hdr, sizeof(hdr)); off += sizeof(hdr);
    memcpy(msg + off, frame, frame_len);  off += frame_len;
    vwifi_medium_rx_bytes(g_dev, msg, off);
}

static const uint8_t *last_tx_frame(uint16_t *len_out)
{
    const struct mock_event *found = NULL;
    for (size_t i = 0; i < mock_backend_event_count(g_mock); i++) {
        const struct mock_event *e = &mock_backend_events(g_mock)[i];
        if (e->kind == MOCK_EV_MEDIUM_TX && e->medium_len > 4 + MEDIUM_HDR_LEN)
            found = e;
    }
    if (!found) { *len_out = 0; return NULL; }
    *len_out = (uint16_t)(found->medium_len - 4 - MEDIUM_HDR_LEN);
    return found->medium_buf + 4 + MEDIUM_HDR_LEN;
}

/* Push an 802.3 frame down the TX ring. */
static void tx_8023(const uint8_t *dst, uint16_t ethertype,
                    const uint8_t *payload, uint16_t plen)
{
    uint8_t eth[512];
    memcpy(eth, dst, 6);
    memcpy(eth + 6, STA_MAC, 6);
    eth[12] = (uint8_t)(ethertype >> 8);
    eth[13] = (uint8_t)(ethertype & 0xFF);
    memcpy(eth + 14, payload, plen);

    memcpy(g_ram_va + TX_FRAME_OFFSET, eth, 14 + plen);
    struct vwifi_tx_desc *tx =
        (struct vwifi_tx_desc *)(g_ram_va + TX_OFFSET
                                 + (g_tx_slot % RING_ENTRIES) * sizeof(*tx));
    memset(tx, 0, sizeof(*tx));
    tx->frame_addr = g_ram + TX_FRAME_OFFSET;
    tx->frame_len  = 14 + plen;
    tx->flags      = VWIFI_DESC_F_OWN;
    g_tx_slot++;
    mock_backend_clear_events(g_mock);
    wreg(VWIFI_REG_TX_RING_DOORBELL, g_tx_slot);
}

/* Build a downlink 802.11 data frame and CCMP-encrypt it as the AP
 * would, using the given key + PN. */
static uint16_t ap_build_encrypted(uint8_t *buf, const uint8_t *key,
                                   const uint8_t pn[6], uint8_t key_id,
                                   const uint8_t *dst,
                                   const uint8_t *payload, uint16_t plen)
{
    struct vwifi_aes128 aes;
    uint16_t len = 0;
    int enc;

    memset(buf, 0, 32);
    buf[0] = 0x08;                  /* data */
    buf[1] = 0x02;                  /* FromDS */
    memcpy(buf + 4,  dst, 6);       /* addr1 = DA */
    memcpy(buf + 10, AP_MAC, 6);    /* addr2 = BSSID */
    memcpy(buf + 16, PEER_MAC, 6);  /* addr3 = SA */
    len = 24;
    /* LLC/SNAP + IPv4 ethertype. */
    buf[len++] = 0xAA; buf[len++] = 0xAA; buf[len++] = 0x03;
    buf[len++] = 0x00; buf[len++] = 0x00; buf[len++] = 0x00;
    buf[len++] = 0x08; buf[len++] = 0x00;
    memcpy(buf + len, payload, plen);
    len += plen;

    vwifi_aes128_key_expand(key, &aes);
    enc = vwifi_ccmp_encrypt(&aes, buf, len, pn, key_id, 512);
    assert(enc > 0);
    return (uint16_t)enc;
}

static void associate(void)
{
    uint8_t connbuf[sizeof(struct vwifi_connect_req)];
    struct vwifi_connect_req *cr = (struct vwifi_connect_req *)connbuf;
    uint8_t ap[128];
    uint16_t len;

    memset(connbuf, 0, sizeof(connbuf));
    memcpy(cr->bssid, AP_MAC, 6);
    cr->ssid_len = (uint16_t)strlen("vwifi-wpa2");
    memcpy(cr->ssid, "vwifi-wpa2", cr->ssid_len);
    cr->channel_freq    = 2437;
    cr->auth_algo       = VWIFI_AUTH_OPEN;   /* WPA2 uses open auth */
    cr->cipher_pairwise = VWIFI_CIPHER_CCMP128;
    cr->cipher_group    = VWIFI_CIPHER_CCMP128;
    cr->akm_suite       = VWIFI_AKM_PSK;
    assert(ctrl_send(VWIFI_OP_CONNECT, cr, sizeof(*cr)) == 0);

    /* Auth Response. */
    memset(ap, 0, sizeof(ap));
    ap[0] = (11 << 4);
    memcpy(ap + 4, STA_MAC, 6); memcpy(ap + 10, AP_MAC, 6); memcpy(ap + 16, AP_MAC, 6);
    put16(ap + 24, 0); put16(ap + 26, 2); put16(ap + 28, 0);
    medium_rx(ap, 30);

    /* Assoc Response. */
    memset(ap, 0, sizeof(ap));
    ap[0] = (1 << 4);
    memcpy(ap + 4, STA_MAC, 6); memcpy(ap + 10, AP_MAC, 6); memcpy(ap + 16, AP_MAC, 6);
    put16(ap + 24, 0x0431); put16(ap + 26, 0); put16(ap + 28, 1 | 0xC000);
    len = 30;
    medium_rx(ap, len);
}

int main(void)
{
    g_mock = mock_backend_new(4 * 1024 * 1024, 512);
    mock_backend_set_quiet(g_mock, true);
    g_ram = mock_backend_ram_base(g_mock);
    g_ram_va = mock_backend_ram(g_mock);

    g_dev = malloc(vwifi_dev_sizeof());
    vwifi_dev_init(g_dev, mock_backend_ops_get(), g_mock, "wpa2-test", false, STA_MAC);
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

    struct vwifi_op_mode om = { .mode = VWIFI_MODE_STA };
    assert(ctrl_send(VWIFI_OP_SET_OP_MODE, &om, sizeof(om)) == 0);
    arm_all_rsp_slots();

    /* ---- 1. Associate ---- */
    associate();
    assert(find_event(VWIFI_EV_ASSOC_RESULT) != NULL);
    arm_all_rsp_slots();
    printf("  associated (open auth, WPA2 pending handshake): PASS\n");

    /* ---- 1b. No channel hopping while the handshake is in flight ----
     *
     * A scan sweeps the radio across thirteen channels; the AP's EAPOL
     * goes out on one of them. A Windows guest lost its M1 exactly this
     * way -- wlansvc asked for a scan 0 ms after the connect, the
     * device took it because the association was already over, and the
     * frame arrived with the station three channels away. Refusing it
     * with -EBUSY is what the driver's existing deferral expects. */
    {
        struct vwifi_scan_req sr = { 0 };
        sr.channel_mask_24 = 0x3FFE;
        sr.dwell_ms        = 100;
        assert(ctrl_send(VWIFI_OP_SCAN, &sr, sizeof(sr)) == -16);
        arm_all_rsp_slots();
    }
    printf("  scan refused while the 4-way handshake is pending: PASS\n");

    /* ---- 2. EAPOL before keys must be unencrypted ---- */
    uint8_t eapol[64];
    memset(eapol, 0, sizeof(eapol));
    eapol[0] = 0x02;   /* EAPOL version/type-ish; contents don't matter here */
    for (int i = 0; i < 40; i++) eapol[i] = (uint8_t)(0x10 + i);

    tx_8023(AP_MAC, 0x888E, eapol, 40);
    {
        uint16_t len;
        const uint8_t *f = last_tx_frame(&len);
        assert(f != NULL);
        assert(!(f[1] & 0x40));                 /* Protected bit CLEAR */
        /* LLC/SNAP + EAPOL ethertype should be readable in the clear. */
        assert(f[24] == 0xAA && f[25] == 0xAA && f[26] == 0x03);
        assert(f[30] == 0x88 && f[31] == 0x8E); /* ethertype 0x888E */
        assert(memcmp(f + 32, eapol, 40) == 0); /* payload in the clear */
    }
    printf("  EAPOL before keys sent unencrypted: PASS\n");

    /* ---- 3. Data before keys is also unencrypted ---- */
    uint8_t data[32];
    for (int i = 0; i < 32; i++) data[i] = (uint8_t)(0xC0 + i);
    tx_8023(PEER_MAC, 0x0800, data, 32);
    {
        uint16_t len;
        const uint8_t *f = last_tx_frame(&len);
        assert(f != NULL);
        assert(!(f[1] & 0x40));
    }
    printf("  data before keys sent unencrypted: PASS\n");

    /* ---- 4. Install the PTK (what the 4-way handshake produces) ---- */
    struct vwifi_key k = { 0 };
    k.id.pairwise = 1;
    k.id.key_idx  = 0;
    memcpy(k.id.mac, AP_MAC, 6);
    k.cipher  = VWIFI_CIPHER_CCMP128;
    k.key_len = 16;
    memcpy(k.key, PTK, 16);
    assert(ctrl_send(VWIFI_OP_SET_KEY, &k, sizeof(k)) == 0);
    assert(find_event(VWIFI_EV_KEY_INSTALLED) != NULL);
    arm_all_rsp_slots();
    printf("  PTK installed -> KEY_INSTALLED event: PASS\n");

    /* ---- 4b. And with the key in, the radio is free to sweep again.
     * The refusal above has to be a window, not a new permanent rule --
     * a station that can never scan while associated can never roam. */
    {
        struct vwifi_scan_req sr = { 0 };
        sr.channel_mask_24 = 0x3FFE;
        sr.dwell_ms        = 100;
        assert(ctrl_send(VWIFI_OP_SCAN, &sr, sizeof(sr)) == 0);
        assert(ctrl_send(VWIFI_OP_SCAN_ABORT, NULL, 0) == 0);
        arm_all_rsp_slots();
    }
    printf("  scan allowed once the pairwise key is installed: PASS\n");

    /* ---- 5. Data after keys is CCMP-encrypted and the AP can
     *          decrypt it back to the exact original payload ---- */
    tx_8023(PEER_MAC, 0x0800, data, 32);
    {
        uint16_t len;
        const uint8_t *f = last_tx_frame(&len);
        assert(f != NULL);
        assert(f[1] & 0x40);                    /* Protected bit SET */
        assert(f[24 + 3] & 0x20);               /* ExtIV in CCMP header */
        /* Payload must not be readable. */
        assert(memcmp(f + 24 + 8 + 8, data, 24) != 0);

        /* Now decrypt exactly as hostapd would. */
        uint8_t copy[512];
        struct vwifi_aes128 aes;
        memcpy(copy, f, len);
        vwifi_aes128_key_expand(PTK, &aes);
        int dlen = vwifi_ccmp_decrypt(&aes, copy, len, NULL, NULL);
        assert(dlen > 0);
        /* After decryption: 24-byte hdr, LLC/SNAP, then our payload. */
        assert(copy[24] == 0xAA && copy[25] == 0xAA && copy[26] == 0x03);
        assert(copy[30] == 0x08 && copy[31] == 0x00);
        assert(memcmp(copy + 32, data, 32) == 0);
        assert(dlen == 24 + 8 + 32);
    }
    printf("  data after keys: CCMP-encrypted, AP decrypts payload exactly: PASS\n");

    /* ---- 6. EAPOL after keys STILL unencrypted (rekey path) ---- */
    tx_8023(AP_MAC, 0x888E, eapol, 40);
    {
        uint16_t len;
        const uint8_t *f = last_tx_frame(&len);
        assert(f != NULL);
        assert(!(f[1] & 0x40));
        assert(f[30] == 0x88 && f[31] == 0x8E);
    }
    printf("  EAPOL after keys still unencrypted (rekey): PASS\n");

    /* ---- 7. RX: AP-encrypted frame decrypts to 802.3 ---- */
    uint8_t rxpayload[24];
    for (int i = 0; i < 24; i++) rxpayload[i] = (uint8_t)(0xD0 + i);
    uint8_t apframe[512];
    uint8_t ap_pn[6] = { 0,0,0,0,0,1 };
    uint16_t aplen = ap_build_encrypted(apframe, PTK, ap_pn, 0,
                                        STA_MAC, rxpayload, 24);

    uint32_t tail0 = (uint32_t)vwifi_reg_read(g_dev, VWIFI_REG_RX_RING_TAIL, 4);
    medium_rx(apframe, aplen);
    uint32_t tail1 = (uint32_t)vwifi_reg_read(g_dev, VWIFI_REG_RX_RING_TAIL, 4);
    assert(tail1 != tail0);
    {
        struct vwifi_rx_desc *rd =
            (struct vwifi_rx_desc *)(g_ram_va + RX_OFFSET + tail0 * sizeof(*rd));
        assert(rd->flags & VWIFI_RX_F_DECRYPTED);
        assert(rd->frame_len == 14 + 24);
        const uint8_t *out = g_ram_va + RX_BUFFER_OFFSET + (uint64_t)tail0 * RX_BUFSZ;
        assert(memcmp(out, STA_MAC, 6) == 0);
        assert(memcmp(out + 6, PEER_MAC, 6) == 0);
        assert(out[12] == 0x08 && out[13] == 0x00);
        assert(memcmp(out + 14, rxpayload, 24) == 0);
    }
    printf("  RX: AP-encrypted frame decrypted to correct 802.3: PASS\n");

    /* ---- 8. Replay: same frame again is rejected ---- */
    tail0 = (uint32_t)vwifi_reg_read(g_dev, VWIFI_REG_RX_RING_TAIL, 4);
    medium_rx(apframe, aplen);      /* identical PN */
    tail1 = (uint32_t)vwifi_reg_read(g_dev, VWIFI_REG_RX_RING_TAIL, 4);
    assert(tail0 == tail1);
    printf("  replayed frame (same PN) rejected: PASS\n");

    /* A higher PN is accepted, proving the counter advances not blocks. */
    uint8_t ap_pn2[6] = { 0,0,0,0,0,2 };
    aplen = ap_build_encrypted(apframe, PTK, ap_pn2, 0, STA_MAC, rxpayload, 24);
    tail0 = (uint32_t)vwifi_reg_read(g_dev, VWIFI_REG_RX_RING_TAIL, 4);
    medium_rx(apframe, aplen);
    tail1 = (uint32_t)vwifi_reg_read(g_dev, VWIFI_REG_RX_RING_TAIL, 4);
    assert(tail0 != tail1);
    printf("  higher PN accepted after replay rejection: PASS\n");

    /* ---- 9. Forged frame (bad MIC) dropped ---- */
    uint8_t ap_pn3[6] = { 0,0,0,0,0,9 };
    aplen = ap_build_encrypted(apframe, PTK, ap_pn3, 0, STA_MAC, rxpayload, 24);
    apframe[aplen - 1] ^= 0xFF;     /* corrupt the MIC */
    tail0 = (uint32_t)vwifi_reg_read(g_dev, VWIFI_REG_RX_RING_TAIL, 4);
    medium_rx(apframe, aplen);
    tail1 = (uint32_t)vwifi_reg_read(g_dev, VWIFI_REG_RX_RING_TAIL, 4);
    assert(tail0 == tail1);
    printf("  forged frame (bad MIC) dropped: PASS\n");

    /* A forged frame must NOT have poisoned the replay counter —
     * a legitimate frame at PN 10 must still be accepted. */
    uint8_t ap_pn4[6] = { 0,0,0,0,0,10 };
    aplen = ap_build_encrypted(apframe, PTK, ap_pn4, 0, STA_MAC, rxpayload, 24);
    tail0 = (uint32_t)vwifi_reg_read(g_dev, VWIFI_REG_RX_RING_TAIL, 4);
    medium_rx(apframe, aplen);
    tail1 = (uint32_t)vwifi_reg_read(g_dev, VWIFI_REG_RX_RING_TAIL, 4);
    assert(tail0 != tail1);
    printf("  forged frame did not poison the replay counter: PASS\n");

    /* ---- 10. Group key: broadcast under the GTK ---- */
    struct vwifi_key gk = { 0 };
    gk.id.pairwise = 0;
    gk.id.key_idx  = 1;
    gk.cipher  = VWIFI_CIPHER_CCMP128;
    gk.key_len = 16;
    memcpy(gk.key, GTK, 16);
    arm_all_rsp_slots();
    assert(ctrl_send(VWIFI_OP_SET_KEY, &gk, sizeof(gk)) == 0);

    uint8_t gpn[6] = { 0,0,0,0,0,1 };
    aplen = ap_build_encrypted(apframe, GTK, gpn, 1, BCAST, rxpayload, 24);
    tail0 = (uint32_t)vwifi_reg_read(g_dev, VWIFI_REG_RX_RING_TAIL, 4);
    medium_rx(apframe, aplen);
    tail1 = (uint32_t)vwifi_reg_read(g_dev, VWIFI_REG_RX_RING_TAIL, 4);
    assert(tail0 != tail1);
    {
        struct vwifi_rx_desc *rd =
            (struct vwifi_rx_desc *)(g_ram_va + RX_OFFSET + tail0 * sizeof(*rd));
        assert(rd->flags & VWIFI_RX_F_DECRYPTED);
        const uint8_t *out = g_ram_va + RX_BUFFER_OFFSET + (uint64_t)tail0 * RX_BUFSZ;
        assert(memcmp(out, BCAST, 6) == 0);
        assert(memcmp(out + 14, rxpayload, 24) == 0);
    }
    printf("  broadcast frame under GTK (key id 1) decrypted: PASS\n");

    /* ---- 11. Disconnect clears keys ---- */
    struct vwifi_disconnect_req dr = { .reason_code = 3 };
    arm_all_rsp_slots();
    assert(ctrl_send(VWIFI_OP_DISCONNECT, &dr, sizeof(dr)) == 0);

    /* Re-associate and confirm TX is unencrypted again — the old PTK
     * must be gone, not silently reused against a new association. */
    arm_all_rsp_slots();
    associate();
    arm_all_rsp_slots();
    tx_8023(PEER_MAC, 0x0800, data, 32);
    {
        uint16_t len;
        const uint8_t *f = last_tx_frame(&len);
        assert(f != NULL);
        assert(!(f[1] & 0x40));    /* no stale key */
    }
    printf("  disconnect clears keys (no stale PTK reuse): PASS\n");

    printf("wpa2: PASS\n");
    free(g_dev);
    mock_backend_free(g_mock);
    return 0;
}
