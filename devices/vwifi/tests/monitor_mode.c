/*
 * vwifi-virt — monitor mode + injection test
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * 1. Put the device in MONITOR mode via a SET_OP_MODE control request.
 * 2. Feed a beacon (management) frame in through the medium RX path.
 * 3. Assert it lands in the RX ring with VWIFI_RX_F_RAW set and the
 *    right rssi/channel/tsf metadata.
 * 4. Set a raw filter of DATA-only, feed the beacon again, assert it
 *    is dropped (mgmt not wanted).
 * 5. Inject a frame via the TX ring with VWIFI_TX_F_INJECT and assert
 *    the medium send used the frame's addr2 as tx_mac, not the STA MAC.
 *
 * Build:
 *   cc -Wall -Wextra -O2 -I../src -I. \
 *      ../src/vwifi_device.c mock_backend.c monitor_mode.c -o monitor_mode
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
#define RSP_PAYLOAD_OFFSET  0x11000
#define RX_BUFFER_OFFSET    0x20000
#define TX_FRAME_OFFSET     0x30000
#define RING_ENTRIES        16
#define RX_BUFSZ            4096

static uint64_t g_ram;
static uint8_t *g_ram_va;
static struct vwifi_dev *g_dev;
static uint32_t g_req_slot;
static uint32_t g_rsp_slot;

static void wreg(uint32_t off, uint32_t val) { vwifi_reg_write(g_dev, off, val, 4); }
static uint32_t rreg(uint32_t off) { return (uint32_t)vwifi_reg_read(g_dev, off, 4); }

/* Submit a control request synchronously (doorbell triggers dispatch). */
static int32_t ctrl_send(uint16_t opcode, const void *payload, uint32_t plen)
{
    uint32_t slot = g_req_slot % RING_ENTRIES;

    /* Pre-arm matching rsp slot. */
    struct vwifi_ctrl_rsp_desc *rsp =
        (struct vwifi_ctrl_rsp_desc *)(g_ram_va + CTRL_RSP_OFFSET
                                       + g_rsp_slot % RING_ENTRIES * sizeof(*rsp));
    memset(rsp, 0, sizeof(*rsp));
    rsp->payload_addr = g_ram + RSP_PAYLOAD_OFFSET;
    rsp->payload_len  = 512;
    rsp->flags        = VWIFI_DESC_F_OWN;

    if (payload && plen) {
        memcpy(g_ram_va + REQ_PAYLOAD_OFFSET, payload, plen);
    }

    struct vwifi_ctrl_req_desc *req =
        (struct vwifi_ctrl_req_desc *)(g_ram_va + CTRL_REQ_OFFSET
                                       + slot * sizeof(*req));
    memset(req, 0, sizeof(*req));
    req->opcode       = opcode;
    req->req_id       = 0x1000 + g_req_slot;
    req->payload_addr = g_ram + REQ_PAYLOAD_OFFSET;
    req->payload_len  = plen;
    req->flags        = VWIFI_DESC_F_OWN;

    wreg(VWIFI_REG_CTRL_REQ_RING_DOORBELL, g_req_slot + 1);

    int32_t status = rsp->status;
    g_req_slot++;
    g_rsp_slot++;
    return status;
}

/* Build a length-prefixed medium message carrying one 802.11 frame and
 * push it through the RX path. */
static void medium_inject_rx(const uint8_t *frame, uint16_t frame_len,
                             uint16_t channel_freq, int8_t rssi,
                             const uint8_t tx_mac[6])
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
    hdr.tsf_lo       = 0x11223344;
    hdr.tsf_hi       = 0x55667788;
    hdr.channel_freq = channel_freq;

    memcpy(msg + off, &len_be, 4);        off += 4;
    memcpy(msg + off, &hdr, sizeof(hdr));  off += sizeof(hdr);
    memcpy(msg + off, frame, frame_len);   off += frame_len;

    vwifi_medium_rx_bytes(g_dev, msg, off);
}

int main(void)
{
    struct mock_backend *m = mock_backend_new(2 * 1024 * 1024, 256);
    mock_backend_set_quiet(m, true);
    g_ram = mock_backend_ram_base(m);
    g_ram_va = mock_backend_ram(m);

    uint8_t sta_mac[6] = { 0x00, 0x03, 0x7F, 0xCC, 0xDD, 0x10 };
    g_dev = malloc(vwifi_dev_sizeof());
    vwifi_dev_init(g_dev, mock_backend_ops_get(), m, "mon-test", false, sta_mac);

    /* Simulate the hub socket connecting — sends hello, sets LINK_UP.
     * The real QEMU/vfio-user backend does this on CHR_EVENT_OPENED. */
    vwifi_medium_link_event(g_dev, true);
    assert(rreg(VWIFI_REG_STATUS) & VWIFI_STATUS_LINK_UP);

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

    /* Post RX buffers. */
    for (int i = 0; i < RING_ENTRIES; i++) {
        struct vwifi_rx_desc *d =
            (struct vwifi_rx_desc *)(g_ram_va + RX_OFFSET + i * sizeof(*d));
        memset(d, 0, sizeof(*d));
        d->frame_addr = g_ram + RX_BUFFER_OFFSET + (uint64_t)i * RX_BUFSZ;
        d->buffer_len = RX_BUFSZ;
        d->flags      = VWIFI_DESC_F_OWN;
    }

    wreg(VWIFI_REG_CTRL, VWIFI_CTRL_ENABLE | VWIFI_CTRL_IRQ_ENABLE);
    assert(rreg(VWIFI_REG_STATUS) & VWIFI_STATUS_READY);

    /* --- Set channel 6 (2437 MHz) --- */
    struct vwifi_channel ch = { .primary_freq = 2437 };
    assert(ctrl_send(VWIFI_OP_SET_CHANNEL, &ch, sizeof(ch)) == 0);

    /* --- Enter MONITOR mode --- */
    struct vwifi_op_mode om = { .mode = VWIFI_MODE_MONITOR };
    assert(ctrl_send(VWIFI_OP_SET_OP_MODE, &om, sizeof(om)) == 0);

    /* --- Build a beacon (mgmt, subtype 8) frame --- */
    uint8_t ap_mac[6] = { 0xAA, 0xBB, 0xCC, 0x11, 0x22, 0x33 };
    uint8_t beacon[64];
    memset(beacon, 0, sizeof(beacon));
    beacon[0] = 0x80;                     /* type=mgmt(0) subtype=beacon(8) */
    memset(beacon + 4, 0xFF, 6);          /* addr1 = broadcast */
    memcpy(beacon + 10, ap_mac, 6);       /* addr2 = AP */
    memcpy(beacon + 16, ap_mac, 6);       /* addr3 = BSSID */

    mock_backend_clear_events(m);

    /* Feed it in on channel 6. */
    uint8_t other_tx[6] = { 0xAA, 0xBB, 0xCC, 0x11, 0x22, 0x33 };
    medium_inject_rx(beacon, sizeof(beacon), 2437, -42, other_tx);

    /* Check RX ring slot 0 got the frame with RAW flag. */
    struct vwifi_rx_desc *rx0 =
        (struct vwifi_rx_desc *)(g_ram_va + RX_OFFSET);
    assert(!(rx0->flags & VWIFI_DESC_F_OWN));   /* device gave it to driver */
    assert(rx0->flags & VWIFI_RX_F_RAW);        /* monitor mode => raw */
    assert(rx0->frame_len == sizeof(beacon));
    assert(rx0->rssi == -42);
    assert(rx0->channel_freq == 2437);
    assert(rx0->tsf == 0x5566778811223344ULL);

    /* Verify the frame bytes landed in the buffer. */
    uint8_t *rxbuf = g_ram_va + RX_BUFFER_OFFSET;
    assert(memcmp(rxbuf, beacon, sizeof(beacon)) == 0);

    /* Verify an RX IRQ fired. */
    bool saw_rx_irq = false;
    for (size_t i = 0; i < mock_backend_event_count(m); i++) {
        const struct mock_event *e = &mock_backend_events(m)[i];
        if (e->kind == MOCK_EV_IRQ && e->vec == VWIFI_VEC_RX) saw_rx_irq = true;
    }
    assert(saw_rx_irq);
    printf("  monitor RX raw indication: PASS\n");

    /* --- Raw filter = DATA only; beacon (mgmt) should now drop --- */
    struct vwifi_raw_filter rf = { .mask = VWIFI_RAW_F_DATA };
    assert(ctrl_send(VWIFI_OP_SET_RAW_FILTER, &rf, sizeof(rf)) == 0);

    uint32_t rx_tail_before = rreg(VWIFI_REG_RX_RING_TAIL);
    medium_inject_rx(beacon, sizeof(beacon), 2437, -42, other_tx);
    uint32_t rx_tail_after = rreg(VWIFI_REG_RX_RING_TAIL);
    assert(rx_tail_before == rx_tail_after);   /* dropped, tail unmoved */
    printf("  raw filter gating (mgmt dropped when DATA-only): PASS\n");

    /* --- Injection: send a frame via TX ring with INJECT flag --- */
    uint8_t inj_mac[6] = { 0xDE, 0xAD, 0xBE, 0xEF, 0x00, 0x01 };
    uint8_t probe[32];
    memset(probe, 0, sizeof(probe));
    probe[0] = 0x40;                    /* type=mgmt subtype=probe-req(4) */
    memset(probe + 4, 0xFF, 6);         /* addr1 = broadcast */
    memcpy(probe + 10, inj_mac, 6);     /* addr2 = our injected source */
    memcpy(probe + 16, inj_mac, 6);     /* addr3 */

    memcpy(g_ram_va + TX_FRAME_OFFSET, probe, sizeof(probe));

    struct vwifi_tx_desc *tx0 =
        (struct vwifi_tx_desc *)(g_ram_va + TX_OFFSET);
    memset(tx0, 0, sizeof(*tx0));
    tx0->frame_addr = g_ram + TX_FRAME_OFFSET;
    tx0->frame_len  = sizeof(probe);
    tx0->flags      = VWIFI_DESC_F_OWN | VWIFI_TX_F_INJECT;

    mock_backend_clear_events(m);
    wreg(VWIFI_REG_TX_RING_DOORBELL, 1);

    /* Find the medium-tx event and check the tx_mac in the header. */
    bool found_inject = false;
    for (size_t i = 0; i < mock_backend_event_count(m); i++) {
        const struct mock_event *e = &mock_backend_events(m)[i];
        if (e->kind != MOCK_EV_MEDIUM_TX) continue;
        /* Layout: [4 len][40 hdr][frame]. tx_mac is at hdr offset 8. */
        const uint8_t *hdr = e->medium_buf + 4;
        const uint8_t *txm = hdr + 8;
        assert(memcmp(txm, inj_mac, 6) == 0);   /* used addr2, not STA MAC */
        assert(memcmp(txm, sta_mac, 6) != 0);
        found_inject = true;
    }
    assert(found_inject);
    printf("  injection uses frame addr2 as tx_mac: PASS\n");

    printf("monitor_mode: PASS\n");
    free(g_dev);
    mock_backend_free(m);
    return 0;
}
