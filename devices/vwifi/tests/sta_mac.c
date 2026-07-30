/*
 * vwifi-virt — the configured MAC, from device property to the air
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * `-device vwifi-virt,mac=...` reaches the guest by exactly one route:
 * the QEMU backend hands it to vwifi_dev_init, the device keeps it as
 * its station MAC, and the guest reads it back out of GET_CAPS to set
 * its netdev address. Nothing else carries it.
 *
 * That makes two things worth pinning, because a guest sees neither
 * until it is too late to tell them apart from a driver bug:
 *
 *   1. the MAC survives the reset sequence a driver performs at probe,
 *      which happens BEFORE it ever asks for caps;
 *   2. frames the device puts on the medium are stamped with it, so a
 *      hub peer list and a guest agree on who this station is.
 *
 * Build/run: part of run_tests.sh.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>

#include "vwifi_abi.h"
#include "vwifi_device.h"
#include "mock_backend.h"

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

/* The medium frame header is 40 bytes and each hub write is prefixed
 * with a 4-byte length; tx_mac sits at offset 8 within the header. */
#define MEDIUM_HDR_OFFSET   4
#define TX_MAC_OFFSET       (MEDIUM_HDR_OFFSET + 8)

static uint64_t g_ram;
static uint8_t *g_ram_va;
static struct vwifi_dev *g_dev;
static struct mock_backend *g_mock;
static uint32_t g_req_slot;

/* The MAC a user configures. Deliberately not the device's built-in
 * fallback, so "it happened to work" and "it was ignored" look
 * different. */
static const uint8_t CONFIGURED[6] = { 0x52, 0x54, 0x00, 0xAB, 0xCD, 0xEF };

static void wreg(uint32_t off, uint32_t val) { vwifi_reg_write(g_dev, off, val, 4); }

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

/* Submit a control request; returns (status, payload of the response). */
static int32_t ctrl_send(uint16_t opcode, const void *payload, uint32_t plen,
                         const void **out_payload, uint32_t *out_len)
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
        if (!(r->flags & VWIFI_DESC_F_OWN) &&
            !(r->flags & VWIFI_RSP_F_EVENT) &&
            r->req_id == 0x3000 + (g_req_slot - 1)) {
            if (out_payload) {
                *out_payload = g_ram_va + RSP_PAYLOAD_OFFSET
                    + (uint64_t)i * RSP_PAYLOAD_STRIDE;
            }
            if (out_len) *out_len = r->payload_len;
            return r->status;
        }
    }
    return -999;
}

static void program_rings(void)
{
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
    /* A reset rewinds the device's consumer indices, so the producer
     * side starts over too — as a driver does, it re-arms rings it has
     * just (re)programmed rather than carrying an index across. */
    memset(g_ram_va + CTRL_REQ_OFFSET, 0,
           RING_ENTRIES * sizeof(struct vwifi_ctrl_req_desc));
    g_req_slot = 0;
    arm_all_rsp_slots();
    wreg(VWIFI_REG_CTRL, VWIFI_CTRL_ENABLE | VWIFI_CTRL_IRQ_ENABLE);
    assert(vwifi_reg_read(g_dev, VWIFI_REG_STATUS, 4) & VWIFI_STATUS_READY);
}

static void caps_mac(uint8_t out[6])
{
    const void *payload = NULL;
    uint32_t len = 0;
    assert(ctrl_send(VWIFI_OP_GET_CAPS, NULL, 0, &payload, &len) == 0);
    assert(len >= sizeof(struct vwifi_caps));
    const struct vwifi_caps *c = payload;
    memcpy(out, c->default_mac, 6);
}

/* The tx_mac the device stamped on the last frame it sent to the hub. */
static bool last_tx_mac(uint8_t out[6])
{
    size_t n = mock_backend_event_count(g_mock);
    const struct mock_event *ev = mock_backend_events(g_mock);
    for (size_t i = n; i-- > 0; ) {
        if (ev[i].kind != MOCK_EV_MEDIUM_TX) continue;
        if (ev[i].medium_len < TX_MAC_OFFSET + 6) continue;
        memcpy(out, ev[i].medium_buf + TX_MAC_OFFSET, 6);
        return true;
    }
    return false;
}

static void expect_mac(const char *what, const uint8_t got[6],
                       const uint8_t want[6])
{
    if (memcmp(got, want, 6) == 0) {
        printf("  %-52s PASS\n", what);
        return;
    }
    printf("  %-52s FAIL\n", what);
    printf("    want %02x:%02x:%02x:%02x:%02x:%02x  "
           "got %02x:%02x:%02x:%02x:%02x:%02x\n",
           want[0], want[1], want[2], want[3], want[4], want[5],
           got[0], got[1], got[2], got[3], got[4], got[5]);
    exit(1);
}

int main(void)
{
    uint8_t mac[6];

    g_mock = mock_backend_new(1 * 1024 * 1024, 256);
    mock_backend_set_quiet(g_mock, true);
    g_ram = mock_backend_ram_base(g_mock);
    g_ram_va = mock_backend_ram(g_mock);

    printf("== vwifi-virt station MAC ==\n");

    g_dev = malloc(vwifi_dev_sizeof());
    vwifi_dev_init(g_dev, mock_backend_ops_get(), g_mock, "mac-test",
                   false, CONFIGURED);
    vwifi_medium_link_event(g_dev, true);

    /*
     * A driver's probe path resets the device before it programs
     * anything: the ring bases are latched on a 0->1 edge of ENABLE, so
     * it cannot trust whatever state it inherited. The configured MAC
     * must not be collateral damage of that — it is not ring state, and
     * the guest has no other way to learn it.
     */
    wreg(VWIFI_REG_CTRL, 0);
    wreg(VWIFI_REG_RESET, 1);

    program_rings();

    caps_mac(mac);
    expect_mac("configured MAC survives probe-time reset", mac, CONFIGURED);

    /* And a reset once the guest is up, e.g. a driver re-bind. */
    wreg(VWIFI_REG_CTRL, VWIFI_CTRL_RESET);
    program_rings();
    caps_mac(mac);
    expect_mac("configured MAC survives a later reset", mac, CONFIGURED);

    /*
     * What the rest of the medium sees. A scan is the earliest thing a
     * station transmits, and its probe requests carry the station MAC
     * as the frame header's tx_mac — which is what the hub's peer list
     * reports.
     */
    struct vwifi_scan_req sr = { 0 };
    sr.channel_mask_24 = (1u << 1);
    sr.dwell_ms        = 50;
    sr.num_ssids       = 0;

    mock_backend_clear_events(g_mock);
    assert(ctrl_send(VWIFI_OP_SCAN, &sr, sizeof(sr), NULL, NULL) == 0);
    assert(last_tx_mac(mac));
    expect_mac("frames on the medium are stamped with it", mac, CONFIGURED);

    /* Land the scan before starting another; a scan in flight is
     * -EBUSY. */
    assert(ctrl_send(VWIFI_OP_SCAN_ABORT, NULL, 0, NULL, NULL) == 0);

    /* The guest can still change the address at runtime
     * (ndo_set_mac_address), and that must reach the air too. */
    const uint8_t changed[6] = { 0x52, 0x54, 0x00, 0x11, 0x22, 0x33 };
    assert(ctrl_send(VWIFI_OP_SET_STA_MAC, changed, 6, NULL, NULL) == 0);
    caps_mac(mac);
    expect_mac("SET_STA_MAC is reflected back in caps", mac, changed);

    mock_backend_clear_events(g_mock);
    assert(ctrl_send(VWIFI_OP_SCAN, &sr, sizeof(sr), NULL, NULL) == 0);
    assert(last_tx_mac(mac));
    expect_mac("SET_STA_MAC reaches the medium", mac, changed);

    printf("\nAll station-MAC tests passed.\n");

    free(g_dev);
    mock_backend_free(g_mock);
    return 0;
}
