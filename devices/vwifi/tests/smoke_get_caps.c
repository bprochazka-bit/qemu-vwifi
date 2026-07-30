/*
 * vwifi-virt — smoke test for the portable device
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Constructs a vwifi_dev with the mock backend, wires up 4 rings in
 * fake guest RAM, submits a GET_CAPS control request, and asserts
 * that the response comes back with the expected caps + that an IRQ
 * was raised on VEC_CTRL_RSP.
 *
 * Build:
 *   cc -Wall -Wextra -O2 -I../src -I. \
 *      ../src/vwifi_device.c mock_backend.c smoke_get_caps.c \
 *      -o smoke_get_caps
 * Run: ./smoke_get_caps  (returns 0 on success)
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>

#include "vwifi_abi.h"
#include "vwifi_device.h"
#include "mock_backend.h"

/* Ring layout in fake guest RAM. */
#define CTRL_REQ_OFFSET  0x00000
#define CTRL_REQ_SIZE    16
#define CTRL_RSP_OFFSET  0x01000
#define CTRL_RSP_SIZE    16
#define TX_OFFSET        0x02000
#define TX_SIZE          16
#define RX_OFFSET        0x03000
#define RX_SIZE          16
#define REQ_PAYLOAD_OFFSET  0x10000
#define RSP_PAYLOAD_OFFSET  0x11000

static void write_reg(struct vwifi_dev *d, uint32_t off, uint32_t val)
{
    vwifi_reg_write(d, off, val, 4);
}

static uint32_t read_reg(struct vwifi_dev *d, uint32_t off)
{
    return (uint32_t)vwifi_reg_read(d, off, 4);
}

int main(void)
{
    struct mock_backend *m = mock_backend_new(1 * 1024 * 1024, 128);
    mock_backend_set_quiet(m, true);
    uint64_t ram = mock_backend_ram_base(m);
    uint8_t *ram_va = mock_backend_ram(m);

    uint8_t mac[6] = { 2, 3, 4, 5, 6, 7 };
    struct vwifi_dev *dev = malloc(vwifi_dev_sizeof());
    vwifi_dev_init(dev, mock_backend_ops_get(), m, "test", false, mac);

    /* Sanity: signature reads back correctly before anything else. */
    assert(read_reg(dev, VWIFI_REG_SIGNATURE) == VWIFI_SIGNATURE);
    assert(read_reg(dev, VWIFI_REG_ABI_VERSION) == VWIFI_ABI_VERSION);

    /* Program the four rings, then enable. */
    write_reg(dev, VWIFI_REG_CTRL_REQ_RING_BASE_LO,
              (uint32_t)((ram + CTRL_REQ_OFFSET) & 0xFFFFFFFF));
    write_reg(dev, VWIFI_REG_CTRL_REQ_RING_BASE_HI,
              (uint32_t)((ram + CTRL_REQ_OFFSET) >> 32));
    write_reg(dev, VWIFI_REG_CTRL_REQ_RING_SIZE, CTRL_REQ_SIZE);

    write_reg(dev, VWIFI_REG_CTRL_RSP_RING_BASE_LO,
              (uint32_t)((ram + CTRL_RSP_OFFSET) & 0xFFFFFFFF));
    write_reg(dev, VWIFI_REG_CTRL_RSP_RING_BASE_HI,
              (uint32_t)((ram + CTRL_RSP_OFFSET) >> 32));
    write_reg(dev, VWIFI_REG_CTRL_RSP_RING_SIZE, CTRL_RSP_SIZE);

    write_reg(dev, VWIFI_REG_TX_RING_BASE_LO,
              (uint32_t)((ram + TX_OFFSET) & 0xFFFFFFFF));
    write_reg(dev, VWIFI_REG_TX_RING_BASE_HI,
              (uint32_t)((ram + TX_OFFSET) >> 32));
    write_reg(dev, VWIFI_REG_TX_RING_SIZE, TX_SIZE);

    write_reg(dev, VWIFI_REG_RX_RING_BASE_LO,
              (uint32_t)((ram + RX_OFFSET) & 0xFFFFFFFF));
    write_reg(dev, VWIFI_REG_RX_RING_BASE_HI,
              (uint32_t)((ram + RX_OFFSET) >> 32));
    write_reg(dev, VWIFI_REG_RX_RING_SIZE, RX_SIZE);

    write_reg(dev, VWIFI_REG_CTRL, VWIFI_CTRL_ENABLE | VWIFI_CTRL_IRQ_ENABLE);
    assert(read_reg(dev, VWIFI_REG_STATUS) & VWIFI_STATUS_READY);

    /* Pre-arm the rsp slot 0 so the device has somewhere to write. */
    struct vwifi_ctrl_rsp_desc *rsp0 =
        (struct vwifi_ctrl_rsp_desc *)(ram_va + CTRL_RSP_OFFSET);
    memset(rsp0, 0, sizeof(*rsp0));
    rsp0->payload_addr = ram + RSP_PAYLOAD_OFFSET;
    rsp0->payload_len  = 512;
    rsp0->flags        = VWIFI_DESC_F_OWN;

    /* Build the GET_CAPS request in slot 0. */
    struct vwifi_ctrl_req_desc *req0 =
        (struct vwifi_ctrl_req_desc *)(ram_va + CTRL_REQ_OFFSET);
    memset(req0, 0, sizeof(*req0));
    req0->opcode       = VWIFI_OP_GET_CAPS;
    req0->req_id       = 0xdeadbeef;
    req0->payload_addr = ram + REQ_PAYLOAD_OFFSET;
    req0->payload_len  = 0;
    req0->flags        = VWIFI_DESC_F_OWN;

    mock_backend_clear_events(m);

    /* Ring the doorbell — synchronous dispatch happens inside. */
    write_reg(dev, VWIFI_REG_CTRL_REQ_RING_DOORBELL, 1);

    /* Check the response was written and OWN was cleared. */
    assert(!(rsp0->flags & VWIFI_DESC_F_OWN));
    assert(rsp0->opcode == VWIFI_OP_GET_CAPS);
    assert(rsp0->req_id == 0xdeadbeef);
    assert(rsp0->status == 0);
    assert(rsp0->payload_len == sizeof(struct vwifi_caps));

    /* Check the payload landed at the pointed-to address. */
    struct vwifi_caps *caps =
        (struct vwifi_caps *)(ram_va + RSP_PAYLOAD_OFFSET);
    assert(caps->abi_version == VWIFI_ABI_VERSION);
    assert(caps->caps & VWIFI_CAP_MONITOR);
    assert(caps->caps & VWIFI_CAP_STA);
    assert(memcmp(caps->default_mac, mac, 6) == 0);

    /* Check that an IRQ was raised on the CTRL_RSP vector. */
    size_t n = mock_backend_event_count(m);
    assert(n >= 1);
    const struct mock_event *ev = mock_backend_events(m);
    bool saw_ctrl_rsp_irq = false;
    for (size_t i = 0; i < n; i++) {
        if (ev[i].kind == MOCK_EV_IRQ && ev[i].vec == VWIFI_VEC_CTRL_RSP) {
            saw_ctrl_rsp_irq = true;
            break;
        }
    }
    assert(saw_ctrl_rsp_irq);

    printf("smoke_get_caps: PASS (%zu events observed)\n", n);

    free(dev);
    mock_backend_free(m);
    return 0;
}
