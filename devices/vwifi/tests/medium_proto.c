/*
 * vwifi-virt — medium wire-protocol conformance test
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * This device shares one medium with the hub, the Linux host module and
 * the vwifi-ath9k device. All of them agree on abi/vwifi.h, and a
 * disagreement is invisible at runtime: frames are silently malformed,
 * every peer drops them, and nothing logs an error anywhere. This device
 * was developed against a hand-transcribed stub of that header and got
 * both the hello magic and its byte order wrong, so the hub would never
 * have registered it. These assertions are the guard against a repeat.
 *
 * Covers:
 *   - struct vwifi_frame_hdr size and every field offset, against the
 *     layout table in abi/vwifi.h
 *   - the hello handshake byte-for-byte: big-endian length prefix,
 *     HOST-endian magic, NUL-terminated node id, no flags byte
 *   - a transmitted frame's medium header: magic, version, length,
 *     source MAC, channel, and the frame following it unmodified
 *
 * Build:
 *   cc -Wall -Wextra -O2 -I../src -I../../../abi -I. \
 *      ../src/vwifi_device.c ../src/vwifi_crypto.c mock_backend.c \
 *      medium_proto.c -o medium_proto
 * Run: ./medium_proto  (returns 0 on success)
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stddef.h>
#include <arpa/inet.h>

#include "vwifi_abi.h"
#include "vwifi_device.h"
#include "vwifi.h"
#include "mock_backend.h"

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
#define TEST_CHANNEL_FREQ   2437        /* channel 6 */

#define CHECK(cond, name)                                          \
    do {                                                           \
        printf("  %-58s %s\n", (name), (cond) ? "PASS" : "FAIL");  \
        if (!(cond)) failures++;                                   \
    } while (0)

static int failures;

static uint64_t g_ram;
static uint8_t *g_ram_va;
static struct vwifi_dev *g_dev;
static uint32_t g_req_slot;
static uint32_t g_rsp_slot;

static void wreg(uint32_t off, uint32_t val) { vwifi_reg_write(g_dev, off, val, 4); }

/* Submit a control request synchronously (the doorbell drives dispatch). */
static int32_t ctrl_send(uint16_t opcode, const void *payload, uint32_t plen)
{
    uint32_t slot = g_req_slot % RING_ENTRIES;
    struct vwifi_ctrl_rsp_desc *rsp =
        (struct vwifi_ctrl_rsp_desc *)(g_ram_va + CTRL_RSP_OFFSET
                                       + g_rsp_slot % RING_ENTRIES * sizeof(*rsp));
    struct vwifi_ctrl_req_desc *req;
    int32_t status;

    memset(rsp, 0, sizeof(*rsp));
    rsp->payload_addr = g_ram + RSP_PAYLOAD_OFFSET;
    rsp->payload_len  = 512;
    rsp->flags        = VWIFI_DESC_F_OWN;

    if (payload && plen) {
        memcpy(g_ram_va + REQ_PAYLOAD_OFFSET, payload, plen);
    }

    req = (struct vwifi_ctrl_req_desc *)(g_ram_va + CTRL_REQ_OFFSET
                                         + slot * sizeof(*req));
    memset(req, 0, sizeof(*req));
    req->opcode       = opcode;
    req->req_id       = 0x1000 + g_req_slot;
    req->payload_addr = g_ram + REQ_PAYLOAD_OFFSET;
    req->payload_len  = plen;
    req->flags        = VWIFI_DESC_F_OWN;

    wreg(VWIFI_REG_CTRL_REQ_RING_DOORBELL, g_req_slot + 1);

    status = rsp->status;
    g_req_slot++;
    g_rsp_slot++;
    return status;
}

/* ------------------------------------------------------------------
 * 1. Header layout — must match the table in abi/vwifi.h exactly.
 * ------------------------------------------------------------------ */
static void test_header_layout(void)
{
    printf("=== frame header layout ===\n");

    CHECK(sizeof(struct vwifi_frame_hdr) == 40, "v2 header is 40 bytes");
    CHECK(VWIFI_HDR_SIZE_V1 == 28, "v1 header is 28 bytes");
    CHECK(VWIFI_MAGIC == 0x46495756u, "frame magic is \"VWIF\"");
    CHECK(VWIFI_VERSION == 2, "protocol version is 2");
    CHECK(VWIFI_HELLO_MAGIC == 0x52495756u, "hello magic is \"VWIR\"");

    CHECK(offsetof(struct vwifi_frame_hdr, magic)             ==  0, "offset  0: magic");
    CHECK(offsetof(struct vwifi_frame_hdr, version)           ==  4, "offset  4: version");
    CHECK(offsetof(struct vwifi_frame_hdr, frame_len)         ==  6, "offset  6: frame_len");
    CHECK(offsetof(struct vwifi_frame_hdr, tx_mac)            ==  8, "offset  8: tx_mac");
    CHECK(offsetof(struct vwifi_frame_hdr, rate_code)         == 14, "offset 14: rate_code");
    CHECK(offsetof(struct vwifi_frame_hdr, rssi)              == 15, "offset 15: rssi");
    CHECK(offsetof(struct vwifi_frame_hdr, tsf_lo)            == 16, "offset 16: tsf_lo");
    CHECK(offsetof(struct vwifi_frame_hdr, tsf_hi)            == 20, "offset 20: tsf_hi");
    CHECK(offsetof(struct vwifi_frame_hdr, flags)             == 24, "offset 24: flags");
    CHECK(offsetof(struct vwifi_frame_hdr, channel_freq)      == 28, "offset 28: channel_freq");
    CHECK(offsetof(struct vwifi_frame_hdr, channel_flags)     == 30, "offset 30: channel_flags");
    CHECK(offsetof(struct vwifi_frame_hdr, channel_bond_freq) == 32, "offset 32: channel_bond_freq");
    CHECK(offsetof(struct vwifi_frame_hdr, center_freq1)      == 34, "offset 34: center_freq1");
    CHECK(offsetof(struct vwifi_frame_hdr, center_freq2)      == 36, "offset 36: center_freq2");
}

/* ------------------------------------------------------------------
 * 2. Hello handshake — the exact bytes the hub parses.
 * ------------------------------------------------------------------ */
static void test_hello(struct mock_backend *m)
{
    const char *node_id = "proto-test";
    const struct mock_event *ev = NULL;
    uint32_t len_be, magic_host;
    size_t i, expect_len;

    printf("=== hello handshake ===\n");

    mock_backend_clear_events(m);
    vwifi_medium_link_event(g_dev, true);

    for (i = 0; i < mock_backend_event_count(m); i++) {
        if (mock_backend_events(m)[i].kind == MOCK_EV_MEDIUM_TX) {
            ev = &mock_backend_events(m)[i];
            break;
        }
    }

    CHECK(ev != NULL, "link up emits a hello on the medium");
    if (!ev) return;

    /* [uint32 net len][uint32 magic][node_id\0] and nothing else. */
    expect_len = 4 + 4 + strlen(node_id) + 1;
    CHECK(ev->medium_len == expect_len,
          "hello is len + magic + node_id\\0, no flags byte");

    memcpy(&len_be, ev->medium_buf, 4);
    CHECK(ntohl(len_be) == (uint32_t)(ev->medium_len - 4),
          "length prefix is big-endian, counts bytes after it");

    /* The hub compares this raw uint32 without ntohl(). Sending
     * htonl(VWIFI_HELLO_MAGIC) here is the bug this asserts against. */
    memcpy(&magic_host, ev->medium_buf + 4, 4);
    CHECK(magic_host == VWIFI_HELLO_MAGIC,
          "magic is VWIFI_HELLO_MAGIC in HOST byte order");

    CHECK(ev->medium_len > 8 &&
          strcmp((const char *)ev->medium_buf + 8, node_id) == 0,
          "node id follows the magic, NUL-terminated");
}

/* ------------------------------------------------------------------
 * 3. A transmitted frame carries a well-formed v2 medium header.
 * ------------------------------------------------------------------ */
static void test_tx_header(struct mock_backend *m, const uint8_t sta_mac[6])
{
    struct vwifi_channel ch;
    struct vwifi_op_mode om;
    struct vwifi_tx_desc *tx0;
    struct vwifi_frame_hdr hdr;
    const struct mock_event *ev = NULL;
    uint32_t len_be;
    size_t i;
    uint8_t inj_mac[6] = { 0x02, 0xDE, 0xAD, 0xBE, 0xEF, 0x01 };
    uint8_t probe[32];

    printf("=== transmitted frame header ===\n");

    memset(&ch, 0, sizeof(ch));
    ch.primary_freq = TEST_CHANNEL_FREQ;
    CHECK(ctrl_send(VWIFI_OP_SET_CHANNEL, &ch, sizeof(ch)) == 0,
          "SET_CHANNEL to 2437 MHz accepted");

    memset(&om, 0, sizeof(om));
    om.mode = VWIFI_MODE_MONITOR;
    CHECK(ctrl_send(VWIFI_OP_SET_OP_MODE, &om, sizeof(om)) == 0,
          "SET_OP_MODE monitor accepted");

    /* A probe request, injected so tx_mac comes from the frame itself. */
    memset(probe, 0, sizeof(probe));
    probe[0] = 0x40;                    /* type=mgmt subtype=probe-req */
    memset(probe + 4, 0xFF, 6);         /* addr1 = broadcast */
    memcpy(probe + 10, inj_mac, 6);     /* addr2 = injected source */
    memcpy(probe + 16, inj_mac, 6);     /* addr3 */

    memcpy(g_ram_va + TX_FRAME_OFFSET, probe, sizeof(probe));

    tx0 = (struct vwifi_tx_desc *)(g_ram_va + TX_OFFSET);
    memset(tx0, 0, sizeof(*tx0));
    tx0->frame_addr = g_ram + TX_FRAME_OFFSET;
    tx0->frame_len  = sizeof(probe);
    tx0->flags      = VWIFI_DESC_F_OWN | VWIFI_TX_F_INJECT;

    mock_backend_clear_events(m);
    wreg(VWIFI_REG_TX_RING_DOORBELL, 1);

    for (i = 0; i < mock_backend_event_count(m); i++) {
        if (mock_backend_events(m)[i].kind == MOCK_EV_MEDIUM_TX) {
            ev = &mock_backend_events(m)[i];
            break;
        }
    }

    CHECK(ev != NULL, "injected frame reaches the medium");
    if (!ev) return;

    CHECK(ev->medium_len == 4 + sizeof(hdr) + sizeof(probe),
          "message is len prefix + 40-byte header + frame");

    memcpy(&len_be, ev->medium_buf, 4);
    CHECK(ntohl(len_be) == (uint32_t)(sizeof(hdr) + sizeof(probe)),
          "length prefix covers header + frame");

    memcpy(&hdr, ev->medium_buf + 4, sizeof(hdr));
    CHECK(hdr.magic == VWIFI_MAGIC, "header magic is VWIFI_MAGIC in host byte order");
    CHECK(hdr.version == VWIFI_VERSION, "header version is 2");
    CHECK(hdr.frame_len == sizeof(probe), "frame_len matches the 802.11 frame");
    CHECK(memcmp(hdr.tx_mac, inj_mac, 6) == 0 &&
          memcmp(hdr.tx_mac, sta_mac, 6) != 0,
          "tx_mac is sourced from the frame's addr2");
    CHECK(hdr.channel_freq == TEST_CHANNEL_FREQ,
          "channel_freq is the channel the device is parked on");
    CHECK(hdr.rssi == VWIFI_DEFAULT_RSSI, "rssi defaults to VWIFI_DEFAULT_RSSI");
    CHECK(hdr._reserved[0] == 0 && hdr._reserved[1] == 0, "reserved bytes are zero");
    CHECK(memcmp(ev->medium_buf + 4 + sizeof(hdr), probe, sizeof(probe)) == 0,
          "the 802.11 frame follows the header unmodified");
}

int main(void)
{
    struct mock_backend *m = mock_backend_new(2 * 1024 * 1024, 256);
    uint8_t sta_mac[6] = { 0x00, 0x03, 0x7F, 0xCC, 0xDD, 0x20 };
    struct { uint32_t lo, hi, sz; uint64_t base; } r[4];
    int i;

    mock_backend_set_quiet(m, true);
    g_ram = mock_backend_ram_base(m);
    g_ram_va = mock_backend_ram(m);

    g_dev = malloc(vwifi_dev_sizeof());
    vwifi_dev_init(g_dev, mock_backend_ops_get(), m, "proto-test", false, sta_mac);

    test_header_layout();
    test_hello(m);

    r[0] = (typeof(r[0])){ VWIFI_REG_CTRL_REQ_RING_BASE_LO, VWIFI_REG_CTRL_REQ_RING_BASE_HI,
                           VWIFI_REG_CTRL_REQ_RING_SIZE, g_ram + CTRL_REQ_OFFSET };
    r[1] = (typeof(r[0])){ VWIFI_REG_CTRL_RSP_RING_BASE_LO, VWIFI_REG_CTRL_RSP_RING_BASE_HI,
                           VWIFI_REG_CTRL_RSP_RING_SIZE, g_ram + CTRL_RSP_OFFSET };
    r[2] = (typeof(r[0])){ VWIFI_REG_TX_RING_BASE_LO, VWIFI_REG_TX_RING_BASE_HI,
                           VWIFI_REG_TX_RING_SIZE, g_ram + TX_OFFSET };
    r[3] = (typeof(r[0])){ VWIFI_REG_RX_RING_BASE_LO, VWIFI_REG_RX_RING_BASE_HI,
                           VWIFI_REG_RX_RING_SIZE, g_ram + RX_OFFSET };
    for (i = 0; i < 4; i++) {
        wreg(r[i].lo, (uint32_t)(r[i].base & 0xFFFFFFFF));
        wreg(r[i].hi, (uint32_t)(r[i].base >> 32));
        wreg(r[i].sz, RING_ENTRIES);
    }

    for (i = 0; i < RING_ENTRIES; i++) {
        struct vwifi_rx_desc *d =
            (struct vwifi_rx_desc *)(g_ram_va + RX_OFFSET + i * sizeof(*d));
        memset(d, 0, sizeof(*d));
        d->frame_addr = g_ram + RX_BUFFER_OFFSET + (uint64_t)i * RX_BUFSZ;
        d->buffer_len = RX_BUFSZ;
        d->flags      = VWIFI_DESC_F_OWN;
    }

    wreg(VWIFI_REG_CTRL, VWIFI_CTRL_ENABLE | VWIFI_CTRL_IRQ_ENABLE);

    test_tx_header(m, sta_mac);

    printf("medium_proto: %s\n", failures == 0 ? "PASS" : "FAIL");

    free(g_dev);
    mock_backend_free(m);
    return failures == 0 ? 0 : 1;
}
