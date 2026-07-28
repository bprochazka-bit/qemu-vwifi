/*
 * vwifi-virt — portable device logic
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * This file implements the device's behavior in a form that is
 * independent of QEMU or libvfio-user. All interaction with the
 * host environment (DMA, IRQ raise, medium socket, logging) goes
 * through the vwifi_backend_ops vtable. See vwifi_backend.h.
 *
 * A concrete backend (currently vwifi_qemu.c, planned:
 * vwifi_vfio_user.c) constructs a vwifi_dev via vwifi_dev_init()
 * and drives it via the four public entry points:
 *
 *   vwifi_reg_read()          — guest MMIO read at register offset
 *   vwifi_reg_write()         — guest MMIO write at register offset
 *   vwifi_medium_rx_bytes()   — bytes arrived on the hub socket
 *   vwifi_medium_link_event() — hub socket opened/closed
 *
 * Everything else — ring processing, opcode dispatch, event
 * generation — happens synchronously inside those four calls.
 * The device is single-threaded from its own perspective.
 */

#include <string.h>
#include <errno.h>
#include <arpa/inet.h>   /* htonl/ntohl — no other network deps */

#include "vwifi_device.h"
#include "vwifi_abi.h"      /* device <-> guest driver PCI contract */
#include "vwifi_crypto.h"
#include "vwifi.h"          /* medium wire protocol, shared with the hub */

/*
 * Reassembly buffer for the medium socket.
 *
 * The shared header's default (VWIFI_RXBUF_SIZE) holds exactly one
 * length-prefixed message.  Hold two, so a message that straddles a
 * read() boundary can be buffered whole alongside the tail of the
 * previous one without a second pass.
 */
#define VWIFI_VIRT_RXBUF_SIZE   (2 * VWIFI_MSG_SLOT_SIZE)

/* ============================================================
 * Public state layout
 *
 * The struct is defined here (not in the header) because backends
 * embed it by inclusion in their own state — they see it fully.
 * The device logic accesses only these fields.
 * ============================================================ */

/* Ring bookkeeping (device side). */
struct vwifi_ring {
    uint64_t base_gpa;
    uint32_t size;
    uint32_t mask;
    uint32_t next_idx;
    uint32_t desc_size;
    bool     enabled;
    const char *name;
};

/* ============================================================
 * BSS table
 *
 * Populated from beacons and probe responses seen on the medium.
 * Entries age out after VWIFI_BSS_TTL_US without a re-sighting.
 * Raw IEs from the last-seen frame are kept so the driver can hand
 * them to the OS (Windows needs them for the BSS entry TLV).
 * ============================================================ */

#define VWIFI_BSS_MAX        64
/* Large enough for a full beacon: 802.11 hdr (24) + fixed params (12) +
 * a generous IE set. Real beacons with HT/VHT/RSN/vendor IEs run to a
 * few hundred bytes. */
#define VWIFI_BSS_FRAME_MAX  768
#define VWIFI_BSS_TTL_US     (30ULL * 1000 * 1000)   /* 30 seconds */

struct vwifi_bss {
    bool     valid;
    uint64_t last_seen_us;

    uint8_t  bssid[6];
    uint16_t channel_freq;
    int8_t   rssi;
    uint32_t beacon_period_tu;
    uint16_t capability_info;
    uint64_t tsf;

    uint16_t ssid_len;
    uint8_t  ssid[33];

    /* The complete beacon or probe-response frame as seen on the air.
     * WDI's BSS entry TLV wants the whole frame (WDI_TLV_BEACON_FRAME /
     * WDI_TLV_PROBE_RESPONSE_FRAME as byte blobs) and the OS parses the
     * IEs itself — so keep the frame, not just the IE tail. */
    uint16_t frame_len;
    uint8_t  frame[VWIFI_BSS_FRAME_MAX];
    bool     is_beacon;      /* false => probe response */

    /* Set when reported during the current scan, so we emit one
     * BSS_FOUND per BSS per scan rather than one per beacon. */
    bool     reported_this_scan;
};

/* Scan state machine. */
enum vwifi_scan_state {
    VWIFI_SCAN_IDLE = 0,
    VWIFI_SCAN_DWELL,       /* parked on a channel, collecting */
};

#define VWIFI_SCAN_MAX_CHANNELS  16

struct vwifi_scan {
    enum vwifi_scan_state state;
    uint16_t channels[VWIFI_SCAN_MAX_CHANNELS];  /* freqs to visit */
    uint8_t  num_channels;
    uint8_t  cur_channel;                        /* index into channels[] */
    uint16_t dwell_ms;
    bool     active;                             /* send probe requests */

    /* Directed-scan SSID list. */
    uint8_t  num_ssids;
    uint8_t  ssid_len[4];
    uint8_t  ssids[4][33];

    /* Channel to restore when the scan finishes. */
    struct vwifi_channel saved_channel;
};

/* ============================================================
 * Key store
 *
 * WPA2-PSK installs two keys after the 4-way handshake:
 *   PTK (pairwise) — unicast traffic to/from the AP
 *   GTK (group)    — broadcast/multicast from the AP
 * Each direction keeps its own packet number space. TX PN increments
 * per frame; RX PN is a replay high-water mark.
 * ============================================================ */

struct vwifi_key_slot {
    bool     valid;
    uint16_t cipher;                       /* VWIFI_CIPHER_* */
    uint8_t  key_idx;
    struct vwifi_aes128 aes;               /* expanded key schedule */
    uint8_t  tx_pn[VWIFI_PN_LEN];          /* next PN to transmit */
    uint8_t  rx_pn[VWIFI_PN_LEN];          /* highest PN seen */
    bool     rx_pn_valid;
};

struct vwifi_keys {
    struct vwifi_key_slot pairwise;        /* PTK */
    struct vwifi_key_slot group[4];        /* GTK, indexed by key id */
};

/* ============================================================
 * Timer scheduling
 *
 * The backend gives us exactly one one-shot timer, but three things
 * want deadlines: the scan dwell, the connect exchange timeout, and
 * (Phase 5) periodic beacon transmission. Rather than widen the
 * backend vtable — which every future backend would have to
 * implement — the device multiplexes them itself: each subsystem
 * publishes an absolute deadline, and we arm the host timer for
 * whichever is earliest.
 *
 * A deadline of 0 means "inactive".
 * ============================================================ */

struct vwifi_timers {
    uint64_t scan_us;      /* next scan channel hop */
    uint64_t conn_us;      /* auth/assoc response timeout */
    uint64_t beacon_us;    /* next beacon transmission */
};

/* ============================================================
 * SoftAP (Phase 5)
 *
 * Lets a Windows guest host a network rather than only join one,
 * which makes the virtual medium symmetric: either end of a link can
 * be a Windows VM, or a Linux VM, or the host radio.
 *
 * The device owns the AP-side 802.11 state machine for the same
 * reason it owns the STA side — it can be tested without a Windows
 * guest in the loop.
 *
 * What the device does:
 *   - transmits beacons on a timer
 *   - answers probe requests with probe responses
 *   - answers auth requests (Open System) and assoc requests
 *   - tracks associated stations and their keys
 *   - bridges data frames: STA -> AP -> driver, driver -> AP -> STA
 *
 * What it does not do: the 4-way handshake. Exactly as on the STA
 * side, that runs in the OS (hostapd's authenticator equivalent) over
 * EAPOL frames we pass through in the clear; we just install the keys
 * the OS hands down.
 * ============================================================ */

#define VWIFI_AP_MAX_STA     16
#define VWIFI_AP_IE_MAX     512

struct vwifi_ap_sta {
    bool     valid;
    uint8_t  mac[6];
    uint16_t aid;
    bool     authenticated;
    bool     associated;
    uint64_t last_seen_us;

    /* Per-station pairwise key, installed by the OS after its 4-way
     * handshake completes with this peer. */
    struct vwifi_key_slot ptk;
};

struct vwifi_ap {
    bool     running;
    uint16_t ssid_len;
    uint8_t  ssid[33];
    uint16_t channel_freq;
    uint16_t beacon_interval_tu;    /* in 1024us Time Units */
    uint16_t dtim_period;

    /* Extra IEs the driver wants in beacons and probe responses —
     * for WPA2 this carries the RSN element, which is how a joining
     * station learns the network is protected and which ciphers to
     * negotiate. */
    uint16_t ie_len;
    uint8_t  ies[VWIFI_AP_IE_MAX];

    struct vwifi_ap_sta sta[VWIFI_AP_MAX_STA];
    uint16_t next_aid;

    /* Group key for broadcast/multicast downlink. */
    struct vwifi_key_slot gtk;
};

/* ============================================================
 * Connect / association state machine
 *
 * op_connect -> send Auth Request  -> AUTH_SENT
 *   Auth Response (status 0)       -> send Assoc Request -> ASSOC_SENT
 *   Assoc Response (status 0)      -> ASSOCIATED, emit ASSOC_RESULT
 *   any failure / timeout          -> IDLE, emit ASSOC_RESULT w/ status
 *
 * Once ASSOCIATED the RX path accepts data frames to/from the BSSID
 * and converts them to 802.3 for the driver; the TX path converts
 * 802.3 from the driver into 802.11 data frames.
 * ============================================================ */

enum vwifi_conn_state {
    VWIFI_CONN_IDLE = 0,
    VWIFI_CONN_AUTH_SENT,
    VWIFI_CONN_ASSOC_SENT,
    VWIFI_CONN_ASSOCIATED,
};

/* How long to wait for an Auth/Assoc Response before giving up. */
#define VWIFI_CONN_TIMEOUT_MS  1000

/* Max association-response IEs we keep to hand back to the driver. */
#define VWIFI_ASSOC_IE_MAX  512

struct vwifi_conn {
    enum vwifi_conn_state state;

    uint8_t  bssid[6];
    uint16_t ssid_len;
    uint8_t  ssid[33];
    uint16_t channel_freq;
    uint16_t auth_algo;
    uint16_t cipher_pairwise;
    uint16_t cipher_group;
    uint16_t akm_suite;
    uint16_t aid;

    /* IEs the driver asked us to include in the Assoc Request. */
    uint16_t req_ie_len;
    uint8_t  req_ies[VWIFI_ASSOC_IE_MAX];
};

struct vwifi_dev {
    /* Backend vtable + opaque backend state pointer. */
    const struct vwifi_backend_ops *ops;
    void *be;

    /* Node identity for logging; owned by the backend. */
    const char *node_id;
    bool        verbose;

    /* MMIO register shadow. */
    uint32_t regs[VWIFI_MMIO_SIZE / 4];

    /* Decoded control registers. */
    uint32_t ctrl;
    uint32_t status;
    uint32_t irq_mask;
    uint32_t irq_status;

    /* Four rings. */
    struct vwifi_ring ctrl_req;
    struct vwifi_ring ctrl_rsp;
    struct vwifi_ring tx;
    struct vwifi_ring rx;

    /* Operating state. */
    uint32_t op_mode;
    uint32_t raw_filter;
    struct vwifi_channel channel;
    uint8_t  sta_mac[6];

    /* Scan + BSS table. */
    struct vwifi_scan scan;
    struct vwifi_bss  bss[VWIFI_BSS_MAX];

    /* Connection state. */
    struct vwifi_conn conn;

    /* Installed cipher keys (Phase 4). */
    struct vwifi_keys keys;

    /* SoftAP state (Phase 5). */
    struct vwifi_ap ap;

    /* Multiplexed timer deadlines. */
    struct vwifi_timers timers;

    /* 802.11 transmit sequence counter (12-bit, in seq ctrl field). */
    uint16_t seq_num;

    /* Medium RX reassembly. */
    uint8_t  rxbuf[VWIFI_VIRT_RXBUF_SIZE];
    uint32_t rxbuf_len;

    /* Stats. */
    uint32_t frames_tx;
    uint32_t frames_rx;
    uint32_t drops;
};

static void vwifi_raise_irq(struct vwifi_dev *d, unsigned vec);
static void post_rsp(struct vwifi_dev *d,
                     const struct vwifi_ctrl_rsp_desc *rsp,
                     const void *payload, uint32_t payload_len);
static bool frame_is_beacon_or_probe_resp(const uint8_t *frame, uint16_t len);
static void bss_observe(struct vwifi_dev *d,
                        const struct vwifi_frame_hdr *hdr,
                        const uint8_t *frame, uint16_t frame_len);
static void medium_send_frame(struct vwifi_dev *d, const uint8_t *frame,
                              uint16_t frame_len, uint8_t rate_code,
                              uint16_t channel_freq, uint64_t tsf_us,
                              bool inject);
static void conn_timeout(struct vwifi_dev *d);
static bool conn_rx_mgmt(struct vwifi_dev *d,
                         const uint8_t *frame, uint16_t frame_len);
static bool sta_is_our_data_frame(const struct vwifi_dev *d,
                                  const uint8_t *frame, uint16_t frame_len);
static uint16_t sta_rx_80211_to_8023(struct vwifi_dev *d,
                                     const uint8_t *frame, uint16_t frame_len,
                                     uint8_t *out, uint16_t out_cap);
static uint16_t sta_tx_8023_to_80211(struct vwifi_dev *d,
                                     const uint8_t *eth, uint16_t eth_len,
                                     uint8_t *out, uint16_t out_cap);
struct vwifi_key_slot;
static struct vwifi_key_slot *key_slot_for_rx(struct vwifi_dev *d,
                                              const uint8_t *frame,
                                              uint8_t key_id);
static void timers_rearm(struct vwifi_dev *d);
static void timer_set_rel(struct vwifi_dev *d, uint64_t *slot, uint32_t ms);
static void timer_clear(struct vwifi_dev *d, uint64_t *slot);
static void ap_beacon_tick(struct vwifi_dev *d, uint64_t due_us);
static bool ap_rx_mgmt(struct vwifi_dev *d,
                       const uint8_t *frame, uint16_t frame_len);
static unsigned freq_to_chan(uint16_t freq);

/* ============================================================
 * Logging helpers — thin wrappers that route through the vtable.
 * ============================================================ */

/* (3, 4): fmt is argument 3 and the varargs start at 4. Unlike the
 * vtable member, this one takes them inline, so the second number is a
 * real position rather than 0. Annotating it also makes the compiler
 * format-check every VWIFI_ERR/WARN/INFO/TRACE call site below. */
static void VWIFI_PRINTF_FMT(3, 4)
vwifi_logf(struct vwifi_dev *d, enum vwifi_log_level lvl,
           const char *fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);
    d->ops->log(d->be, d, lvl, fmt, ap);
    va_end(ap);
}

#define VWIFI_ERR(d, ...)   vwifi_logf((d), VWIFI_LOG_ERR,   __VA_ARGS__)
#define VWIFI_WARN(d, ...)  vwifi_logf((d), VWIFI_LOG_WARN,  __VA_ARGS__)
#define VWIFI_INFO(d, ...)  vwifi_logf((d), VWIFI_LOG_INFO,  __VA_ARGS__)
#define VWIFI_TRACE(d, ...) \
    do { if ((d)->verbose) vwifi_logf((d), VWIFI_LOG_TRACE, __VA_ARGS__); } while (0)

/* ============================================================
 * Small helpers
 * ============================================================ */

static inline bool is_pow2(uint32_t v) { return v && !(v & (v - 1)); }

static void ring_reset(struct vwifi_ring *r, const char *name, uint32_t desc_size)
{
    r->base_gpa  = 0;
    r->size      = 0;
    r->mask      = 0;
    r->next_idx  = 0;
    r->desc_size = desc_size;
    r->enabled   = false;
    r->name      = name;
}

static uint64_t ring_desc_gpa(const struct vwifi_ring *r, uint32_t idx)
{
    return r->base_gpa + (uint64_t)(idx & r->mask) * r->desc_size;
}

/* ============================================================
 * IRQ management
 * ============================================================ */

static void vwifi_raise_irq(struct vwifi_dev *d, unsigned vec)
{
    if (vec >= VWIFI_NUM_VECTORS) return;
    if (!(d->ctrl & VWIFI_CTRL_IRQ_ENABLE)) return;
    if (d->irq_mask & (1u << vec)) return;

    d->irq_status |= (1u << vec);
    d->regs[VWIFI_REG_IRQ_STATUS / 4] = d->irq_status;

    d->ops->raise_irq(d->be, vec);
}

/* ============================================================
 * Medium I/O — send-side wrapping in vwifi_frame_hdr v2.
 * The hub hello is triggered externally by vwifi_medium_link_event.
 * ============================================================ */

/*
 * Register with the hub.
 *
 * Wire format per abi/vwifi.h:
 *   [uint32 net len][uint32 VWIFI_HELLO_MAGIC][node_id\0][flags?]
 *
 * Only the length prefix is in network byte order — the magic goes out
 * in host order, because the hub compares the raw uint32 without
 * ntohl().  The trailing optional flags byte is deliberately omitted so
 * the hub reads flags = 0, i.e. NOT VWIFI_HELLO_FLAG_PHYSICAL: this is
 * a simulated station, not a real radio, so it stays subject to the
 * hub's propagation model.  Only the physical bridge sets that flag.
 */
void vwifi_medium_hello(struct vwifi_dev *d)
{
    const char *id = d->node_id ? d->node_id : "vwifi-virt";
    size_t id_len = strlen(id) + 1;
    uint32_t magic = VWIFI_HELLO_MAGIC;
    uint32_t payload_len;
    uint32_t len_be;
    uint8_t  buf[8 + 128];
    uint32_t off = 0;

    if (id_len > sizeof(buf) - 8) id_len = sizeof(buf) - 8;

    payload_len = 4 + (uint32_t)id_len;
    len_be = htonl(payload_len);

    memcpy(buf + off, &len_be, 4); off += 4;
    memcpy(buf + off, &magic,  4); off += 4;
    memcpy(buf + off, id, id_len); off += id_len;

    (void)d->ops->medium_send(d->be, buf, off);

    VWIFI_TRACE(d, "hello sent to hub (node_id=%s)", id);
    d->status |= VWIFI_STATUS_LINK_UP;
    d->regs[VWIFI_REG_STATUS / 4] = d->status;
}

/* Extract the transmitter address (addr2) from an 802.11 frame, if it
 * has one. Management and data frames carry addr2 at offset 10 (after
 * the 2-byte frame control, 2-byte duration, and 6-byte addr1). Some
 * control frames (ACK, CTS) have no addr2 — for those we fall back to
 * the station MAC. Returns true if addr2 was extracted. */
static bool frame_get_addr2(const uint8_t *frame, uint16_t frame_len,
                            uint8_t out_mac[6])
{
    uint8_t type, subtype;

    if (frame_len < 16) return false;   /* need through addr2 */

    type    = (frame[0] >> 2) & 0x3;
    subtype = (frame[0] >> 4) & 0xF;

    /* Control frames (type 1): only some carry addr2. ACK (0xD) and
     * CTS (0xC) do not. Be conservative — skip addr2 for control. */
    if (type == 1) {
        (void)subtype;
        return false;
    }

    memcpy(out_mac, frame + 10, 6);
    return true;
}

static void medium_send_frame(struct vwifi_dev *d, const uint8_t *frame,
                              uint16_t frame_len, uint8_t rate_code,
                              uint16_t channel_freq, uint64_t tsf_us,
                              bool inject)
{
    struct vwifi_frame_hdr hdr;
    uint32_t payload_len = sizeof(hdr) + frame_len;
    uint32_t len_be = htonl(payload_len);
    uint8_t  out[4 + sizeof(hdr) + VWIFI_MAX_FRAME_SIZE];
    uint32_t off = 0;
    uint8_t  tx_mac[6];

    if (frame_len > VWIFI_MAX_FRAME_SIZE) return;

    /* For injected frames, honor the frame's own transmitter address
     * so the medium sees the frame as coming from whoever the injector
     * claims to be — that's the whole point of injection. Fall back to
     * the station MAC if the frame has no addr2 (or on failure). For
     * normal STA-mode TX, always stamp the station MAC. */
    if (inject && frame_get_addr2(frame, frame_len, tx_mac)) {
        /* use extracted addr2 */
    } else {
        memcpy(tx_mac, d->sta_mac, 6);
    }

    memset(&hdr, 0, sizeof(hdr));
    hdr.magic     = VWIFI_MAGIC;
    hdr.version   = VWIFI_VERSION;
    hdr.frame_len = frame_len;
    memcpy(hdr.tx_mac, tx_mac, 6);
    hdr.rate_code = rate_code ? rate_code : VWIFI_DEFAULT_RATE;
    hdr.rssi      = VWIFI_DEFAULT_RSSI;
    hdr.tsf_lo    = (uint32_t)(tsf_us & 0xFFFFFFFF);
    hdr.tsf_hi    = (uint32_t)(tsf_us >> 32);
    hdr.flags     = 0;
    hdr.channel_freq      = channel_freq ? channel_freq : d->channel.primary_freq;
    hdr.channel_flags     = d->channel.flags;
    hdr.channel_bond_freq = d->channel.bond_freq;
    hdr.center_freq1      = d->channel.center_freq1;
    hdr.center_freq2      = d->channel.center_freq2;

    memcpy(out + off, &len_be, 4);       off += 4;
    memcpy(out + off, &hdr, sizeof(hdr)); off += sizeof(hdr);
    memcpy(out + off, frame, frame_len);  off += frame_len;

    if (d->ops->medium_send(d->be, out, off) == 0) {
        d->frames_tx++;
        d->regs[VWIFI_REG_DIAG_FRAMES_TX / 4] = d->frames_tx;
    } else {
        d->drops++;
        d->regs[VWIFI_REG_DIAG_DROPS / 4] = d->drops;
    }
}

/* ============================================================
 * Medium RX — parse incoming length-prefixed messages and deliver
 * matching frames into the RX ring.
 * ============================================================ */

static void medium_deliver_rx(struct vwifi_dev *d,
                              const struct vwifi_frame_hdr *hdr,
                              const uint8_t *frame, uint16_t frame_len)
{
    struct vwifi_rx_desc desc;
    uint64_t desc_gpa;
    uint32_t idx;
    bool desc_decrypted = false;

    if (!d->rx.enabled) {
        d->drops++;
        d->regs[VWIFI_REG_DIAG_DROPS / 4] = d->drops;
        return;
    }

    /* Channel filter. */
    if (hdr->channel_freq && d->channel.primary_freq &&
        hdr->channel_freq != d->channel.primary_freq) {
        return;
    }

    /* Echo suppression. */
    if (memcmp(hdr->tx_mac, d->sta_mac, 6) == 0) {
        return;
    }

    /* Feed beacons and probe responses into the BSS table regardless
     * of operating mode. During a scan this also emits BSS_FOUND
     * events. This happens before any raw filtering because the BSS
     * table is about what's on the air, not what the driver wants
     * delivered. */
    if (frame_is_beacon_or_probe_resp(frame, frame_len)) {
        bss_observe(d, hdr, frame, frame_len);
    }

    /* Management frames addressed to us (Auth/Assoc Response, Deauth)
     * drive the connect state machine. In STA mode they're consumed
     * internally — the driver doesn't need to see them because the
     * device runs the exchange. In monitor mode we fall through so
     * the capture still sees everything. */
    if (d->op_mode == VWIFI_MODE_STA) {
        if (conn_rx_mgmt(d, frame, frame_len)) {
            return;
        }
    }

    /* In SoftAP mode the device answers probes, auth and assoc
     * requests itself — same reasoning as the STA side: the 802.11
     * state machine belongs where it can be tested. */
    if (d->op_mode == VWIFI_MODE_SOFTAP) {
        if (ap_rx_mgmt(d, frame, frame_len)) {
            return;
        }
    }

    /* In STA mode the driver only wants data frames for our BSS,
     * converted to 802.3. Everything else (other BSSes' traffic,
     * beacons, management) stops here. */
    if (d->op_mode == VWIFI_MODE_STA) {
        if (!sta_is_our_data_frame(d, frame, frame_len)) {
            return;
        }
    }

    /* In monitor mode, honor the raw filter: only forward frame types
     * the driver has asked for. The 802.11 frame type is bits 2-3 of
     * the first frame-control byte: 0=mgmt, 1=ctrl, 2=data. */
    if (d->op_mode == VWIFI_MODE_MONITOR && frame_len >= 1) {
        uint8_t ftype = (frame[0] >> 2) & 0x3;
        uint32_t want =
            (ftype == 0) ? VWIFI_RAW_F_MGMT :
            (ftype == 1) ? VWIFI_RAW_F_CTRL :
            (ftype == 2) ? VWIFI_RAW_F_DATA : 0;
        /* If the driver set a non-zero filter and this type isn't in
         * it, drop. A zero filter means "everything" (driver hasn't
         * narrowed yet). */
        if (d->raw_filter && want && !(d->raw_filter & want)) {
            return;
        }
    }

    idx = d->rx.next_idx;
    desc_gpa = ring_desc_gpa(&d->rx, idx);

    if (d->ops->dma_read(d->be, desc_gpa, &desc, sizeof(desc)) != 0) {
        d->drops++;
        d->regs[VWIFI_REG_DIAG_DROPS / 4] = d->drops;
        return;
    }
    if (!(desc.flags & VWIFI_DESC_F_OWN)) {
        d->drops++;
        d->regs[VWIFI_REG_DIAG_DROPS / 4] = d->drops;
        return;
    }

    /* In STA mode, hand the driver an 802.3 frame; the 802.11 header
     * and LLC/SNAP are the device's business, not the driver's. In
     * monitor mode the raw 802.11 frame goes up untouched. */
    {
        uint8_t eth[VWIFI_MAX_FRAME_SIZE];
        uint8_t work[VWIFI_MAX_FRAME_SIZE];
        const uint8_t *out_frame = frame;
        uint16_t out_len = frame_len;
        bool was_decrypted = false;

        if (d->op_mode == VWIFI_MODE_STA) {
            const uint8_t *plain = frame;
            uint16_t plain_len = frame_len;

            /* Decrypt first if the Protected bit is set. */
            if (frame[1] & 0x40) {
                uint8_t pn[VWIFI_PN_LEN], key_id = 0;
                struct vwifi_key_slot *ks;
                int dlen;

                if (frame_len > sizeof(work)) return;
                memcpy(work, frame, frame_len);

                vwifi_ccmp_parse_header(work + vwifi_ccmp_hdr_len(work),
                                        pn, &key_id);
                ks = key_slot_for_rx(d, work, key_id);
                if (!ks) {
                    VWIFI_TRACE(d, "rx dropped: no key for id %u", key_id);
                    d->drops++;
                    d->regs[VWIFI_REG_DIAG_DROPS / 4] = d->drops;
                    return;
                }

                /* Replay check BEFORE spending cycles on decryption:
                 * a replayed frame has a PN we've already accepted. */
                if (ks->rx_pn_valid && vwifi_pn_cmp(pn, ks->rx_pn) <= 0) {
                    VWIFI_TRACE(d, "rx dropped: replay (PN not advancing)");
                    d->drops++;
                    d->regs[VWIFI_REG_DIAG_DROPS / 4] = d->drops;
                    return;
                }

                dlen = vwifi_ccmp_decrypt(&ks->aes, work, frame_len, NULL, NULL);
                if (dlen < 0) {
                    VWIFI_TRACE(d, "rx dropped: %s",
                                dlen == -2 ? "MIC failure" : "malformed CCMP");
                    d->drops++;
                    d->regs[VWIFI_REG_DIAG_DROPS / 4] = d->drops;
                    return;
                }

                /* Only advance the replay counter once the MIC has
                 * verified — otherwise a forged frame with a high PN
                 * could wedge the counter and block real traffic. */
                memcpy(ks->rx_pn, pn, VWIFI_PN_LEN);
                ks->rx_pn_valid = true;

                plain     = work;
                plain_len = (uint16_t)dlen;
                was_decrypted = true;
            }

            out_len = sta_rx_80211_to_8023(d, plain, plain_len,
                                           eth, sizeof(eth));
            if (out_len == 0) {
                /* Not convertible (non-SNAP, malformed, wrong BSS). */
                return;
            }
            out_frame = eth;
        }

        if (out_len > desc.buffer_len) {
            out_len = desc.buffer_len;
        }

        if (d->ops->dma_write(d->be, desc.frame_addr, out_frame, out_len) != 0) {
            d->drops++;
            d->regs[VWIFI_REG_DIAG_DROPS / 4] = d->drops;
            return;
        }
        desc.frame_len = out_len;
        desc_decrypted = was_decrypted;
    }

    desc.rssi         = hdr->rssi;
    desc.rate_code    = hdr->rate_code;
    desc.channel_freq = hdr->channel_freq;
    desc.tsf          = ((uint64_t)hdr->tsf_hi << 32) | hdr->tsf_lo;
    desc.flags        = 0;   /* clear OWN — transfer to driver */
    if (d->op_mode == VWIFI_MODE_MONITOR) {
        desc.flags |= VWIFI_RX_F_RAW;
    }
    if (desc_decrypted) {
        desc.flags |= VWIFI_RX_F_DECRYPTED;
    }

    (void)d->ops->dma_write(d->be, desc_gpa, &desc, sizeof(desc));

    d->rx.next_idx = (idx + 1) & d->rx.mask;
    d->regs[VWIFI_REG_RX_RING_TAIL / 4] = d->rx.next_idx;

    d->frames_rx++;
    d->regs[VWIFI_REG_DIAG_FRAMES_RX / 4] = d->frames_rx;

    vwifi_raise_irq(d, VWIFI_VEC_RX);
}

static void medium_process_rxbuf(struct vwifi_dev *d)
{
    while (d->rxbuf_len >= 4) {
        uint32_t msg_len = ntohl(*(uint32_t *)d->rxbuf);
        uint32_t total = 4 + msg_len;

        if (msg_len > VWIFI_MAX_MSG_SIZE) {
            VWIFI_ERR(d, "oversized medium message %u", msg_len);
            d->rxbuf_len = 0;
            return;
        }
        if (d->rxbuf_len < total) {
            return;
        }

        const uint8_t *msg = d->rxbuf + 4;
        if (msg_len >= VWIFI_HDR_SIZE_MIN) {
            const struct vwifi_frame_hdr *hdr =
                (const struct vwifi_frame_hdr *)msg;

            if (hdr->magic == VWIFI_MAGIC) {
                uint32_t hdr_size = (hdr->version >= 2)
                                    ? VWIFI_HDR_SIZE
                                    : VWIFI_HDR_SIZE_V1;
                if (msg_len >= hdr_size + hdr->frame_len) {
                    medium_deliver_rx(d, hdr, msg + hdr_size, hdr->frame_len);
                }
            }
        }

        memmove(d->rxbuf, d->rxbuf + total, d->rxbuf_len - total);
        d->rxbuf_len -= total;
    }
}

/* Backend calls this whenever bytes have arrived on the hub socket. */
void vwifi_medium_rx_bytes(struct vwifi_dev *d, const void *buf, size_t len)
{
    size_t space = sizeof(d->rxbuf) - d->rxbuf_len;
    if (len > space) len = space;
    memcpy(d->rxbuf + d->rxbuf_len, buf, len);
    d->rxbuf_len += (uint32_t)len;
    medium_process_rxbuf(d);
}

size_t vwifi_medium_rx_space(const struct vwifi_dev *d)
{
    return sizeof(d->rxbuf) - d->rxbuf_len;
}

void vwifi_medium_link_event(struct vwifi_dev *d, bool link_up)
{
    if (link_up) {
        vwifi_medium_hello(d);
    } else {
        VWIFI_TRACE(d, "medium chardev closed");
        d->status &= ~VWIFI_STATUS_LINK_UP;
        d->regs[VWIFI_REG_STATUS / 4] = d->status;
        d->rxbuf_len = 0;
    }
}

/* ============================================================
 * Control-command dispatcher
 * ============================================================ */

/* ============================================================
 * 802.11 beacon / probe-response parsing
 *
 * Management frame layout:
 *   0  frame control (2)
 *   2  duration (2)
 *   4  addr1 (6)   — DA
 *  10  addr2 (6)   — SA (the AP's MAC)
 *  16  addr3 (6)   — BSSID
 *  22  seq ctrl (2)
 *  24  fixed params: timestamp (8), beacon interval (2), capability (2)
 *  36  information elements: [id 1][len 1][data len]...
 * ============================================================ */

#define IEEE80211_MGMT_HDR_LEN     24
#define IEEE80211_BEACON_FIXED_LEN 12
#define IEEE80211_IE_OFFSET        (IEEE80211_MGMT_HDR_LEN + IEEE80211_BEACON_FIXED_LEN)

#define IEEE80211_SUBTYPE_PROBE_REQ   4
#define IEEE80211_SUBTYPE_PROBE_RESP  5
#define IEEE80211_SUBTYPE_BEACON      8

#define IEEE80211_EID_SSID  0

static inline uint16_t get_le16(const uint8_t *p)
{
    return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}

static inline uint64_t get_le64(const uint8_t *p)
{
    uint64_t v = 0;
    for (int i = 7; i >= 0; i--) v = (v << 8) | p[i];
    return v;
}

/* Is this frame a beacon or probe response? */
static bool frame_is_beacon_or_probe_resp(const uint8_t *frame, uint16_t len)
{
    uint8_t type, subtype;
    if (len < IEEE80211_IE_OFFSET) return false;
    type    = (frame[0] >> 2) & 0x3;
    subtype = (frame[0] >> 4) & 0xF;
    if (type != 0) return false;   /* not management */
    return subtype == IEEE80211_SUBTYPE_BEACON ||
           subtype == IEEE80211_SUBTYPE_PROBE_RESP;
}

/* Walk the IE list looking for the SSID element. Returns false if the
 * IEs are malformed. An SSID of length 0 (hidden network) is valid and
 * returns true with *ssid_len = 0. */
static bool ies_find_ssid(const uint8_t *ies, uint16_t ie_len,
                          uint8_t *ssid_out, uint16_t *ssid_len)
{
    uint16_t off = 0;
    *ssid_len = 0;

    while (off + 2 <= ie_len) {
        uint8_t id  = ies[off];
        uint8_t len = ies[off + 1];
        if (off + 2 + len > ie_len) return false;   /* truncated IE */

        if (id == IEEE80211_EID_SSID) {
            if (len > 32) return false;
            memcpy(ssid_out, ies + off + 2, len);
            ssid_out[len] = 0;
            *ssid_len = len;
            return true;
        }
        off += 2 + len;
    }
    /* No SSID IE at all — treat as hidden rather than an error. */
    ssid_out[0] = 0;
    return true;
}

/*
 * Append the Supported Rates element, plus Extended Supported Rates
 * when the set does not fit in the eight octets EID 1 allows.
 *
 * Rates are in units of 500 kbit/s; the high bit marks a rate as
 * "basic", i.e. one the sender requires of anyone joining. The sets
 * below are the ordinary 802.11g and 802.11a ones, which is what every
 * peer on this medium can decode -- the medium's rate model is about
 * airtime and error probability, not about what a receiver can demodulate.
 *
 * This is not decoration. hostapd's copy_supp_rates() rejects an
 * Association Request that carries no Supported Rates element with
 * WLAN_STATUS_UNSPECIFIED_FAILURE (1), which arrives here as a bare
 * "Assoc Response REJECTED status=1" and explains nothing. The same
 * element is required in a Probe Request by 802.11 and some APs decline
 * to answer without it.
 */
#define IEEE80211_EID_SUPP_RATES      1
#define IEEE80211_EID_EXT_SUPP_RATES  50

static uint16_t ie_put_supp_rates(uint8_t *buf, uint16_t len, uint16_t freq)
{
    /* 802.11a: 6/9/12/18/24/36/48/54, basic 6/12/24. No CCK on 5 GHz. */
    static const uint8_t rates_5[] = {
        0x8C, 0x12, 0x98, 0x24, 0xB0, 0x48, 0x60, 0x6C,
    };
    /* 802.11g: CCK 1/2/5.5/11 (basic) then OFDM 6/9/12/18, with
     * 24/36/48/54 spilling into the extended element. */
    static const uint8_t rates_24[] = {
        0x82, 0x84, 0x8B, 0x96, 0x0C, 0x12, 0x18, 0x24,
    };
    static const uint8_t rates_24_ext[] = { 0x30, 0x48, 0x60, 0x6C };

    bool is_5ghz = (freq >= 5000);
    const uint8_t *rates = is_5ghz ? rates_5 : rates_24;

    buf[len++] = IEEE80211_EID_SUPP_RATES;
    buf[len++] = 8;
    memcpy(buf + len, rates, 8);
    len += 8;

    if (!is_5ghz) {
        buf[len++] = IEEE80211_EID_EXT_SUPP_RATES;
        buf[len++] = (uint8_t)sizeof(rates_24_ext);
        memcpy(buf + len, rates_24_ext, sizeof(rates_24_ext));
        len += (uint16_t)sizeof(rates_24_ext);
    }
    return len;
}

/* ============================================================
 * BSS table
 * ============================================================ */

static struct vwifi_bss *bss_find(struct vwifi_dev *d, const uint8_t bssid[6])
{
    for (unsigned i = 0; i < VWIFI_BSS_MAX; i++) {
        if (d->bss[i].valid && memcmp(d->bss[i].bssid, bssid, 6) == 0) {
            return &d->bss[i];
        }
    }
    return NULL;
}

/* Find a free slot, or evict the oldest if the table is full. */
static struct vwifi_bss *bss_alloc(struct vwifi_dev *d, uint64_t now_us)
{
    struct vwifi_bss *oldest = NULL;

    for (unsigned i = 0; i < VWIFI_BSS_MAX; i++) {
        if (!d->bss[i].valid) return &d->bss[i];
        /* Age out stale entries opportunistically. */
        if (now_us - d->bss[i].last_seen_us > VWIFI_BSS_TTL_US) {
            d->bss[i].valid = false;
            return &d->bss[i];
        }
        if (!oldest || d->bss[i].last_seen_us < oldest->last_seen_us) {
            oldest = &d->bss[i];
        }
    }
    return oldest;   /* table full — recycle the least-recently-seen */
}

/* Emit a BSS_FOUND event for one table entry. The payload is a
 * vwifi_bss_entry followed by ie_len bytes of raw IEs. */
static void bss_emit(struct vwifi_dev *d, const struct vwifi_bss *b)
{
    uint8_t payload[sizeof(struct vwifi_bss_entry) + VWIFI_BSS_FRAME_MAX];
    struct vwifi_bss_entry *e = (struct vwifi_bss_entry *)payload;
    uint32_t total;

    memset(e, 0, sizeof(*e));
    memcpy(e->bssid, b->bssid, 6);
    e->channel_freq     = b->channel_freq;
    e->channel_flags    = d->channel.flags;
    e->rssi             = b->rssi;
    e->beacon_period_tu = b->beacon_period_tu;
    e->capability_info  = b->capability_info;
    e->tsf              = b->tsf;
    e->ssid_len         = b->ssid_len;
    memcpy(e->ssid, b->ssid, sizeof(e->ssid));
    /* ie_len carries the FULL frame length; the driver hands the whole
     * frame to WDI as WDI_TLV_BEACON_FRAME / _PROBE_RESPONSE_FRAME. */
    e->ie_len           = b->frame_len;
    if (b->is_beacon) e->capability_info |= VWIFI_BSS_F_BEACON;

    memcpy(payload + sizeof(*e), b->frame, b->frame_len);
    total = sizeof(*e) + b->frame_len;

    VWIFI_TRACE(d, "BSS_FOUND %02x:%02x:%02x:%02x:%02x:%02x "
                   "ssid='%s' freq=%u rssi=%d frame=%u (%s)",
                b->bssid[0], b->bssid[1], b->bssid[2],
                b->bssid[3], b->bssid[4], b->bssid[5],
                b->ssid_len ? (const char *)b->ssid : "<hidden>",
                b->channel_freq, b->rssi, b->frame_len,
                b->is_beacon ? "beacon" : "probe-resp");

    vwifi_post_event(d, VWIFI_EV_BSS_FOUND, payload, total);
}

/*
 * Does this BSS match the scan's SSID list?
 *
 * Two things count as "match everything":
 *
 *   - an empty list (num_ssids == 0), and
 *   - a zero-length SSID *entry*.
 *
 * The second is the wildcard SSID of 802.11, and getting it wrong is
 * expensive: cfg80211 puts one zero-length SSID in essentially every
 * scan request it issues, so `iw dev wlan0 scan` and NetworkManager
 * both arrive here with num_ssids = 1, ssid_len[0] = 0. Comparing that
 * literally -- ssid_len[i] == b->ssid_len -- matches only a hidden AP
 * and silently discards every named network on the air. The scan still
 * tunes each channel, still probes, still fills the BSS table; it just
 * reports nothing, which looks exactly like an empty medium.
 */
static bool bss_matches_scan_ssids(const struct vwifi_dev *d,
                                   const struct vwifi_bss *b)
{
    if (d->scan.num_ssids == 0) return true;
    for (unsigned i = 0; i < d->scan.num_ssids; i++) {
        if (d->scan.ssid_len[i] == 0) return true;      /* wildcard */
        if (d->scan.ssid_len[i] == b->ssid_len &&
            memcmp(d->scan.ssids[i], b->ssid, b->ssid_len) == 0) {
            return true;
        }
    }
    return false;
}

/* Called from the RX path for every beacon / probe response. Updates
 * the table and, if a scan is running, emits BSS_FOUND once per BSS
 * per scan. */
static void bss_observe(struct vwifi_dev *d,
                        const struct vwifi_frame_hdr *hdr,
                        const uint8_t *frame, uint16_t frame_len)
{
    const uint8_t *bssid = frame + 16;      /* addr3 */
    const uint8_t *fixed = frame + IEEE80211_MGMT_HDR_LEN;
    const uint8_t *ies   = frame + IEEE80211_IE_OFFSET;
    uint16_t ie_len = frame_len - IEEE80211_IE_OFFSET;
    uint64_t now = d->ops->now_us(d->be);
    struct vwifi_bss *b;
    uint8_t  ssid[33];
    uint16_t ssid_len;
    uint8_t  subtype = (frame[0] >> 4) & 0xF;

    if (!ies_find_ssid(ies, ie_len, ssid, &ssid_len)) {
        VWIFI_TRACE(d, "malformed IEs in beacon, ignoring");
        return;
    }

    b = bss_find(d, bssid);
    if (!b) {
        b = bss_alloc(d, now);
        if (!b) return;
        memset(b, 0, sizeof(*b));
        memcpy(b->bssid, bssid, 6);
        b->valid = true;
    }

    b->last_seen_us     = now;
    b->channel_freq     = hdr->channel_freq;
    b->rssi             = hdr->rssi;
    b->tsf              = get_le64(fixed);
    b->beacon_period_tu = get_le16(fixed + 8);
    b->capability_info  = get_le16(fixed + 10);
    b->ssid_len         = ssid_len;
    memcpy(b->ssid, ssid, sizeof(b->ssid));
    b->is_beacon        = (subtype == IEEE80211_SUBTYPE_BEACON);

    /* Keep the whole frame — WDI wants it verbatim. */
    b->frame_len = frame_len;
    if (b->frame_len > VWIFI_BSS_FRAME_MAX) b->frame_len = VWIFI_BSS_FRAME_MAX;
    memcpy(b->frame, frame, b->frame_len);

    /* Report during an active scan, once per BSS per scan. */
    if (d->scan.state == VWIFI_SCAN_DWELL &&
        !b->reported_this_scan &&
        bss_matches_scan_ssids(d, b)) {
        b->reported_this_scan = true;
        bss_emit(d, b);
    }
}

/* ============================================================
 * Scan state machine
 *
 * SCAN request -> build channel list -> tune ch[0], arm timer
 *   timer fires -> next channel, tune, re-arm
 *   ...
 *   last channel done -> restore channel, emit SCAN_COMPLETE
 *
 * Beacons arriving during a dwell are picked up by bss_observe()
 * from the normal RX path and emitted as BSS_FOUND events.
 * ============================================================ */

/* 2.4 GHz channel number -> centre frequency. */
static uint16_t chan_to_freq_24(unsigned ch)
{
    if (ch >= 1 && ch <= 13) return (uint16_t)(2407 + ch * 5);
    if (ch == 14)            return 2484;
    return 0;
}

/* 5 GHz channel number -> centre frequency (Phase 5).
 * Channels 36..165 follow a uniform 5 MHz grid from 5000 MHz; the
 * usable set is regulatory, but the medium is virtual so we accept the
 * whole span and let whatever is listening decide. */
static uint16_t chan_to_freq_5(unsigned ch)
{
    if (ch >= 36 && ch <= 165) return (uint16_t)(5000 + ch * 5);
    return 0;
}

/* Frequency -> channel number, either band. Used for the DS Parameter
 * Set IE in beacons. Returns 0 if the frequency isn't on a grid point. */
static unsigned freq_to_chan(uint16_t freq)
{
    if (freq == 2484)                    return 14;
    if (freq >= 2412 && freq <= 2472 && ((freq - 2407) % 5) == 0)
        return (freq - 2407) / 5;
    if (freq >= 5180 && freq <= 5825 && ((freq - 5000) % 5) == 0)
        return (freq - 5000) / 5;
    return 0;
}

/* Tune the device (and thus the medium channel filter) to a freq. */
static void scan_tune(struct vwifi_dev *d, uint16_t freq)
{
    d->channel.primary_freq = freq;
    VWIFI_TRACE(d, "scan: tuned to %u MHz", freq);
}

/* Build and inject a probe request for an active scan. */
static void scan_send_probe_req(struct vwifi_dev *d, unsigned ssid_idx)
{
    /* mgmt header + SSID IE + Supported Rates + Extended Supported Rates */
    uint8_t frame[IEEE80211_MGMT_HDR_LEN + (2 + 32) + (2 + 8) + (2 + 4)];
    uint16_t len = 0;
    uint8_t  ssid_len = 0;
    const uint8_t *ssid = NULL;

    if (ssid_idx < d->scan.num_ssids) {
        ssid_len = d->scan.ssid_len[ssid_idx];
        ssid     = d->scan.ssids[ssid_idx];
    }

    memset(frame, 0, sizeof(frame));
    frame[0] = (IEEE80211_SUBTYPE_PROBE_REQ << 4);   /* mgmt, probe-req */
    frame[1] = 0;
    memset(frame + 4,  0xFF, 6);           /* addr1 = broadcast */
    memcpy(frame + 10, d->sta_mac, 6);     /* addr2 = us */
    memset(frame + 16, 0xFF, 6);           /* addr3 = broadcast BSSID */
    len = IEEE80211_MGMT_HDR_LEN;

    /* SSID IE (zero-length for a broadcast probe). */
    frame[len++] = IEEE80211_EID_SSID;
    frame[len++] = ssid_len;
    if (ssid_len) {
        memcpy(frame + len, ssid, ssid_len);
        len += ssid_len;
    }

    len = ie_put_supp_rates(frame, len, d->channel.primary_freq);

    medium_send_frame(d, frame, len, 0, d->channel.primary_freq,
                      d->ops->now_us(d->be), false);
}

static void scan_finish(struct vwifi_dev *d, int32_t status)
{
    int32_t st = status;

    timer_clear(d, &d->timers.scan_us);
    d->scan.state = VWIFI_SCAN_IDLE;

    /* Restore the pre-scan channel. */
    d->channel = d->scan.saved_channel;

    VWIFI_TRACE(d, "scan complete (status %d), restored %u MHz",
                st, d->channel.primary_freq);

    vwifi_post_event(d, VWIFI_EV_SCAN_COMPLETE, &st, sizeof(st));
}

/* Move to the next channel in the list, or finish. */
static void scan_step(struct vwifi_dev *d)
{
    if (d->scan.cur_channel >= d->scan.num_channels) {
        scan_finish(d, 0);
        return;
    }

    scan_tune(d, d->scan.channels[d->scan.cur_channel]);

    if (d->scan.active) {
        if (d->scan.num_ssids == 0) {
            scan_send_probe_req(d, 0);        /* broadcast probe */
        } else {
            for (unsigned i = 0; i < d->scan.num_ssids; i++) {
                scan_send_probe_req(d, i);    /* one per directed SSID */
            }
        }
    }

    d->scan.cur_channel++;
    timer_set_rel(d, &d->timers.scan_us, d->scan.dwell_ms);
}

/* ============================================================
 * Timer scheduler
 *
 * Arm the host's single one-shot timer for the earliest pending
 * deadline. Called after any deadline changes.
 * ============================================================ */
static void timers_rearm(struct vwifi_dev *d)
{
    uint64_t now = d->ops->now_us(d->be);
    uint64_t next = 0;
    uint32_t ms;

    const uint64_t cands[] = {
        d->timers.scan_us, d->timers.conn_us, d->timers.beacon_us,
    };
    for (unsigned i = 0; i < sizeof(cands)/sizeof(cands[0]); i++) {
        if (cands[i] == 0) continue;            /* inactive */
        if (next == 0 || cands[i] < next) next = cands[i];
    }

    if (next == 0) {
        d->ops->timer_del(d->be);
        return;
    }

    /* Round up; a 0ms rearm would spin. */
    ms = (next <= now) ? 1 : (uint32_t)((next - now + 999) / 1000);
    if (ms == 0) ms = 1;
    d->ops->timer_mod(d->be, ms);
}

static void timer_set_rel(struct vwifi_dev *d, uint64_t *slot, uint32_t ms)
{
    *slot = d->ops->now_us(d->be) + (uint64_t)ms * 1000;
    timers_rearm(d);
}

static void timer_clear(struct vwifi_dev *d, uint64_t *slot)
{
    *slot = 0;
    timers_rearm(d);
}

void vwifi_timer_expired(struct vwifi_dev *d)
{
    uint64_t now = d->ops->now_us(d->be);

    /* Fire every deadline that has come due. Each handler either
     * re-arms its own slot or clears it. */
    if (d->timers.scan_us && now >= d->timers.scan_us) {
        d->timers.scan_us = 0;
        if (d->scan.state == VWIFI_SCAN_DWELL) scan_step(d);
    }
    if (d->timers.conn_us && now >= d->timers.conn_us) {
        d->timers.conn_us = 0;
        if (d->conn.state == VWIFI_CONN_AUTH_SENT ||
            d->conn.state == VWIFI_CONN_ASSOC_SENT) {
            conn_timeout(d);
        }
    }
    if (d->timers.beacon_us && now >= d->timers.beacon_us) {
        uint64_t due = d->timers.beacon_us;
        d->timers.beacon_us = 0;
        if (d->ap.running) ap_beacon_tick(d, due);
    }

    timers_rearm(d);
}

static int32_t op_scan(struct vwifi_dev *d, const void *in_buf, uint32_t in_len)
{
    const struct vwifi_scan_req *req = in_buf;
    const uint8_t *p;
    uint32_t mask;

    if (in_len < sizeof(*req)) return -22;              /* -EINVAL */
    if (d->scan.state != VWIFI_SCAN_IDLE) return -16;   /* -EBUSY */
    if (d->op_mode == VWIFI_MODE_MONITOR) return -1;    /* no scan in NetMon */
    /* Don't hop channels out from under an in-flight auth/assoc
     * exchange — they share the one-shot timer and the radio. */
    if (d->conn.state == VWIFI_CONN_AUTH_SENT ||
        d->conn.state == VWIFI_CONN_ASSOC_SENT) return -16;

    memset(&d->scan, 0, sizeof(d->scan));

    /* Build the channel list from the 2.4 GHz mask. A zero mask means
     * "all supported channels". 5 GHz is not yet supported by the
     * device, so channel_mask_5 is ignored. */
    /* "0 means all" must consider BOTH masks: a request for 5 GHz only
     * sets channel_mask_5 and leaves channel_mask_24 at zero, and must
     * NOT be silently widened to include all of 2.4 GHz. Only when
     * neither band is specified does the caller mean "everything". */
    if (req->channel_mask_24 == 0 && req->channel_mask_5 == 0) {
        mask = 0x3FFE;          /* 2.4 GHz channels 1..13 */
    } else {
        mask = req->channel_mask_24;
    }
    for (unsigned ch = 1; ch <= 14; ch++) {
        if (!(mask & (1u << ch))) continue;
        uint16_t freq = chan_to_freq_24(ch);
        if (!freq) continue;
        if (d->scan.num_channels >= VWIFI_SCAN_MAX_CHANNELS) break;
        d->scan.channels[d->scan.num_channels++] = freq;
    }

    /* 5 GHz (Phase 5). channel_mask_5 is a bitmask over the UNII
     * channel list below, not over raw channel numbers — 165 won't fit
     * in 64 bits otherwise. */
    if (req->channel_mask_5) {
        static const uint8_t unii[] = {
            36, 40, 44, 48, 52, 56, 60, 64,
            100, 104, 108, 112, 116, 120, 124, 128, 132, 136, 140, 144,
            149, 153, 157, 161, 165,
        };
        for (unsigned i = 0; i < sizeof(unii)/sizeof(unii[0]); i++) {
            if (!(req->channel_mask_5 & (1ULL << i))) continue;
            uint16_t freq = chan_to_freq_5(unii[i]);
            if (!freq) continue;
            if (d->scan.num_channels >= VWIFI_SCAN_MAX_CHANNELS) break;
            d->scan.channels[d->scan.num_channels++] = freq;
        }
    }

    if (d->scan.num_channels == 0) return -22;

    d->scan.dwell_ms = req->dwell_ms ? req->dwell_ms : 100;
    d->scan.active   = true;   /* always probe; passive-only is a later knob */

    /* Parse the trailing SSID list: num_ssids × { u8 len; u8 ssid[33] }. */
    d->scan.num_ssids = (req->num_ssids > 4) ? 4 : (uint8_t)req->num_ssids;
    p = (const uint8_t *)in_buf + sizeof(*req);
    for (unsigned i = 0; i < d->scan.num_ssids; i++) {
        uint32_t need = sizeof(*req) + (i + 1) * 34;
        if (in_len < need) { d->scan.num_ssids = (uint8_t)i; break; }
        uint8_t len = p[i * 34];
        if (len > 32) len = 32;
        d->scan.ssid_len[i] = len;
        memcpy(d->scan.ssids[i], p + i * 34 + 1, len);
        d->scan.ssids[i][len] = 0;
    }

    /* Clear per-scan reporting flags so every live BSS is re-reported. */
    for (unsigned i = 0; i < VWIFI_BSS_MAX; i++) {
        d->bss[i].reported_this_scan = false;
    }

    d->scan.saved_channel = d->channel;
    d->scan.cur_channel   = 0;
    d->scan.state         = VWIFI_SCAN_DWELL;

    VWIFI_TRACE(d, "scan start: %u channels, dwell %u ms, %u ssids",
                d->scan.num_channels, d->scan.dwell_ms, d->scan.num_ssids);

    /* Tune the first channel immediately; the timer drives the rest. */
    scan_step(d);
    return 0;
}

static int32_t op_scan_abort(struct vwifi_dev *d)
{
    if (d->scan.state == VWIFI_SCAN_IDLE) return 0;
    VWIFI_TRACE(d, "scan aborted by driver");
    scan_finish(d, -125);   /* -ECANCELED */
    return 0;
}

/* ============================================================
 * Connect / association state machine
 * ============================================================ */

#define IEEE80211_SUBTYPE_ASSOC_REQ    0
#define IEEE80211_SUBTYPE_ASSOC_RESP   1
#define IEEE80211_SUBTYPE_AUTH        11
#define IEEE80211_SUBTYPE_DEAUTH      12
#define IEEE80211_SUBTYPE_DISASSOC    10

#define IEEE80211_AUTH_ALG_OPEN        0
#define IEEE80211_AUTH_ALG_SAE         3

#define IEEE80211_STATUS_SUCCESS       0

/* Frame-control byte 1 bits for data frames. */
#define IEEE80211_FCTL_TODS    0x01
#define IEEE80211_FCTL_FROMDS  0x02

static inline void put_le16(uint8_t *p, uint16_t v)
{
    p[0] = (uint8_t)(v & 0xFF);
    p[1] = (uint8_t)(v >> 8);
}

/* Build the common 24-byte management header. */
static uint16_t mgmt_hdr(struct vwifi_dev *d, uint8_t *buf, uint8_t subtype,
                         const uint8_t da[6], const uint8_t bssid[6])
{
    memset(buf, 0, IEEE80211_MGMT_HDR_LEN);
    buf[0] = (uint8_t)(subtype << 4);      /* type=0 (mgmt) + subtype */
    buf[1] = 0;
    memcpy(buf + 4,  da, 6);               /* addr1 = DA */
    memcpy(buf + 10, d->sta_mac, 6);       /* addr2 = SA (us) */
    memcpy(buf + 16, bssid, 6);            /* addr3 = BSSID */
    put_le16(buf + 22, (uint16_t)(d->seq_num++ << 4));   /* seq ctrl */
    return IEEE80211_MGMT_HDR_LEN;
}

static void conn_send_auth_req(struct vwifi_dev *d)
{
    uint8_t frame[IEEE80211_MGMT_HDR_LEN + 6];
    uint16_t len = mgmt_hdr(d, frame, IEEE80211_SUBTYPE_AUTH,
                            d->conn.bssid, d->conn.bssid);

    /* Auth body: algorithm (2), sequence (2), status (2). */
    put_le16(frame + len, d->conn.auth_algo == VWIFI_AUTH_SAE
                            ? IEEE80211_AUTH_ALG_SAE
                            : IEEE80211_AUTH_ALG_OPEN);
    len += 2;
    put_le16(frame + len, 1);                        /* seq 1 */
    len += 2;
    put_le16(frame + len, IEEE80211_STATUS_SUCCESS); /* reserved in request */
    len += 2;

    VWIFI_TRACE(d, "conn: -> Auth Request (algo %u)", d->conn.auth_algo);
    medium_send_frame(d, frame, len, 0, d->conn.channel_freq,
                      d->ops->now_us(d->be), false);
}

static void conn_send_assoc_req(struct vwifi_dev *d)
{
    /* mgmt header + fixed body + SSID + rates + whatever the driver adds */
    uint8_t frame[IEEE80211_MGMT_HDR_LEN + 4 + (2 + 33) +
                  (2 + 8) + (2 + 4) + VWIFI_ASSOC_IE_MAX];
    uint16_t len = mgmt_hdr(d, frame, IEEE80211_SUBTYPE_ASSOC_REQ,
                            d->conn.bssid, d->conn.bssid);

    /* Assoc body: capability info (2), listen interval (2). */
    put_le16(frame + len, 0x0431);   /* ESS + short preamble/slot */
    len += 2;
    put_le16(frame + len, 10);       /* listen interval */
    len += 2;

    /* SSID IE — required in an association request. */
    frame[len++] = IEEE80211_EID_SSID;
    frame[len++] = (uint8_t)d->conn.ssid_len;
    memcpy(frame + len, d->conn.ssid, d->conn.ssid_len);
    len += d->conn.ssid_len;

    /* Supported (and Extended Supported) Rates. Required: an AP running
     * hostapd answers an Association Request without them with a bare
     * status 1, and the SSID element alone is not enough to associate. */
    len = ie_put_supp_rates(frame, len, d->conn.channel_freq);

    /* Driver-supplied IEs (RSN element for WPA2, etc). */
    if (d->conn.req_ie_len) {
        memcpy(frame + len, d->conn.req_ies, d->conn.req_ie_len);
        len += d->conn.req_ie_len;
    }

    VWIFI_TRACE(d, "conn: -> Assoc Request (ssid='%s', %u extra IE bytes)",
                d->conn.ssid, d->conn.req_ie_len);
    medium_send_frame(d, frame, len, 0, d->conn.channel_freq,
                      d->ops->now_us(d->be), false);
}

static void conn_send_deauth(struct vwifi_dev *d, uint16_t reason)
{
    uint8_t frame[IEEE80211_MGMT_HDR_LEN + 2];
    uint16_t len = mgmt_hdr(d, frame, IEEE80211_SUBTYPE_DEAUTH,
                            d->conn.bssid, d->conn.bssid);
    put_le16(frame + len, reason);
    len += 2;

    VWIFI_TRACE(d, "conn: -> Deauth (reason %u)", reason);
    medium_send_frame(d, frame, len, 0, d->conn.channel_freq,
                      d->ops->now_us(d->be), false);
}

/* Emit ASSOC_RESULT to the driver. */
static void conn_emit_assoc_result(struct vwifi_dev *d, uint16_t status_code,
                                   const uint8_t *resp_ies, uint16_t ie_len)
{
    uint8_t payload[sizeof(struct vwifi_assoc_result) + VWIFI_ASSOC_IE_MAX];
    struct vwifi_assoc_result *r = (struct vwifi_assoc_result *)payload;

    if (ie_len > VWIFI_ASSOC_IE_MAX) ie_len = VWIFI_ASSOC_IE_MAX;

    memset(r, 0, sizeof(*r));
    memcpy(r->bssid, d->conn.bssid, 6);
    r->status_code = status_code;
    r->aid         = d->conn.aid;
    r->ie_len      = ie_len;
    if (ie_len && resp_ies) {
        memcpy(payload + sizeof(*r), resp_ies, ie_len);
    }

    VWIFI_TRACE(d, "conn: ASSOC_RESULT status=%u aid=%u ies=%u",
                status_code, d->conn.aid, ie_len);
    vwifi_post_event(d, VWIFI_EV_ASSOC_RESULT, payload,
                     sizeof(*r) + ie_len);
}

static void conn_fail(struct vwifi_dev *d, uint16_t status_code)
{
    timer_clear(d, &d->timers.conn_us);
    d->conn.state = VWIFI_CONN_IDLE;
    conn_emit_assoc_result(d, status_code, NULL, 0);
}

/* Handle an Auth Response addressed to us. */
static void conn_rx_auth_resp(struct vwifi_dev *d,
                              const uint8_t *frame, uint16_t frame_len)
{
    const uint8_t *body = frame + IEEE80211_MGMT_HDR_LEN;
    uint16_t seq, status;

    if (d->conn.state != VWIFI_CONN_AUTH_SENT) return;
    if (frame_len < IEEE80211_MGMT_HDR_LEN + 6) return;
    if (memcmp(frame + 10, d->conn.bssid, 6) != 0) return;   /* not our AP */

    seq    = get_le16(body + 2);
    status = get_le16(body + 4);

    /* Open System: we sent seq 1, the AP replies with seq 2. */
    if (seq != 2) {
        VWIFI_TRACE(d, "conn: <- Auth Response seq %u (ignoring)", seq);
        return;
    }
    if (status != IEEE80211_STATUS_SUCCESS) {
        VWIFI_TRACE(d, "conn: <- Auth Response REJECTED status=%u", status);
        conn_fail(d, status);
        return;
    }

    VWIFI_TRACE(d, "conn: <- Auth Response OK, associating");
    d->conn.state = VWIFI_CONN_ASSOC_SENT;
    conn_send_assoc_req(d);
    timer_set_rel(d, &d->timers.conn_us, VWIFI_CONN_TIMEOUT_MS);
}

/* Handle an Assoc Response addressed to us. */
static void conn_rx_assoc_resp(struct vwifi_dev *d,
                               const uint8_t *frame, uint16_t frame_len)
{
    const uint8_t *body = frame + IEEE80211_MGMT_HDR_LEN;
    uint16_t status, aid, ie_len;
    const uint8_t *ies;

    if (d->conn.state != VWIFI_CONN_ASSOC_SENT) return;
    if (frame_len < IEEE80211_MGMT_HDR_LEN + 6) return;
    if (memcmp(frame + 10, d->conn.bssid, 6) != 0) return;

    /* Assoc Response body: capability (2), status (2), AID (2), IEs. */
    status = get_le16(body + 2);
    aid    = get_le16(body + 4) & 0x3FFF;   /* top two bits are reserved */
    ies    = body + 6;
    ie_len = (uint16_t)(frame_len - IEEE80211_MGMT_HDR_LEN - 6);

    timer_clear(d, &d->timers.conn_us);

    if (status != IEEE80211_STATUS_SUCCESS) {
        VWIFI_TRACE(d, "conn: <- Assoc Response REJECTED status=%u", status);
        d->conn.state = VWIFI_CONN_IDLE;
        conn_emit_assoc_result(d, status, NULL, 0);
        return;
    }

    d->conn.aid   = aid;
    d->conn.state = VWIFI_CONN_ASSOCIATED;

    VWIFI_TRACE(d, "conn: ASSOCIATED aid=%u", aid);
    conn_emit_assoc_result(d, IEEE80211_STATUS_SUCCESS, ies, ie_len);
}

/* Handle a Deauth/Disassoc from the AP. */
static void conn_rx_deauth(struct vwifi_dev *d,
                           const uint8_t *frame, uint16_t frame_len)
{
    struct vwifi_disconnect_ev ev = { 0 };

    if (d->conn.state == VWIFI_CONN_IDLE) return;
    if (memcmp(frame + 10, d->conn.bssid, 6) != 0) return;

    if (frame_len >= IEEE80211_MGMT_HDR_LEN + 2) {
        ev.reason_code = get_le16(frame + IEEE80211_MGMT_HDR_LEN);
    }
    ev.local = 0;

    timer_clear(d, &d->timers.conn_us);
    d->conn.state = VWIFI_CONN_IDLE;
    memset(&d->keys, 0, sizeof(d->keys));

    VWIFI_TRACE(d, "conn: <- Deauth from AP, reason %u", ev.reason_code);
    vwifi_post_event(d, VWIFI_EV_DISCONNECTED, &ev, sizeof(ev));
}

static int32_t op_connect(struct vwifi_dev *d, const void *in_buf, uint32_t in_len)
{
    const struct vwifi_connect_req *req = in_buf;

    if (in_len < sizeof(*req)) return -22;                  /* -EINVAL */
    if (d->op_mode == VWIFI_MODE_MONITOR) return -1;
    if (d->conn.state != VWIFI_CONN_IDLE) return -16;       /* -EBUSY */
    if (d->scan.state != VWIFI_SCAN_IDLE) return -16;       /* scan in flight */

    memset(&d->conn, 0, sizeof(d->conn));
    memcpy(d->conn.bssid, req->bssid, 6);
    d->conn.ssid_len = (req->ssid_len > 32) ? 32 : req->ssid_len;
    memcpy(d->conn.ssid, req->ssid, d->conn.ssid_len);
    d->conn.channel_freq    = req->channel_freq;
    d->conn.auth_algo       = req->auth_algo;
    d->conn.cipher_pairwise = req->cipher_pairwise;
    d->conn.cipher_group    = req->cipher_group;
    d->conn.akm_suite       = req->akm_suite;

    /* Copy the driver's association IEs (RSN element etc). */
    d->conn.req_ie_len = req->assoc_ie_len;
    if (d->conn.req_ie_len > VWIFI_ASSOC_IE_MAX) {
        d->conn.req_ie_len = VWIFI_ASSOC_IE_MAX;
    }
    if (d->conn.req_ie_len) {
        if (in_len < sizeof(*req) + d->conn.req_ie_len) return -22;
        memcpy(d->conn.req_ies, (const uint8_t *)in_buf + sizeof(*req),
               d->conn.req_ie_len);
    }

    /* Tune to the target's channel for the exchange. */
    if (d->conn.channel_freq) {
        d->channel.primary_freq = d->conn.channel_freq;
    } else {
        d->conn.channel_freq = d->channel.primary_freq;
    }

    VWIFI_TRACE(d, "conn: connecting to %02x:%02x:%02x:%02x:%02x:%02x "
                   "ssid='%s' freq=%u akm=%u",
                d->conn.bssid[0], d->conn.bssid[1], d->conn.bssid[2],
                d->conn.bssid[3], d->conn.bssid[4], d->conn.bssid[5],
                d->conn.ssid, d->conn.channel_freq, d->conn.akm_suite);

    d->conn.state = VWIFI_CONN_AUTH_SENT;
    conn_send_auth_req(d);
    timer_set_rel(d, &d->timers.conn_us, VWIFI_CONN_TIMEOUT_MS);
    return 0;
}

static int32_t op_disconnect(struct vwifi_dev *d,
                             const void *in_buf, uint32_t in_len)
{
    const struct vwifi_disconnect_req *req = in_buf;
    struct vwifi_disconnect_ev ev = { 0 };
    uint16_t reason = 3;   /* STA is leaving */

    if (in_len >= sizeof(*req)) reason = req->reason_code;
    if (d->conn.state == VWIFI_CONN_IDLE) return 0;

    if (d->conn.state == VWIFI_CONN_ASSOCIATED) {
        conn_send_deauth(d, reason);
    }

    timer_clear(d, &d->timers.conn_us);
    d->conn.state = VWIFI_CONN_IDLE;
    /* Keys are per-association; drop them so a stale PTK can never be
     * used against a new AP. */
    memset(&d->keys, 0, sizeof(d->keys));

    ev.reason_code = reason;
    ev.local       = 1;
    vwifi_post_event(d, VWIFI_EV_DISCONNECTED, &ev, sizeof(ev));
    return 0;
}

/* Route a management frame addressed to us into the connect machine.
 * Returns true if the frame was consumed internally. */
static bool conn_rx_mgmt(struct vwifi_dev *d,
                         const uint8_t *frame, uint16_t frame_len)
{
    uint8_t subtype;

    if (frame_len < IEEE80211_MGMT_HDR_LEN) return false;
    if (((frame[0] >> 2) & 0x3) != 0) return false;          /* not mgmt */
    if (memcmp(frame + 4, d->sta_mac, 6) != 0) return false; /* not for us */

    subtype = (frame[0] >> 4) & 0xF;
    switch (subtype) {
    case IEEE80211_SUBTYPE_AUTH:
        conn_rx_auth_resp(d, frame, frame_len);
        return true;
    case IEEE80211_SUBTYPE_ASSOC_RESP:
        conn_rx_assoc_resp(d, frame, frame_len);
        return true;
    case IEEE80211_SUBTYPE_DEAUTH:
    case IEEE80211_SUBTYPE_DISASSOC:
        conn_rx_deauth(d, frame, frame_len);
        return true;
    default:
        return false;
    }
}

/* Connect timeout — no Auth/Assoc Response arrived in time. */
static void conn_timeout(struct vwifi_dev *d)
{
    VWIFI_TRACE(d, "conn: timeout in state %u", d->conn.state);
    /* 802.11 status 16 = "authentication sequence timeout"; use it for
     * both stages so the driver sees a definite failure code. */
    conn_fail(d, 16);
}


/* ============================================================
 * SoftAP implementation (Phase 5)
 * ============================================================ */

/* One Time Unit is 1024 microseconds, not 1000. Getting this wrong
 * makes beacon timing drift ~2.4% — invisible in casual testing,
 * obvious to anything measuring beacon intervals. */
#define TU_TO_US(tu)  ((uint64_t)(tu) * 1024ULL)

static struct vwifi_ap_sta *ap_sta_find(struct vwifi_dev *d, const uint8_t mac[6])
{
    for (unsigned i = 0; i < VWIFI_AP_MAX_STA; i++) {
        if (d->ap.sta[i].valid && memcmp(d->ap.sta[i].mac, mac, 6) == 0) {
            return &d->ap.sta[i];
        }
    }
    return NULL;
}

static struct vwifi_ap_sta *ap_sta_alloc(struct vwifi_dev *d, const uint8_t mac[6])
{
    for (unsigned i = 0; i < VWIFI_AP_MAX_STA; i++) {
        if (!d->ap.sta[i].valid) {
            struct vwifi_ap_sta *st = &d->ap.sta[i];
            memset(st, 0, sizeof(*st));
            memcpy(st->mac, mac, 6);
            st->valid = true;
            st->aid   = d->ap.next_aid++;
            if (d->ap.next_aid > 2007) d->ap.next_aid = 1;  /* 802.11 AID range */
            return st;
        }
    }
    return NULL;   /* table full */
}

/* Build a beacon or probe response. They share a body layout:
 * timestamp(8) beacon-interval(2) capability(2) then IEs. */
static uint16_t ap_build_beacon_like(struct vwifi_dev *d, uint8_t *buf,
                                     uint8_t subtype, const uint8_t *da)
{
    uint16_t len;
    uint16_t caps = 0x0001;   /* ESS */

    /* Advertise privacy when the driver gave us an RSN element. */
    if (d->ap.ie_len) caps |= 0x0010;

    len = mgmt_hdr(d, buf, subtype, da, d->sta_mac);

    /* Timestamp: our TSF. */
    {
        uint64_t tsf = d->ops->now_us(d->be);
        for (int i = 0; i < 8; i++) buf[len + i] = (uint8_t)(tsf >> (8 * i));
        len += 8;
    }
    put_le16(buf + len, d->ap.beacon_interval_tu); len += 2;
    put_le16(buf + len, caps);                     len += 2;

    /* SSID IE. */
    buf[len++] = IEEE80211_EID_SSID;
    buf[len++] = (uint8_t)d->ap.ssid_len;
    memcpy(buf + len, d->ap.ssid, d->ap.ssid_len);
    len += d->ap.ssid_len;

    /* Supported rates IE — a station that sees no rates may refuse to
     * associate, so this is not optional in practice. Band-aware: the
     * CCK rates this used to advertise unconditionally do not exist on
     * 5 GHz, and a station that reads them there sees a beacon whose
     * basic-rate set it cannot satisfy. */
    len = ie_put_supp_rates(buf, len, d->ap.channel_freq);

    /* DS Parameter Set — tells scanners which channel we're really on,
     * which matters when a beacon is heard on an adjacent channel. */
    buf[len++] = 3;    /* EID_DS_PARAMS */
    buf[len++] = 1;
    buf[len++] = (uint8_t)freq_to_chan(d->ap.channel_freq);

    /* Driver-supplied IEs (RSN for WPA2, vendor elements, ...). */
    if (d->ap.ie_len) {
        memcpy(buf + len, d->ap.ies, d->ap.ie_len);
        len += d->ap.ie_len;
    }
    return len;
}

static void ap_beacon_send(struct vwifi_dev *d)
{
    uint8_t frame[256 + VWIFI_AP_IE_MAX];
    uint16_t len;
    static const uint8_t bcast[6] = { 0xFF,0xFF,0xFF,0xFF,0xFF,0xFF };

    if (!d->ap.running) return;

    len = ap_build_beacon_like(d, frame, IEEE80211_SUBTYPE_BEACON, bcast);
    medium_send_frame(d, frame, len, 0, d->ap.channel_freq,
                      d->ops->now_us(d->be), false);
}

/* Send a beacon and schedule the next one.
 *
 * `due_us` is the deadline this beacon was scheduled for, or 0 for the
 * very first beacon. The next deadline advances from `due_us`, NOT from
 * the current time — the host timer only has millisecond granularity,
 * so re-basing on "now" every interval would accumulate the rounding
 * error into steadily drifting beacons. Anything measuring beacon
 * timing (a sniffer, a station's TSF sync) would see the drift. */
static void ap_beacon_tick(struct vwifi_dev *d, uint64_t due_us)
{
    uint64_t base;

    if (!d->ap.running) return;

    ap_beacon_send(d);

    base = due_us ? due_us : d->ops->now_us(d->be);
    d->timers.beacon_us = base + TU_TO_US(d->ap.beacon_interval_tu);
}

static void ap_send_probe_resp(struct vwifi_dev *d, const uint8_t *da)
{
    uint8_t frame[256 + VWIFI_AP_IE_MAX];
    uint16_t len = ap_build_beacon_like(d, frame,
                                        IEEE80211_SUBTYPE_PROBE_RESP, da);
    VWIFI_TRACE(d, "ap: -> Probe Response to %02x:%02x:%02x:%02x:%02x:%02x",
                da[0], da[1], da[2], da[3], da[4], da[5]);
    medium_send_frame(d, frame, len, 0, d->ap.channel_freq,
                      d->ops->now_us(d->be), false);
}

static void ap_send_auth_resp(struct vwifi_dev *d, const uint8_t *da,
                              uint16_t status)
{
    uint8_t frame[IEEE80211_MGMT_HDR_LEN + 6];
    uint16_t len = mgmt_hdr(d, frame, IEEE80211_SUBTYPE_AUTH, da, d->sta_mac);

    put_le16(frame + len, IEEE80211_AUTH_ALG_OPEN); len += 2;
    put_le16(frame + len, 2);                       len += 2;  /* seq 2 */
    put_le16(frame + len, status);                  len += 2;

    VWIFI_TRACE(d, "ap: -> Auth Response status=%u", status);
    medium_send_frame(d, frame, len, 0, d->ap.channel_freq,
                      d->ops->now_us(d->be), false);
}

static void ap_send_assoc_resp(struct vwifi_dev *d, const uint8_t *da,
                               uint16_t status, uint16_t aid)
{
    uint8_t frame[IEEE80211_MGMT_HDR_LEN + 6 + 16];
    uint16_t len = mgmt_hdr(d, frame, IEEE80211_SUBTYPE_ASSOC_RESP,
                            da, d->sta_mac);

    put_le16(frame + len, 0x0001);           len += 2;   /* capability: ESS */
    put_le16(frame + len, status);           len += 2;
    /* Real APs set the top two bits of the AID field. */
    put_le16(frame + len, (uint16_t)(aid | 0xC000)); len += 2;

    /* Supported rates, echoed back. */
    frame[len++] = 1; frame[len++] = 4;
    frame[len++] = 0x82; frame[len++] = 0x84;
    frame[len++] = 0x8B; frame[len++] = 0x96;

    VWIFI_TRACE(d, "ap: -> Assoc Response status=%u aid=%u", status, aid);
    medium_send_frame(d, frame, len, 0, d->ap.channel_freq,
                      d->ops->now_us(d->be), false);
}

/* Does a probe request target us? A zero-length SSID IE is a wildcard
 * probe, which every AP answers. */
static bool ap_probe_matches(struct vwifi_dev *d,
                             const uint8_t *frame, uint16_t frame_len)
{
    const uint8_t *ies = frame + IEEE80211_MGMT_HDR_LEN;
    uint16_t ie_len;
    uint8_t  ssid[33];
    uint16_t ssid_len;

    if (frame_len < IEEE80211_MGMT_HDR_LEN) return false;
    ie_len = frame_len - IEEE80211_MGMT_HDR_LEN;

    if (!ies_find_ssid(ies, ie_len, ssid, &ssid_len)) return false;
    if (ssid_len == 0) return true;                       /* wildcard */
    if (ssid_len != d->ap.ssid_len) return false;
    return memcmp(ssid, d->ap.ssid, ssid_len) == 0;
}

/* Handle a management frame arriving while we are an AP. Returns true
 * if consumed. */
static bool ap_rx_mgmt(struct vwifi_dev *d,
                       const uint8_t *frame, uint16_t frame_len)
{
    uint8_t subtype;
    const uint8_t *sa;
    struct vwifi_ap_sta *st;
    static const uint8_t bcast[6] = { 0xFF,0xFF,0xFF,0xFF,0xFF,0xFF };

    if (!d->ap.running) return false;
    if (frame_len < IEEE80211_MGMT_HDR_LEN) return false;
    if (((frame[0] >> 2) & 0x3) != 0) return false;      /* not mgmt */

    subtype = (frame[0] >> 4) & 0xF;
    sa = frame + 10;   /* addr2 */

    /* Ignore our own transmissions reflected back. */
    if (memcmp(sa, d->sta_mac, 6) == 0) return false;

    switch (subtype) {
    case IEEE80211_SUBTYPE_PROBE_REQ:
        /* Probe requests are broadcast; answer if we match. */
        if (ap_probe_matches(d, frame, frame_len)) {
            ap_send_probe_resp(d, sa);
        }
        return true;

    case IEEE80211_SUBTYPE_AUTH:
        if (memcmp(frame + 4, d->sta_mac, 6) != 0) return false;  /* not to us */
        {
            uint16_t seq = get_le16(frame + IEEE80211_MGMT_HDR_LEN + 2);
            if (seq != 1) return true;   /* only Open System seq 1 */
        }
        st = ap_sta_find(d, sa);
        if (!st) st = ap_sta_alloc(d, sa);
        if (!st) {
            /* Station table full — 802.11 status 17 is
             * "association denied: AP unable to handle more STAs". */
            ap_send_auth_resp(d, sa, 17);
            return true;
        }
        st->authenticated = true;
        st->last_seen_us  = d->ops->now_us(d->be);
        VWIFI_TRACE(d, "ap: <- Auth Request from "
                       "%02x:%02x:%02x:%02x:%02x:%02x, accepting",
                    sa[0], sa[1], sa[2], sa[3], sa[4], sa[5]);
        ap_send_auth_resp(d, sa, IEEE80211_STATUS_SUCCESS);
        return true;

    case IEEE80211_SUBTYPE_ASSOC_REQ:
        if (memcmp(frame + 4, d->sta_mac, 6) != 0) return false;
        st = ap_sta_find(d, sa);
        if (!st || !st->authenticated) {
            /* 802.11 status 1 = unspecified failure. A real AP would
             * send a Deauth with reason 6/7 here; status 1 is enough to
             * make the station restart from Auth. */
            ap_send_assoc_resp(d, sa, 1, 0);
            return true;
        }
        st->associated   = true;
        st->last_seen_us = d->ops->now_us(d->be);
        VWIFI_TRACE(d, "ap: <- Assoc Request, associating aid=%u", st->aid);
        ap_send_assoc_resp(d, sa, IEEE80211_STATUS_SUCCESS, st->aid);

        /* Tell the driver a station joined so the OS can start its
         * authenticator (the 4-way handshake) for this peer. */
        {
            struct vwifi_assoc_result ev;
            memset(&ev, 0, sizeof(ev));
            memcpy(ev.bssid, sa, 6);      /* the peer, from the AP's view */
            ev.status_code = 0;
            ev.aid         = st->aid;
            vwifi_post_event(d, VWIFI_EV_ASSOC_RESULT, &ev, sizeof(ev));
        }
        return true;

    case IEEE80211_SUBTYPE_DEAUTH:
    case IEEE80211_SUBTYPE_DISASSOC:
        st = ap_sta_find(d, sa);
        if (st) {
            struct vwifi_disconnect_ev ev = { 0 };
            if (frame_len >= IEEE80211_MGMT_HDR_LEN + 2) {
                ev.reason_code = get_le16(frame + IEEE80211_MGMT_HDR_LEN);
            }
            VWIFI_TRACE(d, "ap: <- Deauth from station, reason %u",
                        ev.reason_code);
            memset(st, 0, sizeof(*st));
            vwifi_post_event(d, VWIFI_EV_DISCONNECTED, &ev, sizeof(ev));
        }
        return true;

    default:
        (void)bcast;
        return false;
    }
}

static int32_t op_start_ap(struct vwifi_dev *d, const void *in_buf, uint32_t in_len)
{
    const struct vwifi_ap_config *cfg = in_buf;

    if (in_len < sizeof(*cfg)) return -22;
    if (d->op_mode != VWIFI_MODE_SOFTAP) return -1;
    if (d->ap.running) return -16;   /* -EBUSY */

    memset(&d->ap, 0, sizeof(d->ap));
    d->ap.ssid_len = (cfg->ssid_len > 32) ? 32 : cfg->ssid_len;
    memcpy(d->ap.ssid, cfg->ssid, d->ap.ssid_len);
    d->ap.channel_freq       = cfg->channel_freq ? cfg->channel_freq
                                                 : d->channel.primary_freq;
    d->ap.beacon_interval_tu = cfg->beacon_interval_tu ? cfg->beacon_interval_tu
                                                       : 100;
    d->ap.dtim_period        = cfg->dtim_period ? cfg->dtim_period : 1;
    d->ap.next_aid           = 1;

    d->ap.ie_len = cfg->ie_len;
    if (d->ap.ie_len > VWIFI_AP_IE_MAX) d->ap.ie_len = VWIFI_AP_IE_MAX;
    if (d->ap.ie_len) {
        if (in_len < sizeof(*cfg) + d->ap.ie_len) return -22;
        memcpy(d->ap.ies, (const uint8_t *)in_buf + sizeof(*cfg), d->ap.ie_len);
    }

    /* The AP owns the radio channel while running. */
    d->channel.primary_freq = d->ap.channel_freq;
    d->ap.running = true;

    VWIFI_TRACE(d, "ap: started ssid='%s' freq=%u bi=%u TU ies=%u",
                d->ap.ssid, d->ap.channel_freq,
                d->ap.beacon_interval_tu, d->ap.ie_len);

    /* First beacon immediately, then on the interval. */
    ap_beacon_tick(d, 0);
    timers_rearm(d);
    return 0;
}

static int32_t op_stop_ap(struct vwifi_dev *d)
{
    if (!d->ap.running) return 0;

    /* Deauth every associated station so they don't linger waiting on a
     * BSS that has gone away. */
    for (unsigned i = 0; i < VWIFI_AP_MAX_STA; i++) {
        if (!d->ap.sta[i].valid) continue;
        {
            uint8_t frame[IEEE80211_MGMT_HDR_LEN + 2];
            uint16_t len = mgmt_hdr(d, frame, IEEE80211_SUBTYPE_DEAUTH,
                                    d->ap.sta[i].mac, d->sta_mac);
            put_le16(frame + len, 3);   /* STA is leaving */
            len += 2;
            medium_send_frame(d, frame, len, 0, d->ap.channel_freq,
                              d->ops->now_us(d->be), false);
        }
    }

    VWIFI_TRACE(d, "ap: stopped");
    memset(&d->ap, 0, sizeof(d->ap));
    timer_clear(d, &d->timers.beacon_us);
    return 0;
}

/* ============================================================
 * STA-mode data path: 802.3 <-> 802.11 conversion
 *
 * An 802.3 frame is:
 *   dst(6) src(6) ethertype(2) payload...
 *
 * An 802.11 data frame carrying the same payload is:
 *   hdr(24) LLC/SNAP(8) payload...
 * where LLC/SNAP is AA AA 03 00 00 00 followed by the ethertype.
 *
 * Address mapping for an infrastructure STA:
 *   uplink   (ToDS=1, FromDS=0): addr1=BSSID, addr2=SA(us), addr3=DA
 *   downlink (ToDS=0, FromDS=1): addr1=DA(us), addr2=BSSID, addr3=SA
 *
 * Doing this in the device rather than the Windows driver keeps the
 * kernel-mode code simple and — more usefully — makes the conversion
 * testable without a Windows guest.
 * ============================================================ */

#define IEEE80211_DATA_HDR_LEN  24
#define LLC_SNAP_LEN             8
#define ETH_HDR_LEN             14

static const uint8_t llc_snap_hdr[6] = { 0xAA, 0xAA, 0x03, 0x00, 0x00, 0x00 };

/* Convert an 802.3 frame from the driver into an 802.11 data frame.
 * Returns the 802.11 frame length, or 0 if the input is malformed. */
static uint16_t sta_tx_8023_to_80211(struct vwifi_dev *d,
                                     const uint8_t *eth, uint16_t eth_len,
                                     uint8_t *out, uint16_t out_cap)
{
    uint16_t payload_len;
    uint16_t len = 0;

    if (eth_len < ETH_HDR_LEN) return 0;
    payload_len = eth_len - ETH_HDR_LEN;
    if (IEEE80211_DATA_HDR_LEN + LLC_SNAP_LEN + payload_len > out_cap) return 0;

    memset(out, 0, IEEE80211_DATA_HDR_LEN);
    out[0] = 0x08;                       /* type=2 (data), subtype=0 */
    out[1] = IEEE80211_FCTL_TODS;        /* uplink to the AP */
    memcpy(out + 4,  d->conn.bssid, 6);  /* addr1 = BSSID (RA) */
    memcpy(out + 10, d->sta_mac, 6);     /* addr2 = SA (us) */
    memcpy(out + 16, eth, 6);            /* addr3 = DA (from 802.3 dst) */
    put_le16(out + 22, (uint16_t)(d->seq_num++ << 4));
    len = IEEE80211_DATA_HDR_LEN;

    /* LLC/SNAP + ethertype copied straight from the 802.3 header. */
    memcpy(out + len, llc_snap_hdr, sizeof(llc_snap_hdr));
    len += (uint16_t)sizeof(llc_snap_hdr);
    out[len++] = eth[12];
    out[len++] = eth[13];

    memcpy(out + len, eth + ETH_HDR_LEN, payload_len);
    len += payload_len;
    return len;
}

/* Convert a received 802.11 data frame into an 802.3 frame for the
 * driver. Returns the 802.3 length, or 0 if the frame isn't a
 * well-formed data frame for our BSS. */
static uint16_t sta_rx_80211_to_8023(struct vwifi_dev *d,
                                     const uint8_t *frame, uint16_t frame_len,
                                     uint8_t *out, uint16_t out_cap)
{
    const uint8_t *da, *sa, *body;
    uint16_t payload_len, hdr_len = IEEE80211_DATA_HDR_LEN;
    uint8_t  subtype = (frame[0] >> 4) & 0xF;
    bool     tods    = (frame[1] & IEEE80211_FCTL_TODS)   != 0;
    bool     fromds  = (frame[1] & IEEE80211_FCTL_FROMDS) != 0;

    /* QoS data frames carry an extra 2-byte QoS control field. */
    if (subtype & 0x08) hdr_len += 2;

    if (frame_len < hdr_len + LLC_SNAP_LEN + 2) return 0;

    /* We only handle downlink frames from our AP. */
    if (!fromds || tods) return 0;
    if (memcmp(frame + 10, d->conn.bssid, 6) != 0) return 0;   /* addr2=BSSID */

    da = frame + 4;    /* addr1 */
    sa = frame + 16;   /* addr3 */

    body = frame + hdr_len;
    if (memcmp(body, llc_snap_hdr, sizeof(llc_snap_hdr)) != 0) {
        /* Not LLC/SNAP encapsulated — can't express as 802.3. */
        return 0;
    }

    payload_len = frame_len - hdr_len - LLC_SNAP_LEN;
    if (ETH_HDR_LEN + payload_len > out_cap) return 0;

    memcpy(out,     da, 6);
    memcpy(out + 6, sa, 6);
    out[12] = body[6];    /* ethertype high */
    out[13] = body[7];    /* ethertype low  */
    memcpy(out + ETH_HDR_LEN, body + LLC_SNAP_LEN, payload_len);

    return (uint16_t)(ETH_HDR_LEN + payload_len);
}

/* Is this a data frame belonging to our current BSS? */
static bool sta_is_our_data_frame(const struct vwifi_dev *d,
                                  const uint8_t *frame, uint16_t frame_len)
{
    if (frame_len < IEEE80211_DATA_HDR_LEN) return false;
    if (((frame[0] >> 2) & 0x3) != 2) return false;   /* not data */
    if (d->conn.state != VWIFI_CONN_ASSOCIATED) return false;
    /* Downlink from our AP, addressed to us or to a group address. */
    if (!(frame[1] & IEEE80211_FCTL_FROMDS)) return false;
    if (memcmp(frame + 10, d->conn.bssid, 6) != 0) return false;
    if (memcmp(frame + 4, d->sta_mac, 6) != 0 && !(frame[4] & 0x01)) {
        return false;   /* not unicast to us and not multicast/broadcast */
    }
    return true;
}

/* ============================================================
 * Cipher keys (Phase 4)
 * ============================================================ */

#define ETHERTYPE_EAPOL  0x888E

/* The 4-way handshake runs over EAPOL frames carried as ordinary data
 * frames. Those MUST go out unencrypted — the keys don't exist yet
 * when the handshake starts, and the handshake's own frames stay in
 * the clear throughout. Getting this wrong deadlocks WPA2: we'd
 * encrypt with a key the AP hasn't derived, it drops the frame, and
 * the handshake never completes. */
static bool eth_is_eapol(const uint8_t *eth, uint16_t eth_len)
{
    if (eth_len < ETH_HDR_LEN) return false;
    return (((uint16_t)eth[12] << 8) | eth[13]) == ETHERTYPE_EAPOL;
}

static struct vwifi_key_slot *key_slot_for_tx(struct vwifi_dev *d)
{
    /* A STA always transmits under the PTK; only an AP uses the GTK
     * for downlink group traffic. */
    return d->keys.pairwise.valid ? &d->keys.pairwise : NULL;
}

static struct vwifi_key_slot *key_slot_for_rx(struct vwifi_dev *d,
                                              const uint8_t *frame,
                                              uint8_t key_id)
{
    bool group = (frame[4] & 0x01) != 0;   /* addr1 is a group address */

    if (group) {
        if (key_id < 4 && d->keys.group[key_id].valid) {
            return &d->keys.group[key_id];
        }
        return NULL;
    }
    return d->keys.pairwise.valid ? &d->keys.pairwise : NULL;
}

static int32_t op_set_key(struct vwifi_dev *d, const void *in_buf, uint32_t in_len)
{
    const struct vwifi_key *k = in_buf;
    struct vwifi_key_slot *slot;
    struct vwifi_key_id ev;

    if (in_len < sizeof(*k)) return -22;
    if (k->cipher != VWIFI_CIPHER_CCMP128) {
        VWIFI_ERR(d, "unsupported cipher %u (only CCMP-128)", k->cipher);
        return -95;   /* -EOPNOTSUPP */
    }
    if (k->key_len != VWIFI_CCMP_KEY_LEN) {
        VWIFI_ERR(d, "bad key length %u for CCMP-128", k->key_len);
        return -22;
    }

    if (k->id.pairwise) {
        slot = &d->keys.pairwise;
    } else {
        if (k->id.key_idx >= 4) return -22;
        slot = &d->keys.group[k->id.key_idx];
    }

    memset(slot, 0, sizeof(*slot));
    vwifi_aes128_key_expand(k->key, &slot->aes);
    slot->cipher      = k->cipher;
    slot->key_idx     = k->id.key_idx;
    slot->valid       = true;
    slot->rx_pn_valid = false;
    /* TX PN starts at 1; PN 0 is never transmitted. */
    memset(slot->tx_pn, 0, VWIFI_PN_LEN);
    slot->tx_pn[VWIFI_PN_LEN - 1] = 1;

    VWIFI_TRACE(d, "key installed: %s idx=%u cipher=%u",
                k->id.pairwise ? "pairwise" : "group",
                k->id.key_idx, k->cipher);

    ev = k->id;
    vwifi_post_event(d, VWIFI_EV_KEY_INSTALLED, &ev, sizeof(ev));
    return 0;
}

static int32_t op_del_key(struct vwifi_dev *d, const void *in_buf, uint32_t in_len)
{
    const struct vwifi_key_id *id = in_buf;

    if (in_len < sizeof(*id)) return -22;

    if (id->pairwise) {
        memset(&d->keys.pairwise, 0, sizeof(d->keys.pairwise));
    } else {
        if (id->key_idx >= 4) return -22;
        memset(&d->keys.group[id->key_idx], 0, sizeof(d->keys.group[0]));
    }
    VWIFI_TRACE(d, "key deleted: %s idx=%u",
                id->pairwise ? "pairwise" : "group", id->key_idx);
    return 0;
}

static void post_rsp(struct vwifi_dev *d,
                     const struct vwifi_ctrl_rsp_desc *rsp,
                     const void *payload, uint32_t payload_len)
{
    struct vwifi_ctrl_rsp_desc out;
    uint64_t desc_gpa;
    uint32_t idx;

    if (!d->ctrl_rsp.enabled) return;

    idx = d->ctrl_rsp.next_idx;
    desc_gpa = ring_desc_gpa(&d->ctrl_rsp, idx);

    if (d->ops->dma_read(d->be, desc_gpa, &out, sizeof(out)) != 0) return;
    if (!(out.flags & VWIFI_DESC_F_OWN)) {
        VWIFI_ERR(d, "no free ctrl-rsp slot at idx %u (opcode 0x%04x)",
                  idx, rsp->opcode);
        d->drops++;
        d->regs[VWIFI_REG_DIAG_DROPS / 4] = d->drops;
        return;
    }

    if (payload && payload_len) {
        if (payload_len > out.payload_len) payload_len = out.payload_len;
        d->ops->dma_write(d->be, out.payload_addr, payload, payload_len);
    }

    out.opcode      = rsp->opcode;
    out.flags       = rsp->flags & ~VWIFI_DESC_F_OWN;
    out.req_id      = rsp->req_id;
    out.status      = rsp->status;
    out.payload_len = payload_len;

    d->ops->dma_write(d->be, desc_gpa, &out, sizeof(out));

    d->ctrl_rsp.next_idx = (idx + 1) & d->ctrl_rsp.mask;
    d->regs[VWIFI_REG_CTRL_RSP_RING_TAIL / 4] = d->ctrl_rsp.next_idx;

    vwifi_raise_irq(d, VWIFI_VEC_CTRL_RSP);
}

/* Public helper for the phase-2+ event stream. */
void vwifi_post_event(struct vwifi_dev *d, uint16_t event_code,
                      const void *payload, uint32_t payload_len)
{
    struct vwifi_ctrl_rsp_desc rsp = {
        .opcode      = event_code,
        .flags       = VWIFI_RSP_F_EVENT,
        .req_id      = 0,
        .status      = 0,
        .payload_len = payload_len,
    };
    post_rsp(d, &rsp, payload, payload_len);
}

static int32_t op_get_caps(struct vwifi_dev *d, void *out_buf, uint32_t *out_len)
{
    struct vwifi_caps *c = out_buf;
    if (*out_len < sizeof(*c)) return -1;

    memset(c, 0, sizeof(*c));
    c->abi_version = VWIFI_ABI_VERSION;
    c->caps = VWIFI_CAP_MONITOR | VWIFI_CAP_INJECT | VWIFI_CAP_STA |
              VWIFI_CAP_WPA2 | VWIFI_CAP_SOFTAP;
    memcpy(c->default_mac, d->sta_mac, 6);
    c->supported_channels_24 = 0x3FFE;       /* channels 1..13 */
    /* 25 UNII channels, indexed as in op_scan's table. */
    c->supported_channels_5  = 0x1FFFFFFULL;
    c->max_scan_ssids        = 4;
    c->max_bss_entries       = 64;

    *out_len = sizeof(*c);
    return 0;
}

static int32_t op_set_op_mode(struct vwifi_dev *d,
                              const void *in_buf, uint32_t in_len)
{
    const struct vwifi_op_mode *m = in_buf;
    if (in_len < sizeof(*m)) return -1;
    if (m->mode > VWIFI_MODE_SOFTAP) return -1;
    VWIFI_TRACE(d, "op_mode %u -> %u", d->op_mode, m->mode);
    d->op_mode = m->mode;
    return 0;
}

static int32_t op_set_channel(struct vwifi_dev *d,
                              const void *in_buf, uint32_t in_len)
{
    const struct vwifi_channel *c = in_buf;
    if (in_len < sizeof(*c)) return -1;
    d->channel = *c;
    VWIFI_TRACE(d, "channel -> %u MHz (flags 0x%04x)",
                c->primary_freq, c->flags);
    return 0;
}

static int32_t op_set_raw_filter(struct vwifi_dev *d,
                                 const void *in_buf, uint32_t in_len)
{
    const struct vwifi_raw_filter *f = in_buf;
    if (in_len < sizeof(*f)) return -1;
    d->raw_filter = f->mask;
    VWIFI_TRACE(d, "raw_filter -> 0x%08x", f->mask);
    return 0;
}

static int32_t op_set_sta_mac(struct vwifi_dev *d,
                              const void *in_buf, uint32_t in_len)
{
    if (in_len < 6) return -1;
    memcpy(d->sta_mac, in_buf, 6);
    VWIFI_TRACE(d, "sta_mac -> %02x:%02x:%02x:%02x:%02x:%02x",
                d->sta_mac[0], d->sta_mac[1], d->sta_mac[2],
                d->sta_mac[3], d->sta_mac[4], d->sta_mac[5]);
    return 0;
}

static void handle_ctrl_req(struct vwifi_dev *d,
                            const struct vwifi_ctrl_req_desc *req)
{
    uint8_t  in_buf[2048];
    uint8_t  out_buf[2048];
    uint32_t in_len  = req->payload_len;
    uint32_t out_len = sizeof(out_buf);
    struct vwifi_ctrl_rsp_desc rsp = {
        .opcode = req->opcode,
        .req_id = req->req_id,
        .flags  = 0,
        .status = 0,
    };
    int32_t status = 0;
    const void *out_payload = NULL;
    uint32_t    out_payload_len = 0;

    if (in_len > sizeof(in_buf)) { status = -1; goto respond; }
    if (in_len && req->payload_addr) {
        if (d->ops->dma_read(d->be, req->payload_addr, in_buf, in_len) != 0) {
            status = -5;   /* -EIO */
            goto respond;
        }
    }

    switch (req->opcode) {
    case VWIFI_OP_NOP:            status = 0; break;
    case VWIFI_OP_GET_CAPS:
        status = op_get_caps(d, out_buf, &out_len);
        if (status == 0) { out_payload = out_buf; out_payload_len = out_len; }
        break;
    case VWIFI_OP_SET_STA_MAC:    status = op_set_sta_mac(d, in_buf, in_len); break;
    case VWIFI_OP_SET_OP_MODE:    status = op_set_op_mode(d, in_buf, in_len); break;
    case VWIFI_OP_SET_CHANNEL:    status = op_set_channel(d, in_buf, in_len); break;
    case VWIFI_OP_SET_RAW_FILTER: status = op_set_raw_filter(d, in_buf, in_len); break;
    case VWIFI_OP_SCAN:           status = op_scan(d, in_buf, in_len); break;
    case VWIFI_OP_SCAN_ABORT:     status = op_scan_abort(d); break;
    case VWIFI_OP_CONNECT:        status = op_connect(d, in_buf, in_len); break;
    case VWIFI_OP_DISCONNECT:     status = op_disconnect(d, in_buf, in_len); break;
    case VWIFI_OP_SET_KEY:        status = op_set_key(d, in_buf, in_len); break;
    case VWIFI_OP_DEL_KEY:        status = op_del_key(d, in_buf, in_len); break;
    case VWIFI_OP_START_AP:       status = op_start_ap(d, in_buf, in_len); break;
    case VWIFI_OP_STOP_AP:        status = op_stop_ap(d); break;
    default:
        VWIFI_ERR(d, "unknown opcode 0x%04x", req->opcode);
        status = -22;   /* -EINVAL */
        break;
    }

respond:
    rsp.status = status;
    post_rsp(d, &rsp, out_payload, out_payload_len);
}

static void process_ctrl_req_ring(struct vwifi_dev *d)
{
    while (d->ctrl_req.enabled) {
        uint64_t desc_gpa = ring_desc_gpa(&d->ctrl_req, d->ctrl_req.next_idx);
        struct vwifi_ctrl_req_desc req;

        if (d->ops->dma_read(d->be, desc_gpa, &req, sizeof(req)) != 0) break;
        if (!(req.flags & VWIFI_DESC_F_OWN)) break;

        handle_ctrl_req(d, &req);

        req.flags &= ~VWIFI_DESC_F_OWN;
        d->ops->dma_write(d->be, desc_gpa, &req, sizeof(req));

        d->ctrl_req.next_idx = (d->ctrl_req.next_idx + 1) & d->ctrl_req.mask;
    }
}

/* ============================================================
 * TX ring drain — send guest 802.11 frames to the medium.
 * The caller (backend) should supply a virtual-clock timestamp
 * as we can't call qemu_clock_get_us from portable code.
 * ============================================================ */

static void process_tx_ring(struct vwifi_dev *d)
{
    uint8_t frame[VWIFI_MAX_FRAME_SIZE];

    while (d->tx.enabled) {
        uint64_t desc_gpa = ring_desc_gpa(&d->tx, d->tx.next_idx);
        struct vwifi_tx_desc desc;

        if (d->ops->dma_read(d->be, desc_gpa, &desc, sizeof(desc)) != 0) break;
        if (!(desc.flags & VWIFI_DESC_F_OWN)) break;

        if (desc.frame_len > sizeof(frame)) {
            VWIFI_ERR(d, "tx frame too large: %u", desc.frame_len);
            d->drops++;
        } else if (desc.frame_len > 0 && desc.frame_addr) {
            if (d->ops->dma_read(d->be, desc.frame_addr,
                                 frame, desc.frame_len) == 0) {
                if (!(d->status & VWIFI_STATUS_LINK_UP)) {
                    VWIFI_TRACE(d, "tx dropped: medium link down");
                    d->drops++;
                } else {
                    bool inject = (desc.flags & VWIFI_TX_F_INJECT) != 0;

                    if (!inject && d->op_mode == VWIFI_MODE_STA) {
                        /* STA mode: the driver hands us 802.3; build
                         * the 802.11 data frame here. */
                        uint8_t frame80211[VWIFI_MAX_FRAME_SIZE];
                        uint16_t len;

                        if (d->conn.state != VWIFI_CONN_ASSOCIATED) {
                            VWIFI_TRACE(d, "tx dropped: not associated");
                            d->drops++;
                        } else {
                            len = sta_tx_8023_to_80211(d, frame, desc.frame_len,
                                                       frame80211,
                                                       sizeof(frame80211));
                            if (len == 0) {
                                VWIFI_TRACE(d, "tx dropped: bad 802.3 frame");
                                d->drops++;
                            } else {
                                /* Encrypt unless this is EAPOL (the
                                 * handshake must run in the clear) or
                                 * no key is installed yet. */
                                struct vwifi_key_slot *ks = key_slot_for_tx(d);
                                bool eapol = eth_is_eapol(frame, desc.frame_len);

                                if (ks && !eapol) {
                                    int elen = vwifi_ccmp_encrypt(
                                        &ks->aes, frame80211, len,
                                        ks->tx_pn, ks->key_idx,
                                        sizeof(frame80211));
                                    if (elen < 0) {
                                        VWIFI_TRACE(d, "tx dropped: ccmp encrypt failed");
                                        d->drops++;
                                        goto tx_done;
                                    }
                                    len = (uint16_t)elen;
                                    vwifi_pn_increment(ks->tx_pn);
                                } else if (eapol) {
                                    VWIFI_TRACE(d, "tx: EAPOL frame, sending unencrypted");
                                }

                                medium_send_frame(d, frame80211, len,
                                                  desc.rate_code,
                                                  desc.channel_freq,
                                                  d->ops->now_us(d->be), false);
                            }
                        }
                    tx_done: ;
                    } else {
                        /* Monitor mode or explicit injection: raw. */
                        medium_send_frame(d, frame, desc.frame_len,
                                          desc.rate_code, desc.channel_freq,
                                          d->ops->now_us(d->be), inject);
                    }
                }
            } else {
                d->drops++;
            }
        }

        desc.flags &= ~VWIFI_DESC_F_OWN;
        d->ops->dma_write(d->be, desc_gpa, &desc, sizeof(desc));

        d->tx.next_idx = (d->tx.next_idx + 1) & d->tx.mask;
    }

    d->regs[VWIFI_REG_DIAG_FRAMES_TX / 4] = d->frames_tx;
    d->regs[VWIFI_REG_DIAG_DROPS / 4] = d->drops;
}

/* ============================================================
 * Ring enable/disable
 * ============================================================ */

static void enable_rings(struct vwifi_dev *d)
{
    struct {
        struct vwifi_ring *r;
        uint32_t base_lo_reg, base_hi_reg, size_reg;
    } rings[] = {
        { &d->ctrl_req, VWIFI_REG_CTRL_REQ_RING_BASE_LO,
                        VWIFI_REG_CTRL_REQ_RING_BASE_HI,
                        VWIFI_REG_CTRL_REQ_RING_SIZE },
        { &d->ctrl_rsp, VWIFI_REG_CTRL_RSP_RING_BASE_LO,
                        VWIFI_REG_CTRL_RSP_RING_BASE_HI,
                        VWIFI_REG_CTRL_RSP_RING_SIZE },
        { &d->tx,       VWIFI_REG_TX_RING_BASE_LO,
                        VWIFI_REG_TX_RING_BASE_HI,
                        VWIFI_REG_TX_RING_SIZE },
        { &d->rx,       VWIFI_REG_RX_RING_BASE_LO,
                        VWIFI_REG_RX_RING_BASE_HI,
                        VWIFI_REG_RX_RING_SIZE },
    };

    for (unsigned i = 0; i < sizeof(rings)/sizeof(rings[0]); i++) {
        struct vwifi_ring *r = rings[i].r;
        uint64_t lo = d->regs[rings[i].base_lo_reg / 4];
        uint64_t hi = d->regs[rings[i].base_hi_reg / 4];
        uint32_t sz = d->regs[rings[i].size_reg  / 4];

        r->base_gpa = (hi << 32) | lo;
        r->size     = sz;
        r->enabled  = false;

        if (!r->base_gpa || !is_pow2(sz)) {
            VWIFI_ERR(d, "ring %s invalid (base=0x%lx size=%u)",
                      r->name, (unsigned long)r->base_gpa, sz);
            continue;
        }
        r->mask = sz - 1;
        r->next_idx = 0;
        r->enabled = true;
        VWIFI_TRACE(d, "ring %s enabled: base=0x%lx size=%u",
                    r->name, (unsigned long)r->base_gpa, sz);
    }

    d->status |= VWIFI_STATUS_READY;
    d->regs[VWIFI_REG_STATUS / 4] = d->status;
}

static void disable_rings(struct vwifi_dev *d)
{
    d->ctrl_req.enabled = false;
    d->ctrl_rsp.enabled = false;
    d->tx.enabled       = false;
    d->rx.enabled       = false;
    d->status &= ~VWIFI_STATUS_READY;
    d->regs[VWIFI_REG_STATUS / 4] = d->status;
}

void vwifi_device_reset(struct vwifi_dev *d, bool link_up)
{
    disable_rings(d);

    /* Cancel any in-flight scan and drop its timer. */
    if (d->ops && d->ops->timer_del) {
        d->ops->timer_del(d->be);
    }
    memset(&d->scan, 0, sizeof(d->scan));
    memset(d->bss, 0, sizeof(d->bss));
    memset(&d->conn, 0, sizeof(d->conn));
    memset(&d->keys, 0, sizeof(d->keys));
    memset(&d->ap, 0, sizeof(d->ap));
    memset(&d->timers, 0, sizeof(d->timers));
    d->seq_num = 0;

    d->ctrl       = 0;
    d->irq_mask   = 0;
    d->irq_status = 0;
    d->op_mode    = VWIFI_MODE_IDLE;
    d->raw_filter = 0;
    d->rxbuf_len  = 0;
    memset(&d->channel, 0, sizeof(d->channel));

    memset(d->regs, 0, sizeof(d->regs));
    d->regs[VWIFI_REG_SIGNATURE / 4]   = VWIFI_SIGNATURE;
    d->regs[VWIFI_REG_ABI_VERSION / 4] = VWIFI_ABI_VERSION;
    d->regs[VWIFI_REG_CAPS / 4]        = VWIFI_CAP_MONITOR | VWIFI_CAP_INJECT |
                                         VWIFI_CAP_STA | VWIFI_CAP_WPA2;
    d->regs[VWIFI_REG_NUM_VECTORS / 4] = VWIFI_NUM_VECTORS;
    d->status = link_up ? VWIFI_STATUS_LINK_UP : 0;
    d->regs[VWIFI_REG_STATUS / 4] = d->status;
}

/* ============================================================
 * Public entry points from the backend
 * ============================================================ */

void vwifi_dev_init(struct vwifi_dev *d,
                    const struct vwifi_backend_ops *ops, void *be,
                    const char *node_id, bool verbose,
                    const uint8_t default_mac[6])
{
    memset(d, 0, sizeof(*d));
    d->ops     = ops;
    d->be      = be;
    d->node_id = node_id;
    d->verbose = verbose;
    memcpy(d->sta_mac, default_mac, 6);

    ring_reset(&d->ctrl_req, "ctrl-req", sizeof(struct vwifi_ctrl_req_desc));
    ring_reset(&d->ctrl_rsp, "ctrl-rsp", sizeof(struct vwifi_ctrl_rsp_desc));
    ring_reset(&d->tx,       "tx",       sizeof(struct vwifi_tx_desc));
    ring_reset(&d->rx,       "rx",       sizeof(struct vwifi_rx_desc));

    vwifi_device_reset(d, false);
}

size_t vwifi_dev_sizeof(void) { return sizeof(struct vwifi_dev); }

uint64_t vwifi_reg_read(struct vwifi_dev *d, uint32_t addr, unsigned size)
{
    if (addr + size > VWIFI_MMIO_SIZE) return 0;
    uint32_t val = d->regs[addr / 4];
    if (size == 4) return val;
    if (size == 2) return (val >> ((addr & 2) * 8)) & 0xFFFF;
    if (size == 1) return (val >> ((addr & 3) * 8)) & 0xFF;
    return 0;
}

void vwifi_reg_write(struct vwifi_dev *d, uint32_t addr, uint64_t data,
                     unsigned size)
{
    if (addr + size > VWIFI_MMIO_SIZE) return;

    if (size != 4) {
        uint32_t cur   = d->regs[addr / 4];
        uint32_t shift = (addr & 3) * 8;
        uint32_t mask  = (size == 1) ? 0xFFu : (size == 2 ? 0xFFFFu : ~0u);
        cur &= ~(mask << shift);
        cur |= (data & mask) << shift;
        d->regs[addr / 4] = cur;
        return;
    }

    uint32_t val = (uint32_t)data;
    uint32_t reg = addr & ~3u;

    switch (reg) {
    case VWIFI_REG_SIGNATURE:
    case VWIFI_REG_ABI_VERSION:
    case VWIFI_REG_CAPS:
    case VWIFI_REG_NUM_VECTORS:
    case VWIFI_REG_STATUS:
    case VWIFI_REG_IRQ_STATUS:
    case VWIFI_REG_CTRL_RSP_RING_TAIL:
    case VWIFI_REG_RX_RING_TAIL:
    case VWIFI_REG_DIAG_FRAMES_TX:
    case VWIFI_REG_DIAG_FRAMES_RX:
    case VWIFI_REG_DIAG_DROPS:
        VWIFI_TRACE(d, "write to RO reg 0x%03x ignored", reg);
        return;

    case VWIFI_REG_RESET:
        if (val) vwifi_device_reset(d, d->status & VWIFI_STATUS_LINK_UP);
        return;

    case VWIFI_REG_CTRL: {
        uint32_t old = d->ctrl;
        d->ctrl = val;
        d->regs[reg / 4] = val;
        if (val & VWIFI_CTRL_RESET) {
            vwifi_device_reset(d, d->status & VWIFI_STATUS_LINK_UP);
            return;
        }
        if ((val & VWIFI_CTRL_ENABLE) && !(old & VWIFI_CTRL_ENABLE)) {
            enable_rings(d);
        } else if (!(val & VWIFI_CTRL_ENABLE) && (old & VWIFI_CTRL_ENABLE)) {
            disable_rings(d);
        }
        return;
    }

    case VWIFI_REG_IRQ_MASK:
        d->irq_mask = val;
        d->regs[reg / 4] = val;
        return;

    case VWIFI_REG_CTRL_REQ_RING_BASE_LO:
    case VWIFI_REG_CTRL_REQ_RING_BASE_HI:
    case VWIFI_REG_CTRL_REQ_RING_SIZE:
    case VWIFI_REG_CTRL_RSP_RING_BASE_LO:
    case VWIFI_REG_CTRL_RSP_RING_BASE_HI:
    case VWIFI_REG_CTRL_RSP_RING_SIZE:
    case VWIFI_REG_TX_RING_BASE_LO:
    case VWIFI_REG_TX_RING_BASE_HI:
    case VWIFI_REG_TX_RING_SIZE:
    case VWIFI_REG_RX_RING_BASE_LO:
    case VWIFI_REG_RX_RING_BASE_HI:
    case VWIFI_REG_RX_RING_SIZE:
        d->regs[reg / 4] = val;
        return;

    case VWIFI_REG_CTRL_REQ_RING_DOORBELL:
        d->regs[reg / 4] = val;
        process_ctrl_req_ring(d);
        return;

    case VWIFI_REG_TX_RING_DOORBELL:
        d->regs[reg / 4] = val;
        process_tx_ring(d);
        return;

    case VWIFI_REG_CTRL_RSP_RING_HEAD:
        d->regs[reg / 4] = val;
        if (val == d->ctrl_rsp.next_idx) {
            d->irq_status &= ~(1u << VWIFI_VEC_CTRL_RSP);
            d->regs[VWIFI_REG_IRQ_STATUS / 4] = d->irq_status;
        }
        return;

    case VWIFI_REG_RX_RING_HEAD:
        d->regs[reg / 4] = val;
        if (val == d->rx.next_idx) {
            d->irq_status &= ~(1u << VWIFI_VEC_RX);
            d->regs[VWIFI_REG_IRQ_STATUS / 4] = d->irq_status;
        }
        return;

    default:
        VWIFI_ERR(d, "UNHANDLED write reg=0x%03x val=0x%08x", reg, val);
        d->regs[reg / 4] = val;
        return;
    }
}
