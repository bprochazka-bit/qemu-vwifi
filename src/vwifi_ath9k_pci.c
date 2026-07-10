/*
 * QEMU Virtual Atheros AR9285 (ath9k) PCI-E Device
 *
 * Copyright (c) 2025 Virtual WiFi Project Contributors
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Phase 3: Virtual wireless medium – inter-VM frame exchange via
 *          Unix socket, TX frame extraction, RX injection from
 *          medium, auto-ACK for unicast frames.
 *
 * Usage:
 *   -device vwifi-ath9k,medium=/tmp/vwifi.sock
 *
 * Or without a medium (standalone mode, beacon-only):
 *   -device vwifi-ath9k
 */

#include "qemu/osdep.h"
#include "hw/pci/pci_device.h"
#include "migration/vmstate.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "qemu/units.h"
#include "qemu/timer.h"
#include "qemu/main-loop.h"
#include "qom/object.h"
#include "crypto/aes.h"

#include "vwifi_ath9k_regs.h"
#include "vwifi_ath9k_eeprom.h"
#include "vwifi_ath9k_dma.h"
#include "vwifi_ath9k_ampdu.h"
#include "vwifi_ath9k_crypto.h"
#include "vwifi_ath9k_wep.h"
#include "vwifi_ath9k_tkip.h"
#include "vwifi.h"

#include <sys/socket.h>
#include <sys/un.h>

/* -------------------------------------------------------------------
 *  Compile-time knobs
 * ------------------------------------------------------------------- */
#define ATH9K_VIRT_DEBUG        1
#define ATH9K_MMIO_SIZE         (64 * KiB)
#define ATH9K_REG_COUNT         (ATH9K_MMIO_SIZE / sizeof(uint32_t))

#ifndef PCI_CLASS_NETWORK_OTHER
#define PCI_CLASS_NETWORK_OTHER 0x0280
#endif

#define ATH9K_PM_UNDEFINED  0
#define ATH9K_PM_AWAKE      1
#define ATH9K_PM_FULL_SLEEP 2

#define ATH9K_BEACON_INTERVAL_MS  100

/* AR_PHY(0x37) – Channel select register, written by ar5008_hw_set_channel() */
#define AR_PHY_CHANSEL              0x98DC

/* Maximum 802.11 frame we'll extract from TX DMA */
#define ATH9K_MAX_FRAME_SIZE      VWIFI_MAX_FRAME_SIZE

/* -------------------------------------------------------------------
 *  QOM boilerplate
 * ------------------------------------------------------------------- */
#define TYPE_VWIFI_ATH9K "vwifi-ath9k"
OBJECT_DECLARE_SIMPLE_TYPE(VwifiAth9kState, VWIFI_ATH9K)

struct VwifiAth9kTxQueue {
    uint32_t txdp;
    bool     enabled;
};

struct VwifiAth9kState {
    PCIDevice parent_obj;
    MemoryRegion mmio;
    uint32_t regs[ATH9K_REG_COUNT];
    uint16_t eeprom[ATH9K_EEPROM_4K_SIZE_WORDS];

    /* Interrupt state */
    uint32_t isr;
    uint32_t isr_s0;
    uint32_t isr_s1;
    uint32_t ier;
    uint32_t imr;
    uint32_t imr_s0;
    uint32_t imr_s1;

    uint32_t power_mode;

    /* TX queue state */
    struct VwifiAth9kTxQueue tx_queues[ATH9K_VIRT_NUM_TX_QUEUES];

    /* RX DMA state */
    uint32_t rxdp;
    bool     rx_enabled;
    bool     rxdp_ring_empty;  /* ds_link was 0 on last inject; re-read before next */

    /* PHY state */
    bool     phy_active;

    /* Beacon / SWBA timer */
    QEMUTimer *beacon_timer;
    bool beacon_timer_active;
    QEMUTimer *swba_timer;
    bool swba_timer_active;
    bool swba_pending;          /* true from SWBA raise until driver acknowledges */

    /* Cached beacon for auto-retransmit when driver TX path stalls */
    uint8_t  cached_beacon[256];
    uint16_t cached_beacon_len;
    int64_t  last_beacon_tx_ms;  /* virtual clock time of last beacon TX */
    uint32_t tsf_lo;
    uint32_t tsf_hi;

    /* --- Phase 3: Virtual wireless medium --- */
    char       *macaddr;          /* -device vwifi-ath9k,macaddr=00:03:7f:... */
    char       *medium_path;      /* -device vwifi-ath9k,medium=/tmp/x.sock */
    char       *node_id;          /* -device vwifi-ath9k,node_id=ap1 (stable identity) */
    QEMUTimer  *medium_reconnect_timer;
    int         medium_fd;        /* connected socket fd, or -1 */
    bool        medium_connected; /* true when socket is open */

    /* Stream reassembly buffer for length-prefixed messages */
    uint8_t     medium_rxbuf[VWIFI_RXBUF_SIZE];
    uint32_t    medium_rxbuf_used;  /* bytes currently in rxbuf */

    /* Our MAC address (read from EEPROM at realize) */
    uint8_t     our_mac[6];

    /* Channel state (decoded from PHY register writes) */
    uint16_t    current_channel_freq;       /* MHz, e.g. 2412 */
    uint16_t    current_channel_flags;      /* band/width flags */

    /* Debug bookkeeping */
    uint64_t read_count;
    uint64_t write_count;
    uint64_t unhandled_read_count;
    uint64_t unhandled_write_count;
    uint64_t tx_frames_processed;
    uint64_t rx_frames_injected;
    uint64_t tx_frames_to_medium;
    uint64_t rx_frames_from_medium;

};

/* -------------------------------------------------------------------
 *  Logging
 * ------------------------------------------------------------------- */
#define vwifi_ath9k_trace(fmt, ...)                                              \
    do {                                                                   \
        if (ATH9K_VIRT_DEBUG) {                                            \
            qemu_log_mask(LOG_GUEST_ERROR,                                 \
                          "vwifi-ath9k: " fmt "\n", ## __VA_ARGS__);        \
        }                                                                  \
    } while (0)

#define vwifi_ath9k_warn(fmt, ...)                                               \
    qemu_log_mask(LOG_UNIMP,                                               \
                  "vwifi-ath9k: WARNING: " fmt "\n", ## __VA_ARGS__)

#define vwifi_ath9k_error(fmt, ...)                                              \
    qemu_log_mask(LOG_GUEST_ERROR,                                         \
                  "vwifi-ath9k: ERROR: " fmt "\n", ## __VA_ARGS__)

/* -------------------------------------------------------------------
 *  Register name lookup
 * ------------------------------------------------------------------- */
static const char *vwifi_ath9k_reg_name(hwaddr addr)
{
    static char buf[32];

    switch ((uint32_t)addr) {
    case AR_CR: return "AR_CR";
    case AR_RXDP: return "AR_RXDP";
    case AR_CFG: return "AR_CFG";
    case AR_RXCFG: return "AR_RXCFG";
    case AR_IER: return "AR_IER";
    case AR_TXCFG: return "AR_TXCFG";
    case AR_MIBC: return "AR_MIBC";
    case AR_ISR: return "AR_ISR";
    case AR_ISR_S0: return "AR_ISR_S0";
    case AR_ISR_S1: return "AR_ISR_S1";
    case AR_IMR: return "AR_IMR";
    case AR_IMR_S0: return "AR_IMR_S0";
    case AR_IMR_S1: return "AR_IMR_S1";
    case AR_SREV: return "AR_SREV";
    case AR_EEPROM: return "AR_EEPROM";
    case AR_EEPROM_STATUS_DATA: return "AR_EEPROM_STATUS_DATA";
    case AR_Q_TXE: return "AR_Q_TXE";
    case AR_Q_TXD: return "AR_Q_TXD";
    case AR_RTC_RC: return "AR_RTC_RC";
    case AR_RTC_RESET: return "AR_RTC_RESET";
    case AR_RTC_STATUS: return "AR_RTC_STATUS";
    case AR_RTC_FORCE_WAKE: return "AR_RTC_FORCE_WAKE";
    case AR_RTC_PLL_CONTROL: return "AR_RTC_PLL_CONTROL";
    case AR_GPIO_IN_OUT: return "AR_GPIO_IN_OUT";
    case AR_INTR_SYNC_CAUSE: return "AR_INTR_SYNC_CAUSE";
    case AR_INTR_ASYNC_CAUSE: return "AR_INTR_ASYNC_CAUSE";
    case AR_WA: return "AR_WA";
    case AR_STA_ID0: return "AR_STA_ID0";
    case AR_STA_ID1: return "AR_STA_ID1";
    case AR_BSS_ID0: return "AR_BSS_ID0";
    case AR_BSS_ID1: return "AR_BSS_ID1";
    case AR_DIAG_SW: return "AR_DIAG_SW";
    case AR_RX_FILTER: return "AR_RX_FILTER";
    case AR_PHY_ACTIVE: return "AR_PHY_ACTIVE";
    case AR_PHY_MODE: return "AR_PHY_MODE";
    case AR_PHY_CHANSEL: return "AR_PHY_CHANSEL";
    case AR_PHY_AGC_CONTROL: return "AR_PHY_AGC_CONTROL";
    case AR_PHY_RFBUS_REQ: return "AR_PHY_RFBUS_REQ";
    case AR_PHY_RFBUS_GRANT: return "AR_PHY_RFBUS_GRANT";
    case AR_TIMER_MODE: return "AR_TIMER_MODE";
    case AR_BEACON_PERIOD: return "AR_BEACON_PERIOD";
    case AR_PCU_MISC_MODE2: return "AR_PCU_MISC_MODE2";
    default:
        if (addr >= AR_Q0_TXDP && addr < AR_Q0_TXDP + AR_NUM_QCU * 4) {
            snprintf(buf, sizeof(buf), "AR_QTXDP(%d)",
                     (int)(addr - AR_Q0_TXDP) / 4);
            return buf;
        }
        snprintf(buf, sizeof(buf), "REG(0x%04x)", (uint32_t)addr);
        return buf;
    }
}


/* -------------------------------------------------------------------
 *  Safe register helpers
 * ------------------------------------------------------------------- */
static inline uint32_t vwifi_ath9k_reg_read_raw(VwifiAth9kState *s, hwaddr addr)
{
    if (addr < ATH9K_MMIO_SIZE) {
        return s->regs[addr / 4];
    }
    vwifi_ath9k_error("read out of MMIO range: 0x%" HWADDR_PRIx, addr);
    return 0xdeadbeef;
}

static inline void vwifi_ath9k_reg_write_raw(VwifiAth9kState *s, hwaddr addr,
                                       uint32_t val)
{
    if (addr < ATH9K_MMIO_SIZE) {
        s->regs[addr / 4] = val;
        return;
    }
    vwifi_ath9k_error("write out of MMIO range: 0x%" HWADDR_PRIx, addr);
}

/* -------------------------------------------------------------------
 *  EEPROM emulation (serial EEPROM protocol)
 *
 *  The kernel's ath_pci_eeprom_read() does:
 *    1. REG_READ(ah, AR5416_EEPROM_OFFSET + (word_off << AR5416_EEPROM_S))
 *       On real HW this MMIO read triggers the EEPROM controller.
 *    2. Poll AR_EEPROM_STATUS_DATA until BUSY bit clears.
 *    3. Read the 16-bit word from AR_EEPROM_STATUS_DATA bits [15:0].
 *
 *  We emulate this: any read in the 0x2000..0x2800 range looks up the
 *  EEPROM word and loads it into the status register (not busy).
 *  The read itself returns 0 (the trigger address is not data).
 * ------------------------------------------------------------------- */
#define AR5416_EEPROM_OFFSET  0x2000
#define AR5416_EEPROM_S       2

static void vwifi_ath9k_eeprom_trigger(VwifiAth9kState *s, hwaddr addr)
{
    uint32_t word_off = (addr - AR5416_EEPROM_OFFSET) >> AR5416_EEPROM_S;
    uint16_t val = 0;

    if (word_off < ATH9K_EEPROM_4K_SIZE_WORDS) {
        val = s->eeprom[word_off];
    }

    /* Store in AR_EEPROM_STATUS_DATA: value in low 16 bits, not busy */
    s->regs[AR_EEPROM_STATUS_DATA / 4] = (uint32_t)val;
}

/* -------------------------------------------------------------------
 *  Interrupt delivery
 * ------------------------------------------------------------------- */
static void vwifi_ath9k_update_irq(VwifiAth9kState *s)
{
    bool level = (s->ier & AR_IER_ENABLE) && (s->isr & s->imr);
    pci_set_irq(&s->parent_obj, level ? 1 : 0);
}

static void vwifi_ath9k_raise_irq(VwifiAth9kState *s, uint32_t isr_bits)
{
    s->isr |= isr_bits;
    if (!(s->ier & AR_IER_ENABLE)) {
        /* IRQ blocked by IER — frame will be invisible to guest */
        if (isr_bits & AR_ISR_RXOK) {
            static uint32_t blocked_count;
            blocked_count++;
            if (blocked_count <= 5 || (blocked_count % 100) == 0) {
                vwifi_ath9k_trace("RXOK blocked by IER=0 (count=%u, isr=0x%08x)",
                            blocked_count, s->isr);
            }
        }
    }
    vwifi_ath9k_update_irq(s);
}

/* -------------------------------------------------------------------
 *  TX DMA engine
 * ------------------------------------------------------------------- */

/* ================================================================
 *  Medium reconnect support
 *
 *  If the medium hub restarts or the socket breaks, we automatically
 *  retry every 2 seconds.  On reconnect, we re-send the node_id hello.
 * ================================================================ */

#define MEDIUM_RECONNECT_MS  2000   /* retry every 2 seconds */
#define VWIFI_HELLO_MAGIC    0x52495756  /* "VWIR" – vwifi registration */
#define HELLO_MAGIC          VWIFI_HELLO_MAGIC

/* Forward declarations */
static void vwifi_ath9k_reconnect_cb(void *opaque);
static void vwifi_ath9k_fd_read(void *opaque);

/*
 * Try to connect (or reconnect) to the medium hub.
 * On success, sends a hello message with our node_id.
 * Returns true if connected.
 */
static bool vwifi_ath9k_try_connect(VwifiAth9kState *s)
{
    struct sockaddr_un sun;
    int fd;

    if (!s->medium_path || s->medium_path[0] == '\0')
        return false;
    if (s->medium_connected)
        return true;

    fd = socket(AF_UNIX, SOCK_STREAM, 0);
    if (fd < 0)
        return false;

    memset(&sun, 0, sizeof(sun));
    sun.sun_family = AF_UNIX;
    strncpy(sun.sun_path, s->medium_path, sizeof(sun.sun_path) - 1);

    if (connect(fd, (struct sockaddr *)&sun, sizeof(sun)) < 0) {
        close(fd);
        return false;
    }

    s->medium_fd = fd;
    s->medium_connected = true;
    s->medium_rxbuf_used = 0;
    qemu_set_fd_handler(fd, vwifi_ath9k_fd_read, NULL, s);

    /* Send hello with node_id if configured */
    if (s->node_id && s->node_id[0] != '\0') {
        size_t id_len = strlen(s->node_id) + 1; /* include NUL */
        uint32_t payload_len = 4 + (uint32_t)id_len;
        uint32_t net_len = htonl(payload_len);
        uint32_t hello_magic = HELLO_MAGIC;
        uint8_t hello_buf[4 + 4 + 32];
        memcpy(hello_buf, &net_len, 4);
        memcpy(hello_buf + 4, &hello_magic, 4);
        memcpy(hello_buf + 8, s->node_id, id_len);
        ssize_t wr = write(fd, hello_buf, 4 + payload_len);
        (void)wr;
    }

    qemu_log_mask(LOG_GUEST_ERROR,
                  "vwifi-ath9k: medium connected to %s "
                  "(node_id=%s, MAC %02x:%02x:%02x:%02x:%02x:%02x)\n",
                  s->medium_path,
                  (s->node_id && s->node_id[0]) ? s->node_id : "(auto)",
                  s->our_mac[0], s->our_mac[1], s->our_mac[2],
                  s->our_mac[3], s->our_mac[4], s->our_mac[5]);

    /* Cancel reconnect timer since we're connected now */
    timer_del(s->medium_reconnect_timer);

    return true;
}

/*
 * Start the reconnect timer.  Called when we detect a disconnect.
 */
static void vwifi_ath9k_schedule_reconnect(VwifiAth9kState *s)
{
    if (!s->medium_path || s->medium_path[0] == '\0')
        return;
    qemu_log_mask(LOG_GUEST_ERROR,
                  "vwifi-ath9k: medium disconnected, will retry in %dms\n",
                  MEDIUM_RECONNECT_MS);
    timer_mod(s->medium_reconnect_timer,
              qemu_clock_get_ms(QEMU_CLOCK_REALTIME) + MEDIUM_RECONNECT_MS);
}

/*
 * Reconnect timer callback — runs in QEMU main loop context.
 */
static void vwifi_ath9k_reconnect_cb(void *opaque)
{
    VwifiAth9kState *s = VWIFI_ATH9K(opaque);

    if (s->medium_connected)
        return;

    if (!vwifi_ath9k_try_connect(s)) {
        /* Still can't connect, try again later */
        timer_mod(s->medium_reconnect_timer,
                  qemu_clock_get_ms(QEMU_CLOCK_REALTIME)
                  + MEDIUM_RECONNECT_MS);
    }
}

/*
 * Send a frame to the virtual medium via the chardev socket.
 * Wire format: [uint32_t length (network BE)] [vwifi_frame_hdr] [payload]
 */
static void vwifi_ath9k_send_rate(VwifiAth9kState *s, const uint8_t *frame,
                                   uint16_t frame_len, uint8_t rate_code);

static void vwifi_ath9k_send(VwifiAth9kState *s, const uint8_t *frame,
                              uint16_t frame_len)
{
    vwifi_ath9k_send_rate(s, frame, frame_len, VWIFI_DEFAULT_RATE);
}

static void vwifi_ath9k_send_rate(VwifiAth9kState *s, const uint8_t *frame,
                                   uint16_t frame_len, uint8_t rate_code)
{
    struct vwifi_frame_hdr hdr;
    uint32_t msg_len;
    uint32_t net_len;
    uint8_t sendbuf[4 + VWIFI_HDR_SIZE + VWIFI_MAX_FRAME_SIZE];

    if (!s->medium_connected || s->medium_fd < 0 ||
        frame_len == 0 ||
        frame_len > VWIFI_MAX_FRAME_SIZE) {
        return;
    }

    memset(&hdr, 0, sizeof(hdr));
    hdr.magic = VWIFI_MAGIC;
    hdr.version = VWIFI_VERSION;
    hdr.frame_len = frame_len;
    memcpy(hdr.tx_mac, s->our_mac, 6);
    hdr.rate_code = rate_code;
    hdr.rssi = VWIFI_DEFAULT_RSSI;
    hdr.tsf_lo = s->tsf_lo;
    hdr.tsf_hi = s->tsf_hi;
    hdr.flags = 0;
    hdr.channel_freq = s->current_channel_freq;
    hdr.channel_flags = s->current_channel_flags;
    hdr.channel_bond_freq = 0;
    hdr.center_freq1 = 0;  /* AR9285 is 11n only, no VHT */
    hdr.center_freq2 = 0;

    msg_len = (uint32_t)(VWIFI_HDR_SIZE + frame_len);
    net_len = htonl(msg_len);

    /* Build the wire message in a single buffer for atomic write */
    memcpy(sendbuf, &net_len, 4);
    memcpy(sendbuf + 4, &hdr, VWIFI_HDR_SIZE);
    memcpy(sendbuf + 4 + VWIFI_HDR_SIZE, frame, frame_len);

    /* Write the entire wire message atomically (best-effort) */
    {
        uint32_t total = 4 + msg_len;
        uint32_t off = 0;
        while (off < total) {
            ssize_t n = write(s->medium_fd, sendbuf + off, total - off);
            if (n <= 0) {
                if (n < 0 && (errno == EINTR || errno == EAGAIN)) {
                    continue;
                }
                vwifi_ath9k_warn("MEDIUM TX: write failed, disconnecting");
                qemu_set_fd_handler(s->medium_fd, NULL, NULL, NULL);
                close(s->medium_fd);
                s->medium_fd = -1;
                s->medium_connected = false;
                vwifi_ath9k_schedule_reconnect(s);
                return;
            }
            off += (uint32_t)n;
        }
    }

    s->tx_frames_to_medium++;
    vwifi_ath9k_trace("MEDIUM TX: %u bytes to medium (total %" PRIu64 ")",
                frame_len, s->tx_frames_to_medium);
}

/* -------------------------------------------------------------------
 *  Hardware crypto offload (CCMP / AES-CCM)
 *
 *  Real ath9k programs keys into the on-chip key cache and the MAC does
 *  CCMP on the data path.  We shadow the key cache in regs[] (the driver's
 *  MMIO writes land there like any other register) and perform the same
 *  AES-CCM transform in software here.  See vwifi_ath9k_crypto.h.
 * ------------------------------------------------------------------- */

/*
 * Reconstruct the 16 octets of key material stored in cache entry `slot`.
 * The layout matches ath_hw_set_keycache_entry(): key0..key4 split as
 * 4/2/4/2/4 little-endian octets.  This is cipher-independent; the caller
 * checks the slot's key type and uses as many octets as that cipher needs.
 */
static void vwifi_ath9k_keycache_material(VwifiAth9kState *s, unsigned slot,
                                          uint8_t key[16])
{
    uint32_t k0, k1, k2, k3, k4;

    k0 = vwifi_ath9k_reg_read_raw(s, AR_KEYTABLE_KEY0(slot));
    k1 = vwifi_ath9k_reg_read_raw(s, AR_KEYTABLE_KEY1(slot)) & 0xffff;
    k2 = vwifi_ath9k_reg_read_raw(s, AR_KEYTABLE_KEY2(slot));
    k3 = vwifi_ath9k_reg_read_raw(s, AR_KEYTABLE_KEY3(slot)) & 0xffff;
    k4 = vwifi_ath9k_reg_read_raw(s, AR_KEYTABLE_KEY4(slot));

    key[0]  = k0;       key[1]  = k0 >> 8;  key[2]  = k0 >> 16; key[3]  = k0 >> 24;
    key[4]  = k1;       key[5]  = k1 >> 8;
    key[6]  = k2;       key[7]  = k2 >> 8;  key[8]  = k2 >> 16; key[9]  = k2 >> 24;
    key[10] = k3;       key[11] = k3 >> 8;
    key[12] = k4;       key[13] = k4 >> 8;  key[14] = k4 >> 16; key[15] = k4 >> 24;
}

/*
 * Reconstruct the 128-bit CCMP key stored in cache entry `slot`.
 * Returns false if the slot is not a valid CCM (AES-CCM) entry.
 */
static bool vwifi_ath9k_keycache_ccm(VwifiAth9kState *s, unsigned slot,
                                     uint8_t key[16])
{
    if (slot >= AR_KEY_CACHE_SIZE) {
        return false;
    }
    if ((vwifi_ath9k_reg_read_raw(s, AR_KEYTABLE_TYPE(slot)) &
         AR_KEYTABLE_TYPE_MASK) != AR_KEYTABLE_TYPE_CCM) {
        return false;
    }
    vwifi_ath9k_keycache_material(s, slot, key);
    return true;
}

/*
 * Reconstruct the WEP key stored in cache entry `slot`.  Returns the key
 * length in octets (5 = WEP-40, 13 = WEP-104, 16 = WEP-128) or 0 if the slot
 * does not hold a WEP key.  Unused slots read as AR_KEYTABLE_TYPE_CLR (the
 * driver resets every entry to CLR during init), so a genuinely empty slot is
 * rejected here even though WEP-40 happens to be key type 0.
 */
static int vwifi_ath9k_keycache_wep(VwifiAth9kState *s, unsigned slot,
                                    uint8_t key[16])
{
    int keylen;

    if (slot >= AR_KEY_CACHE_SIZE) {
        return 0;
    }
    switch (vwifi_ath9k_reg_read_raw(s, AR_KEYTABLE_TYPE(slot)) &
            AR_KEYTABLE_TYPE_MASK) {
    case AR_KEYTABLE_TYPE_40:  keylen = 5;  break;
    case AR_KEYTABLE_TYPE_104: keylen = 13; break;
    case AR_KEYTABLE_TYPE_128: keylen = 16; break;
    default:                   return 0;
    }
    vwifi_ath9k_keycache_material(s, slot, key);
    return keylen;
}

/*
 * Reconstruct the 128-bit TKIP temporal key stored in cache entry `slot`.
 * Returns false if the slot is not a valid TKIP entry.  Only the temporal key
 * (the cipher key) lives in the main slot, laid out with the same key0..key4
 * split as CCM; the Michael MIC keys the driver programs into the companion
 * slot are not read here because Michael is done in software by mac80211.
 */
static bool vwifi_ath9k_keycache_tkip(VwifiAth9kState *s, unsigned slot,
                                      uint8_t key[16])
{
    if (slot >= AR_KEY_CACHE_SIZE) {
        return false;
    }
    if ((vwifi_ath9k_reg_read_raw(s, AR_KEYTABLE_TYPE(slot)) &
         AR_KEYTABLE_TYPE_MASK) != AR_KEYTABLE_TYPE_TKIP) {
        return false;
    }
    vwifi_ath9k_keycache_material(s, slot, key);
    return true;
}

/*
 * Decrypt a received CCMP MPDU.  On real hardware the MAC performs an
 * associative key-cache lookup (by transmitter address / key id); here we
 * try each populated CCM key and accept the one whose MIC verifies.  Because
 * the CCMP MIC is a cryptographic tag, a wrong key is rejected with
 * overwhelming probability, so this is both robust and faithful to the
 * "hardware picked the right key" behaviour the driver expects.
 *
 * `src` is the encrypted frame; on success the decrypted frame is written to
 * `dst` (same length) and the matching key-cache slot is returned via
 * `*out_keyix`.  Returns false if no key decrypts the frame.
 */
static bool vwifi_ath9k_rx_decrypt(VwifiAth9kState *s, const uint8_t *src,
                                   uint8_t *dst, int len, uint8_t *out_keyix)
{
    unsigned slot;

    for (slot = 0; slot < AR_KEY_CACHE_SIZE; slot++) {
        uint8_t key[16];

        if (!vwifi_ath9k_keycache_ccm(s, slot, key)) {
            continue;
        }
        memcpy(dst, src, len);
        if (vwifi_ccmp_decrypt(key, dst, len) == 0) {
            *out_keyix = (uint8_t)slot;
            return true;
        }
    }
    return false;
}

/*
 * Decrypt a received WEP MPDU.  WEP carries no cryptographic MIC, only a
 * 32-bit ICV, and the KeyID in the IV header only selects one of the (few)
 * default keys — so, as with CCM, we try each populated WEP slot and accept
 * the one whose ICV verifies.  In practice at most four WEP keys are ever
 * programmed, so the 2^-32 per-slot false-accept rate is negligible.
 */
static bool vwifi_ath9k_rx_decrypt_wep(VwifiAth9kState *s, const uint8_t *src,
                                       uint8_t *dst, int len, uint8_t *out_keyix)
{
    unsigned slot;

    for (slot = 0; slot < AR_KEY_CACHE_SIZE; slot++) {
        uint8_t key[16];
        int keylen = vwifi_ath9k_keycache_wep(s, slot, key);

        if (keylen == 0) {
            continue;
        }
        memcpy(dst, src, len);
        if (vwifi_wep_decrypt(key, keylen, dst, len) == 0) {
            *out_keyix = (uint8_t)slot;
            return true;
        }
    }
    return false;
}

/*
 * Decrypt a received TKIP MPDU.  Like WEP, TKIP has only a 32-bit ICV (the
 * Michael MIC is verified later, in software, by mac80211), so we try each
 * populated TKIP slot and accept the temporal key whose ICV verifies.  The
 * per-packet RC4 key is mixed from the TK, the frame's transmitter address
 * (A2) and the TSC in the TKIP header, all read inside vwifi_tkip_decrypt().
 */
static bool vwifi_ath9k_rx_decrypt_tkip(VwifiAth9kState *s, const uint8_t *src,
                                        uint8_t *dst, int len, uint8_t *out_keyix)
{
    unsigned slot;

    for (slot = 0; slot < AR_KEY_CACHE_SIZE; slot++) {
        uint8_t tk[16];

        if (!vwifi_ath9k_keycache_tkip(s, slot, tk)) {
            continue;
        }
        memcpy(dst, src, len);
        if (vwifi_tkip_decrypt(tk, dst, len) == 0) {
            *out_keyix = (uint8_t)slot;
            return true;
        }
    }
    return false;
}

static void vwifi_ath9k_process_tx_queue(VwifiAth9kState *s, int qnum)
{
    uint32_t desc_addr = s->tx_queues[qnum].txdp;
    uint32_t ds_link, ds_data, ds_ctl0, ds_ctl1;
    uint16_t frame_len, buf_len;
    int count = 0;

    /*
     * A-MPDU (802.11n) aggregation state, accumulated across the
     * contiguous run of AR_IsAggr subframe descriptors that make up one
     * aggregate.  We record each subframe's sequence number as it is sent
     * and, on the last subframe (AR_IsAggr without AR_MoreAggr), synthesize
     * the immediate Block-Ack that the peer would have returned.  agg_n == 0
     * for every legacy (non-aggregated) frame, so this leaves the legacy
     * TX path unchanged.
     */
    uint16_t agg_seqnos[VWIFI_ATH9K_MAX_AGGR];
    int      agg_n = 0;

    if (desc_addr == 0) {
        return;
    }

    vwifi_ath9k_trace("TX queue %d: DMA walk at 0x%08x", qnum, desc_addr);

    while (desc_addr != 0 && count < ATH9K_VIRT_MAX_DESC_WALK) {
        uint32_t ds_txstatus9;

        if (pci_dma_read(&s->parent_obj, desc_addr + DESC_OFF_LINK,
                         &ds_link, 4) != 0) {
            break;
        }
        if (pci_dma_read(&s->parent_obj, desc_addr + DESC_OFF_DATA,
                         &ds_data, 4) != 0) {
            break;
        }
        if (pci_dma_read(&s->parent_obj, desc_addr + DESC_OFF_CTL0,
                         &ds_ctl0, 4) != 0) {
            break;
        }
        if (pci_dma_read(&s->parent_obj, desc_addr + DESC_OFF_CTL1,
                         &ds_ctl1, 4) != 0) {
            break;
        }

        ds_link = le32_to_cpu(ds_link);
        ds_data = le32_to_cpu(ds_data);
        ds_ctl0 = le32_to_cpu(ds_ctl0);
        ds_ctl1 = le32_to_cpu(ds_ctl1);

        /*
         * Check if this descriptor was already completed (AR_TxDone).
         * The driver clears status words via cleartxdesc/fill_txdesc
         * before submitting, so if AR_TxDone is set, it's stale.
         */
        if (pci_dma_read(&s->parent_obj, desc_addr + DESC_OFF_TX_STATUS9,
                         &ds_txstatus9, 4) != 0) {
            break;
        }
        if (le32_to_cpu(ds_txstatus9) & AR_TxDone) {
            /*
             * Already processed — skip to the next descriptor.
             * The driver may have appended new frames after this one.
             */
            if (ds_link == 0) {
                /* End of chain — log this as it may explain beacon stalls */
                if (count == 0) {
                    vwifi_ath9k_trace("TX queue %d: chain end at desc 0x%08x "
                                "(stale, link=0, no new frames)", qnum, desc_addr);
                }
            }
            desc_addr = ds_link;
            continue;
        }

        frame_len = ds_ctl0 & AR_FrameLen;
        buf_len   = ds_ctl1 & AR_BufLen;

        vwifi_ath9k_trace("TX desc 0x%08x: link=0x%08x data=0x%08x "
                    "ctl0=0x%08x ctl1=0x%08x frame=%u buf=%u",
                    desc_addr, ds_link, ds_data,
                    ds_ctl0, ds_ctl1, frame_len, buf_len);

        /*
         * Phase 3: Extract the actual frame data from guest memory
         * and forward it to the virtual medium.
         *
         * The ath9k driver may split a single frame across multiple
         * descriptors using AR_TxMore.  We must gather all segments
         * to reconstruct the full frame before sending.
         *
         * The DMA buffers contain:  hdr + padding + body
         * where padding = (hdrlen & 3) bytes for 4-byte alignment.
         * frame_len in ctl0 = OTA length (hdr + body + FCS), no padding.
         * buf_len in ctl1   = DMA length (hdr + padding + body), no FCS.
         *
         * We gather using buf_len, then strip the padding before
         * sending to the medium.
         */
        if (ds_data != 0 && frame_len > 0 &&
            frame_len <= ATH9K_MAX_FRAME_SIZE) {
            /* +MIC headroom so CCMP encryption can append the 8-byte MIC */
            uint8_t frame_buf[ATH9K_MAX_FRAME_SIZE + VWIFI_CCMP_MIC_LEN];
            uint16_t gathered = 0;
            uint16_t seg_len;
            uint32_t seg_desc = desc_addr;
            uint32_t seg_data = ds_data;
            uint32_t seg_ctl1 = ds_ctl1;
            uint32_t seg_link = ds_link;
            int seg_count = 0;

            /* Read TX rate from descriptor ctl3 bits[7:0] (Series 0 rate).
             * This is the hardware rate code the driver selected. */
            uint32_t ds_ctl3 = 0;
            uint8_t tx_rate = VWIFI_DEFAULT_RATE;
            if (pci_dma_read(&s->parent_obj,
                             desc_addr + DESC_OFF_TX_CTL3,
                             &ds_ctl3, 4) == 0) {
                tx_rate = le32_to_cpu(ds_ctl3) & 0xFF;
                if (tx_rate == 0) {
                    tx_rate = VWIFI_DEFAULT_RATE;
                }
            }

            /* Read TX encryption type from descriptor ctl6 (AR_EncrType).
             * The driver sets this + the key-cache index (AR_DestIdx) when
             * it wants the hardware crypto engine to encrypt the frame. */
            uint32_t ds_ctl6 = 0;
            uint8_t  encr_type = AR_ENCR_TYPE_CLEAR;
            if (pci_dma_read(&s->parent_obj,
                             desc_addr + DESC_OFF_TX_CTL6,
                             &ds_ctl6, 4) == 0) {
                ds_ctl6 = le32_to_cpu(ds_ctl6);
                encr_type = (ds_ctl6 & AR_EncrType) >> AR_EncrType_S;
            }

            /* Gather segments using buf_len (actual DMA data) */
            while (gathered < ATH9K_MAX_FRAME_SIZE && seg_count < 64) {
                seg_len = seg_ctl1 & AR_BufLen;
                if (seg_len == 0) break;
                if (gathered + seg_len > ATH9K_MAX_FRAME_SIZE)
                    seg_len = ATH9K_MAX_FRAME_SIZE - gathered;

                if (seg_data != 0 &&
                    pci_dma_read(&s->parent_obj, seg_data,
                                 frame_buf + gathered, seg_len) == 0) {
                    gathered += seg_len;
                } else {
                    break;
                }

                /* Mark this segment's descriptor as done */
                if (seg_desc != desc_addr) {
                    uint32_t txs1 = cpu_to_le32(AR_FrmXmitOK);
                    uint32_t txs9 = cpu_to_le32(AR_TxDone);
                    pci_dma_write(&s->parent_obj,
                                  seg_desc + DESC_OFF_TX_STATUS1,
                                  &txs1, 4);
                    pci_dma_write(&s->parent_obj,
                                  seg_desc + DESC_OFF_TX_STATUS9,
                                  &txs9, 4);
                    count++;
                }

                /* If AR_TxMore is NOT set, this is the last segment */
                if (!(seg_ctl1 & AR_TxMore)) break;

                /* Follow link to next segment descriptor */
                if (seg_link == 0) break;
                seg_desc = seg_link;
                seg_count++;

                /* Read next segment's fields */
                if (pci_dma_read(&s->parent_obj,
                                 seg_desc + DESC_OFF_LINK,
                                 &seg_link, 4) != 0) break;
                if (pci_dma_read(&s->parent_obj,
                                 seg_desc + DESC_OFF_DATA,
                                 &seg_data, 4) != 0) break;
                if (pci_dma_read(&s->parent_obj,
                                 seg_desc + DESC_OFF_CTL1,
                                 &seg_ctl1, 4) != 0) break;
                seg_link = le32_to_cpu(seg_link);
                seg_data = le32_to_cpu(seg_data);
                seg_ctl1 = le32_to_cpu(seg_ctl1);
            }

            if (gathered > 0) {
                /*
                 * A-MPDU: record this subframe's 802.11 sequence number
                 * for the Block-Ack bitmap we write on the last subframe.
                 * The Sequence Control field is in the (unencrypted,
                 * unpadded-prefix) header, so reading it here — before the
                 * crypto/padding transforms below — is safe.
                 */
                if ((ds_ctl1 & AR_IsAggr) && agg_n < VWIFI_ATH9K_MAX_AGGR) {
                    agg_seqnos[agg_n++] =
                        vwifi_ath9k_frame_seqno(frame_buf, gathered);
                }

                /*
                 * Strip padding inserted by the driver.
                 * Padding = (hdrlen & 3) bytes after the 802.11 header.
                 * hdrlen is determined from the Frame Control field.
                 */
                uint16_t send_len = gathered;
                if (gathered >= 2) {
                    uint16_t fc = frame_buf[0] | (frame_buf[1] << 8);
                    uint8_t ftype = (fc >> 2) & 3;
                    uint8_t stype = (fc >> 4) & 0xF;
                    uint16_t hdrlen = 24; /* default */
                    /* Data frames with QoS have 26-byte header */
                    if (ftype == 2 && (stype & 0x8))
                        hdrlen = 26;
                    /* 4-addr (WDS) adds 6 bytes */
                    if ((fc & 0x0300) == 0x0300) /* ToDS + FromDS */
                        hdrlen += 6;
                    uint8_t padsize = hdrlen & 3;
                    if (padsize > 0 && gathered > hdrlen + padsize) {
                        /* Remove padding bytes after header */
                        memmove(frame_buf + hdrlen,
                                frame_buf + hdrlen + padsize,
                                gathered - hdrlen - padsize);
                        send_len = gathered - padsize;
                    }
                }

                /*
                 * Hardware crypto offload: if the driver flagged this frame
                 * for encryption (valid key-cache index + an encr type) and it
                 * carries the Protected bit, run the transform the AR9285
                 * crypto engine would — encrypt the payload in place and append
                 * the cipher's trailer.  mac80211 has already inserted the IV
                 * header (CCMP header for AES, 4-byte WEP IV for WEP) because
                 * ath9k advertises IEEE80211_KEY_FLAG_GENERATE_IV; the driver's
                 * frame_len already reserves the trailer (MIC / ICV).
                 */
                if ((ds_ctl0 & AR_DestIdxValid) &&
                    encr_type == AR_ENCR_TYPE_AES &&
                    vwifi_dot11_protected(frame_buf, send_len)) {
                    unsigned keyix = (ds_ctl1 & AR_DestIdx) >> AR_DestIdx_S;
                    uint8_t key[16];

                    if (vwifi_ath9k_keycache_ccm(s, keyix, key)) {
                        int enc_len = send_len;
                        if (vwifi_ccmp_encrypt(key, frame_buf, &enc_len,
                                               (int)sizeof(frame_buf)) == 0) {
                            vwifi_ath9k_trace("TX CCMP encrypt keyix=%u "
                                        "%u -> %d bytes", keyix, send_len,
                                        enc_len);
                            send_len = enc_len;
                        } else {
                            vwifi_ath9k_warn("TX CCMP encrypt failed "
                                        "(keyix=%u len=%u)", keyix, send_len);
                        }
                    } else {
                        vwifi_ath9k_warn("TX: no CCM key at cache slot %u",
                                    keyix);
                    }
                } else if ((ds_ctl0 & AR_DestIdxValid) &&
                           encr_type == AR_ENCR_TYPE_WEP &&
                           vwifi_dot11_protected(frame_buf, send_len)) {
                    unsigned keyix = (ds_ctl1 & AR_DestIdx) >> AR_DestIdx_S;
                    uint8_t key[16];
                    int keylen = vwifi_ath9k_keycache_wep(s, keyix, key);

                    if (keylen > 0) {
                        int enc_len = send_len;
                        if (vwifi_wep_encrypt(key, keylen, frame_buf, &enc_len,
                                              (int)sizeof(frame_buf)) == 0) {
                            vwifi_ath9k_trace("TX WEP encrypt keyix=%u "
                                        "%u -> %d bytes", keyix, send_len,
                                        enc_len);
                            send_len = enc_len;
                        } else {
                            vwifi_ath9k_warn("TX WEP encrypt failed "
                                        "(keyix=%u len=%u)", keyix, send_len);
                        }
                    } else {
                        vwifi_ath9k_warn("TX: no WEP key at cache slot %u",
                                    keyix);
                    }
                } else if ((ds_ctl0 & AR_DestIdxValid) &&
                           encr_type == AR_ENCR_TYPE_TKIP &&
                           vwifi_dot11_protected(frame_buf, send_len)) {
                    unsigned keyix = (ds_ctl1 & AR_DestIdx) >> AR_DestIdx_S;
                    uint8_t tk[16];

                    if (vwifi_ath9k_keycache_tkip(s, keyix, tk)) {
                        int enc_len = send_len;
                        if (vwifi_tkip_encrypt(tk, frame_buf, &enc_len,
                                               (int)sizeof(frame_buf)) == 0) {
                            vwifi_ath9k_trace("TX TKIP encrypt keyix=%u "
                                        "%u -> %d bytes", keyix, send_len,
                                        enc_len);
                            send_len = enc_len;
                        } else {
                            vwifi_ath9k_warn("TX TKIP encrypt failed "
                                        "(keyix=%u len=%u)", keyix, send_len);
                        }
                    } else {
                        vwifi_ath9k_warn("TX: no TKIP key at cache slot %u",
                                    keyix);
                    }
                }

                /*
                 * Use frame_len - FCS_LEN as the authoritative OTA
                 * frame size.  buf_len (used for DMA gathering) may
                 * include extra alignment bytes beyond the real frame.
                 * Sending those extra bytes corrupts trailing IEs and
                 * causes mac80211 to flag the beacon as corrupt.
                 *
                 * For an encrypted frame the driver's frame_len already
                 * accounts for the appended MIC (bf_frmlen includes
                 * IEEE80211_CCMP_MIC_LEN), so ota_len == send_len here and
                 * the cap below leaves the ciphertext + MIC intact.
                 */
                uint16_t ota_len = frame_len > 4 ? frame_len - 4 : 0;
                if (ota_len > 0 && ota_len < send_len) {
                    send_len = ota_len;
                }
                if (send_len > 0) {
                    vwifi_ath9k_send_rate(s, frame_buf, send_len, tx_rate);

                    /*
                     * Cache beacon frames for auto-retransmit.
                     * Beacon = management frame (type 0) subtype 8.
                     * Frame Control: bits[3:2]=type, bits[7:4]=subtype.
                     * Beacon FC byte 0: 0x80 (subtype=8, type=0).
                     */
                    if (send_len >= 24 && send_len <= 256 &&
                        (frame_buf[0] & 0xFC) == 0x80) {
                        memcpy(s->cached_beacon, frame_buf, send_len);
                        s->cached_beacon_len = send_len;
                        s->last_beacon_tx_ms =
                            qemu_clock_get_ms(QEMU_CLOCK_VIRTUAL);
                    }
                }
            }

            /* Update desc_addr to skip past any extra segment descs */
            if (seg_count > 0 && seg_desc != desc_addr) {
                desc_addr = seg_desc;
                ds_link = seg_link;
            }
        }

        /* Write TX completion status.
         *
         * The ath9k driver reads these to determine:
         *   - Whether the frame was ACKed (status1: AR_FrmXmitOK)
         *   - Which rate series was used (status1: AR_FinalTxIdx)
         *   - How many retries occurred (status1: AR_DataFailCnt)
         *   - RSSI of the received ACK (status0/5: AR_TxRSSI fields)
         *   - Whether TX is complete (status9: AR_TxDone)
         *
         * We report: success on first attempt at series 0, with a
         * realistic ACK RSSI.  This gives the rate control algorithm
         * (minstrel_ht) accurate feedback so it can ramp up rates.
         */
        {
            /*
             * ACK RSSI: use the medium default RSSI converted to
             * the unsigned 0..127 range the hardware uses.
             */
            uint8_t ack_rssi = (uint8_t)((int)(VWIFI_DEFAULT_RSSI) + 95);

            /* status0: per-chain TX RSSI for the received ACK
             *   AR_TxRSSIAnt00[7:0] = chain 0 RSSI */
            uint32_t s0 = (uint32_t)ack_rssi & 0xFF;

            /* status9: AR_TxDone (driver polls this for completion) */
            uint32_t s9 = AR_TxDone;

            /*
             * A-MPDU Block-Ack completion.  On the aggregate's last
             * subframe (AR_IsAggr set, AR_MoreAggr clear) report the
             * immediate BlockAck the peer would have returned: set
             * AR_TxBaStatus in status0, the per-subframe ACK bitmap in
             * status3/4, and the BA window start in AR_SeqNum (status9).
             * ath_tx_complete_aggr() reads exactly these from this
             * (bf_lastbf) descriptor; the intermediate subframes' status
             * words are never consulted, so they keep the plain
             * AR_TxDone/AR_FrmXmitOK written above/below.
             */
            if ((ds_ctl1 & AR_IsAggr) && !(ds_ctl1 & AR_MoreAggr) &&
                agg_n > 0) {
                uint16_t seq_st;
                uint32_t ba_low, ba_high;

                vwifi_ath9k_ba_bitmap(agg_seqnos, agg_n,
                                      &seq_st, &ba_low, &ba_high);

                s0 |= AR_TxBaStatus;
                s9 |= ((uint32_t)seq_st << AR_SeqNum_S) & AR_SeqNum;

                uint32_t txs3 = cpu_to_le32(ba_low);
                uint32_t txs4 = cpu_to_le32(ba_high);
                pci_dma_write(&s->parent_obj,
                              desc_addr + DESC_OFF_TX_STATUS3,
                              &txs3, 4);
                pci_dma_write(&s->parent_obj,
                              desc_addr + DESC_OFF_TX_STATUS4,
                              &txs4, 4);

                vwifi_ath9k_trace("TX A-MPDU complete: %d subframes, "
                            "seq_st=%u ba=%08x%08x", agg_n, seq_st,
                            ba_high, ba_low);
                agg_n = 0;  /* aggregate finished; ready for the next one */
            }

            uint32_t txs0 = cpu_to_le32(s0);

            /* status1: frame status + rate info
             *   AR_FrmXmitOK (bit 0) = 1 (ACK received)
             *   AR_DataFailCnt[11:8] = 0 (no retries)
             *   AR_FinalTxIdx[22:21] = 0 (used rate series 0) */
            uint32_t txs1 = cpu_to_le32(AR_FrmXmitOK);

            /* status5: combined TX RSSI for rate control
             *   AR_TxRSSIAnt10[7:0] = chain 1 RSSI (same for single-chain)
             *   AR_TxRSSICombined[31:24] = combined RSSI */
            uint32_t txs5 = cpu_to_le32(
                ((uint32_t)ack_rssi & 0xFF) |
                ((uint32_t)ack_rssi << AR_TxRSSICombined_S));

            uint32_t txs9 = cpu_to_le32(s9);

            pci_dma_write(&s->parent_obj,
                          desc_addr + DESC_OFF_TX_STATUS0,
                          &txs0, 4);
            pci_dma_write(&s->parent_obj,
                          desc_addr + DESC_OFF_TX_STATUS1,
                          &txs1, 4);
            pci_dma_write(&s->parent_obj,
                          desc_addr + DESC_OFF_TX_STATUS5,
                          &txs5, 4);
            pci_dma_write(&s->parent_obj,
                          desc_addr + DESC_OFF_TX_STATUS9,
                          &txs9, 4);
        }

        s->tx_frames_processed++;
        count++;
        desc_addr = ds_link;
    }

    if (count > 0) {
        s->isr_s0 |= (1 << qnum);
        vwifi_ath9k_raise_irq(s, AR_ISR_TXOK);
    }

    s->tx_queues[qnum].enabled = false;
    s->regs[AR_Q_TXE / 4] &= ~(1 << qnum);
}

/* -------------------------------------------------------------------
 *  RX DMA – Beacon injection
 * ------------------------------------------------------------------- */
static const uint8_t vwifi_ath9k_beacon_template[] = {
    /* Frame Control: beacon */
    0x80, 0x00,
    /* Duration */
    0x00, 0x00,
    /* DA: broadcast */
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    /* SA: our MAC */
    0x00, 0x03, 0x7F, 0xAA, 0xBB, 0xCC,
    /* BSSID */
    0x00, 0x03, 0x7F, 0xAA, 0xBB, 0xCC,
    /* Seq Control (patched) */
    0x00, 0x00,
    /* Timestamp (8 bytes, patched) */
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    /* Beacon Interval: 100 TU */
    0x64, 0x00,
    /* Capability: ESS + short preamble + short slot */
    0x21, 0x04,
    /* SSID: tag=0, len=8 */
    0x00, 0x08,
    'a', 't', 'h', '9', 'k', 'v', 'i', 'r',
    /* Supported Rates: tag=1, len=8 */
    0x01, 0x08,
    0x82, 0x84, 0x8B, 0x96, 0x0C, 0x18, 0x30, 0x48,
    /* DS Parameter Set: tag=3, len=1, chan=1 */
    0x03, 0x01, 0x01,
    /* FCS placeholder */
    0x00, 0x00, 0x00, 0x00,
};
#define VWIFI_ATH9K_BEACON_SIZE  sizeof(vwifi_ath9k_beacon_template)

static void vwifi_ath9k_inject_beacon(VwifiAth9kState *s)
{
    uint32_t desc_addr, ds_link, ds_data, ds_ctl1;
    uint8_t  beacon[VWIFI_ATH9K_BEACON_SIZE];
    uint32_t rx_status[9];
    uint16_t seq_num;

    if (!s->rx_enabled || s->rxdp == 0) {
        return;
    }

    desc_addr = s->rxdp;

    if (pci_dma_read(&s->parent_obj, desc_addr + DESC_OFF_LINK,
                     &ds_link, 4) != 0 ||
        pci_dma_read(&s->parent_obj, desc_addr + DESC_OFF_DATA,
                     &ds_data, 4) != 0 ||
        pci_dma_read(&s->parent_obj, desc_addr + DESC_OFF_CTL1,
                     &ds_ctl1, 4) != 0) {
        return;
    }

    ds_link = le32_to_cpu(ds_link);
    ds_data = le32_to_cpu(ds_data);
    if (ds_data == 0) return;

    /* Advance TSF (microseconds) */
    s->tsf_lo += ATH9K_BEACON_INTERVAL_MS * 1000;
    if (s->tsf_lo < ATH9K_BEACON_INTERVAL_MS * 1000) {
        s->tsf_hi++;
    }

    /* Build beacon */
    memcpy(beacon, vwifi_ath9k_beacon_template, VWIFI_ATH9K_BEACON_SIZE);
    beacon[24] = (uint8_t)(s->tsf_lo);
    beacon[25] = (uint8_t)(s->tsf_lo >> 8);
    beacon[26] = (uint8_t)(s->tsf_lo >> 16);
    beacon[27] = (uint8_t)(s->tsf_lo >> 24);
    beacon[28] = (uint8_t)(s->tsf_hi);
    beacon[29] = (uint8_t)(s->tsf_hi >> 8);
    beacon[30] = (uint8_t)(s->tsf_hi >> 16);
    beacon[31] = (uint8_t)(s->tsf_hi >> 24);
    seq_num = (uint16_t)(s->rx_frames_injected & 0xFFF) << 4;
    beacon[22] = (uint8_t)(seq_num);
    beacon[23] = (uint8_t)(seq_num >> 8);

    /* Write beacon to guest buffer */
    pci_dma_write(&s->parent_obj, ds_data, beacon, VWIFI_ATH9K_BEACON_SIZE);

    /* Build RX status words */
    memset(rx_status, 0, sizeof(rx_status));
    rx_status[0] = cpu_to_le32(
        (40 & 0xFF) | ((uint32_t)ATH9K_RATE_6M << 24));
    rx_status[1] = cpu_to_le32(VWIFI_ATH9K_BEACON_SIZE & AR_DataLen);
    rx_status[2] = cpu_to_le32(s->tsf_lo);
    rx_status[4] = cpu_to_le32((40 & 0xFF) | ((uint32_t)40 << 24));
    rx_status[8] = cpu_to_le32(AR_RxDone | AR_RxFrameOK);

    pci_dma_write(&s->parent_obj, desc_addr + DESC_OFF_RX_STATUS0,
                  rx_status, sizeof(rx_status));

    s->rx_frames_injected++;

    /* Advance RX descriptor pointer */
    if (ds_link != 0) {
        s->rxdp = ds_link;
    } else {
        /* Lazy RXEOL — same as inject_rx_frame */
        s->rxdp_ring_empty = true;
    }

    /* Beacons are low-rate; always deliver IRQ immediately */
    vwifi_ath9k_raise_irq(s, AR_ISR_RXOK);
}

/* -------------------------------------------------------------------
 *  Beacon timer
 * ------------------------------------------------------------------- */
static void vwifi_ath9k_beacon_timer_cb(void *opaque)
{
    VwifiAth9kState *s = VWIFI_ATH9K(opaque);
    if (!s->beacon_timer_active || !s->rx_enabled) return;
    vwifi_ath9k_inject_beacon(s);
    timer_mod(s->beacon_timer,
              qemu_clock_get_ms(QEMU_CLOCK_VIRTUAL) +
              ATH9K_BEACON_INTERVAL_MS);
}

static void vwifi_ath9k_beacon_timer_start(VwifiAth9kState *s)
{
    if (s->beacon_timer_active) return;
    s->beacon_timer_active = true;
    timer_mod(s->beacon_timer,
              qemu_clock_get_ms(QEMU_CLOCK_VIRTUAL) +
              ATH9K_BEACON_INTERVAL_MS);
    vwifi_ath9k_trace("Beacon timer started");
}

static void vwifi_ath9k_beacon_timer_stop(VwifiAth9kState *s)
{
    if (!s->beacon_timer_active) return;
    s->beacon_timer_active = false;
    s->swba_pending = false;
    s->cached_beacon_len = 0;
    s->last_beacon_tx_ms = 0;
    timer_del(s->beacon_timer);
}

/* -------------------------------------------------------------------
 *  SWBA (Software Beacon Alert) timer
 *
 *  On real hardware the MAC fires AR_ISR_SWBA at each TBTT.
 *  The kernel's beacon tasklet then builds a beacon frame, sets up
 *  the TX descriptor, and writes AR_Q_TXE to transmit it.
 *  We emulate this by firing SWBA periodically so the driver sends
 *  real beacons through our TX DMA path → medium.
 * ------------------------------------------------------------------- */
static void vwifi_ath9k_swba_timer_cb(void *opaque)
{
    VwifiAth9kState *s = VWIFI_ATH9K(opaque);
    if (!s->swba_timer_active) return;

    /* Advance TSF (microseconds) */
    s->tsf_lo += ATH9K_BEACON_INTERVAL_MS * 1000;
    if (s->tsf_lo < ATH9K_BEACON_INTERVAL_MS * 1000) {
        s->tsf_hi++;
    }

    /*
     * Self-pacing SWBA delivery.
     *
     * On each timer tick (every 100ms virtual time), we either:
     * (a) Clear swba_pending from the previous tick, OR
     * (b) Raise a new SWBA if the previous one was already cleared.
     *
     * This guarantees at most ONE SWBA per 100ms interval.  Without
     * this, the driver's ISR write-to-clear acknowledges SWBA almost
     * instantly, allowing the next timer tick to fire another SWBA
     * before the driver finishes its ISR handler.  Under load, this
     * creates 30-60 SWBA interrupts between data TX events, starving
     * data and beacon transmission until the link dies.
     *
     * By clearing swba_pending only here (not on ISR write-to-clear),
     * the timer itself paces delivery: raise on tick N, clear on tick
     * N+1, raise on tick N+2, etc.  Worst case: 5 SWBAs/sec (every
     * other tick).  The driver tolerates missed beacons easily.
     */
    if (s->swba_pending) {
        /* Previous SWBA still in flight — just acknowledge it now */
        s->swba_pending = false;
    } else {
        /* Raise new SWBA */
        s->swba_pending = true;
        vwifi_ath9k_raise_irq(s, AR_ISR_SWBA);
    }

    /*
     * Fallback beacon: if we have a cached beacon and the driver hasn't
     * sent one in over 500ms (5 beacon intervals), re-send the cached
     * beacon on the medium ourselves.  This mimics real hardware where
     * the beacon buffer is preloaded and the MAC auto-transmits it at
     * each TBTT regardless of driver involvement.
     *
     * This keeps the link alive during TCP stalls where the driver's
     * beacon tasklet can't produce new beacons because the TX descriptor
     * chain is stale.
     */
    if (s->cached_beacon_len > 0 && s->medium_fd >= 0) {
        int64_t now_ms = qemu_clock_get_ms(QEMU_CLOCK_VIRTUAL);
        if (now_ms - s->last_beacon_tx_ms > 500) {
            /* Update TSF timestamp in cached beacon (bytes 24-31) */
            if (s->cached_beacon_len >= 32) {
                s->cached_beacon[24] = (uint8_t)(s->tsf_lo);
                s->cached_beacon[25] = (uint8_t)(s->tsf_lo >> 8);
                s->cached_beacon[26] = (uint8_t)(s->tsf_lo >> 16);
                s->cached_beacon[27] = (uint8_t)(s->tsf_lo >> 24);
                s->cached_beacon[28] = (uint8_t)(s->tsf_hi);
                s->cached_beacon[29] = (uint8_t)(s->tsf_hi >> 8);
                s->cached_beacon[30] = (uint8_t)(s->tsf_hi >> 16);
                s->cached_beacon[31] = (uint8_t)(s->tsf_hi >> 24);
            }
            vwifi_ath9k_send(s, s->cached_beacon, s->cached_beacon_len);
            s->last_beacon_tx_ms = now_ms;
        }
    }

    timer_mod(s->swba_timer,
              qemu_clock_get_ms(QEMU_CLOCK_VIRTUAL) +
              ATH9K_BEACON_INTERVAL_MS);
}

static void vwifi_ath9k_swba_timer_start(VwifiAth9kState *s)
{
    if (s->swba_timer_active) return;
    s->swba_timer_active = true;
    timer_mod(s->swba_timer,
              qemu_clock_get_ms(QEMU_CLOCK_VIRTUAL) +
              ATH9K_BEACON_INTERVAL_MS);
    vwifi_ath9k_trace("SWBA timer started");
}

static void vwifi_ath9k_swba_timer_stop(VwifiAth9kState *s)
{
    if (!s->swba_timer_active) return;
    s->swba_timer_active = false;
    timer_del(s->swba_timer);
    vwifi_ath9k_trace("SWBA timer stopped");
}

static inline bool in_range(hwaddr addr, uint32_t base, uint32_t count)
{
    return addr >= base && addr < base + count * 4;
}

/* -------------------------------------------------------------------
 *  Phase 3: RX injection from virtual medium
 *
 *  When a frame arrives from the medium (another VM), inject it into
 *  the RX DMA path the same way beacons are injected.
 * ------------------------------------------------------------------- */
/*
 * Inject one received frame into the RX descriptor ring.  Returns true if a
 * frame was written into a descriptor (the caller batches these and raises a
 * single RXOK once the whole burst is drained — see vwifi_ath9k_fd_read).
 * Returns false when nothing was injected (RX disabled, ring empty/RXEOL, or
 * a DMA error); RXEOL is still raised inline as it is an error condition, not
 * a normal completion.
 */
static bool vwifi_ath9k_inject_rx_frame(VwifiAth9kState *s,
                                  const struct vwifi_frame_hdr *hdr,
                                  const uint8_t *frame, uint16_t frame_len)
{
    uint32_t desc_addr, ds_link, ds_data, ds_ctl1;
    uint32_t rx_status[9];
    uint8_t  rssi_u8;
    uint16_t rx_len;  /* length reported in RX status (includes FCS) */

    if (!s->rx_enabled || s->rxdp == 0) {
        return false;
    }

    desc_addr = s->rxdp;

    /*
     * Lazy RXEOL: if the ring was empty on the last inject, re-read
     * ds_link from the current descriptor.  The driver's RXOK tasklet
     * may have processed the previous frame, recycled its descriptor,
     * and relinked it — giving us a new ds_link to follow.
     */
    if (s->rxdp_ring_empty) {
        if (pci_dma_read(&s->parent_obj, desc_addr + DESC_OFF_LINK,
                         &ds_link, 4) != 0) {
            return false;
        }
        ds_link = le32_to_cpu(ds_link);
        if (ds_link != 0) {
            /* Driver relinked — advance past the already-written descriptor */
            vwifi_ath9k_trace("RX ring refilled: 0x%08x -> 0x%08x",
                        desc_addr, ds_link);
            s->rxdp = ds_link;
            s->rxdp_ring_empty = false;
            desc_addr = ds_link;
        } else {
            /* Still empty — NOW raise RXEOL for real */
            vwifi_ath9k_trace("RX ring still empty at 0x%08x — RXEOL", desc_addr);
            vwifi_ath9k_raise_irq(s, AR_ISR_RXEOL);
            s->rx_enabled = false;
            s->rxdp_ring_empty = false;
            return false;
        }
    }

    /* Read the current RX descriptor */
    if (pci_dma_read(&s->parent_obj, desc_addr + DESC_OFF_LINK,
                     &ds_link, 4) != 0 ||
        pci_dma_read(&s->parent_obj, desc_addr + DESC_OFF_DATA,
                     &ds_data, 4) != 0 ||
        pci_dma_read(&s->parent_obj, desc_addr + DESC_OFF_CTL1,
                     &ds_ctl1, 4) != 0) {
        return false;
    }

    ds_link = le32_to_cpu(ds_link);
    ds_data = le32_to_cpu(ds_data);
    if (ds_data == 0) {
        return false;
    }

    /*
     * Hardware crypto offload (RX): if the frame carries the Protected bit,
     * decrypt it with a matching key from the key cache before DMAing it to
     * the driver, exactly as the AR9285 crypto engine would.  mac80211 relies
     * on this: ath9k_cmn_rx_skb_postprocess() sets RX_FLAG_DECRYPTED only when
     * the RX status reports a valid key index and no decrypt error, after
     * which mac80211 strips the IV header and the trailing MIC/ICV.  If we
     * cannot decrypt (unknown/absent key, or an unsupported cipher such as
     * TKIP), we flag a decrypt error so the frame is dropped rather than
     * delivered as garbage.
     *
     * The cipher is inferred from the IV header and, for the extended-IV
     * ciphers, from which key verifies: CCMP and TKIP both set the ExtIV bit
     * (8-octet header) while WEP does not (4-octet header).  For an ExtIV frame
     * we try CCM first (64-bit MIC) and then TKIP (32-bit ICV); a network only
     * ever programs one pairwise cipher, so at most one set of slots is
     * populated and the MIC/ICV check selects the right key.
     *
     * This path is frame-type agnostic: it keys off the Protected bit, so
     * individually-addressed protected management frames (802.11w / PMF, which
     * WPA3 requires) decrypt through the same CCM engine as data frames.
     * Group-addressed robust management frames use BIP (not the Protected bit)
     * and are verified by mac80211 in software, so they pass through untouched.
     */
    uint8_t rx_decbuf[ATH9K_MAX_FRAME_SIZE];
    uint8_t rx_keyix = 0;
    const char *rx_cipher = "WEP";
    bool    rx_decrypted = false;
    bool    rx_decrypt_err = false;
    if (vwifi_dot11_protected(frame, frame_len)) {
        int rx_hdrlen = vwifi_wep_hdrlen(frame, frame_len);
        bool ext_iv = vwifi_wep_is_extiv(frame, frame_len, rx_hdrlen);
        bool ok = false;

        if (frame_len > (int)sizeof(rx_decbuf)) {
            ok = false;
        } else if (ext_iv) {
            if (vwifi_ath9k_rx_decrypt(s, frame, rx_decbuf, frame_len,
                                       &rx_keyix)) {
                ok = true;
                rx_cipher = "CCMP";
            } else if (vwifi_ath9k_rx_decrypt_tkip(s, frame, rx_decbuf,
                                                   frame_len, &rx_keyix)) {
                ok = true;
                rx_cipher = "TKIP";
            }
        } else {
            ok = vwifi_ath9k_rx_decrypt_wep(s, frame, rx_decbuf, frame_len,
                                            &rx_keyix);
        }

        if (ok) {
            frame = rx_decbuf;
            rx_decrypted = true;
            vwifi_ath9k_trace("RX %s decrypt ok keyix=%u len=%u",
                        rx_cipher, rx_keyix, frame_len);
        } else {
            rx_decrypt_err = true;
            vwifi_ath9k_warn("RX %s decrypt failed (len=%u) — dropping",
                        ext_iv ? "CCMP/TKIP" : "WEP", frame_len);
        }
    }

    /*
     * Re-insert the 802.11 header alignment padding the guest's ath9k
     * driver expects on RX -- the mirror image of the TX padding strip.
     *
     * Real AR9285 hardware DMAs (hdrlen & 3) padding bytes between the
     * 802.11 header and the payload so the payload is 4-byte aligned.
     * The driver removes them (memmove header forward + skb_pull) before
     * handing the frame to mac80211.  If we DON'T insert them, the driver
     * strips real payload bytes instead: for a QoS-Data frame (hdrlen=26,
     * padsize=2) that eats the first two bytes of the LLC/SNAP header
     * (AA AA), so mac80211 fails SNAP demux and silently drops all L3
     * traffic.  hdrlen is derived from Frame Control exactly as on TX.
     */
    uint16_t hdrlen = 24;
    uint8_t  padsize = 0;
    if (frame_len >= 2) {
        uint16_t fc = frame[0] | (frame[1] << 8);
        uint8_t ftype = (fc >> 2) & 3;
        uint8_t stype = (fc >> 4) & 0xF;
        if (ftype == 2 && (stype & 0x8))   /* Data frame with QoS */
            hdrlen = 26;
        if ((fc & 0x0300) == 0x0300)       /* 4-addr (ToDS + FromDS) */
            hdrlen += 6;
        padsize = hdrlen & 3;
    }

    if (padsize > 0 && frame_len > hdrlen) {
        static const uint8_t pad_bytes[4] = {0x00, 0x00, 0x00, 0x00};
        /* header, then padding, then payload */
        pci_dma_write(&s->parent_obj, ds_data, frame, hdrlen);
        pci_dma_write(&s->parent_obj, ds_data + hdrlen, pad_bytes, padsize);
        pci_dma_write(&s->parent_obj, ds_data + hdrlen + padsize,
                      frame + hdrlen, frame_len - hdrlen);
    } else {
        padsize = 0;  /* nothing to align (e.g. management frames) */
        pci_dma_write(&s->parent_obj, ds_data, frame, frame_len);
    }

    /*
     * Append 4 dummy FCS bytes after the (padded) frame.
     * Real hardware includes FCS in the received data and reports
     * DataLen inclusive of FCS.  The driver trims FCS_LEN (4) bytes.
     * Without this, the driver would strip 4 real bytes from the end.
     */
    {
        static const uint8_t dummy_fcs[4] = {0x00, 0x00, 0x00, 0x00};
        pci_dma_write(&s->parent_obj, ds_data + frame_len + padsize,
                      dummy_fcs, 4);
    }
    rx_len = frame_len + padsize + 4;  /* DataLen includes padding + FCS */

    /* Convert signed RSSI to unsigned (ath9k expects 0..127 range) */
    rssi_u8 = (uint8_t)((int)(hdr->rssi) + 95);  /* noise floor -95 */
    if (rssi_u8 > 127) {
        rssi_u8 = 127;
    }

    /* Build RX status words */
    memset(rx_status, 0, sizeof(rx_status));

    /* status0: RSSI ant0 | rate code */
    rx_status[0] = cpu_to_le32(
        ((uint32_t)rssi_u8 & 0xFF) |
        ((uint32_t)hdr->rate_code << AR_RxRate_S));

    /* status1: data length (including FCS) */
    rx_status[1] = cpu_to_le32(rx_len & AR_DataLen);

    /* status2: RX timestamp (TSF low) */
    rx_status[2] = cpu_to_le32(hdr->tsf_lo);

    /* status4: per-chain RSSI (Ant10) + combined RSSI.
     * AR_RxRSSIAnt10 = bits[7:0], AR_RxRSSICombined = bits[31:24].
     * The ath9k driver reads combined RSSI from ds_rxstatus4. */
    rx_status[4] = cpu_to_le32(
        ((uint32_t)rssi_u8 & 0xFF) |
        ((uint32_t)rssi_u8 << AR_RxRSSICombined_S));

    /* status5: EVM data (zero is fine) */

    /*
     * status8: RxDone + RxFrameOK, plus hardware-crypto key status.
     *  - Decrypted OK: report a valid key index so the driver marks the
     *    frame RX_FLAG_DECRYPTED and mac80211 skips software crypto.
     *  - Decrypt failed: report AR_DecryptCRCErr so the driver drops it.
     */
    {
        uint32_t s8 = AR_RxDone | AR_RxFrameOK;
        if (rx_decrypted) {
            s8 |= AR_RxKeyIdxValid |
                  (((uint32_t)rx_keyix << AR_KeyIdx_S) & AR_KeyIdx);
        } else if (rx_decrypt_err) {
            s8 |= AR_DecryptCRCErr;
        }
        rx_status[8] = cpu_to_le32(s8);
    }

    pci_dma_write(&s->parent_obj, desc_addr + DESC_OFF_RX_STATUS0,
                  rx_status, sizeof(rx_status));

    s->rx_frames_injected++;
    s->rx_frames_from_medium++;

    /* Advance RX descriptor pointer */
    if (ds_link != 0) {
        s->rxdp = ds_link;
    } else {
        /*
         * Bug 13 fix: Lazy RXEOL.
         *
         * ds_link == 0 means we've used the last prepared descriptor.
         * DON'T immediately raise RXEOL!  The driver will soon process
         * this frame (via the RXOK interrupt below), recycle the
         * descriptor, and patch ds_link of a predecessor to re-extend
         * the chain.
         *
         * We flag the ring as empty so the NEXT inject attempt will
         * re-read ds_link.  If the driver has relinked by then, we
         * continue smoothly without any RXEOL at all.  If not, THEN
         * we raise RXEOL.
         *
         * This matches real hardware behavior where the DMA engine
         * pauses at the end of the chain and only signals RXEOL when
         * a new frame arrives and there's truly nowhere to put it.
         */
        s->rxdp_ring_empty = true;
        vwifi_ath9k_trace("RX ring end at 0x%08x — deferring RXEOL", desc_addr);
    }

    return true;
}

/*
 * Process a complete medium message (after length prefix has been stripped).
 * Validates the header and injects the frame into the RX path.
 */
/* Returns true if the frame was injected into the RX ring (so the caller can
 * coalesce the RXOK interrupt across a whole drained burst). */
static bool vwifi_ath9k_process_msg(VwifiAth9kState *s,
                                     const uint8_t *msg, uint32_t msg_len)
{
    struct vwifi_frame_hdr hdr;
    const uint8_t *frame;
    uint32_t hdr_size;

    /* Accept v1 (28 bytes) or v2 (40 bytes) headers */
    if (msg_len < VWIFI_HDR_SIZE_MIN) {
        vwifi_ath9k_warn("MEDIUM RX: message too short (%u bytes)", msg_len);
        return false;
    }

    /* Peek at version to determine header size */
    memset(&hdr, 0, sizeof(hdr));
    memcpy(&hdr, msg, VWIFI_HDR_SIZE_V1);  /* copy v1 portion */

    if (hdr.magic != VWIFI_MAGIC) {
        vwifi_ath9k_warn("MEDIUM RX: bad magic 0x%08x", hdr.magic);
        return false;
    }
    if (hdr.version == 1) {
        hdr_size = VWIFI_HDR_SIZE_V1;
    } else if (hdr.version == 2) {
        hdr_size = VWIFI_HDR_SIZE;
        if (msg_len >= VWIFI_HDR_SIZE) {
            memcpy(&hdr, msg, VWIFI_HDR_SIZE);  /* copy full v2 header */
        } else {
            hdr_size = VWIFI_HDR_SIZE_V1;  /* truncated v2, treat as v1 */
        }
    } else {
        vwifi_ath9k_warn("MEDIUM RX: unknown version %u", hdr.version);
        return false;
    }
    if (hdr.frame_len == 0 || hdr.frame_len > VWIFI_MAX_FRAME_SIZE) {
        vwifi_ath9k_warn("MEDIUM RX: bad frame_len %u", hdr.frame_len);
        return false;
    }
    if (msg_len < hdr_size + hdr.frame_len) {
        vwifi_ath9k_warn("MEDIUM RX: truncated (msg=%u, need=%u)",
                   msg_len, hdr_size + hdr.frame_len);
        return false;
    }

    /* Don't inject our own frames back (the hub broadcasts to all) */
    if (memcmp(hdr.tx_mac, s->our_mac, 6) == 0) {
        return false;
    }

    frame = msg + hdr_size;

    vwifi_ath9k_trace("MEDIUM RX: %u byte frame from "
                "%02x:%02x:%02x:%02x:%02x:%02x (total %" PRIu64 ")",
                hdr.frame_len,
                hdr.tx_mac[0], hdr.tx_mac[1], hdr.tx_mac[2],
                hdr.tx_mac[3], hdr.tx_mac[4], hdr.tx_mac[5],
                s->rx_frames_from_medium + 1);

    /*
     * If RX DMA is off (rxdp == 0 during a full reset), silently drop.
     *
     * Real AR9285 hardware has no buffering when RX DMA is stopped —
     * frames are simply lost.  The driver expects this and handles it:
     *  - Beacons: arrive every 100ms; missing a few during a brief
     *    reset window is normal and well within beacon-loss tolerance.
     *  - Data frames: TCP handles retransmission.
     *
     * Note: we do NOT check IER here.  IER is disabled during normal
     * ISR processing (not just resets), and the descriptor ring is
     * perfectly valid during that time.
     */
    if (!s->rx_enabled || s->rxdp == 0) {
        return false;
    }

    return vwifi_ath9k_inject_rx_frame(s, &hdr, frame, hdr.frame_len);
}

/* -------------------------------------------------------------------
 *  FD read handler for the medium socket
 *
 *  Registered via qemu_set_fd_handler() — called when the socket
 *  has data available.  We reassemble length-prefixed messages.
 * ------------------------------------------------------------------- */
static void vwifi_ath9k_fd_read(void *opaque)
{
    VwifiAth9kState *s = VWIFI_ATH9K(opaque);
    uint32_t avail, msg_len, net_len, consumed;
    ssize_t n;

    if (s->medium_fd < 0) {
        return;
    }

    avail = VWIFI_RXBUF_SIZE - s->medium_rxbuf_used;
    if (avail == 0) {
        vwifi_ath9k_error("MEDIUM RX: buffer full, resetting");
        s->medium_rxbuf_used = 0;
        return;
    }

    n = read(s->medium_fd, s->medium_rxbuf + s->medium_rxbuf_used, avail);
    if (n <= 0) {
        if (n < 0 && (errno == EINTR || errno == EAGAIN)) {
            return;
        }
        vwifi_ath9k_trace("MEDIUM: socket closed (read returned %zd)", n);
        qemu_set_fd_handler(s->medium_fd, NULL, NULL, NULL);
        close(s->medium_fd);
        s->medium_fd = -1;
        s->medium_connected = false;
        s->medium_rxbuf_used = 0;
        vwifi_ath9k_schedule_reconnect(s);
        return;
    }
    s->medium_rxbuf_used += (uint32_t)n;

    /*
     * Drain every complete message this read produced, then raise a single
     * RXOK for the whole burst.  The ath9k RX tasklet drains the descriptor
     * ring until it hits a descriptor without AR_RxDone, so one interrupt
     * per burst is equivalent to one-per-frame for delivery but far cheaper
     * — and it is exactly how an A-MPDU's de-aggregated subframes complete
     * on real hardware (one RXOK for the aggregate, not one per MPDU).
     */
    bool any_rx = false;
    while (s->medium_rxbuf_used >= 4) {
        memcpy(&net_len, s->medium_rxbuf, 4);
        msg_len = ntohl(net_len);

        if (msg_len > VWIFI_MAX_MSG_SIZE) {
            vwifi_ath9k_error("MEDIUM RX: absurd msg_len %u, resetting buffer",
                        msg_len);
            s->medium_rxbuf_used = 0;
            if (any_rx) {
                vwifi_ath9k_raise_irq(s, AR_ISR_RXOK);
            }
            return;
        }

        if (s->medium_rxbuf_used < 4 + msg_len) {
            break;
        }

        if (vwifi_ath9k_process_msg(s, s->medium_rxbuf + 4, msg_len)) {
            any_rx = true;
        }

        consumed = 4 + msg_len;
        if (consumed < s->medium_rxbuf_used) {
            memmove(s->medium_rxbuf, s->medium_rxbuf + consumed,
                    s->medium_rxbuf_used - consumed);
        }
        s->medium_rxbuf_used -= consumed;
    }

    if (any_rx) {
        vwifi_ath9k_raise_irq(s, AR_ISR_RXOK);
    }
}


/* -------------------------------------------------------------------
 *  MMIO read handler
 * ------------------------------------------------------------------- */
static uint64_t vwifi_ath9k_mmio_read(void *opaque, hwaddr addr, unsigned size)
{
    VwifiAth9kState *s = VWIFI_ATH9K(opaque);
    uint32_t val = 0;

    s->read_count++;

    /* EEPROM trigger: reading 0x2000..0x2800 loads word into status reg */
    if (addr >= AR5416_EEPROM_OFFSET &&
        addr < AR5416_EEPROM_OFFSET + (ATH9K_EEPROM_4K_SIZE_WORDS << AR5416_EEPROM_S)) {
        vwifi_ath9k_eeprom_trigger(s, addr);
        return 0;  /* the trigger read itself returns 0 on real HW */
    }

    addr &= ~3ULL;

    switch (addr) {
    case AR_SREV:
        return ATH9K_SREV_AR9285_V12;

    case AR_CR:
    case AR_CFG:
    case AR_RXCFG:
    case AR_TXCFG:
    case AR_MIBC:
        val = vwifi_ath9k_reg_read_raw(s, addr);
        break;

    case AR_RXDP:
        val = s->rxdp;
        break;

    /* Interrupts – ISR is read-to-clear */
    case AR_ISR:
        val = s->isr;
        s->isr = 0;
        vwifi_ath9k_update_irq(s);
        return val;
    case AR_ISR_S0:
        val = s->isr_s0; s->isr_s0 = 0;
        break;
    case AR_ISR_S1:
        val = s->isr_s1; s->isr_s1 = 0;
        break;
    case AR_ISR_S2:
    case AR_ISR_S3:
    case AR_ISR_S4:
    case AR_ISR_S5:
        val = 0;
        break;

    case AR_IMR:
        val = s->imr;
        break;
    case AR_IMR_S0:
        val = s->imr_s0;
        break;
    case AR_IMR_S1:
        val = s->imr_s1;
        break;
    case AR_IMR_S2:
    case AR_IMR_S3:
    case AR_IMR_S4:
    case AR_IMR_S5:
        val = vwifi_ath9k_reg_read_raw(s, addr);
        break;
    case AR_IER:
        val = s->ier;
        break;

    case AR_INTR_SYNC_CAUSE:
        val = 0;
        break;
    case AR_INTR_SYNC_ENABLE:
    case AR_INTR_ASYNC_ENABLE:
    case AR_INTR_ASYNC_MASK:
        val = vwifi_ath9k_reg_read_raw(s, addr);
        break;
    case AR_INTR_ASYNC_CAUSE:
        val = (s->isr & s->imr) ? AR_INTR_MAC_IRQ : 0;
        break;

    /* TX queues */
    case AR_Q_TXE:
        val = s->regs[AR_Q_TXE / 4];
        break;
    case AR_Q_TXD:
        val = 0;
        break;

    /* RTC / Power */
    case AR_PM_STATE:
        val = 0;
        break;
    case AR_RTC_STATUS:
        val = AR_RTC_STATUS_ON;
        break;
    case AR_RTC_RC:
    case AR_RTC_RESET:
    case AR_RTC_FORCE_WAKE:
    case AR_RTC_PLL_CONTROL:
    case AR_RTC_PLL_CONTROL2:
    case AR_HOST_TIMEOUT:
        val = vwifi_ath9k_reg_read_raw(s, addr);
        break;

    /* EEPROM status */
    case AR_EEPROM:
        val = 0;  /* AR_EEPROM_ABSENT bit clear = EEPROM present */
        break;
    case AR_EEPROM_STATUS_DATA:
        /* vwifi_ath9k_eeprom_trigger() stores the word here; BUSY=0 means ready */
        val = vwifi_ath9k_reg_read_raw(s, addr);
        break;

    /* GPIO */
    case AR_GPIO_IN_OUT:
        val = 0;
        break;
    case AR_GPIO_OE_OUT:
    case AR_GPIO_INPUT_EN_VAL:
        val = vwifi_ath9k_reg_read_raw(s, addr);
        break;

    /* PCIe / WA */
    case AR_PCIE_PM_CTRL:
    case AR_WA:
        val = vwifi_ath9k_reg_read_raw(s, addr);
        break;

    /* Station / BSS */
    case AR_STA_ID0:
    case AR_STA_ID1:
    case AR_BSS_ID0:
    case AR_BSS_ID1:
    case AR_RX_FILTER:
    case AR_DIAG_SW:
    case AR_PCU_MISC_MODE2:
    case AR_PHY_ERR:
    case AR_RXBUF_READ:
        val = vwifi_ath9k_reg_read_raw(s, addr);
        break;

    /* Observation / DMA debug: all idle */
    case AR_OBS:
    case AR_DMADBG_0: case AR_DMADBG_1: case AR_DMADBG_2: case AR_DMADBG_3:
    case AR_DMADBG_4: case AR_DMADBG_5: case AR_DMADBG_6: case AR_DMADBG_7:
        val = 0;
        break;

    /* Beacon / Timer */
    case AR_TIME_OUT:
    case AR_RSSI_THR:
    case AR_USEC:
    case AR_BEACON_PERIOD:
    case AR_DBA_PERIOD:
    case AR_TIMER_MODE:
    case AR_NEXT_TBTT_TIMER:
    case AR_NEXT_DMA_BEACON_ALERT:
    case AR_NEXT_SWBA:
    case AR_NEXT_TIM:
    case AR_NEXT_DTIM:
        val = vwifi_ath9k_reg_read_raw(s, addr);
        break;

    /* PHY */
    case AR_PHY_ACTIVE:
        val = s->phy_active ? AR_PHY_ACTIVE_EN : 0;
        break;
    case AR_PHY_MODE:
    case AR_PHY_CCA:
        val = vwifi_ath9k_reg_read_raw(s, addr);
        break;
    case AR_PHY_AGC_CONTROL:
        /* Cal bits auto-clear: calibration "completes" instantly */
        val = vwifi_ath9k_reg_read_raw(s, addr) &
              ~(AR_PHY_AGC_CONTROL_CAL | AR_PHY_AGC_CONTROL_NF);
        break;
    case AR_PHY_RFBUS_REQ:
        val = vwifi_ath9k_reg_read_raw(s, addr);
        break;
    case AR_PHY_RFBUS_GRANT:
        val = AR_PHY_RFBUS_GRANT_EN; /* always grant */
        break;

    /* Global IFS / DCU */
    case AR_D_GBL_IFS_SIFS:
    case AR_D_GBL_IFS_SLOT:
    case AR_D_GBL_IFS_EIFS:
    case AR_D_GBL_IFS_MISC:
    case AR_D_FPCTL:
        val = vwifi_ath9k_reg_read_raw(s, addr);
        break;

    default:
        /* Per-queue TX descriptor pointers */
        if (in_range(addr, AR_Q0_TXDP, AR_NUM_QCU)) {
            val = s->tx_queues[(addr - AR_Q0_TXDP) / 4].txdp;
            break;
        }
        /* Per-queue/DCU config registers */
        if (in_range(addr, AR_Q0_CBRCFG, AR_NUM_QCU) ||
            in_range(addr, AR_Q0_RDYTIMECFG, AR_NUM_QCU) ||
            in_range(addr, AR_Q0_MISC, AR_NUM_QCU) ||
            in_range(addr, AR_Q0_STS, AR_NUM_QCU) ||
            in_range(addr, AR_D0_QCUMASK, AR_NUM_DCU) ||
            in_range(addr, AR_D0_LCL_IFS, AR_NUM_DCU) ||
            in_range(addr, AR_D0_RETRY_LIMIT, AR_NUM_DCU) ||
            in_range(addr, AR_D0_CHNTIME, AR_NUM_DCU) ||
            in_range(addr, AR_D0_MISC, AR_NUM_DCU) ||
            in_range(addr, AR_D0_SEQNUM, AR_NUM_DCU)) {
            val = vwifi_ath9k_reg_read_raw(s, addr);
            break;
        }
        /* Hardware key cache – shadow storage, read back silently */
        if (addr >= AR_KEYTABLE_0 && addr < AR_KEYTABLE_END) {
            val = vwifi_ath9k_reg_read_raw(s, addr);
            break;
        }
        /* PHY register space */
        if (addr >= AR_PHY_BASE && addr < ATH9K_MMIO_SIZE) {
            val = vwifi_ath9k_reg_read_raw(s, addr);
            break;
        }
        /* Truly unhandled */
        val = vwifi_ath9k_reg_read_raw(s, addr);
        s->unhandled_read_count++;
        if (s->unhandled_read_count <= 200) {
            vwifi_ath9k_warn("UNHANDLED read  %-24s = 0x%08x",
                       vwifi_ath9k_reg_name(addr), val);
        }
        return val;
    }

    return val;
}


/* -------------------------------------------------------------------
 *  MMIO write handler
 * ------------------------------------------------------------------- */
static void vwifi_ath9k_mmio_write(void *opaque, hwaddr addr, uint64_t val64,
                             unsigned size)
{
    VwifiAth9kState *s = VWIFI_ATH9K(opaque);
    uint32_t val = (uint32_t)val64;
    int q;

    s->write_count++;
    addr &= ~3ULL;

    switch (addr) {

    /* --- MAC / DMA core --- */
    case AR_CR:
        vwifi_ath9k_reg_write_raw(s, addr, val);
        if (val & AR_CR_RXE) {
            s->rx_enabled = true;
            s->rxdp_ring_empty = false;  /* driver is providing fresh ring */
            vwifi_ath9k_trace("RX DMA enabled");
            /* Phase 2 standalone mode: self-inject beacons into own RX.
             * When medium is connected, SWBA timer handles beacons
             * via the driver's real TX path instead. */
            if (!s->medium_connected) {
                vwifi_ath9k_beacon_timer_start(s);
            }
        }
        if (val & AR_CR_RXD) {
            /*
             * Do NOT set rx_enabled = false.
             *
             * On real AR9285 hardware, AR_CR_RXD tells the DMA engine to
             * stop writing received frames to host memory.  The radio
             * itself keeps receiving; frames land in a small hardware
             * FIFO and are available when DMA is re-enabled.
             *
             * The ath9k driver writes AR_CR_RXD during every calibration
             * reset (~every 2 seconds) and during ath_stoprecv().  The
             * disable-to-enable window is very brief in wall-clock time
             * but in QEMU's single-threaded TCG model the vCPU cannot
             * run while our I/O handlers run.  If we set rx_enabled =
             * false here, beacon frames delivered by the event loop
             * between the RXD write and the next RXE write are lost.
             * Because the event loop can deliver dozens of beacons in
             * that window, the driver sees beacon loss and disconnects.
             *
             * Instead we rely on the rxdp == 0 guard inside
             * inject_rx_frame and medium_process_msg.  During the reset
             * the driver writes AR_RXDP = 0 (which prevents injection)
             * and then writes a fresh AR_RXDP + AR_CR_RXE.
             *
             * For Phase 2 standalone beacons we still stop the timer
             * to avoid injecting into a half-rebuilt ring.
             */
            vwifi_ath9k_trace("RX DMA disable requested (ignored for medium)");
            if (!s->medium_connected) {
                vwifi_ath9k_beacon_timer_stop(s);
            }
        }
        break;

    case AR_CFG:
    case AR_RXCFG:
    case AR_TXCFG:
    case AR_MIBC:
        vwifi_ath9k_reg_write_raw(s, addr, val);
        break;

    case AR_RXDP:
        s->rxdp = val;
        s->rxdp_ring_empty = false;  /* fresh chain from driver */
        vwifi_ath9k_trace("WRITE AR_RXDP = 0x%08x", val);
        vwifi_ath9k_reg_write_raw(s, addr, val);
        break;

    /* --- Interrupts --- */
    case AR_IER:
        s->ier = val;
        vwifi_ath9k_reg_write_raw(s, addr, val);
        vwifi_ath9k_trace("WRITE AR_IER = 0x%08x (irqs %s)", val,
                    (val & AR_IER_ENABLE) ? "ENABLED" : "DISABLED");
        vwifi_ath9k_update_irq(s);
        break;
    case AR_IMR:
        vwifi_ath9k_reg_write_raw(s, addr, val);   /* raw value for readback */
        /*
         * The ath9k driver routes RXOK via AR_IMR_S5 and TXOK via
         * AR_IMR_S0 (secondary interrupt registers), so neither bit
         * appears in the primary AR_IMR.  On real hardware, the
         * composite PCI interrupt includes secondary contributions.
         *
         * We don't fully implement the secondary register chains,
         * so we augment the internal mask with RXOK and TXOK to
         * ensure update_irq fires when frames are received or
         * transmitted.  The raw register value is preserved for
         * driver readback via reg_read_raw.
         */
        s->imr = val | AR_ISR_RXOK | AR_ISR_TXOK;
        vwifi_ath9k_trace("WRITE AR_IMR = 0x%08x", val);
        vwifi_ath9k_update_irq(s);
        break;
    case AR_IMR_S0:
        s->imr_s0 = val;
        vwifi_ath9k_reg_write_raw(s, addr, val);
        break;
    case AR_IMR_S1:
        s->imr_s1 = val;
        vwifi_ath9k_reg_write_raw(s, addr, val);
        break;
    case AR_IMR_S2:
    case AR_IMR_S3:
    case AR_IMR_S4:
    case AR_IMR_S5:
        vwifi_ath9k_reg_write_raw(s, addr, val);
        break;

    case AR_ISR:
        /* Write-to-clear */
        if (val != 0) {
            vwifi_ath9k_trace("WRITE AR_ISR (clear) = 0x%08x, isr was 0x%08x -> 0x%08x",
                        val, s->isr, s->isr & ~val);
        }
        s->isr &= ~val;
        vwifi_ath9k_update_irq(s);
        break;

    case AR_INTR_SYNC_ENABLE:
    case AR_INTR_SYNC_CAUSE:
    case AR_INTR_ASYNC_ENABLE:
    case AR_INTR_ASYNC_MASK:
        vwifi_ath9k_reg_write_raw(s, addr, val);
        break;

    /* --- TX queues --- */
    case AR_Q_TXE:
        vwifi_ath9k_trace("WRITE AR_Q_TXE = 0x%08x", val);
        /* Set bits enable queues; process each newly enabled queue */
        for (q = 0; q < ATH9K_VIRT_NUM_TX_QUEUES; q++) {
            if ((val & (1 << q)) && !s->tx_queues[q].enabled) {
                s->tx_queues[q].enabled = true;
                s->regs[AR_Q_TXE / 4] |= (1 << q);
                vwifi_ath9k_process_tx_queue(s, q);
            }
        }
        break;

    case AR_Q_TXD:
        /* Disable TX queues */
        for (q = 0; q < ATH9K_VIRT_NUM_TX_QUEUES; q++) {
            if (val & (1 << q)) {
                s->tx_queues[q].enabled = false;
                s->regs[AR_Q_TXE / 4] &= ~(1 << q);
            }
        }
        break;

    /* --- RTC / Power --- */
    case AR_RTC_RC:
        vwifi_ath9k_reg_write_raw(s, addr, val);
        s->regs[AR_RTC_STATUS / 4] = AR_RTC_STATUS_ON;
        break;
    case AR_RTC_RESET:
        vwifi_ath9k_reg_write_raw(s, addr, val);
        s->regs[AR_RTC_STATUS / 4] = AR_RTC_STATUS_ON;
        break;
    case AR_RTC_FORCE_WAKE:
        vwifi_ath9k_reg_write_raw(s, addr, val);
        if (val & AR_RTC_FORCE_WAKE_EN) {
            s->power_mode = ATH9K_PM_AWAKE;
            s->regs[AR_RTC_STATUS / 4] = AR_RTC_STATUS_ON;
        }
        break;
    case AR_RTC_PLL_CONTROL:
    case AR_RTC_PLL_CONTROL2:
    case AR_HOST_TIMEOUT:
        vwifi_ath9k_reg_write_raw(s, addr, val);
        break;

    /* --- EEPROM --- */
    case AR_EEPROM:
        vwifi_ath9k_reg_write_raw(s, addr, val);
        break;

    /* --- GPIO --- */
    case AR_GPIO_OE_OUT:
    case AR_GPIO_IN_OUT:
    case AR_GPIO_INPUT_EN_VAL:
        vwifi_ath9k_reg_write_raw(s, addr, val);
        break;

    /* --- PCIe / WA --- */
    case AR_PCIE_PM_CTRL:
    case AR_WA:
        vwifi_ath9k_reg_write_raw(s, addr, val);
        break;

    /* --- Station / BSS --- */
    case AR_STA_ID0:
    case AR_STA_ID1:
    case AR_BSS_ID0:
    case AR_BSS_ID1:
        vwifi_ath9k_reg_write_raw(s, addr, val);
        break;

    /* --- RX filter / Diagnostic --- */
    case AR_RX_FILTER:
    case AR_DIAG_SW:
    case AR_PCU_MISC_MODE2:
    case AR_OBS:
        vwifi_ath9k_reg_write_raw(s, addr, val);
        break;

    /* --- Beacon / Timer --- */
    case AR_TIME_OUT:
    case AR_RSSI_THR:
    case AR_USEC:
    case AR_BEACON_PERIOD:
    case AR_DBA_PERIOD:
    case AR_NEXT_TBTT_TIMER:
    case AR_NEXT_DMA_BEACON_ALERT:
    case AR_NEXT_SWBA:
    case AR_NEXT_TIM:
    case AR_NEXT_DTIM:
        vwifi_ath9k_reg_write_raw(s, addr, val);
        break;
    case AR_TIMER_MODE:
        vwifi_ath9k_reg_write_raw(s, addr, val);
        /* Start or stop SWBA timer based on AR_SWBA_TIMER_EN bit */
        if (val & AR_SWBA_TIMER_EN) {
            vwifi_ath9k_swba_timer_start(s);
        } else {
            vwifi_ath9k_swba_timer_stop(s);
        }
        break;

    /* --- PHY --- */
    case AR_PHY_ACTIVE:
        s->phy_active = !!(val & AR_PHY_ACTIVE_EN);
        vwifi_ath9k_reg_write_raw(s, addr, val);
        break;
    case AR_PHY_MODE:
    case AR_PHY_AGC_CONTROL:
    case AR_PHY_CCA:
    case AR_PHY_RFBUS_REQ:
        vwifi_ath9k_reg_write_raw(s, addr, val);
        break;

    /*
     * Channel select register – AR_PHY(0x37) = 0x98DC.
     *
     * The ath9k driver writes the channel divider here during
     * ar5008_hw_set_channel().  For 2 GHz:
     *   channelSel = CHANSEL_2G(freq) = ((freq - 704) * 2) / 5
     *   reg = (channelSel << 2) | (bModeSynth << 1) | 1
     * We reverse-decode the frequency so we can tag outgoing medium
     * frames with the correct channel, enabling channel-aware filtering
     * in the hub.
     */
    case AR_PHY_CHANSEL:
    {
        uint16_t chansel = (val >> 2) & 0xFFF;
        uint16_t freq = 0;
        if (chansel >= 683 && chansel <= 707) {
            /* 2.4 GHz: freq = (chansel * 5 + 1) / 2 + 704 */
            freq = (uint16_t)((chansel * 5 + 1) / 2 + 704);
        } else if (chansel >= 128 && chansel <= 682) {
            /* 5 GHz: chansel = (freq - 4800) / 5, reversed bits
             * For simplicity, treat as freq = chansel * 5 + 4800.
             * This is approximate; 5 GHz support can be refined later. */
            freq = (uint16_t)(chansel * 5 + 4800);
        }
        if (freq != 0 && freq != s->current_channel_freq) {
            s->current_channel_freq = freq;
            s->current_channel_flags = (freq < 3000) ?
                VWIFI_CHAN_FLAG_2GHZ : VWIFI_CHAN_FLAG_5GHZ;
            vwifi_ath9k_trace("CHANNEL: tuned to %u MHz", freq);
        }
        vwifi_ath9k_reg_write_raw(s, addr, val);
        break;
    }

    /* --- Global IFS --- */
    case AR_D_GBL_IFS_SIFS:
    case AR_D_GBL_IFS_SLOT:
    case AR_D_GBL_IFS_EIFS:
    case AR_D_GBL_IFS_MISC:
    case AR_D_FPCTL:
        vwifi_ath9k_reg_write_raw(s, addr, val);
        break;

    default:
        /* Per-queue TX descriptor pointers */
        if (in_range(addr, AR_Q0_TXDP, AR_NUM_QCU)) {
            q = (addr - AR_Q0_TXDP) / 4;
            s->tx_queues[q].txdp = val;
            vwifi_ath9k_trace("WRITE AR_QTXDP(%d) = 0x%08x", q, val);
            vwifi_ath9k_reg_write_raw(s, addr, val);
            break;
        }
        /* Per-queue/DCU config registers */
        if (in_range(addr, AR_Q0_CBRCFG, AR_NUM_QCU) ||
            in_range(addr, AR_Q0_RDYTIMECFG, AR_NUM_QCU) ||
            in_range(addr, AR_Q0_MISC, AR_NUM_QCU) ||
            in_range(addr, AR_Q0_STS, AR_NUM_QCU) ||
            in_range(addr, AR_Q0_ONESHOTARM_SC, AR_NUM_QCU) ||
            in_range(addr, AR_Q0_ONESHOTARM_CC, AR_NUM_QCU) ||
            in_range(addr, AR_D0_QCUMASK, AR_NUM_DCU) ||
            in_range(addr, AR_D0_LCL_IFS, AR_NUM_DCU) ||
            in_range(addr, AR_D0_RETRY_LIMIT, AR_NUM_DCU) ||
            in_range(addr, AR_D0_CHNTIME, AR_NUM_DCU) ||
            in_range(addr, AR_D0_MISC, AR_NUM_DCU) ||
            in_range(addr, AR_D0_SEQNUM, AR_NUM_DCU)) {
            vwifi_ath9k_reg_write_raw(s, addr, val);
            break;
        }
        /* Hardware key cache (WEP/TKIP/CCMP offload).
         * The driver programs keys here via ath_hw_set_keycache_entry();
         * we shadow the writes in regs[] and read the key material back at
         * TX/RX to perform CCMP, TKIP or WEP.  Stored silently (no UNHANDLED). */
        if (addr >= AR_KEYTABLE_0 && addr < AR_KEYTABLE_END) {
            vwifi_ath9k_reg_write_raw(s, addr, val);
            break;
        }
        /* PHY register space */
        if (addr >= AR_PHY_BASE && addr < ATH9K_MMIO_SIZE) {
            vwifi_ath9k_reg_write_raw(s, addr, val);
            break;
        }
        /* Truly unhandled */
        vwifi_ath9k_reg_write_raw(s, addr, val);
        s->unhandled_write_count++;
        if (s->unhandled_write_count <= 200) {
            vwifi_ath9k_warn("UNHANDLED write %-24s = 0x%08x",
                       vwifi_ath9k_reg_name(addr), val);
        }
        return;
    }
}


/* -------------------------------------------------------------------
 *  Memory region operations
 * ------------------------------------------------------------------- */
static const MemoryRegionOps vwifi_ath9k_mmio_ops = {
    .read  = vwifi_ath9k_mmio_read,
    .write = vwifi_ath9k_mmio_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .impl = {
        .min_access_size = 2,
        .max_access_size = 4,
    },
    .valid = {
        .min_access_size = 2,
        .max_access_size = 4,
    },
};

/* -------------------------------------------------------------------
 *  Device lifecycle
 * ------------------------------------------------------------------- */
static void vwifi_ath9k_init_registers(VwifiAth9kState *s)
{
    int i;

    memset(s->regs, 0, sizeof(s->regs));

    s->regs[AR_RTC_STATUS / 4] = AR_RTC_STATUS_ON;
    s->regs[AR_WA / 4] = AR9285_WA_DEFAULT;
    s->regs[AR_CFG / 4] = 0;

    s->power_mode = ATH9K_PM_AWAKE;
    s->isr = 0;
    s->isr_s0 = 0;
    s->isr_s1 = 0;
    s->ier = 0;
    s->imr = 0;
    s->imr_s0 = 0;
    s->imr_s1 = 0;

    s->rxdp = 0;
    s->rx_enabled = false;
    s->rxdp_ring_empty = false;
    s->phy_active = false;

    for (i = 0; i < ATH9K_VIRT_NUM_TX_QUEUES; i++) {
        s->tx_queues[i].txdp = 0;
        s->tx_queues[i].enabled = false;
    }

    s->tsf_lo = 0;
    s->tsf_hi = 0;
    s->beacon_timer_active = false;
    s->swba_pending = false;
    s->swba_timer_active = false;

    s->read_count = 0;
    s->cached_beacon_len = 0;
    s->last_beacon_tx_ms = 0;
    s->write_count = 0;
    s->unhandled_read_count = 0;
    s->unhandled_write_count = 0;
    s->tx_frames_processed = 0;
    s->rx_frames_injected = 0;
    s->tx_frames_to_medium = 0;
    s->rx_frames_from_medium = 0;

    /* Medium state (socket connection is set up in realize) */
    s->medium_fd = -1;
    s->medium_connected = false;
    s->medium_rxbuf_used = 0;
}

static void vwifi_ath9k_realize(PCIDevice *pci_dev, Error **errp)
{
    VwifiAth9kState *s = VWIFI_ATH9K(pci_dev);
    uint8_t *pci_conf = pci_dev->config;

    pci_config_set_interrupt_pin(pci_conf, 1);
    pci_conf[PCI_CLASS_PROG] = 0x00;

    memory_region_init_io(&s->mmio, OBJECT(s), &vwifi_ath9k_mmio_ops, s,
                          "vwifi-ath9k-mmio", ATH9K_MMIO_SIZE);
    pci_register_bar(pci_dev, 0, PCI_BASE_ADDRESS_SPACE_MEMORY, &s->mmio);

    vwifi_ath9k_init_registers(s);
    ath9k_eeprom_init_4k(s->eeprom, ATH9K_EEPROM_4K_SIZE_WORDS);

    /* --- MAC address assignment ---
     * Priority: explicit macaddr= property > auto-random.
     * We patch the EEPROM words so the driver reads the right MAC
     * from its normal EEPROM path.  Word layout (little-endian):
     *   eeprom[7] = mac[0] | (mac[1] << 8)   (byte offset 14)
     *   eeprom[8] = mac[2] | (mac[3] << 8)
     *   eeprom[9] = mac[4] | (mac[5] << 8)
     */
    {
        uint8_t mac[6];
        bool need_patch = false;

        if (s->macaddr && s->macaddr[0] != '\0') {
            /* Parse user-supplied MAC: "xx:xx:xx:xx:xx:xx" */
            unsigned m[6];
            if (sscanf(s->macaddr, "%x:%x:%x:%x:%x:%x",
                       &m[0], &m[1], &m[2], &m[3], &m[4], &m[5]) == 6) {
                for (int i = 0; i < 6; i++) mac[i] = (uint8_t)m[i];
                need_patch = true;
            } else {
                qemu_log_mask(LOG_GUEST_ERROR,
                              "vwifi-ath9k: bad macaddr '%s' – "
                              "expected xx:xx:xx:xx:xx:xx, using random\n",
                              s->macaddr);
            }
        }

        if (!need_patch) {
            /* Auto-generate: Atheros OUI 00:03:7F + 3 random bytes.
             * Similar to qemu_macaddr_default_if_unset() but we use
             * the Atheros OUI so the driver's OUI checks pass. */
            mac[0] = 0x00;
            mac[1] = 0x03;
            mac[2] = 0x7F;
            uint32_t r = g_random_int();
            mac[3] = (uint8_t)(r >> 16);
            mac[4] = (uint8_t)(r >> 8);
            mac[5] = (uint8_t)(r);
            /* Ensure unicast (bit 0 of first byte clear) and
             * locally-administered bit stays clear (real Atheros OUI). */
            need_patch = true;
        }

        if (need_patch) {
            /* MAC is at byte offset EEP4K_OFF_HDR_MACADDR (140),
             * which is word offset 70 in the uint16_t array. */
            int mac_word = EEP4K_OFF_HDR_MACADDR / 2;
            s->eeprom[mac_word + 0] = (uint16_t)(mac[0] | (mac[1] << 8));
            s->eeprom[mac_word + 1] = (uint16_t)(mac[2] | (mac[3] << 8));
            s->eeprom[mac_word + 2] = (uint16_t)(mac[4] | (mac[5] << 8));

            /* Recompute EEPROM checksum.  The kernel checksums the struct
             * portion (words 64..511), so we XOR only those words,
             * skipping the checksum word itself (word 65 = byte 130). */
            {
                int csum_word = EEP4K_OFF_HDR_CHECKSUM / 2;
                uint16_t xsum = 0;
                for (int i = EEP4K_STRUCT_START / 2;
                     i < ATH9K_EEPROM_4K_SIZE_WORDS; i++) {
                    if (i == csum_word) continue;
                    xsum ^= s->eeprom[i];
                }
                s->eeprom[csum_word] = xsum ^ 0xFFFF;
            }
        }
    }

    /* Extract our MAC address from EEPROM for medium TX headers. */
    {
        int mac_word = EEP4K_OFF_HDR_MACADDR / 2;
        s->our_mac[0] = (uint8_t)(s->eeprom[mac_word + 0]);
        s->our_mac[1] = (uint8_t)(s->eeprom[mac_word + 0] >> 8);
        s->our_mac[2] = (uint8_t)(s->eeprom[mac_word + 1]);
        s->our_mac[3] = (uint8_t)(s->eeprom[mac_word + 1] >> 8);
        s->our_mac[4] = (uint8_t)(s->eeprom[mac_word + 2]);
        s->our_mac[5] = (uint8_t)(s->eeprom[mac_word + 2] >> 8);
    }

    /* Create beacon timer (Phase 2 standalone self-injection) */
    s->beacon_timer = timer_new_ms(QEMU_CLOCK_VIRTUAL,
                                   vwifi_ath9k_beacon_timer_cb, s);

    /* Create SWBA timer (Phase 3 real beacon TX via driver tasklet) */
    s->swba_timer = timer_new_ms(QEMU_CLOCK_VIRTUAL,
                                 vwifi_ath9k_swba_timer_cb, s);

    /* Create medium reconnect timer (QEMU_CLOCK_REALTIME so it fires
     * even when the guest is idle / vCPU is halted) */
    s->medium_reconnect_timer = timer_new_ms(QEMU_CLOCK_REALTIME,
                                              vwifi_ath9k_reconnect_cb, s);

    /* Initial connection attempt */
    if (s->medium_path && s->medium_path[0] != '\0') {
        if (!vwifi_ath9k_try_connect(s)) {
            qemu_log_mask(LOG_GUEST_ERROR,
                          "vwifi-ath9k: medium %s not available, "
                          "will retry every %dms\n",
                          s->medium_path, MEDIUM_RECONNECT_MS);
            vwifi_ath9k_schedule_reconnect(s);
        }
    } else {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "vwifi-ath9k: no medium path – "
                      "standalone beacon-only mode\n");
    }

    qemu_log_mask(LOG_GUEST_ERROR,
                  "vwifi-ath9k: device realized – "
                  "Vendor 0x%04x Device 0x%04x "
                  "(AR9285 virtual, phase-3)\n",
                  ATHEROS_VENDOR_ID, AR9285_DEVID_PCIE);
}

static void vwifi_ath9k_exit(PCIDevice *pci_dev)
{
    VwifiAth9kState *s = VWIFI_ATH9K(pci_dev);

    vwifi_ath9k_beacon_timer_stop(s);
    timer_free(s->beacon_timer);
    vwifi_ath9k_swba_timer_stop(s);
    timer_free(s->swba_timer);
    timer_del(s->medium_reconnect_timer);
    timer_free(s->medium_reconnect_timer);

    if (s->medium_fd >= 0) {
        qemu_set_fd_handler(s->medium_fd, NULL, NULL, NULL);
        close(s->medium_fd);
        s->medium_fd = -1;
        s->medium_connected = false;
    }

    g_free(s->medium_path);
    s->medium_path = NULL;
    g_free(s->macaddr);
    s->macaddr = NULL;
    g_free(s->node_id);
    s->node_id = NULL;

    qemu_log_mask(LOG_GUEST_ERROR,
                  "vwifi-ath9k: device destroyed – "
                  "reads: %" PRIu64 " writes: %" PRIu64 " "
                  "TX: %" PRIu64 " RX: %" PRIu64 " "
                  "medium-TX: %" PRIu64 " medium-RX: %" PRIu64 "\n",
                  s->read_count, s->write_count,
                  s->tx_frames_processed, s->rx_frames_injected,
                  s->tx_frames_to_medium, s->rx_frames_from_medium);
}

/* -------------------------------------------------------------------
 *  Migration (VMState)
 * ------------------------------------------------------------------- */
static const VMStateDescription vmstate_vwifi_ath9k = {
    .name = TYPE_VWIFI_ATH9K,
    .version_id = 3,
    .minimum_version_id = 3,
    .fields = (const VMStateField[]) {
        VMSTATE_PCI_DEVICE(parent_obj, VwifiAth9kState),
        VMSTATE_UINT32_ARRAY(regs, VwifiAth9kState, ATH9K_REG_COUNT),
        VMSTATE_UINT16_ARRAY(eeprom, VwifiAth9kState,
                             ATH9K_EEPROM_4K_SIZE_WORDS),
        VMSTATE_UINT32(isr, VwifiAth9kState),
        VMSTATE_UINT32(isr_s0, VwifiAth9kState),
        VMSTATE_UINT32(isr_s1, VwifiAth9kState),
        VMSTATE_UINT32(ier, VwifiAth9kState),
        VMSTATE_UINT32(imr, VwifiAth9kState),
        VMSTATE_UINT32(imr_s0, VwifiAth9kState),
        VMSTATE_UINT32(imr_s1, VwifiAth9kState),
        VMSTATE_UINT32(power_mode, VwifiAth9kState),
        VMSTATE_UINT32(rxdp, VwifiAth9kState),
        VMSTATE_BOOL(rx_enabled, VwifiAth9kState),
        VMSTATE_BOOL(rxdp_ring_empty, VwifiAth9kState),
        VMSTATE_BOOL(phy_active, VwifiAth9kState),
        VMSTATE_UINT32(tsf_lo, VwifiAth9kState),
        VMSTATE_UINT32(tsf_hi, VwifiAth9kState),
        VMSTATE_UINT16(current_channel_freq, VwifiAth9kState),
        VMSTATE_UINT16(current_channel_flags, VwifiAth9kState),
        VMSTATE_END_OF_LIST()
    },
};

/* -------------------------------------------------------------------
 *  QOM string property for "medium" – uses object_property_add_str()
 *  instead of DEFINE_PROP_STRING (which requires hw/qdev-properties.h)
 * ------------------------------------------------------------------- */
static char *vwifi_ath9k_get_medium(Object *obj, Error **errp)
{
    VwifiAth9kState *s = VWIFI_ATH9K(obj);
    return g_strdup(s->medium_path ? s->medium_path : "");
}

static void vwifi_ath9k_set_medium(Object *obj, const char *value, Error **errp)
{
    VwifiAth9kState *s = VWIFI_ATH9K(obj);
    g_free(s->medium_path);
    s->medium_path = g_strdup(value);
}

static char *vwifi_ath9k_get_macaddr(Object *obj, Error **errp)
{
    VwifiAth9kState *s = VWIFI_ATH9K(obj);
    return g_strdup(s->macaddr ? s->macaddr : "");
}

static void vwifi_ath9k_set_macaddr(Object *obj, const char *value, Error **errp)
{
    VwifiAth9kState *s = VWIFI_ATH9K(obj);
    g_free(s->macaddr);
    s->macaddr = g_strdup(value);
}

static char *vwifi_ath9k_get_node_id(Object *obj, Error **errp)
{
    VwifiAth9kState *s = VWIFI_ATH9K(obj);
    return g_strdup(s->node_id ? s->node_id : "");
}

static void vwifi_ath9k_set_node_id(Object *obj, const char *value, Error **errp)
{
    VwifiAth9kState *s = VWIFI_ATH9K(obj);
    g_free(s->node_id);
    s->node_id = g_strdup(value);
}

static void vwifi_ath9k_instance_init(Object *obj)
{
    object_property_add_str(obj, "medium",
                            vwifi_ath9k_get_medium,
                            vwifi_ath9k_set_medium);
    object_property_add_str(obj, "macaddr",
                            vwifi_ath9k_get_macaddr,
                            vwifi_ath9k_set_macaddr);
    object_property_add_str(obj, "node_id",
                            vwifi_ath9k_get_node_id,
                            vwifi_ath9k_set_node_id);
}

static void vwifi_ath9k_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    PCIDeviceClass *k = PCI_DEVICE_CLASS(klass);

    k->realize   = vwifi_ath9k_realize;
    k->exit      = vwifi_ath9k_exit;
    k->vendor_id = ATHEROS_VENDOR_ID;
    k->device_id = AR9285_DEVID_PCIE;
    k->revision  = 0x01;
    k->class_id  = PCI_CLASS_NETWORK_OTHER;

    k->subsystem_vendor_id = ATHEROS_VENDOR_ID;
    k->subsystem_id        = 0x3099;

    dc->desc  = "Virtual Atheros AR9285 802.11n (vwifi-ath9k)";
    dc->vmsd  = &vmstate_vwifi_ath9k;

    set_bit(DEVICE_CATEGORY_NETWORK, dc->categories);
}

static const TypeInfo vwifi_ath9k_info = {
    .name          = TYPE_VWIFI_ATH9K,
    .parent        = TYPE_PCI_DEVICE,
    .instance_size = sizeof(VwifiAth9kState),
    .instance_init = vwifi_ath9k_instance_init,
    .class_init    = vwifi_ath9k_class_init,
    .interfaces    = (InterfaceInfo[]) {
        { INTERFACE_PCIE_DEVICE },
        { },
    },
};

static void vwifi_ath9k_register_types(void)
{
    type_register_static(&vwifi_ath9k_info);
}

type_init(vwifi_ath9k_register_types)
