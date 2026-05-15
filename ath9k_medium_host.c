/*
 * ath9k_medium_host — Host-side virtual wireless interface
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * This kernel module registers a mac80211 virtual radio on the host,
 * creating a standard wlanX interface.  Frames transmitted by the host's
 * wifi stack are forwarded to userspace (via a character device), where
 * the ath9k_host_relay daemon sends them to the ath9k_medium_hub.
 * Frames received from the hub are injected back into mac80211.
 *
 * Architecture:
 *
 *   hostapd / wpa_supplicant / iw
 *         |
 *     mac80211  (wlanX)
 *         |
 *   ath9k_medium_host.ko  ← this module
 *         |  /dev/ath9k_medium
 *   ath9k_host_relay       (userspace daemon)
 *         |  unix socket
 *   ath9k_medium_hub
 *         |
 *   QEMU VMs (ath9k-virt)
 *
 * The char device uses the same length-prefixed wire protocol as the
 * hub, so the relay daemon is just a bidirectional byte pump.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/wait.h>
#include <linux/poll.h>
#include <linux/miscdevice.h>
#include <linux/uaccess.h>
#include <linux/workqueue.h>
#include <linux/skbuff.h>
#include <linux/etherdevice.h>
#include <linux/platform_device.h>
#include <net/mac80211.h>

#include "ath9k_medium.h"

#define DRV_NAME        "ath9k_medium_host"
#define DRV_VERSION     "1.0"

/* -------------------------------------------------------------------
 *  Module parameters
 * ------------------------------------------------------------------- */

/* MAC address for the virtual radio.  Default uses Atheros OUI. */
static char *macaddr = "00:03:7F:CC:DD:01";
module_param(macaddr, charp, 0444);
MODULE_PARM_DESC(macaddr, "MAC address for the virtual radio (default: 00:03:7F:CC:DD:01)");

/* -------------------------------------------------------------------
 *  Per-radio state
 * ------------------------------------------------------------------- */
struct ath9k_host_priv {
    struct ieee80211_hw     *hw;

    /* Current operating state */
    bool                    started;
    int                     channel_freq;       /* MHz */
    enum nl80211_band       channel_band;
    u8                      mac_addr[ETH_ALEN];

    /*
     * state_lock protects:
     *   - tx_queue (instead of relying on the sk_buff_head's own
     *     internal lock, so the relay_connected check + enqueue can
     *     happen atomically inside a single critical section)
     *   - relay_connected
     *   - tx_queues_stopped
     *
     * .tx runs in mac80211 softirq context, while chrdev_open / read /
     * release run in process context. Without this lock the open
     * EBUSY check is racy (two relays could both attach), and a
     * release that flips relay_connected=false then purges the queue
     * can be interleaved with a .tx that already passed the
     * relay_connected check -- the late-enqueued skb leaks until the
     * module is unloaded.
     */
    spinlock_t              state_lock;

    /* TX queue: frames going from mac80211 → chardev → relay → hub */
    struct sk_buff_head     tx_queue;
    wait_queue_head_t       tx_waitq;           /* relay polls for TX */
    bool                    tx_queues_stopped;  /* true between stop_queues/wake_queues */

    /* RX queue: frames coming from hub → relay → chardev → mac80211 */
    struct work_struct      rx_work;

    /* Chardev state (guarded by state_lock) */
    bool                    relay_connected;

    /* Statistics */
    u64                     tx_count;
    u64                     rx_count;
    u64                     tx_drop_count;
};

static struct ath9k_host_priv *g_priv;  /* single-radio for now */
static struct platform_device *g_pdev; /* fake parent device for wiphy */

/* -------------------------------------------------------------------
 *  2.4 GHz band definition (matches AR9285 capabilities)
 * ------------------------------------------------------------------- */

/* Supported rates for 2.4 GHz (802.11b/g/n) */
static struct ieee80211_rate ath9k_host_rates_2ghz[] = {
    { .bitrate = 10,  .hw_value = 0x1B, .flags = IEEE80211_RATE_SHORT_PREAMBLE },
    { .bitrate = 20,  .hw_value = 0x1A, .flags = IEEE80211_RATE_SHORT_PREAMBLE },
    { .bitrate = 55,  .hw_value = 0x19, .flags = IEEE80211_RATE_SHORT_PREAMBLE },
    { .bitrate = 110, .hw_value = 0x18, .flags = IEEE80211_RATE_SHORT_PREAMBLE },
    { .bitrate = 60,  .hw_value = 0x0B },
    { .bitrate = 90,  .hw_value = 0x0F },
    { .bitrate = 120, .hw_value = 0x0A },
    { .bitrate = 180, .hw_value = 0x0E },
    { .bitrate = 240, .hw_value = 0x09 },
    { .bitrate = 360, .hw_value = 0x0D },
    { .bitrate = 480, .hw_value = 0x08 },
    { .bitrate = 540, .hw_value = 0x0C },
};

/* 2.4 GHz channels (1-14) */
static struct ieee80211_channel ath9k_host_channels_2ghz[] = {
    { .band = NL80211_BAND_2GHZ, .center_freq = 2412, .hw_value = 1 },
    { .band = NL80211_BAND_2GHZ, .center_freq = 2417, .hw_value = 2 },
    { .band = NL80211_BAND_2GHZ, .center_freq = 2422, .hw_value = 3 },
    { .band = NL80211_BAND_2GHZ, .center_freq = 2427, .hw_value = 4 },
    { .band = NL80211_BAND_2GHZ, .center_freq = 2432, .hw_value = 5 },
    { .band = NL80211_BAND_2GHZ, .center_freq = 2437, .hw_value = 6 },
    { .band = NL80211_BAND_2GHZ, .center_freq = 2442, .hw_value = 7 },
    { .band = NL80211_BAND_2GHZ, .center_freq = 2447, .hw_value = 8 },
    { .band = NL80211_BAND_2GHZ, .center_freq = 2452, .hw_value = 9 },
    { .band = NL80211_BAND_2GHZ, .center_freq = 2457, .hw_value = 10 },
    { .band = NL80211_BAND_2GHZ, .center_freq = 2462, .hw_value = 11 },
    { .band = NL80211_BAND_2GHZ, .center_freq = 2467, .hw_value = 12 },
    { .band = NL80211_BAND_2GHZ, .center_freq = 2472, .hw_value = 13 },
    { .band = NL80211_BAND_2GHZ, .center_freq = 2484, .hw_value = 14 },
};

static struct ieee80211_supported_band ath9k_host_band_2ghz = {
    .band       = NL80211_BAND_2GHZ,
    .channels   = ath9k_host_channels_2ghz,
    .n_channels = ARRAY_SIZE(ath9k_host_channels_2ghz),
    .bitrates   = ath9k_host_rates_2ghz,
    .n_bitrates = ARRAY_SIZE(ath9k_host_rates_2ghz),
    .ht_cap     = {
        .ht_supported   = true,
        .cap            = IEEE80211_HT_CAP_SUP_WIDTH_20_40 |
                          IEEE80211_HT_CAP_SGI_20 |
                          IEEE80211_HT_CAP_SGI_40 |
                          IEEE80211_HT_CAP_DSSSCCK40 |
                          IEEE80211_HT_CAP_RX_STBC |
                          IEEE80211_HT_CAP_TX_STBC,
        .ampdu_factor   = IEEE80211_HT_MAX_AMPDU_64K,
        .ampdu_density  = IEEE80211_HT_MPDU_DENSITY_8,
        .mcs = {
            /* NSS=1 (MCS 0..7) AND NSS=2 (MCS 8..15) supported. */
            .rx_mask    = { 0xFF, 0xFF, 0, 0, 0, 0, 0, 0, 0, 0 },
            .rx_highest = cpu_to_le16(144),     /* MCS15 HT20 SGI */
            .tx_params  = IEEE80211_HT_MCS_TX_DEFINED,
        },
    },
};

/* -------------------------------------------------------------------
 *  5 GHz band definition (for VHT / HE support)
 *
 *  We declare a representative slice of the U-NII bands that are
 *  channel-bondable for VHT80/160: U-NII-1, U-NII-2A, and U-NII-3.
 *  This is enough for mac80211 to allow association on VHT/HE-capable
 *  channels without claiming DFS-restricted channels we don't model.
 * ------------------------------------------------------------------- */

/* OFDM rates for 5 GHz (no CCK). */
static struct ieee80211_rate ath9k_host_rates_5ghz[] = {
    { .bitrate = 60,  .hw_value = 0x0B },
    { .bitrate = 90,  .hw_value = 0x0F },
    { .bitrate = 120, .hw_value = 0x0A },
    { .bitrate = 180, .hw_value = 0x0E },
    { .bitrate = 240, .hw_value = 0x09 },
    { .bitrate = 360, .hw_value = 0x0D },
    { .bitrate = 480, .hw_value = 0x08 },
    { .bitrate = 540, .hw_value = 0x0C },
};

static struct ieee80211_channel ath9k_host_channels_5ghz[] = {
    /* U-NII-1 (5150-5250 MHz) */
    { .band = NL80211_BAND_5GHZ, .center_freq = 5180, .hw_value = 36 },
    { .band = NL80211_BAND_5GHZ, .center_freq = 5200, .hw_value = 40 },
    { .band = NL80211_BAND_5GHZ, .center_freq = 5220, .hw_value = 44 },
    { .band = NL80211_BAND_5GHZ, .center_freq = 5240, .hw_value = 48 },
    /* U-NII-2A (5250-5350 MHz) */
    { .band = NL80211_BAND_5GHZ, .center_freq = 5260, .hw_value = 52 },
    { .band = NL80211_BAND_5GHZ, .center_freq = 5280, .hw_value = 56 },
    { .band = NL80211_BAND_5GHZ, .center_freq = 5300, .hw_value = 60 },
    { .band = NL80211_BAND_5GHZ, .center_freq = 5320, .hw_value = 64 },
    /* U-NII-3 (5725-5825 MHz) */
    { .band = NL80211_BAND_5GHZ, .center_freq = 5745, .hw_value = 149 },
    { .band = NL80211_BAND_5GHZ, .center_freq = 5765, .hw_value = 153 },
    { .band = NL80211_BAND_5GHZ, .center_freq = 5785, .hw_value = 157 },
    { .band = NL80211_BAND_5GHZ, .center_freq = 5805, .hw_value = 161 },
    { .band = NL80211_BAND_5GHZ, .center_freq = 5825, .hw_value = 165 },
};

static struct ieee80211_supported_band ath9k_host_band_5ghz = {
    .band       = NL80211_BAND_5GHZ,
    .channels   = ath9k_host_channels_5ghz,
    .n_channels = ARRAY_SIZE(ath9k_host_channels_5ghz),
    .bitrates   = ath9k_host_rates_5ghz,
    .n_bitrates = ARRAY_SIZE(ath9k_host_rates_5ghz),
    .ht_cap     = {
        .ht_supported   = true,
        .cap            = IEEE80211_HT_CAP_SUP_WIDTH_20_40 |
                          IEEE80211_HT_CAP_SGI_20 |
                          IEEE80211_HT_CAP_SGI_40 |
                          IEEE80211_HT_CAP_RX_STBC |
                          IEEE80211_HT_CAP_TX_STBC,
        .ampdu_factor   = IEEE80211_HT_MAX_AMPDU_64K,
        .ampdu_density  = IEEE80211_HT_MPDU_DENSITY_8,
        .mcs = {
            .rx_mask    = { 0xFF, 0xFF, 0, 0, 0, 0, 0, 0, 0, 0 },
            .rx_highest = cpu_to_le16(144),
            .tx_params  = IEEE80211_HT_MCS_TX_DEFINED,
        },
    },
    /* .vht_cap and HE iftype_data populated in subsequent commits. */
};

/* -------------------------------------------------------------------
 *  ieee80211_ops — mac80211 callbacks
 * ------------------------------------------------------------------- */

/*
 * .tx — Called by mac80211 to transmit a frame.
 *
 * We wrap the 802.11 frame in an ath9k_medium_frame_hdr, prepend the
 * 4-byte length prefix, and enqueue it for the relay daemon to read
 * from the chardev.
 *
 * Backpressure model:
 *   - At HIGH watermark we ask mac80211 to stop the TX queues, so the
 *     stack stops giving us frames before we have to drop them.
 *   - The relay-side dequeue (chardev_read) wakes the queues again
 *     once the depth drops below LOW.
 *   - If, despite backpressure, the queue is full when a frame arrives
 *     (e.g. the relay has died and the queues haven't been stopped yet,
 *     or a single producer races past the watermark), we drop the
 *     frame and report a TX failure to mac80211 (no STAT_ACK flag) so
 *     its retry/rate-control logic responds correctly. We do NOT fake
 *     an ACK on the drop path -- that would lie to the upper layers.
 */
#define ATH9K_TX_QUEUE_MAX  1024
#define ATH9K_TX_QUEUE_HIGH 768
#define ATH9K_TX_QUEUE_LOW  256

static void ath9k_host_tx(struct ieee80211_hw *hw,
                          struct ieee80211_tx_control *control,
                          struct sk_buff *skb)
{
    struct ath9k_host_priv *priv = hw->priv;
    struct ieee80211_tx_info *tx_info = IEEE80211_SKB_CB(skb);
    struct ieee80211_hdr *hdr = (struct ieee80211_hdr *)skb->data;
    struct sk_buff *msg;
    struct ath9k_medium_frame_hdr fhdr;
    unsigned long flags;
    bool stop_queues_now = false;
    bool dropped = false;
    u32 wire_len, qlen;
    __be32 len_be;
    u8 rate_code = 0x0B;  /* default: 6 Mbps OFDM */

    /* Quick fast-path check before we do work; the authoritative
     * relay_connected check happens again under the lock below. */
    if (!priv->started || !READ_ONCE(priv->relay_connected)) {
        ieee80211_free_txskb(hw, skb);
        priv->tx_drop_count++;
        return;
    }

    /* Extract rate code from tx_info if available */
    if (tx_info->control.rates[0].idx >= 0 &&
        tx_info->control.rates[0].idx < ARRAY_SIZE(ath9k_host_rates_2ghz)) {
        rate_code = ath9k_host_rates_2ghz[tx_info->control.rates[0].idx].hw_value;
    }

    /* Build the medium frame header */
    memset(&fhdr, 0, sizeof(fhdr));
    fhdr.magic      = cpu_to_le32(ATH9K_MEDIUM_MAGIC);
    fhdr.version    = cpu_to_le16(ATH9K_MEDIUM_VERSION);
    fhdr.frame_len  = cpu_to_le16(skb->len);
    memcpy(fhdr.tx_mac, hdr->addr2, ETH_ALEN);
    fhdr.rate_code  = rate_code;
    fhdr.rssi       = (int8_t)-30;      /* nominal RSSI */
    fhdr.tsf_lo     = cpu_to_le32(0);   /* hub doesn't use this for TX */
    fhdr.tsf_hi     = cpu_to_le32(0);
    fhdr.flags      = cpu_to_le32(0);   /* TTL=0, hub will stamp it */

    /* Build wire message: [len_be][fhdr][802.11 frame] */
    wire_len = ATH9K_MEDIUM_HDR_SIZE + skb->len;
    msg = alloc_skb(4 + wire_len, GFP_ATOMIC);
    if (!msg) {
        ieee80211_free_txskb(hw, skb);
        priv->tx_drop_count++;
        return;
    }

    len_be = htonl(wire_len);
    skb_put_data(msg, &len_be, 4);
    skb_put_data(msg, &fhdr, ATH9K_MEDIUM_HDR_SIZE);
    skb_put_data(msg, skb->data, skb->len);

    /*
     * Critical section: re-check relay_connected, evaluate the
     * watermark, and enqueue under the lock so chrdev_release can't
     * tear down the queue between our check and our enqueue.
     */
    spin_lock_irqsave(&priv->state_lock, flags);

    if (!priv->relay_connected) {
        dropped = true;
    } else {
        qlen = skb_queue_len(&priv->tx_queue);

        if (qlen >= ATH9K_TX_QUEUE_HIGH && !priv->tx_queues_stopped) {
            priv->tx_queues_stopped = true;
            stop_queues_now = true;
        }

        if (qlen < ATH9K_TX_QUEUE_MAX) {
            __skb_queue_tail(&priv->tx_queue, msg);
        } else {
            dropped = true;
        }
    }

    spin_unlock_irqrestore(&priv->state_lock, flags);

    /* mac80211 hooks called outside the lock */
    if (stop_queues_now)
        ieee80211_stop_queues(hw);

    if (dropped) {
        kfree_skb(msg);
        priv->tx_drop_count++;
        ieee80211_tx_info_clear_status(tx_info);
        /* No IEEE80211_TX_STAT_ACK -> mac80211 sees no-ACK -> retries
         * (or for the !relay_connected case, the frame is lost cleanly). */
        ieee80211_tx_status_irqsafe(hw, skb);
        return;
    }

    wake_up_interruptible(&priv->tx_waitq);

    /* Frame is queued for transmission. Report success with the
     * synthetic ACK; the medium has no real over-the-air ACK. */
    ieee80211_tx_info_clear_status(tx_info);
    tx_info->flags |= IEEE80211_TX_STAT_ACK;
    ieee80211_tx_status_irqsafe(hw, skb);
    priv->tx_count++;
}

static int ath9k_host_start(struct ieee80211_hw *hw)
{
    struct ath9k_host_priv *priv = hw->priv;

    priv->started = true;
    pr_info(DRV_NAME ": radio started\n");
    return 0;
}

static void ath9k_host_stop(struct ieee80211_hw *hw, bool suspend)
{
    struct ath9k_host_priv *priv = hw->priv;
    unsigned long flags;
    bool was_stopped;

    priv->started = false;

    spin_lock_irqsave(&priv->state_lock, flags);
    was_stopped = priv->tx_queues_stopped;
    priv->tx_queues_stopped = false;
    spin_unlock_irqrestore(&priv->state_lock, flags);

    /* Leave the TX queues in the woken state so a subsequent start
     * doesn't inherit a stale stop. */
    if (was_stopped)
        ieee80211_wake_queues(hw);

    pr_info(DRV_NAME ": radio stopped (TX: %llu RX: %llu drops: %llu)\n",
            priv->tx_count, priv->rx_count, priv->tx_drop_count);
}

static int ath9k_host_add_interface(struct ieee80211_hw *hw,
                                    struct ieee80211_vif *vif)
{
    pr_info(DRV_NAME ": add_interface type=%d\n", vif->type);
    return 0;
}

static void ath9k_host_remove_interface(struct ieee80211_hw *hw,
                                        struct ieee80211_vif *vif)
{
    pr_info(DRV_NAME ": remove_interface\n");
}

static int ath9k_host_config(struct ieee80211_hw *hw, u32 changed)
{
    struct ath9k_host_priv *priv = hw->priv;
    struct ieee80211_conf *conf = &hw->conf;

    if (changed & IEEE80211_CONF_CHANGE_CHANNEL) {
        priv->channel_freq = conf->chandef.chan->center_freq;
        priv->channel_band = conf->chandef.chan->band;
        pr_info(DRV_NAME ": config channel %d MHz\n", priv->channel_freq);
    }

    return 0;
}

static void ath9k_host_configure_filter(struct ieee80211_hw *hw,
                                        unsigned int changed_flags,
                                        unsigned int *total_flags,
                                        u64 multicast)
{
    /*
     * Accept everything — we're a virtual radio on a virtual medium.
     * The mac80211 layer does its own filtering above us.
     */
    *total_flags &= (FIF_ALLMULTI | FIF_BCN_PRBRESP_PROMISC |
                     FIF_CONTROL | FIF_OTHER_BSS | FIF_PSPOLL |
                     FIF_PROBE_REQ);
}

static void ath9k_host_bss_info_changed(struct ieee80211_hw *hw,
                                        struct ieee80211_vif *vif,
                                        struct ieee80211_bss_conf *info,
                                        u64 changed)
{
    /* Accept BSS info changes silently */
}

static int ath9k_host_conf_tx(struct ieee80211_hw *hw,
                              struct ieee80211_vif *vif,
                              unsigned int link_id,
                              u16 queue,
                              const struct ieee80211_tx_queue_params *params)
{
    return 0;
}

static void ath9k_host_sw_scan_start(struct ieee80211_hw *hw,
                                     struct ieee80211_vif *vif,
                                     const u8 *mac_addr)
{
    pr_info(DRV_NAME ": sw scan start\n");
}

static void ath9k_host_sw_scan_complete(struct ieee80211_hw *hw,
                                        struct ieee80211_vif *vif)
{
    pr_info(DRV_NAME ": sw scan complete\n");
}

static int ath9k_host_set_rts_threshold(struct ieee80211_hw *hw, u32 value)
{
    return 0;
}

static const struct ieee80211_ops ath9k_host_ops = {
    .tx                 = ath9k_host_tx,
    .wake_tx_queue      = ieee80211_handle_wake_tx_queue,
    .start              = ath9k_host_start,
    .stop               = ath9k_host_stop,
    .add_interface      = ath9k_host_add_interface,
    .remove_interface   = ath9k_host_remove_interface,
    .config             = ath9k_host_config,
    .configure_filter   = ath9k_host_configure_filter,
    .bss_info_changed   = ath9k_host_bss_info_changed,
    .conf_tx            = ath9k_host_conf_tx,
    .sw_scan_start      = ath9k_host_sw_scan_start,
    .sw_scan_complete   = ath9k_host_sw_scan_complete,
    .set_rts_threshold  = ath9k_host_set_rts_threshold,
    /* Channel context emulation — required in 6.12+ */
    .add_chanctx        = ieee80211_emulate_add_chanctx,
    .remove_chanctx     = ieee80211_emulate_remove_chanctx,
    .change_chanctx     = ieee80211_emulate_change_chanctx,
    .switch_vif_chanctx = ieee80211_emulate_switch_vif_chanctx,
};

/* -------------------------------------------------------------------
 *  RX path — inject frames from relay into mac80211
 * ------------------------------------------------------------------- */

/*
 * Called from the chardev write path (process context).
 * Parses the wire-format message and injects the 802.11 frame
 * into mac80211 via ieee80211_rx_irqsafe().
 */
static void ath9k_host_rx_frame(struct ath9k_host_priv *priv,
                                const u8 *data, size_t len)
{
    struct ath9k_medium_frame_hdr fhdr;
    struct ieee80211_rx_status *rx_status;
    struct sk_buff *skb;
    u16 frame_len;
    size_t hdr_len;

    /* Accept either v1 (28-byte) or v2 (40-byte) headers.  v1 peers
     * (older qemu-vwifi components, or anything that hasn't been
     * upgraded yet) still send 28-byte headers per the documented
     * backward-compat contract; rejecting them here would silently
     * black-hole every legacy peer. */
    if (len < ATH9K_MEDIUM_HDR_SIZE_V1) {
        pr_warn(DRV_NAME ": rx: short message (%zu bytes)\n", len);
        return;
    }

    hdr_len = (len >= ATH9K_MEDIUM_HDR_SIZE)
              ? ATH9K_MEDIUM_HDR_SIZE
              : ATH9K_MEDIUM_HDR_SIZE_V1;

    /* Zero the local fhdr first so v2-only fields (channel_freq etc.)
     * read as 0 ("unknown") when copying from a v1 sender. */
    memset(&fhdr, 0, sizeof(fhdr));
    memcpy(&fhdr, data, hdr_len);

    if (le32_to_cpu(fhdr.magic) != ATH9K_MEDIUM_MAGIC) {
        pr_warn(DRV_NAME ": rx: bad magic 0x%08x\n", le32_to_cpu(fhdr.magic));
        return;
    }

    frame_len = le16_to_cpu(fhdr.frame_len);
    if (hdr_len + frame_len > len) {
        pr_warn(DRV_NAME ": rx: truncated frame (need %zu, have %zu)\n",
                hdr_len + frame_len, len);
        return;
    }

    if (frame_len == 0 || frame_len > ATH9K_MEDIUM_MAX_FRAME) {
        return;
    }

    /* Don't inject our own frames back */
    if (ether_addr_equal(fhdr.tx_mac, priv->mac_addr)) {
        return;
    }

    /* Allocate skb for mac80211 */
    skb = dev_alloc_skb(frame_len);
    if (!skb) {
        return;
    }

    skb_put_data(skb, data + hdr_len, frame_len);

    /* Fill in rx_status for mac80211 */
    rx_status = IEEE80211_SKB_RXCB(skb);
    memset(rx_status, 0, sizeof(*rx_status));

    rx_status->freq     = priv->channel_freq ? priv->channel_freq : 2412;
    rx_status->band     = NL80211_BAND_2GHZ;
    rx_status->signal   = (s8)fhdr.rssi;
    rx_status->antenna  = 0;

    /*
     * Map the rate_code back to a rate index.
     * mac80211 needs either a rate_idx or MCS index.
     * For legacy OFDM/CCK rates, we search our rate table.
     */
    rx_status->rate_idx = 4;  /* default: 6 Mbps (index 4 in our table) */
    {
        int i;
        for (i = 0; i < ARRAY_SIZE(ath9k_host_rates_2ghz); i++) {
            if (ath9k_host_rates_2ghz[i].hw_value == fhdr.rate_code) {
                rx_status->rate_idx = i;
                break;
            }
        }
    }

    /* Inject into mac80211 */
    ieee80211_rx_irqsafe(priv->hw, skb);
    priv->rx_count++;
}

/* -------------------------------------------------------------------
 *  Character device — interface to the relay daemon
 *
 *  read()  — relay reads TX frames destined for the hub
 *  write() — relay pushes RX frames received from the hub
 *  poll()  — relay waits for TX frames or chardev readiness
 * ------------------------------------------------------------------- */

static int ath9k_host_chrdev_open(struct inode *inode, struct file *filp)
{
    struct ath9k_host_priv *priv = g_priv;
    unsigned long flags;
    bool taken;

    if (!priv) {
        return -ENODEV;
    }

    /* Atomically claim the single-relay slot. Without the lock two
     * concurrent open()s could both observe relay_connected=false and
     * both succeed, leaving two relays sharing the same tx_queue. */
    spin_lock_irqsave(&priv->state_lock, flags);
    taken = priv->relay_connected;
    if (!taken)
        priv->relay_connected = true;
    spin_unlock_irqrestore(&priv->state_lock, flags);

    if (taken)
        return -EBUSY;  /* only one relay at a time */

    filp->private_data = priv;
    pr_info(DRV_NAME ": relay daemon connected\n");
    return 0;
}

static int ath9k_host_chrdev_release(struct inode *inode, struct file *filp)
{
    struct ath9k_host_priv *priv = filp->private_data;
    struct sk_buff_head purge;
    unsigned long flags;
    bool was_stopped;

    if (!priv)
        return 0;

    __skb_queue_head_init(&purge);

    /*
     * Flip relay_connected and detach the queue under the lock so any
     * .tx running concurrently observes !relay_connected and either
     * (a) drops the frame in the early fast-path check, or (b) takes
     * the lock after us and sees relay_connected=false in the
     * authoritative re-check. Either way no new skbs can land on the
     * queue we're about to free.
     */
    spin_lock_irqsave(&priv->state_lock, flags);
    priv->relay_connected = false;
    was_stopped = priv->tx_queues_stopped;
    priv->tx_queues_stopped = false;
    skb_queue_splice_init(&priv->tx_queue, &purge);
    spin_unlock_irqrestore(&priv->state_lock, flags);

    /* Wake mac80211 so a future relay reattach starts from a clean
     * state (otherwise the queues stay stopped across reconnect). */
    if (was_stopped)
        ieee80211_wake_queues(priv->hw);

    /* Free orphaned skbs outside the lock. */
    __skb_queue_purge(&purge);

    pr_info(DRV_NAME ": relay daemon disconnected\n");
    return 0;
}

/*
 * read() — Relay daemon reads TX frames to forward to the hub.
 *
 * Returns one complete wire message per read() call:
 *   [uint32_t length (network order)] [frame_hdr] [802.11 frame]
 *
 * Blocks until a frame is available (unless O_NONBLOCK).
 */
static ssize_t ath9k_host_chrdev_read(struct file *filp, char __user *buf,
                                      size_t count, loff_t *ppos)
{
    struct ath9k_host_priv *priv = filp->private_data;
    struct sk_buff *skb;
    unsigned long flags;
    bool wake_now = false;
    ssize_t len;
    int ret;

    /* Wait for a TX frame */
    if (skb_queue_empty(&priv->tx_queue)) {
        if (filp->f_flags & O_NONBLOCK) {
            return -EAGAIN;
        }
        ret = wait_event_interruptible(priv->tx_waitq,
                                       !skb_queue_empty(&priv->tx_queue));
        if (ret) {
            return ret;
        }
    }

    /* Dequeue and evaluate the low-watermark wake under the lock so we
     * see a consistent qlen and tx_queues_stopped pair. */
    spin_lock_irqsave(&priv->state_lock, flags);
    skb = __skb_dequeue(&priv->tx_queue);
    if (skb && priv->tx_queues_stopped &&
        skb_queue_len(&priv->tx_queue) <= ATH9K_TX_QUEUE_LOW) {
        priv->tx_queues_stopped = false;
        wake_now = true;
    }
    spin_unlock_irqrestore(&priv->state_lock, flags);

    if (!skb) {
        return -EAGAIN;
    }

    if (wake_now)
        ieee80211_wake_queues(priv->hw);

    len = skb->len;
    if (len > count) {
        /* Caller buffer too small — drop the frame */
        kfree_skb(skb);
        return -EMSGSIZE;
    }

    if (copy_to_user(buf, skb->data, len)) {
        kfree_skb(skb);
        return -EFAULT;
    }

    kfree_skb(skb);
    return len;
}

/*
 * write() — Relay daemon pushes RX frames received from the hub.
 *
 * The relay writes complete wire messages:
 *   [uint32_t length (network order)] [frame_hdr] [802.11 frame]
 *
 * Multiple messages may be written in a single write() call;
 * we parse them one by one.
 */
static ssize_t ath9k_host_chrdev_write(struct file *filp,
                                       const char __user *buf,
                                       size_t count, loff_t *ppos)
{
    struct ath9k_host_priv *priv = filp->private_data;
    u8 *kbuf;
    size_t offset = 0;

    if (!priv || !priv->started) {
        return -ENODEV;
    }

    if (count > ATH9K_MEDIUM_MAX_MSG * 8) {
        return -EINVAL;
    }

    kbuf = kvmalloc(count, GFP_KERNEL);
    if (!kbuf) {
        return -ENOMEM;
    }

    if (copy_from_user(kbuf, buf, count)) {
        kvfree(kbuf);
        return -EFAULT;
    }

    /* Parse one or more wire messages */
    while (offset + 4 <= count) {
        __be32 len_be;
        u32 payload_len;

        memcpy(&len_be, kbuf + offset, 4);
        payload_len = ntohl(len_be);

        /* Accept v1 (28-byte) and v2 (40-byte) headers; ath9k_host_rx_frame
         * detects which from `len` and zeros the missing v2 fields. */
        if (payload_len < ATH9K_MEDIUM_HDR_SIZE_V1 ||
            payload_len > ATH9K_MEDIUM_HDR_SIZE + ATH9K_MEDIUM_MAX_FRAME) {
            break;  /* bad length — stop parsing */
        }

        if (offset + 4 + payload_len > count) {
            break;  /* incomplete message */
        }

        /* Inject this frame */
        ath9k_host_rx_frame(priv, kbuf + offset + 4, payload_len);
        offset += 4 + payload_len;
    }

    kvfree(kbuf);
    return count;
}

static __poll_t ath9k_host_chrdev_poll(struct file *filp,
                                       struct poll_table_struct *wait)
{
    struct ath9k_host_priv *priv = filp->private_data;
    __poll_t mask = 0;

    poll_wait(filp, &priv->tx_waitq, wait);

    if (!skb_queue_empty(&priv->tx_queue)) {
        mask |= EPOLLIN | EPOLLRDNORM;   /* TX frames available to read */
    }

    mask |= EPOLLOUT | EPOLLWRNORM;       /* Always writable (RX injection) */

    return mask;
}

static const struct file_operations ath9k_host_chrdev_fops = {
    .owner   = THIS_MODULE,
    .open    = ath9k_host_chrdev_open,
    .release = ath9k_host_chrdev_release,
    .read    = ath9k_host_chrdev_read,
    .write   = ath9k_host_chrdev_write,
    .poll    = ath9k_host_chrdev_poll,
};

static struct miscdevice ath9k_host_miscdev = {
    .minor  = MISC_DYNAMIC_MINOR,
    .name   = ATH9K_MEDIUM_CHRDEV_NAME,
    .fops   = &ath9k_host_chrdev_fops,
};

/* -------------------------------------------------------------------
 *  MAC address parsing
 * ------------------------------------------------------------------- */
static int parse_mac(const char *str, u8 *mac)
{
    int ret;
    unsigned int m[6];
    int i;

    ret = sscanf(str, "%02x:%02x:%02x:%02x:%02x:%02x",
                 &m[0], &m[1], &m[2], &m[3], &m[4], &m[5]);
    if (ret != 6) {
        return -EINVAL;
    }

    for (i = 0; i < 6; i++) {
        mac[i] = (u8)m[i];
    }
    return 0;
}

/* -------------------------------------------------------------------
 *  Module init / exit
 * ------------------------------------------------------------------- */
static int __init ath9k_host_init(void)
{
    struct ieee80211_hw *hw;
    struct ath9k_host_priv *priv;
    u8 mac[ETH_ALEN];
    int ret;

    /* Parse MAC address */
    ret = parse_mac(macaddr, mac);
    if (ret) {
        pr_err(DRV_NAME ": invalid MAC address '%s'\n", macaddr);
        return ret;
    }

    /*
     * Create a platform device as the parent for wiphy.
     * mac80211 / cfg80211 requires a non-NULL struct device * parent
     * for sysfs registration.  This is the same approach used by
     * mac80211_hwsim.
     */
    g_pdev = platform_device_register_simple(DRV_NAME, -1, NULL, 0);
    if (IS_ERR(g_pdev)) {
        pr_err(DRV_NAME ": platform_device_register failed: %ld\n",
               PTR_ERR(g_pdev));
        g_pdev = NULL;
        return -ENOMEM;
    }

    /* Allocate ieee80211_hw with our private data */
    hw = ieee80211_alloc_hw(sizeof(*priv), &ath9k_host_ops);
    if (!hw) {
        pr_err(DRV_NAME ": ieee80211_alloc_hw failed\n");
        ret = -ENOMEM;
        goto err_free_pdev;
    }

    priv = hw->priv;
    memset(priv, 0, sizeof(*priv));
    priv->hw = hw;
    memcpy(priv->mac_addr, mac, ETH_ALEN);

    spin_lock_init(&priv->state_lock);
    skb_queue_head_init(&priv->tx_queue);
    init_waitqueue_head(&priv->tx_waitq);

    /* Set up hardware capabilities */
    SET_IEEE80211_DEV(hw, &g_pdev->dev);
    SET_IEEE80211_PERM_ADDR(hw, mac);

    ieee80211_hw_set(hw, SIGNAL_DBM);
    ieee80211_hw_set(hw, MFP_CAPABLE);
    ieee80211_hw_set(hw, SUPPORTS_PS);
    ieee80211_hw_set(hw, HOST_BROADCAST_PS_BUFFERING);
    ieee80211_hw_set(hw, RX_INCLUDES_FCS);
    ieee80211_hw_set(hw, SW_CRYPTO_CONTROL);

    hw->queues = 4;  /* AC_VO, AC_VI, AC_BE, AC_BK */

    hw->wiphy->bands[NL80211_BAND_2GHZ] = &ath9k_host_band_2ghz;
    hw->wiphy->bands[NL80211_BAND_5GHZ] = &ath9k_host_band_5ghz;

    hw->wiphy->interface_modes =
        BIT(NL80211_IFTYPE_STATION) |
        BIT(NL80211_IFTYPE_AP) |
        BIT(NL80211_IFTYPE_ADHOC) |
        BIT(NL80211_IFTYPE_MONITOR) |
        BIT(NL80211_IFTYPE_MESH_POINT);

    hw->wiphy->n_addresses = 1;

    /* Set regulatory domain hint */
    hw->wiphy->regulatory_flags |= REGULATORY_CUSTOM_REG;

    /* Register with mac80211 */
    ret = ieee80211_register_hw(hw);
    if (ret) {
        pr_err(DRV_NAME ": ieee80211_register_hw failed: %d\n", ret);
        goto err_free_hw;
    }

    /* Register the character device */
    ret = misc_register(&ath9k_host_miscdev);
    if (ret) {
        pr_err(DRV_NAME ": misc_register failed: %d\n", ret);
        goto err_unreg_hw;
    }

    g_priv = priv;
    priv->channel_freq = 2412;  /* default to channel 1 */
    priv->channel_band = NL80211_BAND_2GHZ;

    pr_info(DRV_NAME " v" DRV_VERSION ": registered — "
            "MAC %pM, chardev /dev/" ATH9K_MEDIUM_CHRDEV_NAME "\n", mac);

    return 0;

err_unreg_hw:
    ieee80211_unregister_hw(hw);
err_free_hw:
    ieee80211_free_hw(hw);
err_free_pdev:
    platform_device_unregister(g_pdev);
    g_pdev = NULL;
    return ret;
}

static void __exit ath9k_host_exit(void)
{
    struct ath9k_host_priv *priv = g_priv;

    if (!priv) {
        return;
    }

    g_priv = NULL;

    misc_deregister(&ath9k_host_miscdev);

    /* Drain the TX queue */
    skb_queue_purge(&priv->tx_queue);
    wake_up_interruptible(&priv->tx_waitq);

    ieee80211_unregister_hw(priv->hw);
    ieee80211_free_hw(priv->hw);

    if (g_pdev) {
        platform_device_unregister(g_pdev);
        g_pdev = NULL;
    }

    pr_info(DRV_NAME ": unloaded\n");
}

module_init(ath9k_host_init);
module_exit(ath9k_host_exit);

MODULE_AUTHOR("Virtual WiFi Project Contributors");
MODULE_DESCRIPTION("Host-side mac80211 radio for ath9k virtual wireless medium");
MODULE_LICENSE("GPL");
MODULE_VERSION(DRV_VERSION);
