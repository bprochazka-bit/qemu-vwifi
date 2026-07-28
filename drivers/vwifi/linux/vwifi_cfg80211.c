// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * vwifi — cfg80211 glue for the vwifi-virt QEMU device
 *
 * This is a FULL-MAC driver: the device runs the 802.11 state machine,
 * so cfg80211 talks to it directly and mac80211 is not involved at all.
 * Registering with mac80211 instead would mean running a second,
 * competing state machine on top of one that has already scanned,
 * authenticated, associated and installed keys.
 *
 * Each cfg80211 operation becomes one control opcode, and each device
 * event becomes one cfg80211 notification. That symmetry is the whole
 * driver.
 */

#include <linux/etherdevice.h>
#include <linux/ieee80211.h>
#include <net/cfg80211.h>

#include "vwifi_drv.h"

/* struct vwifi_scan_req carries at most four trailing SSID entries. */
#define VWIFI_MAX_SCAN_SSIDS	4
#define VWIFI_SCAN_TIMEOUT_MS	10000

/* ============================================================
 * Channel tables
 * ============================================================ */

/*
 * The 5 GHz mask in struct vwifi_caps is a bitmask over THIS list, not
 * over raw channel numbers — 165 does not fit in 64 bits otherwise.
 * The order must match the device's table in op_scan(); if you add a
 * channel here, add it there in the same position.
 */
static const u8 vwifi_unii_channels[] = {
	36, 40, 44, 48, 52, 56, 60, 64,
	100, 104, 108, 112, 116, 120, 124, 128, 132, 136, 140, 144,
	149, 153, 157, 161, 165,
};

/* Legacy rates only. The device reports HT/VHT capability, but the
 * medium's rate handling is per-frame metadata rather than a negotiated
 * rate set, so advertising a legacy set keeps cfg80211 honest. */
static struct ieee80211_rate vwifi_rates[] = {
	{ .bitrate = 10,  .hw_value = 0x02 },
	{ .bitrate = 20,  .hw_value = 0x04 },
	{ .bitrate = 55,  .hw_value = 0x0B },
	{ .bitrate = 110, .hw_value = 0x16 },
	{ .bitrate = 60,  .hw_value = 0x0C },
	{ .bitrate = 90,  .hw_value = 0x12 },
	{ .bitrate = 120, .hw_value = 0x18 },
	{ .bitrate = 180, .hw_value = 0x24 },
	{ .bitrate = 240, .hw_value = 0x30 },
	{ .bitrate = 360, .hw_value = 0x48 },
	{ .bitrate = 480, .hw_value = 0x60 },
	{ .bitrate = 540, .hw_value = 0x6C },
};

/* Skip the four CCK rates when describing the 5 GHz band. */
#define VWIFI_CCK_RATES 4

static u16 vwifi_chan_to_freq_24(u8 ch)
{
	if (ch == 14)
		return 2484;
	if (ch >= 1 && ch <= 13)
		return 2407 + 5 * ch;
	return 0;
}

static int vwifi_build_channels(struct vwifi_priv *p)
{
	unsigned int i, n;

	/* 2.4 GHz: the mask is over raw channel numbers, bit N = channel N. */
	n = 0;
	for (i = 1; i <= 14; i++)
		if (p->caps.supported_channels_24 & BIT(i))
			n++;

	if (n) {
		p->channels_2ghz = devm_kcalloc(p->dev, n,
						sizeof(*p->channels_2ghz),
						GFP_KERNEL);
		if (!p->channels_2ghz)
			return -ENOMEM;

		n = 0;
		for (i = 1; i <= 14; i++) {
			u16 freq;

			if (!(p->caps.supported_channels_24 & BIT(i)))
				continue;
			freq = vwifi_chan_to_freq_24(i);
			if (!freq)
				continue;
			p->channels_2ghz[n].band        = NL80211_BAND_2GHZ;
			p->channels_2ghz[n].center_freq = freq;
			p->channels_2ghz[n].hw_value    = freq;
			p->channels_2ghz[n].max_power   = 20;
			n++;
		}
		p->n_channels_2ghz = n;

		p->band_2ghz.band       = NL80211_BAND_2GHZ;
		p->band_2ghz.channels   = p->channels_2ghz;
		p->band_2ghz.n_channels = p->n_channels_2ghz;
		p->band_2ghz.bitrates   = vwifi_rates;
		p->band_2ghz.n_bitrates = ARRAY_SIZE(vwifi_rates);
		p->wiphy->bands[NL80211_BAND_2GHZ] = &p->band_2ghz;
	}

	/* 5 GHz: the mask indexes vwifi_unii_channels[]. */
	n = 0;
	for (i = 0; i < ARRAY_SIZE(vwifi_unii_channels); i++)
		if (p->caps.supported_channels_5 & BIT_ULL(i))
			n++;

	if (n) {
		p->channels_5ghz = devm_kcalloc(p->dev, n,
						sizeof(*p->channels_5ghz),
						GFP_KERNEL);
		if (!p->channels_5ghz)
			return -ENOMEM;

		n = 0;
		for (i = 0; i < ARRAY_SIZE(vwifi_unii_channels); i++) {
			if (!(p->caps.supported_channels_5 & BIT_ULL(i)))
				continue;
			p->channels_5ghz[n].band        = NL80211_BAND_5GHZ;
			p->channels_5ghz[n].center_freq =
				5000 + 5 * vwifi_unii_channels[i];
			p->channels_5ghz[n].hw_value =
				p->channels_5ghz[n].center_freq;
			p->channels_5ghz[n].max_power = 20;
			n++;
		}
		p->n_channels_5ghz = n;

		p->band_5ghz.band       = NL80211_BAND_5GHZ;
		p->band_5ghz.channels   = p->channels_5ghz;
		p->band_5ghz.n_channels = p->n_channels_5ghz;
		p->band_5ghz.bitrates   = vwifi_rates + VWIFI_CCK_RATES;
		p->band_5ghz.n_bitrates =
			ARRAY_SIZE(vwifi_rates) - VWIFI_CCK_RATES;
		p->wiphy->bands[NL80211_BAND_5GHZ] = &p->band_5ghz;
	}

	if (!p->n_channels_2ghz && !p->n_channels_5ghz) {
		dev_err(p->dev, "device reports no usable channels\n");
		return -ENODEV;
	}
	return 0;
}

/* ============================================================
 * Cipher / AKM translation
 * ============================================================ */

static u16 vwifi_cipher_from_nl(u32 suite)
{
	switch (suite) {
	case WLAN_CIPHER_SUITE_WEP40:	return VWIFI_CIPHER_WEP40;
	case WLAN_CIPHER_SUITE_WEP104:	return VWIFI_CIPHER_WEP104;
	case WLAN_CIPHER_SUITE_TKIP:	return VWIFI_CIPHER_TKIP;
	case WLAN_CIPHER_SUITE_CCMP:	return VWIFI_CIPHER_CCMP128;
	case WLAN_CIPHER_SUITE_GCMP_256:return VWIFI_CIPHER_GCMP256;
	default:			return VWIFI_CIPHER_NONE;
	}
}

static u16 vwifi_akm_from_nl(u32 suite)
{
	switch (suite) {
	case WLAN_AKM_SUITE_PSK:	return VWIFI_AKM_PSK;
	case WLAN_AKM_SUITE_SAE:	return VWIFI_AKM_SAE;
	case WLAN_AKM_SUITE_8021X:	return VWIFI_AKM_8021X;
	default:			return VWIFI_AKM_NONE;
	}
}

static u16 vwifi_auth_from_nl(enum nl80211_auth_type t)
{
	switch (t) {
	case NL80211_AUTHTYPE_SHARED_KEY:	return VWIFI_AUTH_SHARED;
	case NL80211_AUTHTYPE_SAE:		return VWIFI_AUTH_SAE;
	default:				return VWIFI_AUTH_OPEN;
	}
}

/* ============================================================
 * Scan bookkeeping
 *
 * p->scan_req is touched from the cfg80211 ops (under the wiphy lock)
 * and from the response workqueue (which is NOT under it), so every
 * transition goes through ring_lock. Claiming rather than reading means
 * exactly one context can ever complete a given request -- completing
 * one twice is a use-after-free, since cfg80211 owns the memory.
 * ============================================================ */

struct cfg80211_scan_request *vwifi_scan_claim(struct vwifi_priv *p)
{
	struct cfg80211_scan_request *req;
	unsigned long flags;

	spin_lock_irqsave(&p->ring_lock, flags);
	req = p->scan_req;
	p->scan_req = NULL;
	spin_unlock_irqrestore(&p->ring_lock, flags);
	return req;
}

/*
 * If SCAN_COMPLETE is ever lost, scan_req would stay set and every
 * later scan would fail with -EBUSY for the lifetime of the module.
 */
void vwifi_scan_timeout_fn(struct work_struct *work)
{
	struct vwifi_priv *p = container_of(to_delayed_work(work),
					    struct vwifi_priv, scan_timeout);
	struct cfg80211_scan_request *req = vwifi_scan_claim(p);
	struct cfg80211_scan_info info = { .aborted = true };

	if (!req)
		return;

	dev_warn(p->dev, "scan timed out with no SCAN_COMPLETE\n");
	cfg80211_scan_done(req, &info);
}

/* ============================================================
 * cfg80211 operations
 * ============================================================ */

static int vwifi_op_scan(struct wiphy *wiphy,
			 struct cfg80211_scan_request *request)
{
	struct vwifi_priv *p = wiphy_priv(wiphy);
	u8 buf[sizeof(struct vwifi_scan_req) + 4 * 34];
	struct vwifi_scan_req *req = (void *)buf;
	u32 len = sizeof(*req);
	unsigned long irqflags;
	unsigned int i, j;
	bool busy;
	int ret;

	spin_lock_irqsave(&p->ring_lock, irqflags);
	busy = p->scan_req != NULL;
	spin_unlock_irqrestore(&p->ring_lock, irqflags);
	if (busy)
		return -EBUSY;

	memset(buf, 0, sizeof(buf));
	req->dwell_ms = 100;

	/* Translate the requested channel list into the device's two
	 * masks. An empty mask means "all", which is what cfg80211 asking
	 * for every channel amounts to anyway. */
	for (i = 0; i < request->n_channels; i++) {
		u32 freq = request->channels[i]->center_freq;

		if (freq == 2484) {
			req->channel_mask_24 |= BIT(14);
		} else if (freq >= 2412 && freq <= 2472) {
			req->channel_mask_24 |= BIT((freq - 2407) / 5);
		} else if (freq >= 5000) {
			u8 ch = (freq - 5000) / 5;

			for (j = 0; j < ARRAY_SIZE(vwifi_unii_channels); j++) {
				if (vwifi_unii_channels[j] == ch) {
					req->channel_mask_5 |= BIT_ULL(j);
					break;
				}
			}
		}
	}

	/* Directed SSIDs, capped at what the device advertised. */
	req->num_ssids = min_t(u16, request->n_ssids, VWIFI_MAX_SCAN_SSIDS);

	for (i = 0; i < req->num_ssids; i++) {
		u8 *entry = buf + sizeof(*req) + i * 34;
		u8 ssid_len = min_t(u8, request->ssids[i].ssid_len, 32);

		entry[0] = ssid_len;
		memcpy(entry + 1, request->ssids[i].ssid, ssid_len);
		len += 34;
	}

	/*
	 * Submit BEFORE publishing the request. vwifi_ctrl_cmd() drains
	 * the response ring inline, so publishing first would let a stale
	 * SCAN_COMPLETE from the previous scan complete this brand-new
	 * one -- cfg80211 warns, and the new scan then hangs forever.
	 */
	ret = vwifi_ctrl_cmd(p, VWIFI_OP_SCAN, buf, len, NULL, 0, NULL);
	if (ret)
		return ret;

	spin_lock_irqsave(&p->ring_lock, irqflags);
	p->scan_req = request;
	spin_unlock_irqrestore(&p->ring_lock, irqflags);

	schedule_delayed_work(&p->scan_timeout,
			      msecs_to_jiffies(VWIFI_SCAN_TIMEOUT_MS));
	return 0;
}

static int vwifi_op_connect(struct wiphy *wiphy, struct net_device *ndev,
			    struct cfg80211_connect_params *sme)
{
	struct vwifi_priv *p = wiphy_priv(wiphy);
	u8 *buf;
	struct vwifi_connect_req *req;
	u32 len;
	int ret;

	if (sme->ssid_len > 32)
		return -EINVAL;

	len = sizeof(*req) + sme->ie_len;
	buf = kzalloc(len, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	req = (void *)buf;
	if (sme->bssid)
		ether_addr_copy(req->bssid, sme->bssid);
	req->ssid_len = sme->ssid_len;
	memcpy(req->ssid, sme->ssid, sme->ssid_len);
	if (sme->channel)
		req->channel_freq = sme->channel->center_freq;

	req->auth_algo = vwifi_auth_from_nl(sme->auth_type);
	req->cipher_pairwise = sme->crypto.n_ciphers_pairwise ?
		vwifi_cipher_from_nl(sme->crypto.ciphers_pairwise[0]) :
		VWIFI_CIPHER_NONE;
	req->cipher_group = sme->crypto.cipher_group ?
		vwifi_cipher_from_nl(sme->crypto.cipher_group) :
		VWIFI_CIPHER_NONE;
	req->akm_suite = sme->crypto.n_akm_suites ?
		vwifi_akm_from_nl(sme->crypto.akm_suites[0]) :
		VWIFI_AKM_NONE;

	if (sme->ie_len) {
		req->assoc_ie_len = sme->ie_len;
		memcpy(buf + sizeof(*req), sme->ie, sme->ie_len);
	}

	ret = vwifi_ctrl_cmd(p, VWIFI_OP_CONNECT, buf, len, NULL, 0, NULL);
	kfree(buf);

	/*
	 * Return the error and report NOTHING. A non-zero return from
	 * .connect means the attempt never started, and cfg80211 tears its
	 * own state down and answers userspace with the errno. Calling
	 * cfg80211_connect_result() as well would fire a CONNECT event at
	 * a wdev that has just been reset, after userspace already saw the
	 * failure -- which desynchronizes wpa_supplicant's state machine.
	 */
	return ret;
}

static int vwifi_op_disconnect(struct wiphy *wiphy, struct net_device *ndev,
			       u16 reason_code)
{
	struct vwifi_priv *p = wiphy_priv(wiphy);
	struct vwifi_disconnect_req req = { .reason_code = reason_code };

	return vwifi_ctrl_cmd(p, VWIFI_OP_DISCONNECT, &req, sizeof(req),
			      NULL, 0, NULL);
}

static int vwifi_op_add_key(struct wiphy *wiphy, struct net_device *ndev,
			    int link_id, u8 key_index, bool pairwise,
			    const u8 *mac_addr, struct key_params *params)
{
	struct vwifi_priv *p = wiphy_priv(wiphy);
	struct vwifi_key key;

	if (params->key_len > sizeof(key.key))
		return -EINVAL;

	memset(&key, 0, sizeof(key));
	if (mac_addr)
		ether_addr_copy(key.id.mac, mac_addr);
	key.id.key_idx  = key_index;
	key.id.pairwise = pairwise ? 1 : 0;
	key.cipher      = vwifi_cipher_from_nl(params->cipher);
	key.key_len     = params->key_len;
	memcpy(key.key, params->key, params->key_len);

	return vwifi_ctrl_cmd(p, VWIFI_OP_SET_KEY, &key, sizeof(key),
			      NULL, 0, NULL);
}

static int vwifi_op_del_key(struct wiphy *wiphy, struct net_device *ndev,
			    int link_id, u8 key_index, bool pairwise,
			    const u8 *mac_addr)
{
	struct vwifi_priv *p = wiphy_priv(wiphy);
	struct vwifi_key_id id;

	memset(&id, 0, sizeof(id));
	if (mac_addr)
		ether_addr_copy(id.mac, mac_addr);
	id.key_idx  = key_index;
	id.pairwise = pairwise ? 1 : 0;

	return vwifi_ctrl_cmd(p, VWIFI_OP_DEL_KEY, &id, sizeof(id),
			      NULL, 0, NULL);
}

/*
 * The device tracks the default key implicitly from what has been
 * installed, so there is nothing to send. Returning an error here would
 * fail association, so accept and ignore.
 */
static int vwifi_op_set_default_key(struct wiphy *wiphy,
				    struct net_device *ndev, int link_id,
				    u8 key_index, bool unicast, bool multicast)
{
	return 0;
}

static int vwifi_op_get_station(struct wiphy *wiphy, struct net_device *ndev,
				const u8 *mac, struct station_info *sinfo)
{
	struct vwifi_priv *p = wiphy_priv(wiphy);

	if (!p->connected || !ether_addr_equal(mac, p->bssid))
		return -ENOENT;

	sinfo->filled = BIT_ULL(NL80211_STA_INFO_TX_PACKETS) |
			BIT_ULL(NL80211_STA_INFO_RX_PACKETS) |
			BIT_ULL(NL80211_STA_INFO_TX_BYTES) |
			BIT_ULL(NL80211_STA_INFO_RX_BYTES);
	sinfo->tx_packets = ndev->stats.tx_packets;
	sinfo->rx_packets = ndev->stats.rx_packets;
	sinfo->tx_bytes   = ndev->stats.tx_bytes;
	sinfo->rx_bytes   = ndev->stats.rx_bytes;
	return 0;
}

static int vwifi_op_change_iface(struct wiphy *wiphy, struct net_device *ndev,
				 enum nl80211_iftype type,
				 struct vif_params *params)
{
	struct vwifi_priv *p = wiphy_priv(wiphy);
	struct vwifi_op_mode mode;

	/*
	 * STA is the only mode this driver can actually deliver. Monitor
	 * would need a radiotap path that does not exist yet; accepting
	 * the switch would stop the device doing 802.11 framing and leave
	 * the interface silent with no way back short of a reload.
	 */
	if (type != NL80211_IFTYPE_STATION)
		return -EOPNOTSUPP;
	mode.mode = VWIFI_MODE_STA;

	return vwifi_ctrl_cmd(p, VWIFI_OP_SET_OP_MODE, &mode, sizeof(mode),
			      NULL, 0, NULL);
}

static const struct cfg80211_ops vwifi_cfg80211_ops = {
	.scan			= vwifi_op_scan,
	.connect		= vwifi_op_connect,
	.disconnect		= vwifi_op_disconnect,
	.add_key		= vwifi_op_add_key,
	.del_key		= vwifi_op_del_key,
	.set_default_key	= vwifi_op_set_default_key,
	.get_station		= vwifi_op_get_station,
	.change_virtual_intf	= vwifi_op_change_iface,
};

const struct cfg80211_ops *vwifi_cfg80211_ops_get(void)
{
	return &vwifi_cfg80211_ops;
}

/* ============================================================
 * Device events -> cfg80211 notifications
 * ============================================================ */

static void handle_bss_found(struct vwifi_priv *p, const void *payload, u32 len)
{
	const struct vwifi_bss_entry *bss = payload;
	struct cfg80211_inform_bss data = {};
	struct ieee80211_channel *chan;
	const struct ieee80211_mgmt *mgmt;
	struct cfg80211_bss *cbss;
	u32 frame_len;

	if (len < sizeof(*bss))
		return;

	frame_len = bss->ie_len;
	if (frame_len < offsetof(struct ieee80211_mgmt, u.beacon.variable) ||
	    sizeof(*bss) + frame_len > len)
		return;

	chan = ieee80211_get_channel(p->wiphy, bss->channel_freq);
	if (!chan)
		return;

	data.chan   = chan;
	data.signal = bss->rssi * 100;	/* dBm -> mBm */

	/*
	 * The device sends the whole beacon or probe response, not just
	 * its IE tail, so hand cfg80211 the frame and let it parse. That
	 * keeps IE handling in one place rather than duplicating the
	 * parser here.
	 */
	mgmt = (const struct ieee80211_mgmt *)((const u8 *)payload +
					       sizeof(*bss));

	cbss = cfg80211_inform_bss_frame_data(p->wiphy, &data,
					      (struct ieee80211_mgmt *)mgmt,
					      frame_len, GFP_KERNEL);
	if (cbss)
		cfg80211_put_bss(p->wiphy, cbss);
}

static void handle_scan_complete(struct vwifi_priv *p,
				 const void *payload, u32 len)
{
	struct cfg80211_scan_info info = {};
	struct cfg80211_scan_request *req = vwifi_scan_claim(p);

	/* No claim means no scan of ours is outstanding -- a stale
	 * completion from a previous one, or the timeout beat us here. */
	if (!req)
		return;

	cancel_delayed_work(&p->scan_timeout);

	if (len >= sizeof(s32))
		info.aborted = (*(const s32 *)payload) != 0;

	cfg80211_scan_done(req, &info);
}

static void handle_assoc_result(struct vwifi_priv *p,
				const void *payload, u32 len)
{
	const struct vwifi_assoc_result *ar = payload;
	const u8 *resp_ie = NULL;
	u16 resp_ie_len = 0;

	if (len < sizeof(*ar) || !p->ndev)
		return;

	if (ar->ie_len && sizeof(*ar) + ar->ie_len <= len) {
		resp_ie = (const u8 *)payload + sizeof(*ar);
		resp_ie_len = ar->ie_len;
	}

	if (ar->status_code == 0) {
		ether_addr_copy(p->bssid, ar->bssid);
		p->connected = true;
		netif_carrier_on(p->ndev);
	}

	cfg80211_connect_result(p->ndev, ar->bssid, NULL, 0,
				resp_ie, resp_ie_len,
				ar->status_code, GFP_KERNEL);
}

static void handle_disconnected(struct vwifi_priv *p,
				const void *payload, u32 len)
{
	const struct vwifi_disconnect_ev *ev = payload;
	u16 reason = WLAN_REASON_UNSPECIFIED;
	bool local = false;

	if (len >= sizeof(*ev)) {
		reason = ev->reason_code;
		local = ev->local != 0;
	}

	if (!p->ndev)
		return;

	p->connected = false;
	eth_zero_addr(p->bssid);
	netif_carrier_off(p->ndev);

	cfg80211_disconnected(p->ndev, reason, NULL, 0, local, GFP_KERNEL);
}

static void handle_link_state(struct vwifi_priv *p,
			      const void *payload, u32 len)
{
	u32 up;

	if (len < sizeof(u32) || !p->ndev)
		return;

	up = *(const u32 *)payload;
	dev_info(p->dev, "medium link %s\n", up ? "up" : "down");

	/*
	 * This is the hub connection, not the 802.11 association. Losing
	 * it means the association is gone too, but the device sends its
	 * own DISCONNECTED for that — do not synthesise one here.
	 */
	if (!up && p->connected)
		netif_carrier_off(p->ndev);
}

void vwifi_handle_event(struct vwifi_priv *p, u16 event,
			const void *payload, u32 len)
{
	switch (event) {
	case VWIFI_EV_BSS_FOUND:
		handle_bss_found(p, payload, len);
		break;
	case VWIFI_EV_SCAN_COMPLETE:
		handle_scan_complete(p, payload, len);
		break;
	case VWIFI_EV_ASSOC_RESULT:
		handle_assoc_result(p, payload, len);
		break;
	case VWIFI_EV_DISCONNECTED:
		handle_disconnected(p, payload, len);
		break;
	case VWIFI_EV_KEY_INSTALLED:
		/* Informational: cfg80211 already believes the key is in. */
		break;
	case VWIFI_EV_LINK_STATE:
		handle_link_state(p, payload, len);
		break;
	case VWIFI_EV_MGMT_RX:
		/* Only needed once this driver reports management frames to
		 * userspace (SAE, ANQP). STA-mode association is handled in
		 * the device, so nothing to do yet. */
		break;
	default:
		dev_dbg(p->dev, "unhandled event 0x%04x\n", event);
		break;
	}
}

/* ============================================================
 * Registration
 * ============================================================ */

static const struct ieee80211_iface_limit vwifi_iface_limits[] = {
	{
		.max = 1,
		.types = BIT(NL80211_IFTYPE_STATION),
	},
};

static const struct ieee80211_iface_combination vwifi_iface_combos[] = {
	{
		.limits = vwifi_iface_limits,
		.n_limits = ARRAY_SIZE(vwifi_iface_limits),
		.max_interfaces = 1,
		.num_different_channels = 1,
	},
};

/*
 * CCMP only. The device's SET_KEY accepts VWIFI_CIPHER_CCMP128 with a
 * 16-byte key and rejects everything else outright, so advertising
 * WEP/TKIP/GCMP would let wpa_supplicant negotiate a cipher that then
 * fails at .add_key -- mid-four-way-handshake, leaving an associated
 * but unkeyed link and a deauth loop. GCMP-256 is appended only if the
 * device claims WPA3.
 */
static const u32 vwifi_cipher_suites[] = {
	WLAN_CIPHER_SUITE_CCMP,
	WLAN_CIPHER_SUITE_GCMP_256,	/* only exposed with VWIFI_CAP_WPA3 */
};
#define VWIFI_N_CIPHERS_BASE 1

int vwifi_cfg80211_init(struct vwifi_priv *p)
{
	struct net_device *ndev;
	struct vwifi_op_mode mode;
	u32 rsp_len = 0;
	int ret;

	/* Ask the device what it is before describing it to cfg80211. */
	ret = vwifi_ctrl_cmd(p, VWIFI_OP_GET_CAPS, NULL, 0,
			     &p->caps, sizeof(p->caps), &rsp_len);
	if (ret) {
		dev_err(p->dev, "GET_CAPS failed: %d\n", ret);
		return ret;
	}
	if (rsp_len < sizeof(p->caps)) {
		dev_err(p->dev, "short GET_CAPS response (%u bytes)\n", rsp_len);
		return -EPROTO;
	}

	ret = vwifi_build_channels(p);
	if (ret)
		return ret;

	/* Clamp to what vwifi_op_scan can actually encode -- the scan
	 * request carries at most four SSIDs. Advertising the device's
	 * number unchecked would also truncate into wiphy's u8. */
	p->wiphy->max_scan_ssids = min_t(u16, p->caps.max_scan_ssids ?: 4,
					 VWIFI_MAX_SCAN_SSIDS);
	/* struct vwifi_scan_req has nowhere to put probe-request IEs, so
	 * claiming support for them would be a lie. */
	p->wiphy->max_scan_ie_len = 0;
	p->wiphy->signal_type = CFG80211_SIGNAL_TYPE_MBM;
	/*
	 * STA only. The device supports monitor mode, but this driver has
	 * no radiotap path and drops every RX_F_RAW frame, so offering
	 * NL80211_IFTYPE_MONITOR would give userspace an interface that
	 * switches successfully and then goes permanently silent.
	 */
	p->wiphy->interface_modes = BIT(NL80211_IFTYPE_STATION);
	p->wiphy->iface_combinations = vwifi_iface_combos;
	p->wiphy->n_iface_combinations = ARRAY_SIZE(vwifi_iface_combos);
	p->wiphy->cipher_suites = vwifi_cipher_suites;
	p->wiphy->n_cipher_suites = (p->caps.caps & VWIFI_CAP_WPA3) ?
		ARRAY_SIZE(vwifi_cipher_suites) : VWIFI_N_CIPHERS_BASE;

	ret = wiphy_register(p->wiphy);
	if (ret) {
		dev_err(p->dev, "wiphy_register failed: %d\n", ret);
		return ret;
	}

	ndev = alloc_netdev(0, "wlan%d", NET_NAME_ENUM, ether_setup);
	if (!ndev) {
		ret = -ENOMEM;
		goto err_wiphy;
	}

	p->ndev = ndev;
	ndev->netdev_ops = &vwifi_netdev_ops;
	ndev->needs_free_netdev = true;
	SET_NETDEV_DEV(ndev, p->dev);

	p->wdev.wiphy  = p->wiphy;
	p->wdev.netdev = ndev;
	p->wdev.iftype = NL80211_IFTYPE_STATION;
	ndev->ieee80211_ptr = &p->wdev;

	/* An all-zero or multicast MAC from the device would register fine
	 * and then fail eth_validate_addr() at every ip-link-up, which is
	 * an opaque way to be broken. */
	if (is_valid_ether_addr(p->caps.default_mac))
		eth_hw_addr_set(ndev, p->caps.default_mac);
	else {
		dev_warn(p->dev,
			 "device reported an invalid MAC; using a random one\n");
		eth_hw_addr_random(ndev);
	}

	ret = register_netdev(ndev);
	if (ret) {
		dev_err(p->dev, "register_netdev failed: %d\n", ret);
		goto err_netdev;
	}

	netif_carrier_off(ndev);

	/* Park the device in STA mode and tell it the MAC we registered. */
	mode.mode = VWIFI_MODE_STA;
	vwifi_ctrl_cmd(p, VWIFI_OP_SET_OP_MODE, &mode, sizeof(mode),
		       NULL, 0, NULL);
	vwifi_ctrl_cmd(p, VWIFI_OP_SET_STA_MAC, ndev->dev_addr, ETH_ALEN,
		       NULL, 0, NULL);

	return 0;

err_netdev:
	free_netdev(ndev);
	p->ndev = NULL;
err_wiphy:
	wiphy_unregister(p->wiphy);
	return ret;
}

void vwifi_cfg80211_deinit(struct vwifi_priv *p)
{
	if (p->ndev) {
		unregister_netdev(p->ndev);
		p->ndev = NULL;
	}
	if (p->wiphy)
		wiphy_unregister(p->wiphy);
}
