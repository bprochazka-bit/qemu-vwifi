// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * vwifi — monitor mode and raw injection
 *
 * Two directions, one interface:
 *
 *   RX   the device hands up whole 802.11 frames with VWIFI_RX_F_RAW
 *        set, plus the metadata it recovered from the medium header
 *        (RSSI, rate code, channel, TSF). This file turns that metadata
 *        into a radiotap header so tcpdump and Wireshark see a normal
 *        802.11 capture.
 *
 *   TX   userspace writes a radiotap header followed by an 802.11 frame
 *        to the monitor netdev. This file strips the radiotap and marks
 *        the descriptor VWIFI_TX_F_INJECT, which tells the device to put
 *        the frame on the medium verbatim -- no 802.3 translation, no
 *        encryption, and the transmitter address taken from the frame's
 *        own addr2 rather than the station MAC.
 *
 * Injection is what makes the medium testable from inside a guest: you
 * can forge a beacon, a deauth, or a malformed frame and watch what the
 * other peers do with it.
 */

#include <linux/etherdevice.h>
#include <linux/ieee80211.h>
#include <linux/if_arp.h>	/* ARPHRD_IEEE80211_RADIOTAP */
#include <net/ieee80211_radiotap.h>

#include "vwifi_drv.h"

/*
 * The radiotap header we prepend on RX.
 *
 * Radiotap requires fields in ascending bit order, each naturally
 * aligned relative to the start of the header. This layout satisfies
 * both, which is why it is a fixed struct rather than assembled at
 * runtime: offsets 8 (u64), 16, 17, 18 (u16 pair), 22.
 */
struct vwifi_rx_radiotap {
	u8	it_version;
	u8	it_pad;
	__le16	it_len;
	__le32	it_present;

	__le64	tsft;		/* offset 8,  8-aligned */
	u8	flags;		/* offset 16 */
	u8	rate;		/* offset 17, 500 kbps units */
	__le16	chan_freq;	/* offset 18, 2-aligned */
	__le16	chan_flags;	/* offset 20 */
	s8	dbm_antsignal;	/* offset 22 */
} __packed;

#define VWIFI_RT_PRESENT					\
	(BIT(IEEE80211_RADIOTAP_TSFT) |				\
	 BIT(IEEE80211_RADIOTAP_FLAGS) |			\
	 BIT(IEEE80211_RADIOTAP_RATE) |				\
	 BIT(IEEE80211_RADIOTAP_CHANNEL) |			\
	 BIT(IEEE80211_RADIOTAP_DBM_ANTSIGNAL))

/*
 * Medium rate code -> radiotap rate, in 500 kbps units.
 *
 * The low codes are the ath9k legacy set the medium inherited (see
 * devices/ath9k/src/vwifi_ath9k_dma.h and the rate-code namespace in
 * abi/vwifi.h). Codes at 0x80 and above are HT/VHT/HE MCS values, which
 * radiotap cannot express in this field at all -- reporting one as a
 * legacy rate would be worse than reporting nothing, so those come back
 * as 0 and the MCS field is left for a later revision.
 */
static u8 vwifi_rate_to_radiotap(u8 rate_code)
{
	switch (rate_code) {
	/* CCK */
	case 0x1B: return 2;	/*  1   Mbps */
	case 0x1A: return 4;	/*  2   Mbps */
	case 0x19: return 11;	/*  5.5 Mbps */
	case 0x18: return 22;	/* 11   Mbps */
	/* OFDM */
	case 0x0B: return 12;	/*  6   Mbps */
	case 0x0F: return 18;	/*  9   Mbps */
	case 0x0A: return 24;	/* 12   Mbps */
	case 0x0E: return 36;	/* 18   Mbps */
	case 0x09: return 48;	/* 24   Mbps */
	case 0x0D: return 72;	/* 36   Mbps */
	case 0x08: return 96;	/* 48   Mbps */
	case 0x0C: return 108;	/* 54   Mbps */
	default:   return 0;	/* HT/VHT/HE MCS, or unknown */
	}
}

/*
 * Deliver one raw 802.11 frame to the monitor interface.
 *
 * Returns false if the frame could not be delivered, so the caller can
 * count it. Called from the RX interrupt, so everything here is
 * GFP_ATOMIC and non-sleeping.
 */
bool vwifi_monitor_rx(struct vwifi_priv *p, const struct vwifi_rx_desc *desc,
		      const void *data)
{
	struct net_device *ndev = p->ndev;
	struct vwifi_rx_radiotap *rt;
	struct sk_buff *skb;
	u16 chan_flags;

	/*
	 * Radiotap alignment is a silent failure mode: get an offset wrong
	 * and every capture decodes as plausible garbage rather than
	 * erroring. Pin the layout to the spec here so a future field
	 * insertion cannot quietly shift it.
	 */
	BUILD_BUG_ON(offsetof(struct vwifi_rx_radiotap, it_len) != 2);
	BUILD_BUG_ON(offsetof(struct vwifi_rx_radiotap, it_present) != 4);
	BUILD_BUG_ON(offsetof(struct vwifi_rx_radiotap, tsft) != 8);
	BUILD_BUG_ON(offsetof(struct vwifi_rx_radiotap, flags) != 16);
	BUILD_BUG_ON(offsetof(struct vwifi_rx_radiotap, rate) != 17);
	BUILD_BUG_ON(offsetof(struct vwifi_rx_radiotap, chan_freq) != 18);
	BUILD_BUG_ON(offsetof(struct vwifi_rx_radiotap, chan_flags) != 20);
	BUILD_BUG_ON(offsetof(struct vwifi_rx_radiotap, dbm_antsignal) != 22);
	BUILD_BUG_ON(sizeof(struct vwifi_rx_radiotap) != 23);

	if (!ndev || !netif_running(ndev))
		return false;

	skb = netdev_alloc_skb(ndev, sizeof(*rt) + desc->frame_len);
	if (!skb)
		return false;

	rt = skb_put(skb, sizeof(*rt));
	memset(rt, 0, sizeof(*rt));
	rt->it_version = 0;
	rt->it_len     = cpu_to_le16(sizeof(*rt));
	rt->it_present = cpu_to_le32(VWIFI_RT_PRESENT);

	rt->tsft = cpu_to_le64(desc->tsf);

	/*
	 * The medium delivers de-aggregated MPDUs whose FCS the device
	 * does not reproduce, so say so rather than letting a capture tool
	 * try to validate four bytes that are not there.
	 */
	rt->flags = 0;
	if (desc->flags & VWIFI_RX_F_DECRYPTED)
		rt->flags |= IEEE80211_RADIOTAP_F_WEP;

	rt->rate = vwifi_rate_to_radiotap(desc->rate_code);

	chan_flags = (desc->channel_freq >= 5000) ?
		IEEE80211_CHAN_5GHZ : IEEE80211_CHAN_2GHZ;
	chan_flags |= rt->rate && desc->rate_code >= 0x18 ?
		IEEE80211_CHAN_CCK : IEEE80211_CHAN_OFDM;

	rt->chan_freq  = cpu_to_le16(desc->channel_freq);
	rt->chan_flags = cpu_to_le16(chan_flags);
	rt->dbm_antsignal = desc->rssi;

	skb_put_data(skb, data, desc->frame_len);

	/*
	 * A monitor netdev is ARPHRD_IEEE80211_RADIOTAP, so there is no
	 * Ethernet header to parse: point the mac header at the radiotap
	 * and hand it up unclassified.
	 */
	skb_reset_mac_header(skb);
	skb->ip_summed  = CHECKSUM_UNNECESSARY;
	skb->pkt_type   = PACKET_OTHERHOST;
	skb->protocol   = htons(ETH_P_802_2);

	ndev->stats.rx_packets++;
	ndev->stats.rx_bytes += skb->len;

	netif_rx(skb);
	return true;
}

/*
 * Strip the radiotap header from an injected frame.
 *
 * Returns the offset of the 802.11 frame, or a negative errno. We only
 * need it_len to find the payload; the rest of the radiotap fields are
 * transmit hints (rate, channel) that the medium either derives itself
 * or takes from the device's current channel.
 */
int vwifi_monitor_strip_radiotap(const struct sk_buff *skb)
{
	const struct ieee80211_radiotap_header *rt;
	u16 it_len;

	if (skb->len < sizeof(*rt))
		return -EINVAL;

	rt = (const struct ieee80211_radiotap_header *)skb->data;
	if (rt->it_version != 0)
		return -EINVAL;

	it_len = le16_to_cpu(rt->it_len);

	/* Must cover the fixed header and leave a plausible frame behind
	 * it -- 10 bytes reaches through addr1, the shortest thing worth
	 * putting on the air. */
	if (it_len < sizeof(*rt) || it_len + 10 > skb->len)
		return -EINVAL;

	return it_len;
}

/*
 * Enter or leave monitor mode.
 *
 * The netdev's link type changes with it, which is why this refuses to
 * run on a live interface: userspace and libpcap both latch the type at
 * open, and changing it underneath them produces captures that decode
 * as the wrong protocol rather than an error.
 */
int vwifi_monitor_set_mode(struct vwifi_priv *p, bool monitor)
{
	struct vwifi_op_mode mode;
	struct vwifi_raw_filter filter;
	int ret;

	if (!p->ndev)
		return -ENODEV;
	if (netif_running(p->ndev))
		return -EBUSY;

	if (monitor && !(p->caps.caps & VWIFI_CAP_MONITOR))
		return -EOPNOTSUPP;

	mode.mode = monitor ? VWIFI_MODE_MONITOR : VWIFI_MODE_STA;
	ret = vwifi_ctrl_cmd(p, VWIFI_OP_SET_OP_MODE, &mode, sizeof(mode),
			     NULL, 0, NULL);
	if (ret)
		return ret;

	if (monitor) {
		/*
		 * Mask 0 means "no filtering" to the device, which is what a
		 * monitor interface wants. Set it explicitly: a previous
		 * session may have narrowed it, and inheriting that would
		 * silently drop whole frame types.
		 */
		filter.mask = 0;
		ret = vwifi_ctrl_cmd(p, VWIFI_OP_SET_RAW_FILTER,
				     &filter, sizeof(filter), NULL, 0, NULL);
		if (ret)
			dev_warn(p->dev,
				 "could not clear the raw filter: %d\n", ret);
	}

	p->ndev->type = monitor ? ARPHRD_IEEE80211_RADIOTAP : ARPHRD_ETHER;
	p->monitor = monitor;

	dev_info(p->dev, "%s mode\n", monitor ? "monitor" : "station");
	return 0;
}
