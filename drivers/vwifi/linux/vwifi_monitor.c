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
#include <asm/unaligned.h>

#include "vwifi_drv.h"

/*
 * Radiotap on RX.
 *
 * The header is built rather than memcpy'd from a fixed struct, because
 * which fields are present depends on the frame: a legacy frame gets
 * RATE, an HT frame gets MCS, a VHT frame gets VHT. Radiotap requires
 * fields in ascending bit order, each aligned to its own size relative
 * to the start of the header, so the builder tracks alignment
 * explicitly and vwifi_rt_align() is the only thing allowed to move the
 * cursor.
 *
 * Bit order used here: TSFT(0) FLAGS(1) RATE(2) CHANNEL(3)
 * DBM_ANTSIGNAL(5) MCS(19) VHT(21).
 */

/* Worst case: 8 hdr + 8 tsft + 1 flags + 1 pad + 4 chan + 1 signal
 * + 1 pad + 12 vht = 36. Round up for headroom. */
#define VWIFI_RT_MAX	48

struct vwifi_rt_builder {
	u8	buf[VWIFI_RT_MAX];
	u32	len;
	u32	present;
};

static void vwifi_rt_align(struct vwifi_rt_builder *b, u32 size)
{
	while (b->len & (size - 1))
		b->buf[b->len++] = 0;
}

static void vwifi_rt_put(struct vwifi_rt_builder *b, u32 bit,
			 const void *data, u32 size, u32 align)
{
	vwifi_rt_align(b, align);
	if (WARN_ON_ONCE(b->len + size > VWIFI_RT_MAX))
		return;
	memcpy(b->buf + b->len, data, size);
	b->len += size;
	b->present |= BIT(bit);
}

/*
 * Decode a medium rate code into what radiotap needs.
 *
 * The medium's rate-code namespace (abi/vwifi.h) already encodes
 * bandwidth, spatial streams and MCS index in the code itself:
 *
 *   0x00..0x1F  legacy OFDM / CCK
 *   0x80..0x8F  HT20,  NSS 1 then 2   (MCS 0-7, 8-15)
 *   0x90..0x9F  HT40,  NSS 1 then 2
 *   0xA0/0xB0   VHT80, NSS 1 / 2      (MCS 0-9)
 *   0xC0/0xD0   VHT160,NSS 1 / 2
 *   0xE0/0xF0   HE80,  NSS 1 / 2      (MCS 0-11)
 *
 * So nothing is lost on the wire -- the information radiotap wants is
 * all recoverable here.
 */
enum vwifi_rate_kind { VWIFI_RATE_LEGACY, VWIFI_RATE_HT, VWIFI_RATE_VHT,
		       VWIFI_RATE_HE };

struct vwifi_rate_info {
	enum vwifi_rate_kind kind;
	u8 legacy_500kbps;	/* LEGACY */
	u8 mcs;			/* HT: 0-15 (NSS folded in). VHT/HE: 0-11 */
	u8 nss;			/* VHT/HE */
	u8 bw;			/* 0=20 1=40 2=80 3=160 */
	bool cck;
};

static void vwifi_decode_rate(u8 code, struct vwifi_rate_info *out)
{
	static const struct { u8 code; u8 rate; bool cck; } legacy[] = {
		{ 0x1B,   2, true  },	/*  1   Mbps */
		{ 0x1A,   4, true  },	/*  2   Mbps */
		{ 0x19,  11, true  },	/*  5.5 Mbps */
		{ 0x18,  22, true  },	/* 11   Mbps */
		{ 0x0B,  12, false },	/*  6   Mbps */
		{ 0x0F,  18, false },	/*  9   Mbps */
		{ 0x0A,  24, false },	/* 12   Mbps */
		{ 0x0E,  36, false },	/* 18   Mbps */
		{ 0x09,  48, false },	/* 24   Mbps */
		{ 0x0D,  72, false },	/* 36   Mbps */
		{ 0x08,  96, false },	/* 48   Mbps */
		{ 0x0C, 108, false },	/* 54   Mbps */
	};
	unsigned int i;

	memset(out, 0, sizeof(*out));

	if (code < 0x80) {
		out->kind = VWIFI_RATE_LEGACY;
		for (i = 0; i < ARRAY_SIZE(legacy); i++) {
			if (legacy[i].code == code) {
				out->legacy_500kbps = legacy[i].rate;
				out->cck = legacy[i].cck;
				return;
			}
		}
		return;		/* unknown legacy code: rate 0 */
	}

	if (code < 0xA0) {
		/* HT. Bit 4 selects HT40, bit 3 selects NSS 2, which in
		 * radiotap's MCS numbering is just MCS + 8. */
		out->kind = VWIFI_RATE_HT;
		out->bw   = (code & 0x10) ? 1 : 0;
		out->mcs  = (code & 0x07) + ((code & 0x08) ? 8 : 0);
		out->nss  = (code & 0x08) ? 2 : 1;
		return;
	}

	if (code < 0xE0) {
		out->kind = VWIFI_RATE_VHT;
		out->bw   = (code < 0xC0) ? 2 : 3;	/* VHT80 / VHT160 */
		out->nss  = ((code & 0x10) ? 2 : 1);
		out->mcs  = code & 0x0F;
		if (out->mcs > 9)
			out->mcs = 9;
		return;
	}

	/* HE. Radiotap's HE field is a six-word structure whose useful
	 * subset needs more than the medium encodes; report it as VHT80,
	 * which is what the rate actually is on this medium, rather than
	 * emit a half-filled HE field a decoder would misread. */
	out->kind = VWIFI_RATE_VHT;
	out->bw   = 2;
	out->nss  = (code & 0x10) ? 2 : 1;
	out->mcs  = min_t(u8, code & 0x0F, 9);
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
	struct vwifi_rt_builder b = { .len = 8 };	/* fixed header first */
	struct vwifi_rate_info ri;
	struct sk_buff *skb;
	__le16 chan[2];
	__le64 tsft;
	u8 flags, sig;

	if (!ndev || !netif_running(ndev))
		return false;

	vwifi_decode_rate(desc->rate_code, &ri);

	tsft = cpu_to_le64(desc->tsf);
	vwifi_rt_put(&b, IEEE80211_RADIOTAP_TSFT, &tsft, sizeof(tsft), 8);

	/*
	 * No FCS flag. Frames on this medium carry no FCS by convention --
	 * the physical bridge strips it from real captures and the QEMU
	 * devices never add one -- so claiming IEEE80211_RADIOTAP_F_FCS
	 * would point a capture tool at four bytes of payload.
	 */
	flags = 0;
	if (desc->flags & VWIFI_RX_F_DECRYPTED)
		flags |= IEEE80211_RADIOTAP_F_WEP;
	vwifi_rt_put(&b, IEEE80211_RADIOTAP_FLAGS, &flags, 1, 1);

	/* RATE is legacy-only by spec; for HT and above it must be absent
	 * and the MCS/VHT field carries the rate instead. */
	if (ri.kind == VWIFI_RATE_LEGACY && ri.legacy_500kbps)
		vwifi_rt_put(&b, IEEE80211_RADIOTAP_RATE,
			     &ri.legacy_500kbps, 1, 1);

	chan[0] = cpu_to_le16(desc->channel_freq);
	chan[1] = cpu_to_le16((desc->channel_freq >= 5000) ?
			      IEEE80211_CHAN_5GHZ : IEEE80211_CHAN_2GHZ);
	if (ri.kind == VWIFI_RATE_LEGACY)
		chan[1] |= cpu_to_le16(ri.cck ? IEEE80211_CHAN_CCK :
						IEEE80211_CHAN_OFDM);
	vwifi_rt_put(&b, IEEE80211_RADIOTAP_CHANNEL, chan, sizeof(chan), 2);

	sig = (u8)desc->rssi;
	vwifi_rt_put(&b, IEEE80211_RADIOTAP_DBM_ANTSIGNAL, &sig, 1, 1);

	if (ri.kind == VWIFI_RATE_HT) {
		u8 mcs[3];

		mcs[0] = IEEE80211_RADIOTAP_MCS_HAVE_MCS |
			 IEEE80211_RADIOTAP_MCS_HAVE_BW;
		mcs[1] = ri.bw ? IEEE80211_RADIOTAP_MCS_BW_40 :
				 IEEE80211_RADIOTAP_MCS_BW_20;
		mcs[2] = ri.mcs;
		vwifi_rt_put(&b, IEEE80211_RADIOTAP_MCS, mcs, sizeof(mcs), 1);
	} else if (ri.kind == VWIFI_RATE_VHT) {
		struct {
			__le16	known;
			u8	flags;
			u8	bandwidth;
			u8	mcs_nss[4];
			u8	coding;
			u8	group_id;
			__le16	partial_aid;
		} __packed vht = {};

		vht.known	= cpu_to_le16(IEEE80211_RADIOTAP_VHT_KNOWN_BANDWIDTH);
		/* Radiotap bandwidth index: 0=20 1=40 4=80 11=160. */
		vht.bandwidth	= (ri.bw == 3) ? 11 : 4;
		/* User 0: MCS in the high nibble, NSS in the low. */
		vht.mcs_nss[0]	= (ri.mcs << 4) | (ri.nss & 0x0F);
		vwifi_rt_put(&b, IEEE80211_RADIOTAP_VHT, &vht, sizeof(vht), 2);
	}

	/* Fixed header last: it_len is only known now. */
	b.buf[0] = 0;				/* it_version */
	b.buf[1] = 0;				/* it_pad */
	put_unaligned_le16(b.len, &b.buf[2]);	/* it_len */
	put_unaligned_le32(b.present, &b.buf[4]);

	skb = netdev_alloc_skb(ndev, b.len + desc->frame_len);
	if (!skb)
		return false;

	skb_put_data(skb, b.buf, b.len);
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
