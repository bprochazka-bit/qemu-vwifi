// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * vwifi — netdev data path
 *
 * In STA mode the device does the 802.11 framing: the driver hands it
 * plain 802.3 frames and gets 802.3 back. That is what makes this a
 * short file — there is no LLC/SNAP encapsulation, no ToDS/FromDS
 * address juggling and no crypto here, because all of it happens on the
 * other side of the ring.
 */

#include <linux/etherdevice.h>
#include <linux/netdevice.h>
#include <linux/skbuff.h>

#include "vwifi_drv.h"

static struct vwifi_priv *ndev_priv(struct net_device *ndev)
{
	/* The netdev carries no private area of its own; the state lives
	 * in the wiphy, which outlives it. */
	return wiphy_priv(ndev->ieee80211_ptr->wiphy);
}

static int vwifi_ndo_open(struct net_device *ndev)
{
	netif_start_queue(ndev);
	return 0;
}

static int vwifi_ndo_stop(struct net_device *ndev)
{
	netif_stop_queue(ndev);
	return 0;
}

static netdev_tx_t vwifi_ndo_start_xmit(struct sk_buff *skb,
					struct net_device *ndev)
{
	struct vwifi_priv *p = ndev_priv(ndev);
	struct vwifi_tx_desc *d;
	unsigned long flags;
	u32 idx;

	if (skb->len > VWIFI_TX_BUF_SIZE) {
		ndev->stats.tx_dropped++;
		dev_kfree_skb_any(skb);
		return NETDEV_TX_OK;
	}

	spin_lock_irqsave(&p->ring_lock, flags);

	idx = p->tx.head & p->tx.mask;
	d = (struct vwifi_tx_desc *)((u8 *)p->tx.desc +
				     (size_t)idx * p->tx.desc_size);

	/*
	 * A descriptor still owned by the device means the ring is full.
	 *
	 * Drop rather than netif_stop_queue(): the device consumes the
	 * whole TX ring synchronously on every doorbell write and never
	 * raises VWIFI_VEC_TX_COMPLETE, so there is no completion
	 * interrupt to wake a stopped queue with. Stopping it here would
	 * wedge transmit permanently. A full ring therefore means the
	 * device is not consuming at all, which is worth saying out loud.
	 */
	if (d->flags & VWIFI_DESC_F_OWN) {
		spin_unlock_irqrestore(&p->ring_lock, flags);
		net_warn_ratelimited("%s: TX ring full, device not draining\n",
				     ndev->name);
		ndev->stats.tx_dropped++;
		dev_kfree_skb_any(skb);
		return NETDEV_TX_OK;
	}

	memcpy((u8 *)p->tx.bufs + (size_t)idx * p->tx.buf_size,
	       skb->data, skb->len);

	memset(d, 0, sizeof(*d));
	d->frame_addr = p->tx.bufs_dma + (dma_addr_t)idx * p->tx.buf_size;
	d->frame_len  = skb->len;
	d->flags      = VWIFI_DESC_F_OWN;

	wmb();	/* descriptor complete before the doorbell */
	p->tx.head++;
	vwifi_wr(p, VWIFI_REG_TX_RING_DOORBELL, p->tx.head & p->tx.mask);

	spin_unlock_irqrestore(&p->ring_lock, flags);

	ndev->stats.tx_packets++;
	ndev->stats.tx_bytes += skb->len;
	dev_kfree_skb_any(skb);
	return NETDEV_TX_OK;
}

/*
 * Called from the RX ring drain with one completed descriptor. In STA
 * mode `data` is an 802.3 frame; in monitor mode it is raw 802.11 with
 * VWIFI_RX_F_RAW set, which this driver does not yet surface.
 */
void vwifi_rx_frame(struct vwifi_priv *p, const struct vwifi_rx_desc *desc,
		    const void *data)
{
	struct net_device *ndev = p->ndev;
	struct sk_buff *skb;

	if (!ndev || !netif_running(ndev))
		return;

	if (desc->flags & VWIFI_RX_F_RAW) {
		/* Monitor-mode capture. Delivering these means a radiotap
		 * header and a monitor interface; neither exists yet. */
		ndev->stats.rx_dropped++;
		return;
	}

	if (desc->frame_len < ETH_HLEN) {
		ndev->stats.rx_length_errors++;
		return;
	}

	skb = netdev_alloc_skb(ndev, desc->frame_len + NET_IP_ALIGN);
	if (!skb) {
		ndev->stats.rx_dropped++;
		return;
	}

	skb_reserve(skb, NET_IP_ALIGN);
	skb_put_data(skb, data, desc->frame_len);

	skb->protocol = eth_type_trans(skb, ndev);
	skb->ip_summed = CHECKSUM_UNNECESSARY;

	ndev->stats.rx_packets++;
	ndev->stats.rx_bytes += desc->frame_len;

	netif_rx(skb);
}

static int vwifi_ndo_set_mac(struct net_device *ndev, void *addr)
{
	struct vwifi_priv *p = ndev_priv(ndev);
	struct sockaddr *sa = addr;
	int ret;

	if (!is_valid_ether_addr(sa->sa_data))
		return -EADDRNOTAVAIL;
	if (netif_running(ndev))
		return -EBUSY;

	ret = vwifi_ctrl_cmd(p, VWIFI_OP_SET_STA_MAC, sa->sa_data, ETH_ALEN,
			     NULL, 0, NULL);
	if (ret)
		return ret;

	eth_hw_addr_set(ndev, sa->sa_data);
	return 0;
}

const struct net_device_ops vwifi_netdev_ops = {
	.ndo_open		= vwifi_ndo_open,
	.ndo_stop		= vwifi_ndo_stop,
	.ndo_start_xmit		= vwifi_ndo_start_xmit,
	.ndo_set_mac_address	= vwifi_ndo_set_mac,
	.ndo_validate_addr	= eth_validate_addr,
};
