/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * vwifi — Linux full-MAC driver for the vwifi-virt QEMU device
 *
 * Driver-private definitions. The wire contract lives in
 * ../../../abi/vwifi_abi.h and is shared verbatim with the device; do
 * not restate anything from it here.
 */

#ifndef VWIFI_DRV_H
#define VWIFI_DRV_H

#include <linux/pci.h>
#include <linux/netdevice.h>
#include <linux/workqueue.h>
#include <linux/completion.h>
#include <linux/mutex.h>
#include <linux/spinlock.h>
#include <net/cfg80211.h>

#include "vwifi_abi.h"

#define VWIFI_DRV_NAME "vwifi"

/*
 * Ring sizes. All must be powers of two — the device masks indices
 * rather than comparing against a length.
 */
#define VWIFI_CTRL_REQ_RING_ENTRIES  16
#define VWIFI_CTRL_RSP_RING_ENTRIES  32
#define VWIFI_TX_RING_ENTRIES        128
#define VWIFI_RX_RING_ENTRIES        128

/*
 * Per-slot buffer sizes.
 *
 * A BSS_FOUND event carries a whole beacon, not just its IE tail, so
 * the response payload has to be frame-sized rather than struct-sized.
 */
#define VWIFI_CTRL_PAYLOAD_SIZE      4096
#define VWIFI_TX_BUF_SIZE            2048
#define VWIFI_RX_BUF_SIZE            2048

/* How long to wait for a control response before giving up. The device
 * services the doorbell synchronously from the vCPU thread, so this is
 * a backstop against a wedged device, not a normal timeout. */
#define VWIFI_CTRL_TIMEOUT_MS        2000

/* A DMA-coherent ring: descriptors plus, optionally, one payload
 * buffer per descriptor carved out of a second coherent allocation. */
struct vwifi_ring {
	void		*desc;		/* descriptor array (coherent) */
	dma_addr_t	 desc_dma;
	size_t		 desc_size;	/* bytes per descriptor */
	u32		 entries;
	u32		 mask;

	void		*bufs;		/* entries × buf_size (coherent) */
	dma_addr_t	 bufs_dma;
	u32		 buf_size;

	u32		 head;		/* driver's index into the ring */
};

struct vwifi_priv {
	struct pci_dev		*pdev;
	struct device		*dev;
	void __iomem		*mmio;

	struct wiphy		*wiphy;
	struct net_device	*ndev;
	struct wireless_dev	 wdev;

	struct vwifi_ring	 ctrl_req;
	struct vwifi_ring	 ctrl_rsp;
	struct vwifi_ring	 tx;
	struct vwifi_ring	 rx;

	/* Control transport. One request in flight at a time: the device
	 * completes synchronously, so pipelining would buy nothing and
	 * cost a request-tracking table. */
	struct mutex		 ctrl_lock;	/* serializes submitters */
	/* The response ring is drained from two places -- inline after a
	 * doorbell, and from the workqueue an interrupt schedules -- so it
	 * needs a lock of its own. Always innermost. */
	struct mutex		 rsp_lock;
	spinlock_t		 ring_lock;	/* guards ring indices vs IRQ */
	struct completion	 ctrl_done;
	u32			 ctrl_req_id;	/* next id to issue */
	u32			 ctrl_pending_id;	/* id we are waiting on */
	s32			 ctrl_status;		/* result, once done */
	u32			 ctrl_rsp_len;		/* payload bytes */
	void			*ctrl_rsp_buf;		/* copy of the payload */

	struct work_struct	 rsp_work;	/* rsp ring -> process context */

	/* Channel/band description handed to cfg80211. Must outlive
	 * wiphy_register(), so it lives here rather than on the stack. */
	struct ieee80211_channel *channels_2ghz;
	int			 n_channels_2ghz;
	struct ieee80211_channel *channels_5ghz;
	int			 n_channels_5ghz;
	struct ieee80211_supported_band band_2ghz;
	struct ieee80211_supported_band band_5ghz;

	/* Scan in progress, if any. Owned by the rsp work / cfg80211
	 * callbacks, both of which run under the wiphy lock. */
	struct cfg80211_scan_request *scan_req;

	/* Connection state, for cfg80211_connect_result / _disconnected. */
	bool			 connected;
	u8			 bssid[ETH_ALEN];

	struct vwifi_caps	 caps;
	bool			 rings_enabled;
};

/* MMIO helpers. The device's registers are all 32-bit. */
static inline u32 vwifi_rd(struct vwifi_priv *p, u32 off)
{
	return ioread32(p->mmio + off);
}

static inline void vwifi_wr(struct vwifi_priv *p, u32 off, u32 val)
{
	iowrite32(val, p->mmio + off);
}

/* vwifi_main.c */
int  vwifi_ctrl_cmd(struct vwifi_priv *p, u16 opcode,
		    const void *payload, u32 payload_len,
		    void *rsp_out, u32 rsp_cap, u32 *rsp_len);
void vwifi_rx_poll(struct vwifi_priv *p);
int  vwifi_rings_alloc(struct vwifi_priv *p);
void vwifi_rings_free(struct vwifi_priv *p);
void vwifi_rings_arm(struct vwifi_priv *p);

/* vwifi_cfg80211.c */
const struct cfg80211_ops *vwifi_cfg80211_ops_get(void);
int  vwifi_cfg80211_init(struct vwifi_priv *p);
void vwifi_cfg80211_deinit(struct vwifi_priv *p);
void vwifi_handle_event(struct vwifi_priv *p, u16 event,
			const void *payload, u32 len);

/* vwifi_net.c */
extern const struct net_device_ops vwifi_netdev_ops;
void vwifi_rx_frame(struct vwifi_priv *p, const struct vwifi_rx_desc *desc,
		    const void *data);

#endif /* VWIFI_DRV_H */
