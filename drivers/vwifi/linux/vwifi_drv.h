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
/* struct vwifi_channel's flags field is defined in terms of the medium
 * protocol's VWIFI_CHAN_FLAG_*, so the driver needs that header too.
 * Both are kernel-safe and share no symbols. */
#include "vwifi.h"

#define VWIFI_DRV_NAME "vwifi"

/* Must match PACKAGE_VERSION in dkms.conf. dkms-install.sh refuses to
 * stage the driver if the two disagree. */
#define VWIFI_DRV_VERSION "1.0.0"

/*
 * Ring sizes. All must be powers of two — the device masks indices
 * rather than comparing against a length.
 */
#define VWIFI_CTRL_REQ_RING_ENTRIES  16
#define VWIFI_CTRL_RSP_RING_ENTRIES  32
/* 64 x 2048 = 128 KiB of coherent memory per data ring. Larger rings
 * mean larger contiguous allocations, which start failing on a
 * fragmented long-lived guest; this is already far more than a single
 * STA data path keeps in flight. */
#define VWIFI_TX_RING_ENTRIES        64
#define VWIFI_RX_RING_ENTRIES        64

/*
 * Per-slot buffer sizes.
 *
 * Request and response are sized differently on purpose. The device
 * reads a control request into a 2048-byte stack buffer and rejects
 * anything larger outright, so offering more would only produce
 * requests it refuses. Responses have no such limit and a BSS_FOUND
 * event carries a whole beacon rather than just its IE tail, so the
 * response slot is frame-sized.
 */
#define VWIFI_CTRL_REQ_PAYLOAD_SIZE  2048	/* must match the device */
#define VWIFI_CTRL_RSP_PAYLOAD_SIZE  4096
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
	/* The handshake fields below are written by the submitter and read
	 * by whichever context drains the response ring, so they are
	 * guarded by ring_lock rather than by ctrl_lock -- those are
	 * different locks and ctrl_lock alone excludes nothing here. */
	u32			 ctrl_pending_id;	/* id we are waiting on */
	s32			 ctrl_status;		/* device status, once done */
	u32			 ctrl_rsp_len;		/* payload bytes reported */
	void			*ctrl_rsp_buf;		/* where to copy payload */
	u32			 ctrl_rsp_cap;		/* ...and how much fits */

	struct work_struct	 rsp_work;	/* rsp ring -> process context */

	/* Channel/band description handed to cfg80211. Must outlive
	 * wiphy_register(), so it lives here rather than on the stack. */
	struct ieee80211_channel *channels_2ghz;
	int			 n_channels_2ghz;
	struct ieee80211_channel *channels_5ghz;
	int			 n_channels_5ghz;
	struct ieee80211_supported_band band_2ghz;
	struct ieee80211_supported_band band_5ghz;

	/*
	 * Scan in progress, if any. Guarded by ring_lock: the response
	 * work runs on the system workqueue and does NOT hold the wiphy
	 * lock, so it is not serialized against the cfg80211 callbacks.
	 * Always take it via vwifi_scan_claim() so exactly one context
	 * can complete a given request.
	 */
	struct cfg80211_scan_request *scan_req;
	/* Backstop: if SCAN_COMPLETE is ever lost, scan_req would stay set
	 * and every future scan would return -EBUSY forever. */
	struct delayed_work	 scan_timeout;

	/* Connection state, for cfg80211_connect_result / _disconnected. */
	bool			 connected;
	u8			 bssid[ETH_ALEN];

	struct vwifi_caps	 caps;
	bool			 rings_enabled;
	/* Monitor mode: changes the netdev link type and the meaning of
	 * everything on the TX and RX rings, so it is tracked here rather
	 * than inferred from wdev->iftype at each use. */
	bool			 monitor;
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
struct cfg80211_scan_request *vwifi_scan_claim(struct vwifi_priv *p);
void vwifi_scan_timeout_fn(struct work_struct *work);
int  vwifi_cfg80211_init(struct vwifi_priv *p);
void vwifi_cfg80211_deinit(struct vwifi_priv *p);
void vwifi_handle_event(struct vwifi_priv *p, u16 event,
			const void *payload, u32 len);

/* vwifi_monitor.c */
bool vwifi_monitor_rx(struct vwifi_priv *p, const struct vwifi_rx_desc *desc,
		      const void *data);
int  vwifi_monitor_strip_radiotap(const struct sk_buff *skb);
int  vwifi_monitor_set_mode(struct vwifi_priv *p, bool monitor);

/* vwifi_net.c */
extern const struct net_device_ops vwifi_netdev_ops;
void vwifi_rx_frame(struct vwifi_priv *p, const struct vwifi_rx_desc *desc,
		    const void *data);

#endif /* VWIFI_DRV_H */
