// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * vwifi — Linux full-MAC driver for the vwifi-virt QEMU device
 *
 * PCI attach, MMIO, MSI-X, the four DMA rings, and the control
 * transport that everything else is built on.
 *
 * Ring ownership, which is the thing to get right:
 *
 *   ctrl req   driver produces. Fill descriptor, set OWN, ring the
 *              doorbell. Device clears OWN when it has consumed it.
 *   ctrl rsp   device produces into slots the DRIVER pre-arms with a
 *              payload buffer and OWN set. The device only writes to a
 *              slot that has OWN set, and clears it. So "OWN clear" is
 *              how the driver recognises a completed response — which
 *              also means every consumed slot must be re-armed.
 *   tx         driver produces, same shape as ctrl req.
 *   rx         driver pre-arms with a buffer and OWN set; device fills
 *              and clears OWN, exactly like ctrl rsp.
 *
 * Responses and asynchronous events share the ctrl rsp ring; an event
 * is flagged VWIFI_RSP_F_EVENT and carries req_id 0.
 */

#include <linux/module.h>
#include <linux/pci.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/etherdevice.h>

#include "vwifi_drv.h"

/* ============================================================
 * Ring allocation
 * ============================================================ */

static int ring_alloc(struct vwifi_priv *p, struct vwifi_ring *r,
		      size_t desc_size, u32 entries, u32 buf_size)
{
	r->desc_size = desc_size;
	r->entries   = entries;
	r->mask      = entries - 1;
	r->buf_size  = buf_size;
	r->head      = 0;

	r->desc = dma_alloc_coherent(p->dev, desc_size * entries,
				     &r->desc_dma, GFP_KERNEL);
	if (!r->desc)
		return -ENOMEM;

	if (buf_size) {
		r->bufs = dma_alloc_coherent(p->dev, (size_t)buf_size * entries,
					     &r->bufs_dma, GFP_KERNEL);
		if (!r->bufs) {
			dma_free_coherent(p->dev, desc_size * entries,
					  r->desc, r->desc_dma);
			r->desc = NULL;
			return -ENOMEM;
		}
	}
	return 0;
}

static void ring_free(struct vwifi_priv *p, struct vwifi_ring *r)
{
	if (r->desc) {
		dma_free_coherent(p->dev, r->desc_size * r->entries,
				  r->desc, r->desc_dma);
		r->desc = NULL;
	}
	if (r->bufs) {
		dma_free_coherent(p->dev, (size_t)r->buf_size * r->entries,
				  r->bufs, r->bufs_dma);
		r->bufs = NULL;
	}
}

static void *ring_desc(struct vwifi_ring *r, u32 idx)
{
	return (u8 *)r->desc + (size_t)(idx & r->mask) * r->desc_size;
}

static void *ring_buf(struct vwifi_ring *r, u32 idx)
{
	return (u8 *)r->bufs + (size_t)(idx & r->mask) * r->buf_size;
}

static dma_addr_t ring_buf_dma(struct vwifi_ring *r, u32 idx)
{
	return r->bufs_dma + (dma_addr_t)(idx & r->mask) * r->buf_size;
}

int vwifi_rings_alloc(struct vwifi_priv *p)
{
	int ret;

	ret = ring_alloc(p, &p->ctrl_req, sizeof(struct vwifi_ctrl_req_desc),
			 VWIFI_CTRL_REQ_RING_ENTRIES, VWIFI_CTRL_REQ_PAYLOAD_SIZE);
	if (ret)
		return ret;

	ret = ring_alloc(p, &p->ctrl_rsp, sizeof(struct vwifi_ctrl_rsp_desc),
			 VWIFI_CTRL_RSP_RING_ENTRIES, VWIFI_CTRL_RSP_PAYLOAD_SIZE);
	if (ret)
		goto err_rsp;

	ret = ring_alloc(p, &p->tx, sizeof(struct vwifi_tx_desc),
			 VWIFI_TX_RING_ENTRIES, VWIFI_TX_BUF_SIZE);
	if (ret)
		goto err_tx;

	ret = ring_alloc(p, &p->rx, sizeof(struct vwifi_rx_desc),
			 VWIFI_RX_RING_ENTRIES, VWIFI_RX_BUF_SIZE);
	if (ret)
		goto err_rx;

	return 0;

err_rx:
	ring_free(p, &p->tx);
err_tx:
	ring_free(p, &p->ctrl_rsp);
err_rsp:
	ring_free(p, &p->ctrl_req);
	return ret;
}

void vwifi_rings_free(struct vwifi_priv *p)
{
	ring_free(p, &p->rx);
	ring_free(p, &p->tx);
	ring_free(p, &p->ctrl_rsp);
	ring_free(p, &p->ctrl_req);
}

/* Pre-arm the two device-produced rings and program all four into MMIO. */
void vwifi_rings_arm(struct vwifi_priv *p)
{
	u32 i;

	memset(p->ctrl_req.desc, 0,
	       p->ctrl_req.desc_size * p->ctrl_req.entries);
	memset(p->tx.desc, 0, p->tx.desc_size * p->tx.entries);

	for (i = 0; i < p->ctrl_rsp.entries; i++) {
		struct vwifi_ctrl_rsp_desc *d = ring_desc(&p->ctrl_rsp, i);

		memset(d, 0, sizeof(*d));
		d->payload_addr = ring_buf_dma(&p->ctrl_rsp, i);
		d->payload_len  = p->ctrl_rsp.buf_size;
		d->flags        = VWIFI_DESC_F_OWN;
	}


	for (i = 0; i < p->rx.entries; i++) {
		struct vwifi_rx_desc *d = ring_desc(&p->rx, i);

		memset(d, 0, sizeof(*d));
		d->frame_addr = ring_buf_dma(&p->rx, i);
		d->buffer_len = p->rx.buf_size;
		d->flags      = VWIFI_DESC_F_OWN;
	}

	p->ctrl_req.head = 0;
	p->ctrl_rsp.head = 0;
	p->tx.head = 0;
	p->rx.head = 0;

	vwifi_wr(p, VWIFI_REG_CTRL_REQ_RING_BASE_LO,
		 lower_32_bits(p->ctrl_req.desc_dma));
	vwifi_wr(p, VWIFI_REG_CTRL_REQ_RING_BASE_HI,
		 upper_32_bits(p->ctrl_req.desc_dma));
	vwifi_wr(p, VWIFI_REG_CTRL_REQ_RING_SIZE, p->ctrl_req.entries);

	vwifi_wr(p, VWIFI_REG_CTRL_RSP_RING_BASE_LO,
		 lower_32_bits(p->ctrl_rsp.desc_dma));
	vwifi_wr(p, VWIFI_REG_CTRL_RSP_RING_BASE_HI,
		 upper_32_bits(p->ctrl_rsp.desc_dma));
	vwifi_wr(p, VWIFI_REG_CTRL_RSP_RING_SIZE, p->ctrl_rsp.entries);

	vwifi_wr(p, VWIFI_REG_TX_RING_BASE_LO, lower_32_bits(p->tx.desc_dma));
	vwifi_wr(p, VWIFI_REG_TX_RING_BASE_HI, upper_32_bits(p->tx.desc_dma));
	vwifi_wr(p, VWIFI_REG_TX_RING_SIZE, p->tx.entries);

	vwifi_wr(p, VWIFI_REG_RX_RING_BASE_LO, lower_32_bits(p->rx.desc_dma));
	vwifi_wr(p, VWIFI_REG_RX_RING_BASE_HI, upper_32_bits(p->rx.desc_dma));
	vwifi_wr(p, VWIFI_REG_RX_RING_SIZE, p->rx.entries);

	wmb();	/* rings visible to the device before it is enabled */
	vwifi_wr(p, VWIFI_REG_CTRL, VWIFI_CTRL_ENABLE | VWIFI_CTRL_IRQ_ENABLE);
	p->rings_enabled = true;
}

/* ============================================================
 * Control response ring — responses and events
 * ============================================================ */

static void rsp_ring_process(struct vwifi_priv *p)
{
	unsigned int guard = p->ctrl_rsp.entries;

	/* Called both inline from vwifi_ctrl_cmd() and from the workqueue;
	 * two drainers walking the same ring would corrupt it. */
	mutex_lock(&p->rsp_lock);

	while (guard--) {
		struct vwifi_ctrl_rsp_desc *d;
		u16 opcode, flags;
		u32 payload_len, idx, req_id;
		s32 status;
		void *payload;

		idx = p->ctrl_rsp.head & p->ctrl_rsp.mask;
		d = ring_desc(&p->ctrl_rsp, idx);

		/* Device clears OWN when it fills a slot. Still set means
		 * nothing new here. */
		if (READ_ONCE(d->flags) & VWIFI_DESC_F_OWN)
			break;

		/* Do not let the rest of the descriptor, or the payload, be
		 * read before the OWN bit that publishes them. Coherent
		 * memory guarantees visibility, not ordering. */
		dma_rmb();

		opcode      = d->opcode;
		flags       = d->flags;
		status      = d->status;
		req_id      = d->req_id;
		payload_len = d->payload_len;
		payload     = ring_buf(&p->ctrl_rsp, idx);

		if (payload_len > p->ctrl_rsp.buf_size)
			payload_len = p->ctrl_rsp.buf_size;

		if (flags & VWIFI_RSP_F_EVENT) {
			vwifi_handle_event(p, opcode, payload, payload_len);
		} else {
			bool matched = false;
			unsigned long lflags;
			u32 copy;

			/*
			 * The submitter publishes the pending id and its
			 * destination buffer under ring_lock; match and copy
			 * under the same lock so a request that has already
			 * timed out cannot have its buffer written after the
			 * submitter walked away from it.
			 */
			spin_lock_irqsave(&p->ring_lock, lflags);
			if (req_id && req_id == p->ctrl_pending_id) {
				matched = true;
				p->ctrl_status  = status;
				p->ctrl_rsp_len = payload_len;
				/*
				 * payload_len is device-controlled. Clamping
				 * it to the ring buffer is not enough: the
				 * destination is the CALLER's buffer, which
				 * is usually much smaller. Without this the
				 * device can overrun it at will.
				 */
				copy = min(payload_len, p->ctrl_rsp_cap);
				if (p->ctrl_rsp_buf && copy)
					memcpy(p->ctrl_rsp_buf, payload, copy);
			}
			spin_unlock_irqrestore(&p->ring_lock, lflags);

			if (matched)
				complete(&p->ctrl_done);
			else
				dev_dbg(p->dev,
					"unmatched ctrl response op=0x%04x req=%u\n",
					opcode, req_id);
		}

		/* Re-arm: the device will not reuse a slot whose OWN is
		 * clear, so a slot we forget to re-arm is a slot lost. */
		memset(d, 0, sizeof(*d));
		d->payload_addr = ring_buf_dma(&p->ctrl_rsp, idx);
		d->payload_len  = p->ctrl_rsp.buf_size;
		/* Buffer address and length must land before OWN hands the
		 * slot back, or the device can DMA to a half-written one. */
		dma_wmb();
		WRITE_ONCE(d->flags, VWIFI_DESC_F_OWN);

		p->ctrl_rsp.head++;
		vwifi_wr(p, VWIFI_REG_CTRL_RSP_RING_HEAD,
			 p->ctrl_rsp.head & p->ctrl_rsp.mask);
	}

	mutex_unlock(&p->rsp_lock);
}

/*
 * Responses and events are processed in process context, not the IRQ
 * handler: the cfg80211 calls they turn into want to be able to sleep
 * and to take the wiphy lock.
 */
static void rsp_work_fn(struct work_struct *work)
{
	struct vwifi_priv *p = container_of(work, struct vwifi_priv, rsp_work);

	rsp_ring_process(p);
}

/* ============================================================
 * Control transport
 * ============================================================ */

int vwifi_ctrl_cmd(struct vwifi_priv *p, u16 opcode,
		   const void *payload, u32 payload_len,
		   void *rsp_out, u32 rsp_cap, u32 *rsp_len)
{
	struct vwifi_ctrl_req_desc *d;
	unsigned long remaining, lflags;
	u32 idx;
	int ret = 0;
	s32 status;

	if (payload_len > p->ctrl_req.buf_size)
		return -EINVAL;
	if (!p->rings_enabled)
		return -ENODEV;

	mutex_lock(&p->ctrl_lock);

	idx = p->ctrl_req.head & p->ctrl_req.mask;
	d = ring_desc(&p->ctrl_req, idx);

	if (payload && payload_len)
		memcpy(ring_buf(&p->ctrl_req, idx), payload, payload_len);

	/* Arm the completion BEFORE the doorbell. The device services the
	 * doorbell synchronously from the vCPU thread and raises MSI-X
	 * before the write returns, so the response can be processed the
	 * instant we ring — arming afterwards would race and hang. */
	reinit_completion(&p->ctrl_done);

	spin_lock_irqsave(&p->ring_lock, lflags);
	p->ctrl_pending_id = ++p->ctrl_req_id;
	p->ctrl_rsp_buf    = rsp_out;
	p->ctrl_rsp_cap    = rsp_out ? rsp_cap : 0;
	p->ctrl_status     = 0;
	p->ctrl_rsp_len    = 0;
	spin_unlock_irqrestore(&p->ring_lock, lflags);

	memset(d, 0, sizeof(*d));
	d->opcode       = opcode;
	d->req_id       = p->ctrl_pending_id;
	d->payload_addr = payload_len ? ring_buf_dma(&p->ctrl_req, idx) : 0;
	d->payload_len  = payload_len;
	dma_wmb();	/* descriptor body before the OWN that publishes it */
	WRITE_ONCE(d->flags, VWIFI_DESC_F_OWN);

	wmb();	/* descriptor complete before the doorbell */
	p->ctrl_req.head++;
	vwifi_wr(p, VWIFI_REG_CTRL_REQ_RING_DOORBELL,
		 p->ctrl_req.head & p->ctrl_req.mask);

	/*
	 * The response is typically already in memory by the time the
	 * doorbell write retires. Drain the ring here rather than waiting
	 * for the workqueue, so the common case costs no scheduling.
	 */
	rsp_ring_process(p);

	remaining = wait_for_completion_timeout(
			&p->ctrl_done,
			msecs_to_jiffies(VWIFI_CTRL_TIMEOUT_MS));
	if (!remaining) {
		dev_err(p->dev, "control opcode 0x%04x timed out\n", opcode);
		ret = -ETIMEDOUT;
		goto out;
	}

	spin_lock_irqsave(&p->ring_lock, lflags);
	status = p->ctrl_status;
	if (rsp_len)
		*rsp_len = min(p->ctrl_rsp_len, rsp_cap);
	spin_unlock_irqrestore(&p->ring_lock, lflags);

	/*
	 * Pass the device's errno through rather than flattening it. The
	 * device returns -EBUSY for "a scan or connect is already in
	 * flight", which is the difference between a supplicant retrying
	 * and giving up.
	 */
	if (status < 0)
		ret = status;

out:
	spin_lock_irqsave(&p->ring_lock, lflags);
	p->ctrl_pending_id = 0;
	p->ctrl_rsp_buf    = NULL;
	p->ctrl_rsp_cap    = 0;
	spin_unlock_irqrestore(&p->ring_lock, lflags);
	mutex_unlock(&p->ctrl_lock);
	return ret;
}

/* ============================================================
 * RX data ring
 * ============================================================ */

void vwifi_rx_poll(struct vwifi_priv *p)
{
	unsigned int budget = p->rx.entries;

	while (budget--) {
		struct vwifi_rx_desc *d;
		u32 idx = p->rx.head & p->rx.mask;
		struct vwifi_rx_desc snapshot;

		d = ring_desc(&p->rx, idx);
		if (READ_ONCE(d->flags) & VWIFI_DESC_F_OWN)
			break;

		dma_rmb();	/* descriptor and frame after the OWN test */
		snapshot = *d;
		if (snapshot.frame_len && snapshot.frame_len <= p->rx.buf_size)
			vwifi_rx_frame(p, &snapshot, ring_buf(&p->rx, idx));

		/* Re-post the buffer. */
		memset(d, 0, sizeof(*d));
		d->frame_addr = ring_buf_dma(&p->rx, idx);
		d->buffer_len = p->rx.buf_size;
		dma_wmb();	/* address and length before OWN */
		WRITE_ONCE(d->flags, VWIFI_DESC_F_OWN);

		p->rx.head++;
		vwifi_wr(p, VWIFI_REG_RX_RING_HEAD, p->rx.head & p->rx.mask);
	}
}

/* ============================================================
 * Interrupts
 * ============================================================ */

static irqreturn_t vwifi_irq_ctrl(int irq, void *data)
{
	struct vwifi_priv *p = data;

	/* Defer to process context: this turns into cfg80211 calls. */
	schedule_work(&p->rsp_work);
	return IRQ_HANDLED;
}

static irqreturn_t vwifi_irq_rx(int irq, void *data)
{
	struct vwifi_priv *p = data;

	vwifi_rx_poll(p);
	return IRQ_HANDLED;
}

/*
 * The device currently raises only VWIFI_VEC_CTRL_RSP and VWIFI_VEC_RX.
 * TX_COMPLETE and EVENT are allocated in the ABI and handled here so
 * that a device which starts raising them does not hit an unhandled
 * vector, but neither fires today -- which is exactly why the transmit
 * path must not depend on a completion interrupt (see vwifi_net.c).
 */
static irqreturn_t vwifi_irq_noop(int irq, void *data)
{
	return IRQ_HANDLED;
}

static void vwifi_free_irqs(struct vwifi_priv *p)
{
	int i;

	for (i = 0; i < VWIFI_NUM_VECTORS; i++)
		free_irq(pci_irq_vector(p->pdev, i), p);
	pci_free_irq_vectors(p->pdev);
}

static int vwifi_setup_irqs(struct vwifi_priv *p)
{
	static const struct {
		irq_handler_t fn;
		const char *name;
	} vecs[VWIFI_NUM_VECTORS] = {
		[VWIFI_VEC_CTRL_RSP]    = { vwifi_irq_ctrl, "vwifi-ctrl" },
		[VWIFI_VEC_RX]          = { vwifi_irq_rx,   "vwifi-rx"   },
		[VWIFI_VEC_TX_COMPLETE] = { vwifi_irq_noop, "vwifi-tx"   },
		[VWIFI_VEC_EVENT]       = { vwifi_irq_ctrl, "vwifi-event" },
	};
	int i, ret, nvec;

	nvec = pci_alloc_irq_vectors(p->pdev, VWIFI_NUM_VECTORS,
				     VWIFI_NUM_VECTORS, PCI_IRQ_MSIX);
	if (nvec < 0) {
		dev_err(p->dev, "MSI-X allocation failed: %d\n", nvec);
		return nvec;
	}

	for (i = 0; i < VWIFI_NUM_VECTORS; i++) {
		ret = request_irq(pci_irq_vector(p->pdev, i), vecs[i].fn, 0,
				  vecs[i].name, p);
		if (ret) {
			dev_err(p->dev, "request_irq %d failed: %d\n", i, ret);
			while (--i >= 0)
				free_irq(pci_irq_vector(p->pdev, i), p);
			pci_free_irq_vectors(p->pdev);
			return ret;
		}
	}
	return 0;
}

/* ============================================================
 * PCI attach / detach
 * ============================================================ */

static int vwifi_probe(struct pci_dev *pdev, const struct pci_device_id *id)
{
	struct vwifi_priv *p;
	struct wiphy *wiphy;
	u32 sig, abi;
	int ret;

	ret = pcim_enable_device(pdev);
	if (ret)
		return ret;
	pci_set_master(pdev);

	ret = dma_set_mask_and_coherent(&pdev->dev, DMA_BIT_MASK(64));
	if (ret) {
		ret = dma_set_mask_and_coherent(&pdev->dev, DMA_BIT_MASK(32));
		if (ret) {
			dev_err(&pdev->dev, "no usable DMA mask\n");
			return ret;
		}
	}

	/*
	 * The wiphy owns the private area, so it has to exist before
	 * anything else can be stashed. cfg80211 registration happens
	 * later, once the device has told us what it supports.
	 */
	wiphy = wiphy_new(vwifi_cfg80211_ops_get(), sizeof(struct vwifi_priv));
	if (!wiphy)
		return -ENOMEM;

	p = wiphy_priv(wiphy);
	p->wiphy = wiphy;
	p->pdev  = pdev;
	p->dev   = &pdev->dev;
	pci_set_drvdata(pdev, p);
	set_wiphy_dev(wiphy, &pdev->dev);

	mutex_init(&p->ctrl_lock);
	mutex_init(&p->rsp_lock);
	spin_lock_init(&p->ring_lock);
	init_completion(&p->ctrl_done);
	INIT_WORK(&p->rsp_work, rsp_work_fn);
	INIT_DELAYED_WORK(&p->scan_timeout, vwifi_scan_timeout_fn);

	ret = pcim_iomap_regions(pdev, BIT(0), VWIFI_DRV_NAME);
	if (ret) {
		dev_err(&pdev->dev, "cannot map BAR0\n");
		goto err_wiphy;
	}
	p->mmio = pcim_iomap_table(pdev)[0];

	sig = vwifi_rd(p, VWIFI_REG_SIGNATURE);
	abi = vwifi_rd(p, VWIFI_REG_ABI_VERSION);

	if (sig != VWIFI_SIGNATURE) {
		dev_err(&pdev->dev,
			"bad signature 0x%08x (expected 0x%08x)%s\n",
			sig, VWIFI_SIGNATURE,
			sig == 0xFFFFFFFFu ? " — BAR not decoding" : "");
		ret = -ENODEV;
		goto err_wiphy;
	}

	/*
	 * Refuse to bind on an ABI mismatch rather than limping along.
	 * A changed field offset compiles fine on both sides and then
	 * corrupts memory across the guest/host boundary; this check is
	 * the last line of defence against that.
	 */
	if (abi != VWIFI_ABI_VERSION) {
		dev_err(&pdev->dev,
			"ABI mismatch: device speaks %u, driver speaks %u\n",
			abi, VWIFI_ABI_VERSION);
		ret = -ENODEV;
		goto err_wiphy;
	}

	/*
	 * Put the device back to a known state before programming rings.
	 *
	 * The device latches the ring base addresses only on a 0->1
	 * transition of CTRL_ENABLE. If it is already enabled when we
	 * attach -- kexec, a re-bind, a previous driver that died without
	 * calling remove() -- writing ENABLE again is not an edge, the new
	 * bases are never read, and the device keeps DMAing into the
	 * previous incarnation's physical addresses. Which are now
	 * somebody else's memory.
	 */
	vwifi_wr(p, VWIFI_REG_CTRL, 0);
	vwifi_wr(p, VWIFI_REG_RESET, 1);

	ret = vwifi_rings_alloc(p);
	if (ret)
		goto err_wiphy;

	ret = vwifi_setup_irqs(p);
	if (ret)
		goto err_rings;

	vwifi_rings_arm(p);

	/* enable_rings() on the device side can reject a ring and carry on
	 * with it disabled; READY is how it says the rings took. */
	if (!(vwifi_rd(p, VWIFI_REG_STATUS) & VWIFI_STATUS_READY)) {
		dev_err(&pdev->dev,
			"device did not report READY after enable (status 0x%08x)\n",
			vwifi_rd(p, VWIFI_REG_STATUS));
		ret = -EIO;
		goto err_irq;
	}

	ret = vwifi_cfg80211_init(p);
	if (ret)
		goto err_irq;

	dev_info(&pdev->dev,
		 "vwifi-virt bound: ABI %u, caps 0x%08x, %s\n",
		 abi, vwifi_rd(p, VWIFI_REG_CAPS),
		 (vwifi_rd(p, VWIFI_REG_STATUS) & VWIFI_STATUS_LINK_UP) ?
			"hub link up" : "hub link DOWN");

	return 0;

err_irq:
	vwifi_wr(p, VWIFI_REG_CTRL, 0);
	p->rings_enabled = false;
	vwifi_free_irqs(p);
	/* free_irq() stops new handlers; it does not flush work one of
	 * them already queued. Without this, rsp_work_fn runs against a
	 * freed vwifi_priv and an unmapped BAR. */
	cancel_work_sync(&p->rsp_work);
	cancel_delayed_work_sync(&p->scan_timeout);
err_rings:
	vwifi_rings_free(p);
err_wiphy:
	mutex_destroy(&p->rsp_lock);
	mutex_destroy(&p->ctrl_lock);
	wiphy_free(wiphy);
	return ret;
}

static void vwifi_remove(struct pci_dev *pdev)
{
	struct vwifi_priv *p = pci_get_drvdata(pdev);

	if (!p)
		return;

	/*
	 * Order matters. unregister_netdev() frees the netdev
	 * (needs_free_netdev), and both the response work and the RX
	 * interrupt dereference p->ndev. Quiesce every producer FIRST,
	 * then tear down.
	 *
	 * Disabling the device does not close the window on its own: an
	 * interrupt raised just before the write can still be pending, and
	 * the work it queued still has to be flushed.
	 */
	vwifi_wr(p, VWIFI_REG_CTRL, 0);
	p->rings_enabled = false;

	vwifi_free_irqs(p);
	cancel_work_sync(&p->rsp_work);
	cancel_delayed_work_sync(&p->scan_timeout);

	/* Safe now: any .disconnect cfg80211 issues during unregister
	 * short-circuits on !rings_enabled. */
	vwifi_cfg80211_deinit(p);
	vwifi_rings_free(p);
	mutex_destroy(&p->rsp_lock);
	mutex_destroy(&p->ctrl_lock);
	wiphy_free(p->wiphy);
}

static const struct pci_device_id vwifi_pci_ids[] = {
	{ PCI_DEVICE(VWIFI_PCI_VENDOR_ID, VWIFI_PCI_DEVICE_ID) },
	{ 0, }
};
MODULE_DEVICE_TABLE(pci, vwifi_pci_ids);

static struct pci_driver vwifi_pci_driver = {
	.name     = VWIFI_DRV_NAME,
	.id_table = vwifi_pci_ids,
	.probe    = vwifi_probe,
	.remove   = vwifi_remove,
};

module_pci_driver(vwifi_pci_driver);

MODULE_DESCRIPTION("Full-MAC cfg80211 driver for the vwifi-virt QEMU device");
MODULE_VERSION(VWIFI_DRV_VERSION);
MODULE_LICENSE("GPL");
