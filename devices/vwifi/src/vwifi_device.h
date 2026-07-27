/*
 * vwifi-virt — public device interface
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Backends (QEMU, vfio-user, unit-test mock) include this header
 * to construct and drive a vwifi_dev. The struct is opaque here;
 * backends that need to embed one by value use vwifi_dev_sizeof()
 * to allocate space, then treat it as an opaque handle.
 */

#ifndef VWIFI_DEVICE_H
#define VWIFI_DEVICE_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#include "vwifi_backend.h"

struct vwifi_dev;   /* opaque */

/* Return the size of struct vwifi_dev so backends can allocate. */
size_t vwifi_dev_sizeof(void);

/* Initialize a vwifi_dev in place. `d` must point to at least
 * vwifi_dev_sizeof() bytes of storage. `default_mac` is used as the
 * initial STA MAC and reported in GET_CAPS until the driver changes
 * it. `node_id` is used in the hub hello handshake. */
void vwifi_dev_init(struct vwifi_dev *d,
                    const struct vwifi_backend_ops *ops, void *be,
                    const char *node_id, bool verbose,
                    const uint8_t default_mac[6]);

/* Full device reset. Clears rings, MMIO shadow, op mode, filters.
 * link_up preserves the STATUS_LINK_UP bit across the reset. */
void vwifi_device_reset(struct vwifi_dev *d, bool link_up);

/* Guest MMIO accesses. Called from the backend's MMIO r/w callbacks. */
uint64_t vwifi_reg_read(struct vwifi_dev *d, uint32_t offset, unsigned size);
void     vwifi_reg_write(struct vwifi_dev *d, uint32_t offset,
                         uint64_t data, unsigned size);

/* Hub socket integration. */
void   vwifi_medium_rx_bytes(struct vwifi_dev *d, const void *buf, size_t len);
size_t vwifi_medium_rx_space(const struct vwifi_dev *d);
void   vwifi_medium_link_event(struct vwifi_dev *d, bool link_up);
void   vwifi_medium_hello(struct vwifi_dev *d);

/* Called by the backend when the one-shot timer armed via
 * ops->timer_mod() fires. Drives the scan state machine's channel
 * dwell progression. */
void vwifi_timer_expired(struct vwifi_dev *d);

/* Async event helper used by later phases (SCAN, CONNECT results). */
void vwifi_post_event(struct vwifi_dev *d, uint16_t event_code,
                      const void *payload, uint32_t payload_len);

#endif /* VWIFI_DEVICE_H */
