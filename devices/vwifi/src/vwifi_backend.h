/*
 * vwifi-virt — backend abstraction
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Every operation the portable device logic ("device.c") needs to
 * perform against its host environment goes through the vtable
 * declared here. Two implementations exist (or are planned):
 *
 *   - vwifi_qemu.c       : native QEMU PCIDevice; uses pci_dma_*,
 *                          msix_notify, qemu_chr_fe_write, qemu_log
 *   - vwifi_vfio_user.c  : standalone libvfio-user server; uses
 *                          vfu_sgl_read/write, vfu_irq_trigger,
 *                          raw sockets, syslog/stderr
 *
 * The device logic never includes QEMU or libvfio-user headers.
 * All host interaction happens via the ops in this file.
 */

#ifndef VWIFI_BACKEND_H
#define VWIFI_BACKEND_H

#include <stdint.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stddef.h>

/* Log levels — mapped by each backend to whatever it uses. */
enum vwifi_log_level {
    VWIFI_LOG_ERR   = 0,
    VWIFI_LOG_WARN  = 1,
    VWIFI_LOG_INFO  = 2,
    VWIFI_LOG_TRACE = 3,
};

/* Forward-decl: the device state is opaque to backend impls. */
struct vwifi_dev;

/*
 * Backend operations vtable.
 *
 * All operations take a `void *be` — the backend's own state pointer,
 * which the backend supplies when it constructs the vwifi_dev. The
 * device logic treats it as opaque.
 *
 * Threading: all ops are called from the same thread that owns the
 * vwifi_dev. The backend is responsible for its own synchronization
 * if it services the device from multiple threads.
 *
 * Errors: DMA and medium_send return 0 on success, negative on failure.
 * The device logic surfaces errors as drops (bumps stats) rather than
 * aborting — a paravirtual device can't really "fail" in the middle
 * of servicing a guest.
 */
struct vwifi_backend_ops {
    /* Read `len` bytes from guest physical address `gpa` into `buf`. */
    int (*dma_read)(void *be, uint64_t gpa, void *buf, size_t len);

    /* Write `len` bytes from `buf` to guest physical address `gpa`. */
    int (*dma_write)(void *be, uint64_t gpa, const void *buf, size_t len);

    /* Assert MSI-X vector `vec` (0..VWIFI_NUM_VECTORS-1). Called only
     * after the device has determined the vector is unmasked and IRQs
     * are globally enabled — the backend need not re-check. */
    void (*raise_irq)(void *be, unsigned vec);

    /* Send `len` bytes to the hub socket. Non-blocking; returns 0 on
     * success or if partial-then-completed via internal buffering,
     * negative if the send failed (link down, etc.). */
    int (*medium_send)(void *be, const void *buf, size_t len);

    /* Log a message. Each backend routes this to its native logger.
     * `dev` is provided so the backend can prefix with node_id etc. */
    void (*log)(void *be, const struct vwifi_dev *dev,
                enum vwifi_log_level level, const char *fmt, va_list ap);

    /* Current time in microseconds from a monotonic source. Used to
     * stamp medium frames with a TSF and to age the BSS table. QEMU
     * uses QEMU_CLOCK_VIRTUAL; vfio-user uses CLOCK_MONOTONIC; the
     * mock uses a manually-advanced counter so tests are deterministic. */
    uint64_t (*now_us)(void *be);

    /* Arm (or re-arm) the device's single one-shot timer to fire in
     * `ms` milliseconds. When it fires, the backend must call
     * vwifi_timer_expired(dev). Re-arming replaces any pending
     * deadline. */
    void (*timer_mod)(void *be, uint32_t ms);

    /* Cancel any pending one-shot timer. Safe to call when none is
     * armed. */
    void (*timer_del)(void *be);
};

#endif /* VWIFI_BACKEND_H */
