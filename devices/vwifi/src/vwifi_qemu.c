/*
 * vwifi-virt — QEMU backend
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * The QEMU integration file. Everything QEMU-specific lives here:
 * PCIDevice boilerplate, property parsing, BAR/MSI-X registration,
 * chardev callbacks, and the concrete vwifi_backend_ops instance
 * that adapts QEMU's DMA/IRQ/logging APIs to the portable device.
 */

/*
 * Targets QEMU 10.x, like the vwifi-ath9k device alongside it. Several
 * of the includes and idioms below changed in QEMU 10.0 and are not
 * backward compatible: qdev-properties*.h moved under hw/core/,
 * class_init callbacks take a const void *, Property arrays are const
 * and unterminated, and DeviceClass::reset is gone.
 */
#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "qemu/error-report.h"
#include "qemu/main-loop.h"
#include "qemu/timer.h"
#include "qapi/error.h"
#include "hw/pci/pci.h"
/* pci_dma_read/write live here, not in the DMA header -- which is also
 * why we do not include that header at all: it moved from sysemu/dma.h
 * to system/dma.h in QEMU 10.0, and pci_device.h spans the rename. */
#include "hw/pci/pci_device.h"
#include "hw/pci/msix.h"
#include "hw/core/qdev-properties.h"
#include "hw/core/qdev-properties-system.h"
#include "chardev/char-fe.h"
#include "migration/vmstate.h"
#include "net/net.h"        /* MACAddr, as DEFINE_PROP_MACADDR requires */

#include "vwifi_abi.h"
#include "vwifi_device.h"
#include "vwifi_backend.h"

#define TYPE_VWIFI_VIRT "vwifi-virt"
OBJECT_DECLARE_SIMPLE_TYPE(VWifiState, VWIFI_VIRT)

/* State kept in QEMU space; embeds the portable device by value. */
struct VWifiState {
    PCIDevice     parent_obj;

    MemoryRegion  mmio;
    /* CharBackend up to QEMU 10.1; renamed CharFrontend in 10.2, which
     * is the name that matches what this side of the chardev actually
     * is. Every qemu_chr_fe_* call below takes one of these. */
    CharFrontend  chr;
    char         *node_id;
    bool          verbose;
    /* MACAddr, not a bare uint8_t[6]: DEFINE_PROP_MACADDR type-checks
     * the field against MACAddr and will not compile against an array. */
    MACAddr       default_mac;

    /* One-shot timer backing the device's scan dwell progression. */
    QEMUTimer    *scan_timer;

    /* Portable device. Allocated dynamically since its size is
     * exposed only via vwifi_dev_sizeof() to keep vwifi_device.c
     * changes from forcing header rebuilds. */
    struct vwifi_dev *dev;
};

/* ============================================================
 * Backend vtable implementations — adapters onto QEMU APIs
 * ============================================================ */

static int qemu_be_dma_read(void *be, uint64_t gpa, void *buf, size_t len)
{
    VWifiState *s = be;
    return pci_dma_read(&s->parent_obj, gpa, buf, len) == MEMTX_OK ? 0 : -1;
}

static int qemu_be_dma_write(void *be, uint64_t gpa, const void *buf, size_t len)
{
    VWifiState *s = be;
    return pci_dma_write(&s->parent_obj, gpa, buf, len) == MEMTX_OK ? 0 : -1;
}

static void qemu_be_raise_irq(void *be, unsigned vec)
{
    VWifiState *s = be;
    if (msix_enabled(&s->parent_obj)) {
        msix_notify(&s->parent_obj, vec);
    } else {
        pci_set_irq(&s->parent_obj, 1);
    }
}

static int qemu_be_medium_send(void *be, const void *buf, size_t len)
{
    VWifiState *s = be;
    const uint8_t *p = buf;
    size_t remaining = len;
    while (remaining) {
        int n = qemu_chr_fe_write(&s->chr, p, remaining);
        if (n < 0) {
            if (errno == EAGAIN || errno == EINTR) continue;
            return -1;
        }
        if (n == 0) return -1;
        p += n; remaining -= n;
    }
    return 0;
}

/* Same annotation the vtable member carries (see vwifi_backend.h): fmt
 * is argument 4, and the trailing 0 says the varargs arrive as a
 * va_list. QEMU builds with -Wmissing-format-attribute -Werror, which
 * rejects both an unannotated forwarder and an unannotated pointer. */
static void VWIFI_PRINTF_FMT(4, 0)
qemu_be_log(void *be, const struct vwifi_dev *dev,
            enum vwifi_log_level level, const char *fmt, va_list ap)
{
    VWifiState *s = be;
    (void)dev;

    /* Prefix with node_id for grep-ability. */
    char msg[512];
    vsnprintf(msg, sizeof(msg), fmt, ap);

    switch (level) {
    case VWIFI_LOG_ERR:
    case VWIFI_LOG_WARN:
        qemu_log_mask(LOG_GUEST_ERROR, "vwifi-virt[%s]: %s\n",
                      s->node_id ? s->node_id : "?", msg);
        break;
    case VWIFI_LOG_INFO:
    case VWIFI_LOG_TRACE:
    default:
        qemu_log("vwifi-virt[%s]: %s\n",
                 s->node_id ? s->node_id : "?", msg);
        break;
    }
}

static uint64_t qemu_be_now_us(void *be)
{
    (void)be;
    return (uint64_t)qemu_clock_get_us(QEMU_CLOCK_VIRTUAL);
}

/* One-shot timer backing the scan dwell state machine. */
static void qemu_vwifi_timer_cb(void *opaque)
{
    VWifiState *s = opaque;
    vwifi_timer_expired(s->dev);
}

static void qemu_be_timer_mod(void *be, uint32_t ms)
{
    VWifiState *s = be;
    timer_mod(s->scan_timer,
              qemu_clock_get_ms(QEMU_CLOCK_VIRTUAL) + (int64_t)ms);
}

static void qemu_be_timer_del(void *be)
{
    VWifiState *s = be;
    timer_del(s->scan_timer);
}

static const struct vwifi_backend_ops qemu_backend_ops = {
    .dma_read    = qemu_be_dma_read,
    .dma_write   = qemu_be_dma_write,
    .raise_irq   = qemu_be_raise_irq,
    .medium_send = qemu_be_medium_send,
    .log         = qemu_be_log,
    .now_us      = qemu_be_now_us,
    .timer_mod   = qemu_be_timer_mod,
    .timer_del   = qemu_be_timer_del,
};

/* ============================================================
 * MMIO shim — adapts QEMU's MemoryRegionOps to vwifi_reg_read/write
 * ============================================================ */

static uint64_t vwifi_mmio_read(void *opaque, hwaddr addr, unsigned size)
{
    VWifiState *s = opaque;
    return vwifi_reg_read(s->dev, (uint32_t)addr, size);
}

static void vwifi_mmio_write(void *opaque, hwaddr addr, uint64_t data,
                             unsigned size)
{
    VWifiState *s = opaque;
    vwifi_reg_write(s->dev, (uint32_t)addr, data, size);
}

static const MemoryRegionOps vwifi_mmio_ops = {
    .read  = vwifi_mmio_read,
    .write = vwifi_mmio_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 1, .max_access_size = 4 },
    .impl  = { .min_access_size = 1, .max_access_size = 4 },
};

/* ============================================================
 * Chardev callbacks — forward hub-socket bytes into the device
 * ============================================================ */

static int vwifi_chr_can_read(void *opaque)
{
    VWifiState *s = opaque;
    return (int)vwifi_medium_rx_space(s->dev);
}

static void vwifi_chr_read(void *opaque, const uint8_t *buf, int size)
{
    VWifiState *s = opaque;
    vwifi_medium_rx_bytes(s->dev, buf, (size_t)size);
}

static void vwifi_chr_event(void *opaque, QEMUChrEvent ev)
{
    VWifiState *s = opaque;
    switch (ev) {
    case CHR_EVENT_OPENED: vwifi_medium_link_event(s->dev, true);  break;
    case CHR_EVENT_CLOSED: vwifi_medium_link_event(s->dev, false); break;
    default: break;
    }
}

/* ============================================================
 * PCI lifecycle
 * ============================================================ */

/* const, and no DEFINE_PROP_END_OF_LIST: QEMU 10.0 removed the
 * terminator and device_class_set_props() now takes the array length
 * from ARRAY_SIZE(). */
static const Property vwifi_properties[] = {
    DEFINE_PROP_CHR("chardev", VWifiState, chr),
    DEFINE_PROP_STRING("node_id", VWifiState, node_id),
    DEFINE_PROP_BOOL("verbose", VWifiState, verbose, false),
    DEFINE_PROP_MACADDR("mac", VWifiState, default_mac),
};

static void vwifi_realize(PCIDevice *pdev, Error **errp)
{
    VWifiState *s = VWIFI_VIRT(pdev);
    uint8_t *config = pdev->config;
    Error *local_err = NULL;

    /* Default MAC derivation. */
    if ((s->default_mac.a[0] | s->default_mac.a[1] | s->default_mac.a[2] |
         s->default_mac.a[3] | s->default_mac.a[4] | s->default_mac.a[5]) == 0) {
        s->default_mac.a[0] = 0x00;
        s->default_mac.a[1] = 0x03;
        s->default_mac.a[2] = 0x7F;
        s->default_mac.a[3] = 0xCC;
        s->default_mac.a[4] = 0xDD;
        s->default_mac.a[5] = 0x10 + (PCI_FUNC(pdev->devfn) & 0xF);
    }

    pci_config_set_interrupt_pin(config, 1);

    memory_region_init_io(&s->mmio, OBJECT(s), &vwifi_mmio_ops, s,
                          "vwifi-mmio", VWIFI_MMIO_SIZE);
    pci_register_bar(pdev, 0, PCI_BASE_ADDRESS_SPACE_MEMORY, &s->mmio);

    if (msix_init_exclusive_bar(pdev, VWIFI_NUM_VECTORS, 1, &local_err) < 0) {
        error_propagate(errp, local_err);
        return;
    }
    for (unsigned v = 0; v < VWIFI_NUM_VECTORS; v++) {
        msix_vector_use(pdev, v);
    }

    /* Construct the portable device. The scan timer must exist before
     * vwifi_dev_init, since device reset calls timer_del. */
    s->scan_timer = timer_new_ms(QEMU_CLOCK_VIRTUAL, qemu_vwifi_timer_cb, s);

    s->dev = g_malloc0(vwifi_dev_sizeof());
    vwifi_dev_init(s->dev, &qemu_backend_ops, s,
                   s->node_id, s->verbose, s->default_mac.a);
    if (qemu_chr_fe_backend_connected(&s->chr)) {
        vwifi_medium_link_event(s->dev, true);
    }

    if (qemu_chr_fe_backend_connected(&s->chr)) {
        qemu_chr_fe_set_handlers(&s->chr,
                                 vwifi_chr_can_read, vwifi_chr_read,
                                 vwifi_chr_event, NULL, s, NULL, true);
    }
}

static void vwifi_unrealize(PCIDevice *pdev)
{
    VWifiState *s = VWIFI_VIRT(pdev);

    if (qemu_chr_fe_backend_connected(&s->chr)) {
        qemu_chr_fe_deinit(&s->chr, true);
    }
    for (unsigned v = 0; v < VWIFI_NUM_VECTORS; v++) {
        msix_vector_unuse(pdev, v);
    }
    msix_uninit_exclusive_bar(pdev);

    if (s->scan_timer) {
        timer_free(s->scan_timer);
        s->scan_timer = NULL;
    }
    g_free(s->dev);
    s->dev = NULL;
}

static void vwifi_pci_reset(DeviceState *dev)
{
    VWifiState *s = VWIFI_VIRT(dev);
    if (s->dev) {
        vwifi_device_reset(s->dev,
                           qemu_chr_fe_backend_connected(&s->chr));
    }
}

static const VMStateDescription vwifi_vmstate = {
    .name = "vwifi-virt",
    .unmigratable = 1,
};

static void vwifi_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    PCIDeviceClass *k = PCI_DEVICE_CLASS(klass);

    k->realize   = vwifi_realize;
    k->exit      = vwifi_unrealize;
    k->vendor_id = VWIFI_PCI_VENDOR_ID;
    k->device_id = VWIFI_PCI_DEVICE_ID;
    k->revision  = VWIFI_PCI_REVISION;
    k->class_id  = VWIFI_PCI_CLASS;

    dc->desc  = "Paravirtual virtual-Wi-Fi adapter";
    dc->vmsd  = &vwifi_vmstate;
    /* DeviceClass::reset was removed in QEMU 10.0. This helper keeps the
     * plain (DeviceState *) reset signature; converting to the
     * Resettable phases would be a behavioural change, not a build fix. */
    device_class_set_legacy_reset(dc, vwifi_pci_reset);
    device_class_set_props(dc, vwifi_properties);
    set_bit(DEVICE_CATEGORY_NETWORK, dc->categories);
}

static const TypeInfo vwifi_info = {
    .name          = TYPE_VWIFI_VIRT,
    .parent        = TYPE_PCI_DEVICE,
    .instance_size = sizeof(VWifiState),
    .class_init    = vwifi_class_init,
    /* Left non-const deliberately: it converts to a const-qualified
     * field if TypeInfo has one, and still compiles if it does not.
     * The reverse is not true. Same form as the ath9k device. */
    .interfaces    = (InterfaceInfo[]) {
        { INTERFACE_CONVENTIONAL_PCI_DEVICE },
        { },
    },
};

static void vwifi_register_types(void)
{
    type_register_static(&vwifi_info);
}

type_init(vwifi_register_types)
