/*
 * vwifi_host_ioctl.h — control ABI for the vwifi host kernel module
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * The /dev/vwifi-ctl ioctl interface used to create and destroy host
 * radios at runtime.  This is the contract between vwifi_host.ko and
 * vwifi-ctl (and any other controller driving the module); it is NOT
 * part of the medium wire protocol and no QEMU device model needs it.
 *
 * Split out of vwifi.h so the wire protocol stays free of Linux-only
 * includes: QEMU builds on Windows and macOS hosts, where <sys/ioctl.h>
 * does not exist.
 */

#ifndef VWIFI_HOST_IOCTL_H
#define VWIFI_HOST_IOCTL_H

#ifdef __KERNEL__
#include <linux/types.h>
#include <linux/ioctl.h>
#else
#include <stdint.h>
#include <sys/ioctl.h>
#endif

/* ================================================================
 *  Control device — dynamic radio management
 * ================================================================
 *
 * The module exposes a single control node, /dev/vwifi-ctl, on which
 * userspace issues ioctls to create and destroy radios at runtime.
 * Each radio is an independent mac80211 wiphy (its own wlanX) backed by
 * its own data chardev at /dev/<name>, which one vwifi-host-relay then
 * bridges to a medium hub. This lets a controller (e.g. Nyxus) add and
 * remove host radios on demand rather than being limited to the single
 * radio created at module load.
 */
#define VWIFI_CTL_NAME           "vwifi-ctl"   /* -> /dev/vwifi-ctl */

/* Max length (incl. NUL) of a radio's data-chardev basename. The node
 * appears at /dev/<name>, so this bounds the miscdevice name. Sized to
 * hold "vwifi-" + a 32-char medium name with room to spare. */
#define VWIFI_RADIO_NAME_MAX     40

/* Max length (incl. NUL) of a wiphy name as returned to userspace
 * ("phy0", "phy17", ...). */
#define VWIFI_PHYNAME_MAX        16

/*
 * VWIFI_IOC_CREATE_RADIO — allocate and register a new radio.
 *
 *   in:  name  data-chardev basename; the node is created at /dev/<name>.
 *              Must be unique and contain only [A-Za-z0-9._-]. Required.
 *        mac   desired permanent MAC. All-zero => the driver derives a
 *              stable locally-administered MAC from `name`.
 *   out: radio_id  opaque id assigned to the radio.
 *        phy       wiphy name, e.g. "phy3" (map to the netdev via
 *                  /sys/class/net/<if>/phy80211/name to rename it).
 *        mac_out   the MAC actually assigned.
 *
 * Returns 0 on success, -EEXIST if `name` is already in use, -EINVAL on a
 * malformed name, -ENOSPC/-ENOMEM on resource exhaustion.
 */
struct vwifi_ioc_create {
    char     name[VWIFI_RADIO_NAME_MAX];   /* in  */
    uint8_t  mac[6];                       /* in  (0 => auto) */
    uint8_t  _pad0[2];
    uint32_t radio_id;                     /* out */
    char     phy[VWIFI_PHYNAME_MAX];       /* out */
    uint8_t  mac_out[6];                   /* out */
    uint8_t  _pad1[2];
};

/*
 * VWIFI_IOC_DESTROY_RADIO — unregister and free a radio by name.
 *
 * Returns 0 on success, -ENODEV if no such radio, -EBUSY if a relay still
 * holds its data chardev open (close the relay first).
 */
struct vwifi_ioc_destroy {
    char     name[VWIFI_RADIO_NAME_MAX];   /* in */
};

#define VWIFI_IOC_MAGIC          0xF7
#define VWIFI_IOC_CREATE_RADIO   _IOWR(VWIFI_IOC_MAGIC, 1, struct vwifi_ioc_create)
#define VWIFI_IOC_DESTROY_RADIO  _IOW (VWIFI_IOC_MAGIC, 2, struct vwifi_ioc_destroy)

#endif /* VWIFI_HOST_IOCTL_H */
