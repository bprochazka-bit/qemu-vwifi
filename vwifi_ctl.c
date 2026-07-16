/*
 * vwifi-ctl — Create and destroy vwifi_host radios at runtime
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Talks to the kernel module's control node (/dev/vwifi-ctl) to add and
 * remove independent mac80211 radios on demand. Each radio gets its own
 * data chardev at /dev/<name> (which one vwifi-host-relay then bridges to
 * a medium hub) and its own wlanX netdev, which this tool can rename to a
 * friendly name in the same step.
 *
 * Usage:
 *   vwifi-ctl create --dev <name> [--mac <mac>] [--ifname <newname>]
 *   vwifi-ctl destroy --dev <name>
 *
 * Examples:
 *   # Create a radio at /dev/vwifi-lab and rename its interface to vwm-lab
 *   vwifi-ctl create --dev vwifi-lab --ifname vwm-lab
 *   # Tear it down (after its relay has stopped)
 *   vwifi-ctl destroy --dev vwifi-lab
 *
 * Build:
 *   make vwifi-ctl
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <fcntl.h>
#include <dirent.h>
#include <time.h>
#include <limits.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <net/if.h>
#include <linux/sockios.h>

#include "vwifi.h"

#define CTL_DEV        "/dev/" VWIFI_CTL_NAME
#define SYS_NET        "/sys/class/net"
#define IFNAME_MAX     (IFNAMSIZ - 1)   /* kernel caps names at 15 chars */

static int parse_mac(const char *s, uint8_t mac[6])
{
    unsigned int m[6];
    int i;

    if (sscanf(s, "%02x:%02x:%02x:%02x:%02x:%02x",
               &m[0], &m[1], &m[2], &m[3], &m[4], &m[5]) != 6)
        return -1;
    for (i = 0; i < 6; i++)
        mac[i] = (uint8_t)m[i];
    return 0;
}

/* Read the phy name a netdev belongs to (/sys/class/net/<if>/phy80211/name).
 * Returns 0 and fills out on success, -1 if the iface has no phy link. */
static int netdev_phy(const char *iface, char *out, size_t outlen)
{
    char path[PATH_MAX];
    FILE *f;

    snprintf(path, sizeof(path), SYS_NET "/%s/phy80211/name", iface);
    f = fopen(path, "r");
    if (!f)
        return -1;
    if (!fgets(out, outlen, f)) {
        fclose(f);
        return -1;
    }
    fclose(f);
    out[strcspn(out, "\n")] = '\0';
    return 0;
}

/* Find the netdev backing a given phy. Returns 0 and fills iface on
 * success, -1 if none found. The netdev is created synchronously by
 * ieee80211_register_hw before the CREATE ioctl returns, but the sysfs
 * symlink can lag a hair, so the caller retries. */
static int find_iface_for_phy(const char *phy, char *iface, size_t iflen)
{
    DIR *d = opendir(SYS_NET);
    struct dirent *e;
    int found = -1;

    if (!d)
        return -1;
    while ((e = readdir(d))) {
        char thisphy[64];

        if (e->d_name[0] == '.')
            continue;
        if (strlen(e->d_name) > IFNAME_MAX)
            continue;   /* not a valid interface name */
        if (netdev_phy(e->d_name, thisphy, sizeof(thisphy)) != 0)
            continue;
        if (!strcmp(thisphy, phy)) {
            snprintf(iface, iflen, "%s", e->d_name);
            found = 0;
            break;
        }
    }
    closedir(d);
    return found;
}

static void msleep(long ms)
{
    struct timespec ts = { ms / 1000, (ms % 1000) * 1000000L };

    nanosleep(&ts, NULL);
}

/* Rename `cur` to `newname` and bring it up, via rtnetlink ioctls. The
 * link must be down to rename, so we down/rename/up. Returns 0 on success. */
static int rename_iface(const char *cur, const char *newname)
{
    struct ifreq ifr;
    int fd, ret = -1;

    fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (fd < 0) {
        perror("vwifi-ctl: socket");
        return -1;
    }

    /* Down (rename requires the link be down). */
    memset(&ifr, 0, sizeof(ifr));
    snprintf(ifr.ifr_name, IFNAMSIZ, "%s", cur);
    if (ioctl(fd, SIOCGIFFLAGS, &ifr) < 0) {
        perror("vwifi-ctl: SIOCGIFFLAGS");
        goto out;
    }
    if (ifr.ifr_flags & IFF_UP) {
        ifr.ifr_flags &= ~IFF_UP;
        if (ioctl(fd, SIOCSIFFLAGS, &ifr) < 0) {
            perror("vwifi-ctl: SIOCSIFFLAGS(down)");
            goto out;
        }
    }

    /* Rename. */
    memset(&ifr, 0, sizeof(ifr));
    snprintf(ifr.ifr_name, IFNAMSIZ, "%s", cur);
    snprintf(ifr.ifr_newname, IFNAMSIZ, "%s", newname);
    if (ioctl(fd, SIOCSIFNAME, &ifr) < 0) {
        fprintf(stderr, "vwifi-ctl: rename %s -> %s: %s\n",
                cur, newname, strerror(errno));
        goto out;
    }

    /* Up. */
    memset(&ifr, 0, sizeof(ifr));
    snprintf(ifr.ifr_name, IFNAMSIZ, "%s", newname);
    if (ioctl(fd, SIOCGIFFLAGS, &ifr) == 0) {
        ifr.ifr_flags |= IFF_UP;
        if (ioctl(fd, SIOCSIFFLAGS, &ifr) < 0)
            perror("vwifi-ctl: SIOCSIFFLAGS(up)");
    }

    ret = 0;
out:
    close(fd);
    return ret;
}

/* Locate the radio's netdev (retrying briefly) and rename it. Best-effort:
 * a failure here doesn't invalidate the radio, so we warn and return 0. */
static int rename_radio_iface(const char *phy, const char *ifname)
{
    char cur[IFNAMSIZ];
    char want[IFNAMSIZ];
    int i;

    /* Kernel caps interface names at IFNAMSIZ-1. */
    snprintf(want, sizeof(want), "%.*s", IFNAME_MAX, ifname);

    for (i = 0; i < 40; i++) {
        if (find_iface_for_phy(phy, cur, sizeof(cur)) == 0)
            break;
        msleep(50);
    }
    if (i == 40) {
        fprintf(stderr, "vwifi-ctl: warning: no netdev found for %s; "
                "leaving interface unnamed\n", phy);
        return 0;
    }

    if (!strcmp(cur, want))
        return 0;  /* already named as requested */

    if (rename_iface(cur, want) != 0)
        fprintf(stderr, "vwifi-ctl: warning: could not rename %s to %s\n",
                cur, want);
    return 0;
}

static int do_create(const char *dev, const char *mac, const char *ifname)
{
    struct vwifi_ioc_create req;
    int fd, ret;

    memset(&req, 0, sizeof(req));
    snprintf(req.name, sizeof(req.name), "%s", dev);
    if (mac && parse_mac(mac, req.mac) != 0) {
        fprintf(stderr, "vwifi-ctl: invalid MAC '%s'\n", mac);
        return 2;
    }

    fd = open(CTL_DEV, O_RDWR);
    if (fd < 0) {
        fprintf(stderr, "vwifi-ctl: open(%s): %s\n", CTL_DEV, strerror(errno));
        fprintf(stderr, "vwifi-ctl: is the vwifi_host module loaded?\n");
        return 1;
    }

    ret = ioctl(fd, VWIFI_IOC_CREATE_RADIO, &req);
    close(fd);
    if (ret < 0) {
        fprintf(stderr, "vwifi-ctl: create '%s': %s\n", dev, strerror(errno));
        return 1;
    }

    printf("created radio '%s' id=%u phy=%s mac=%02x:%02x:%02x:%02x:%02x:%02x "
           "dev=/dev/%s\n",
           req.name, req.radio_id, req.phy,
           req.mac_out[0], req.mac_out[1], req.mac_out[2],
           req.mac_out[3], req.mac_out[4], req.mac_out[5], req.name);

    if (ifname && *ifname)
        rename_radio_iface(req.phy, ifname);

    return 0;
}

static int do_destroy(const char *dev)
{
    struct vwifi_ioc_destroy req;
    int fd, ret;

    memset(&req, 0, sizeof(req));
    snprintf(req.name, sizeof(req.name), "%s", dev);

    fd = open(CTL_DEV, O_RDWR);
    if (fd < 0) {
        fprintf(stderr, "vwifi-ctl: open(%s): %s\n", CTL_DEV, strerror(errno));
        return 1;
    }

    ret = ioctl(fd, VWIFI_IOC_DESTROY_RADIO, &req);
    close(fd);
    if (ret < 0) {
        if (errno == ENODEV) {
            /* Already gone — the idempotent teardown target is satisfied. */
            printf("radio '%s' not present (already destroyed)\n", dev);
            return 0;
        }
        fprintf(stderr, "vwifi-ctl: destroy '%s': %s\n", dev, strerror(errno));
        return 1;
    }
    printf("destroyed radio '%s'\n", dev);
    return 0;
}

static void usage(const char *prog)
{
    fprintf(stderr,
        "Usage:\n"
        "  %s create --dev <name> [--mac <mac>] [--ifname <newname>]\n"
        "  %s destroy --dev <name>\n"
        "\n"
        "  --dev <name>      data-chardev basename; node is /dev/<name>\n"
        "  --mac <mac>       radio MAC (aa:bb:cc:dd:ee:ff); default derived\n"
        "                    deterministically from <name>\n"
        "  --ifname <name>   rename the radio's wlanX netdev to this and\n"
        "                    bring it up (capped to %d chars)\n",
        prog, prog, IFNAME_MAX);
}

int main(int argc, char *argv[])
{
    const char *dev = NULL, *mac = NULL, *ifname = NULL;
    const char *cmd;
    int i;

    if (argc < 2) {
        usage(argv[0]);
        return 2;
    }
    cmd = argv[1];

    for (i = 2; i < argc; i++) {
        if (!strcmp(argv[i], "--dev") && i + 1 < argc)
            dev = argv[++i];
        else if (!strcmp(argv[i], "--mac") && i + 1 < argc)
            mac = argv[++i];
        else if (!strcmp(argv[i], "--ifname") && i + 1 < argc)
            ifname = argv[++i];
        else if (!strcmp(argv[i], "-h") || !strcmp(argv[i], "--help")) {
            usage(argv[0]);
            return 0;
        } else {
            fprintf(stderr, "vwifi-ctl: unexpected argument '%s'\n", argv[i]);
            usage(argv[0]);
            return 2;
        }
    }

    if (!dev) {
        fprintf(stderr, "vwifi-ctl: --dev is required\n");
        usage(argv[0]);
        return 2;
    }

    if (!strcmp(cmd, "create"))
        return do_create(dev, mac, ifname);
    if (!strcmp(cmd, "destroy"))
        return do_destroy(dev);

    fprintf(stderr, "vwifi-ctl: unknown command '%s'\n", cmd);
    usage(argv[0]);
    return 2;
}
