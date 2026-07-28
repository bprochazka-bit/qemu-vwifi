/*
 * vwifi-probe — guest-side smoke test for the vwifi-virt device
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * There is no Linux driver for vwifi-virt yet, which would normally mean
 * a Linux guest can tell you nothing about whether the device works.
 * This closes that gap: it maps BAR0 straight from sysfs and reads the
 * read-only registers, so a guest with no driver at all can still answer
 *
 *   - did the device enumerate on the PCI bus?
 *   - does BAR0 decode, and does MMIO reach the device model?
 *   - does it report the ABI version this tree was built against?
 *   - is the hub link up, i.e. is the chardev actually connected?
 *   - are frames moving on the medium?
 *
 * That last pair is the useful one: STATUS.LINK_UP and the DIAG counters
 * exercise the whole QEMU-side path — chardev, medium protocol, frame
 * parsing — without a single line of guest driver.
 *
 * Run as root inside the guest:
 *
 *   ./vwifi-probe            # one shot
 *   ./vwifi-probe -w         # poll every second (watch the counters move)
 *
 * Build (host or guest; needs abi/vwifi_abi.h beside it or on -I):
 *   cc -Wall -O2 -I../../../abi -o vwifi-probe vwifi-probe.c
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <dirent.h>
#include <sys/mman.h>

#include "vwifi_abi.h"

#define PCI_DEVICES_DIR "/sys/bus/pci/devices"

static uint32_t read_hex_file(const char *dir, const char *file, int *ok)
{
    char path[512];
    char buf[64];
    FILE *f;
    unsigned long v = 0;

    *ok = 0;
    snprintf(path, sizeof(path), "%s/%s", dir, file);
    f = fopen(path, "r");
    if (!f) return 0;
    if (fgets(buf, sizeof(buf), f)) {
        v = strtoul(buf, NULL, 16);
        *ok = 1;
    }
    fclose(f);
    return (uint32_t)v;
}

/* Locate the first PCI function matching our vendor/device id. */
static int find_device(char *out, size_t outsz)
{
    DIR *d = opendir(PCI_DEVICES_DIR);
    struct dirent *e;
    int found = 0;

    if (!d) {
        fprintf(stderr, "cannot open %s: %s\n", PCI_DEVICES_DIR, strerror(errno));
        return -1;
    }

    while ((e = readdir(d)) != NULL) {
        char dir[512];
        int ok_v, ok_d;
        uint32_t vendor, device;

        if (e->d_name[0] == '.') continue;
        snprintf(dir, sizeof(dir), "%s/%s", PCI_DEVICES_DIR, e->d_name);

        vendor = read_hex_file(dir, "vendor", &ok_v);
        device = read_hex_file(dir, "device", &ok_d);
        if (!ok_v || !ok_d) continue;

        if (vendor == VWIFI_PCI_VENDOR_ID && device == VWIFI_PCI_DEVICE_ID) {
            snprintf(out, outsz, "%s", dir);
            found = 1;
            break;
        }
    }
    closedir(d);
    return found ? 0 : -1;
}

/* Without a driver bound, memory decoding may be off, and then every
 * BAR read comes back 0xFFFFFFFF. Writing 1 to sysfs `enable` turns it
 * on, which is exactly what a driver's pci_enable_device() would do. */
static void enable_device(const char *dir)
{
    char path[600];
    int fd;

    snprintf(path, sizeof(path), "%s/enable", dir);
    fd = open(path, O_WRONLY);
    if (fd < 0) return;
    if (write(fd, "1", 1) < 0) {
        /* Non-fatal: it may already be enabled. */
    }
    close(fd);
}

static void decode_caps(uint32_t caps)
{
    static const struct { uint32_t bit; const char *name; } tbl[] = {
        { VWIFI_CAP_MONITOR, "MONITOR" },
        { VWIFI_CAP_INJECT,  "INJECT"  },
        { VWIFI_CAP_STA,     "STA"     },
        { VWIFI_CAP_SOFTAP,  "SOFTAP"  },
        { VWIFI_CAP_WPA2,    "WPA2"    },
        { VWIFI_CAP_WPA3,    "WPA3"    },
        { VWIFI_CAP_HT,      "HT"      },
        { VWIFI_CAP_VHT,     "VHT"     },
    };
    size_t i;
    printf("  caps            0x%08x  ", caps);
    for (i = 0; i < sizeof(tbl) / sizeof(tbl[0]); i++) {
        if (caps & tbl[i].bit) printf("%s ", tbl[i].name);
    }
    printf("\n");
}

static void decode_status(uint32_t st)
{
    printf("  status          0x%08x  %s%s%s\n", st,
           (st & VWIFI_STATUS_READY)     ? "READY "     : "",
           (st & VWIFI_STATUS_LINK_UP)   ? "LINK_UP "   : "",
           (st & VWIFI_STATUS_RESETTING) ? "RESETTING " : "");
    if (!(st & VWIFI_STATUS_LINK_UP)) {
        printf("                  ^ hub link is DOWN: the chardev is not\n");
        printf("                    connected. Is vwifi-medium running, and\n");
        printf("                    did QEMU get the right socket path?\n");
    }
}

int main(int argc, char **argv)
{
    char dir[512];
    char res0[600];
    int watch = 0;
    int fd;
    volatile uint32_t *bar;
    uint32_t sig, abi;

    if (argc > 1 && strcmp(argv[1], "-w") == 0) watch = 1;

    if (find_device(dir, sizeof(dir)) != 0) {
        fprintf(stderr,
            "No PCI device %04x:%04x found.\n"
            "The guest is not seeing vwifi-virt at all. Check that QEMU was\n"
            "started with -device vwifi-virt and that this is the right VM.\n",
            VWIFI_PCI_VENDOR_ID, VWIFI_PCI_DEVICE_ID);
        return 1;
    }

    printf("vwifi-virt found at %s\n", strrchr(dir, '/') + 1);
    enable_device(dir);

    snprintf(res0, sizeof(res0), "%s/resource0", dir);
    fd = open(res0, O_RDWR | O_SYNC);
    if (fd < 0) {
        fprintf(stderr, "open %s: %s\n", res0, strerror(errno));
        fprintf(stderr, "(are you root?)\n");
        return 1;
    }

    bar = mmap(NULL, VWIFI_MMIO_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
    if (bar == MAP_FAILED) {
        fprintf(stderr, "mmap BAR0: %s\n", strerror(errno));
        close(fd);
        return 1;
    }

#define REG(off) (bar[(off) / 4])

    sig = REG(VWIFI_REG_SIGNATURE);
    abi = REG(VWIFI_REG_ABI_VERSION);

    printf("\n-- identity --\n");
    printf("  signature       0x%08x  %s\n", sig,
           sig == VWIFI_SIGNATURE ? "OK" :
           sig == 0xFFFFFFFFu ? "BAD (all ones: BAR not decoding)" : "BAD");
    printf("  abi version     %-10u  %s\n", abi,
           abi == VWIFI_ABI_VERSION ? "matches this build" :
                                      "MISMATCH with this build");

    if (sig != VWIFI_SIGNATURE) {
        printf("\nMMIO is not reaching the device model. Everything below is\n"
               "meaningless until the signature reads back correctly.\n");
    }

    decode_caps(REG(VWIFI_REG_CAPS));
    printf("  msix vectors    %u\n", REG(VWIFI_REG_NUM_VECTORS));

    do {
        printf("\n-- state --\n");
        decode_status(REG(VWIFI_REG_STATUS));
        printf("  ctrl            0x%08x\n", REG(VWIFI_REG_CTRL));
        printf("\n-- medium counters --\n");
        printf("  frames tx       %u\n", REG(VWIFI_REG_DIAG_FRAMES_TX));
        printf("  frames rx       %u\n", REG(VWIFI_REG_DIAG_FRAMES_RX));
        printf("  drops           %u\n", REG(VWIFI_REG_DIAG_DROPS));
        if (watch) {
            printf("\n(polling, ^C to stop)\n");
            sleep(1);
        }
    } while (watch);

#undef REG

    munmap((void *)bar, VWIFI_MMIO_SIZE);
    close(fd);
    return sig == VWIFI_SIGNATURE ? 0 : 1;
}
