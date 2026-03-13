/*
 * ath9k_host_relay — Userspace relay between kernel module and medium hub
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * This is a thin bidirectional byte pump that connects:
 *   /dev/ath9k_medium  (kernel module chardev)
 *       ↕
 *   Unix socket to ath9k_medium_hub
 *
 * Both sides speak the same length-prefixed wire protocol, so the relay
 * just copies complete messages in both directions.
 *
 * Usage:
 *   ./ath9k_host_relay /tmp/ath9k.sock [/dev/ath9k_medium]
 *
 * Build:
 *   gcc -Wall -O2 -o ath9k_host_relay ath9k_host_relay.c
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <fcntl.h>
#include <signal.h>
#include <poll.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <arpa/inet.h>  /* ntohl/htonl */

#include "ath9k_medium.h"

#define DEFAULT_CHRDEV  "/dev/" ATH9K_MEDIUM_CHRDEV_NAME
#define READ_BUF_SIZE   (ATH9K_MEDIUM_MAX_MSG)

static volatile sig_atomic_t g_running = 1;

static void sig_handler(int sig)
{
    (void)sig;
    g_running = 0;
}

/*
 * Connect to the hub's Unix socket.
 * Returns the fd on success, -1 on failure.
 */
static int connect_hub(const char *sock_path)
{
    int fd;
    struct sockaddr_un addr;

    fd = socket(AF_UNIX, SOCK_STREAM, 0);
    if (fd < 0) {
        perror("relay: socket(AF_UNIX)");
        return -1;
    }

    memset(&addr, 0, sizeof(addr));
    addr.sun_family = AF_UNIX;
    snprintf(addr.sun_path, sizeof(addr.sun_path), "%s", sock_path);

    if (connect(fd, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        perror("relay: connect to hub");
        close(fd);
        return -1;
    }

    return fd;
}

/*
 * Read exactly `count` bytes from fd.
 * Returns 0 on success, -1 on EOF/error.
 */
static int read_exact(int fd, void *buf, size_t count)
{
    size_t done = 0;
    while (done < count) {
        ssize_t n = read(fd, (char *)buf + done, count - done);
        if (n <= 0) {
            if (n < 0 && errno == EINTR)
                continue;
            return -1;
        }
        done += n;
    }
    return 0;
}

/*
 * Write exactly `count` bytes to fd.
 * Returns 0 on success, -1 on error.
 */
static int write_exact(int fd, const void *buf, size_t count)
{
    size_t done = 0;
    while (done < count) {
        ssize_t n = write(fd, (const char *)buf + done, count - done);
        if (n <= 0) {
            if (n < 0 && errno == EINTR)
                continue;
            return -1;
        }
        done += n;
    }
    return 0;
}

/*
 * Forward one complete wire message from src_fd to dst_fd.
 *
 * Wire format: [uint32_t length (network order)] [payload]
 *
 * For hub→chardev direction, we write the full message including
 * the length prefix, since the chardev write() handler expects it.
 *
 * For chardev→hub direction, the chardev read() returns the full
 * message including the length prefix.
 */
/* (forward_message removed — using directional functions below) */

/*
 * The chardev read() returns a complete wire message (including
 * length prefix) in a single read call.  We forward it to the hub.
 */
static int forward_chardev_to_hub(int chrdev_fd, int hub_fd)
{
    uint8_t buf[READ_BUF_SIZE];
    ssize_t n;

    n = read(chrdev_fd, buf, sizeof(buf));
    if (n <= 0) {
        if (n < 0 && (errno == EAGAIN || errno == EINTR))
            return 0;
        return -1;
    }

    /* Validate minimum size: 4 (len prefix) + 28 (frame header) */
    if (n < (ssize_t)(4 + ATH9K_MEDIUM_HDR_SIZE)) {
        fprintf(stderr, "relay: chardev→hub: short read (%zd bytes)\n", n);
        return 0;
    }

    /* Forward the complete message to the hub */
    if (write_exact(hub_fd, buf, n) < 0) {
        fprintf(stderr, "relay: chardev→hub: write failed: %s\n",
                strerror(errno));
        return -1;
    }

    return 0;
}

/*
 * Read a complete wire message from the hub socket (which is a stream,
 * so we need to do length-prefix framing) and write it to the chardev.
 */
static int forward_hub_to_chardev(int hub_fd, int chrdev_fd)
{
    uint8_t buf[READ_BUF_SIZE];
    uint32_t len_be;
    uint32_t payload_len;

    /* Read the 4-byte length prefix */
    if (read_exact(hub_fd, &len_be, 4) < 0) {
        return -1;
    }

    payload_len = ntohl(len_be);
    if (payload_len < ATH9K_MEDIUM_HDR_SIZE ||
        payload_len > ATH9K_MEDIUM_HDR_SIZE + ATH9K_MEDIUM_MAX_FRAME) {
        fprintf(stderr, "relay: hub→chardev: bad length %u\n", payload_len);
        return -1;
    }

    /* Read the payload */
    if (read_exact(hub_fd, buf + 4, payload_len) < 0) {
        return -1;
    }

    /* Prepend length prefix */
    memcpy(buf, &len_be, 4);

    /* Write complete message to chardev */
    if (write_exact(chrdev_fd, buf, 4 + payload_len) < 0) {
        fprintf(stderr, "relay: hub→chardev: write failed: %s\n",
                strerror(errno));
        return -1;
    }

    return 0;
}

static void usage(const char *prog)
{
    fprintf(stderr,
            "Usage: %s <hub-socket-path> [chardev-path]\n"
            "\n"
            "  hub-socket-path  Path to ath9k_medium_hub Unix socket\n"
            "  chardev-path     Path to kernel module char device\n"
            "                   (default: " DEFAULT_CHRDEV ")\n"
            "\n"
            "Example:\n"
            "  %s /tmp/ath9k.sock\n"
            "  %s /tmp/ath9k.sock /dev/ath9k_medium\n",
            prog, prog, prog);
}

int main(int argc, char *argv[])
{
    const char *hub_path;
    const char *chrdev_path = DEFAULT_CHRDEV;
    int hub_fd = -1;
    int chrdev_fd = -1;
    struct pollfd pfds[2];
    int ret = 1;

    if (argc < 2 || strcmp(argv[1], "-h") == 0) {
        usage(argv[0]);
        return (argc < 2) ? 1 : 0;
    }

    hub_path = argv[1];
    if (argc >= 3) {
        chrdev_path = argv[2];
    }

    signal(SIGINT,  sig_handler);
    signal(SIGTERM, sig_handler);
    signal(SIGPIPE, SIG_IGN);

    /* Open the kernel module's char device */
    chrdev_fd = open(chrdev_path, O_RDWR);
    if (chrdev_fd < 0) {
        fprintf(stderr, "relay: open(%s): %s\n", chrdev_path, strerror(errno));
        fprintf(stderr, "relay: is the ath9k_medium_host module loaded?\n");
        goto out;
    }
    fprintf(stderr, "relay: chardev %s opened (fd=%d)\n", chrdev_path, chrdev_fd);

    /* Connect to the hub */
    hub_fd = connect_hub(hub_path);
    if (hub_fd < 0) {
        fprintf(stderr, "relay: is ath9k_medium_hub running at %s?\n", hub_path);
        goto out;
    }
    fprintf(stderr, "relay: connected to hub %s (fd=%d)\n", hub_path, hub_fd);
    fprintf(stderr, "relay: bridging %s ↔ %s\n", chrdev_path, hub_path);

    /* Main loop: poll both fds and forward messages */
    while (g_running) {
        pfds[0].fd     = chrdev_fd;
        pfds[0].events = POLLIN;
        pfds[0].revents = 0;

        pfds[1].fd     = hub_fd;
        pfds[1].events = POLLIN;
        pfds[1].revents = 0;

        int nready = poll(pfds, 2, 1000);
        if (nready < 0) {
            if (errno == EINTR) continue;
            perror("relay: poll");
            break;
        }
        if (nready == 0) continue;

        /* TX: chardev → hub */
        if (pfds[0].revents & POLLIN) {
            if (forward_chardev_to_hub(chrdev_fd, hub_fd) < 0) {
                fprintf(stderr, "relay: chardev→hub pipe broken\n");
                break;
            }
        }

        /* RX: hub → chardev */
        if (pfds[1].revents & POLLIN) {
            if (forward_hub_to_chardev(hub_fd, chrdev_fd) < 0) {
                fprintf(stderr, "relay: hub→chardev pipe broken\n");
                break;
            }
        }

        /* Check for errors / hangup */
        if ((pfds[0].revents | pfds[1].revents) & (POLLERR | POLLHUP)) {
            fprintf(stderr, "relay: connection lost\n");
            break;
        }
    }

    fprintf(stderr, "relay: shutting down\n");
    ret = 0;

out:
    if (hub_fd >= 0)    close(hub_fd);
    if (chrdev_fd >= 0) close(chrdev_fd);
    return ret;
}
