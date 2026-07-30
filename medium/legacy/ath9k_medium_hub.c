/*
 * ath9k Virtual Wireless Medium Hub
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * A simple fan-out hub for the ath9k-virt virtual wireless medium.
 * Listens on a Unix domain socket; each connected QEMU instance
 * sends length-prefixed 802.11 frames. The hub forwards every
 * frame to all *other* connected clients.
 *
 * Usage:
 *   ./ath9k_medium_hub /tmp/ath9k-medium.sock
 *
 * Then start QEMU instances with:
 *   -chardev socket,id=medium,path=/tmp/ath9k-medium.sock,server=off
 *   -device ath9k-virt,chardev=medium
 *
 * Wire protocol (same as QEMU stream socket backend):
 *   [uint32_t length in network byte order] [payload]
 *
 * Build:
 *   gcc -Wall -O2 -o ath9k_medium_hub ath9k_medium_hub.c
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <signal.h>
#include <errno.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <sys/select.h>
#include <arpa/inet.h>
#include <stdint.h>

#define MAX_CLIENTS         16
#define RECV_BUF_SIZE       (4 + 8192 + 64)  /* length prefix + max frame */
#define MAX_MSG_SIZE        (8192 + 64)       /* max payload after prefix */

static volatile int running = 1;

struct client {
    int     fd;
    uint8_t buf[RECV_BUF_SIZE];
    uint32_t buf_used;
};

static struct client clients[MAX_CLIENTS];
static int num_clients = 0;
static int listen_fd = -1;
static char *socket_path = NULL;

static void cleanup(void)
{
    int i;
    if (listen_fd >= 0) {
        close(listen_fd);
    }
    for (i = 0; i < num_clients; i++) {
        if (clients[i].fd >= 0) {
            close(clients[i].fd);
        }
    }
    if (socket_path) {
        unlink(socket_path);
    }
}

static void sighandler(int sig)
{
    (void)sig;
    running = 0;
}

static void remove_client(int idx)
{
    fprintf(stderr, "hub: client %d disconnected (fd=%d)\n",
            idx, clients[idx].fd);
    close(clients[idx].fd);
    /* Swap with last */
    if (idx < num_clients - 1) {
        clients[idx] = clients[num_clients - 1];
    }
    num_clients--;
}

/*
 * Forward a complete message (including the 4-byte length prefix)
 * to all clients except the sender.
 */
static void forward_to_others(int sender_idx, const uint8_t *msg,
                              uint32_t total_len)
{
    int i;
    for (i = 0; i < num_clients; i++) {
        ssize_t sent, offset;
        if (i == sender_idx) {
            continue;
        }
        offset = 0;
        while ((uint32_t)offset < total_len) {
            sent = write(clients[i].fd, msg + offset,
                         total_len - offset);
            if (sent <= 0) {
                if (sent < 0 && (errno == EAGAIN || errno == EINTR)) {
                    continue;
                }
                /* Client write failed — will be cleaned up on next read */
                break;
            }
            offset += sent;
        }
    }
}

static void handle_client_data(int idx)
{
    struct client *c = &clients[idx];
    ssize_t n;
    uint32_t net_len, msg_len, consumed;

    n = read(c->fd, c->buf + c->buf_used,
             RECV_BUF_SIZE - c->buf_used);
    if (n <= 0) {
        remove_client(idx);
        return;
    }
    c->buf_used += (uint32_t)n;

    /* Process complete messages */
    while (c->buf_used >= 4) {
        memcpy(&net_len, c->buf, 4);
        msg_len = ntohl(net_len);

        if (msg_len > MAX_MSG_SIZE) {
            fprintf(stderr, "hub: client %d sent oversized message (%u), "
                    "disconnecting\n", idx, msg_len);
            remove_client(idx);
            return;
        }

        if (c->buf_used < 4 + msg_len) {
            /* Incomplete — wait for more data */
            break;
        }

        /* Forward the entire wire message (prefix + payload) */
        forward_to_others(idx, c->buf, 4 + msg_len);

        /* Remove processed message from buffer */
        consumed = 4 + msg_len;
        if (consumed < c->buf_used) {
            memmove(c->buf, c->buf + consumed, c->buf_used - consumed);
        }
        c->buf_used -= consumed;
    }
}

int main(int argc, char **argv)
{
    struct sockaddr_un addr;
    fd_set rfds;
    int maxfd, i, client_fd;

    if (argc != 2) {
        fprintf(stderr, "Usage: %s <socket-path>\n", argv[0]);
        return 1;
    }
    socket_path = argv[1];

    signal(SIGINT, sighandler);
    signal(SIGTERM, sighandler);
    signal(SIGPIPE, SIG_IGN);

    /* Clean up stale socket */
    unlink(socket_path);

    listen_fd = socket(AF_UNIX, SOCK_STREAM, 0);
    if (listen_fd < 0) {
        perror("socket");
        return 1;
    }

    memset(&addr, 0, sizeof(addr));
    addr.sun_family = AF_UNIX;
    strncpy(addr.sun_path, socket_path, sizeof(addr.sun_path) - 1);

    if (bind(listen_fd, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        perror("bind");
        close(listen_fd);
        return 1;
    }

    if (listen(listen_fd, 8) < 0) {
        perror("listen");
        cleanup();
        return 1;
    }

    fprintf(stderr, "hub: listening on %s (max %d clients)\n",
            socket_path, MAX_CLIENTS);

    for (i = 0; i < MAX_CLIENTS; i++) {
        clients[i].fd = -1;
        clients[i].buf_used = 0;
    }

    while (running) {
        FD_ZERO(&rfds);
        FD_SET(listen_fd, &rfds);
        maxfd = listen_fd;

        for (i = 0; i < num_clients; i++) {
            FD_SET(clients[i].fd, &rfds);
            if (clients[i].fd > maxfd) {
                maxfd = clients[i].fd;
            }
        }

        struct timeval tv = { .tv_sec = 1, .tv_usec = 0 };
        int ret = select(maxfd + 1, &rfds, NULL, NULL, &tv);
        if (ret < 0) {
            if (errno == EINTR) {
                continue;
            }
            perror("select");
            break;
        }
        if (ret == 0) {
            continue;
        }

        /* Accept new connections */
        if (FD_ISSET(listen_fd, &rfds)) {
            client_fd = accept(listen_fd, NULL, NULL);
            if (client_fd >= 0) {
                if (num_clients >= MAX_CLIENTS) {
                    fprintf(stderr, "hub: max clients reached, "
                            "rejecting connection\n");
                    close(client_fd);
                } else {
                    clients[num_clients].fd = client_fd;
                    clients[num_clients].buf_used = 0;
                    fprintf(stderr, "hub: client %d connected (fd=%d)\n",
                            num_clients, client_fd);
                    num_clients++;
                }
            }
        }

        /* Handle client data (iterate backwards so remove_client is safe) */
        for (i = num_clients - 1; i >= 0; i--) {
            if (FD_ISSET(clients[i].fd, &rfds)) {
                handle_client_data(i);
            }
        }
    }

    fprintf(stderr, "hub: shutting down\n");
    cleanup();
    return 0;
}
