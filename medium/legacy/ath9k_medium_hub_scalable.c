/*
 * ath9k Virtual Wireless Medium Hub — Channel-Aware
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * A channel-aware fan-out hub for the ath9k-virt virtual wireless medium.
 *
 * Features:
 *   - Up to 256 local QEMU clients via Unix domain socket
 *   - TCP listen port for incoming bridge connections from remote hubs
 *   - Outbound TCP upstream connections to remote hubs (-u host:port)
 *   - Channel-aware filtering: devices on different channels don't see
 *     each other's frames (including across TCP bridges)
 *   - HT40 channel bonding: bonded frames only go to peers on the same
 *     bonded channel pair
 *   - TTL-based loop prevention for multi-hub topologies
 *   - Backward compatible with v1 protocol (no channel = broadcast)
 *   - Uses poll() instead of select() -- no FD_SETSIZE limit
 *   - Automatic reconnection to upstreams every 3 seconds
 *
 * Channel filtering:
 *   The hub inspects the v2 medium header's channel_freq field:
 *     - channel_freq == 0: frame is broadcast to all peers (v1 compat)
 *     - channel_freq != 0, channel_bond_freq == 0: deliver to peers on
 *       the same primary channel frequency only
 *     - channel_freq != 0, channel_bond_freq != 0: deliver only to
 *       peers with matching primary AND secondary (bonded) channel
 *   Peers announce their current channel via a special "channel update"
 *   control message, or implicitly via the channel_freq in data frames.
 *
 * Usage:
 *   # Simple local hub:
 *   ./ath9k_medium_hub /tmp/ath9k-medium.sock
 *
 *   # Local hub + TCP listen for bridges:
 *   ./ath9k_medium_hub /tmp/ath9k-medium.sock -t 5550
 *
 *   # Two hosts bridged:
 *   Host A:  ./ath9k_medium_hub /tmp/ath9k.sock -t 5550
 *   Host B:  ./ath9k_medium_hub /tmp/ath9k.sock -u hostA:5550
 *
 * Wire protocol:
 *   [uint32_t length (network byte order)] [payload]
 *   Payload starts with ath9k_medium_frame_hdr (v1=28 bytes, v2=40 bytes).
 *   The 'flags' field byte 27 is used as a TTL counter.
 *   v2 adds channel_freq, channel_flags, channel_bond_freq after flags.
 *
 * Build:
 *   gcc -Wall -O2 -o ath9k_medium_hub ath9k_medium_hub_scalable.c
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <signal.h>
#include <errno.h>
#include <fcntl.h>
#include <poll.h>
#include <netdb.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <sys/types.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <stdint.h>
#include <time.h>

/* -------------------------------------------------------------------
 *  Configuration
 * ------------------------------------------------------------------- */
#define MAX_PEERS           256     /* Local clients + bridge connections */
#define MAX_UPSTREAMS       16      /* Max -u upstream arguments */
#define RECV_BUF_SIZE       (4 + 8192 + 64)
#define MAX_MSG_SIZE        (8192 + 64)

/*
 * TTL lives in the MSB of the 'flags' field in ath9k_medium_frame_hdr.
 * The flags field is at byte offset 24 from the start of the header.
 * We treat byte 27 (the fourth byte of flags, big-endian MSB) as TTL.
 */
#define TTL_BYTE_OFFSET     27      /* byte within payload carrying TTL */
#define DEFAULT_TTL         8       /* max hub hops */
#define MIN_HDR_SIZE_V1     28      /* v1 ath9k_medium_frame_hdr */
#define MIN_HDR_SIZE_V2     40      /* v2 with channel fields */

#define UPSTREAM_RETRY_SEC  3       /* seconds between reconnect attempts */

/* V2 header field offsets within the payload (after 4-byte length prefix) */
#define OFF_MAGIC           0
#define OFF_VERSION         4
#define OFF_FRAME_LEN       6
#define OFF_TX_MAC          8
#define OFF_RATE_CODE       14
#define OFF_RSSI            15
#define OFF_TSF_LO          16
#define OFF_TSF_HI          20
#define OFF_FLAGS           24
#define OFF_CHAN_FREQ        28      /* uint16_t LE: center freq MHz */
#define OFF_CHAN_FLAGS       30      /* uint16_t LE: channel flags */
#define OFF_CHAN_BOND_FREQ   32      /* uint16_t LE: secondary freq MHz (HT40) */

/* Protocol versions */
#define MEDIUM_VERSION_V1   1
#define MEDIUM_VERSION_V2   2

/* -------------------------------------------------------------------
 *  Data structures
 * ------------------------------------------------------------------- */
static volatile int running = 1;
static int verbose = 0;

struct peer {
    int         fd;
    uint8_t     buf[RECV_BUF_SIZE];
    uint32_t    buf_used;
    int         is_bridge;      /* 1 = TCP bridge, 0 = local QEMU client */
    char        label[64];      /* human-readable name for logging */
    /* Channel state: learned from last data frame sent by this peer */
    uint16_t    channel_freq;       /* 0 = unknown (accept all) */
    uint16_t    channel_bond_freq;  /* 0 = no bonding */
};

static struct peer peers[MAX_PEERS];
static int num_peers = 0;

static int unix_listen_fd = -1;
static int tcp_listen_fd = -1;
static char *socket_path = NULL;

struct upstream_addr {
    char host[256];
    char port[16];
};
static struct upstream_addr upstreams[MAX_UPSTREAMS];
static int num_upstreams = 0;

struct upstream_state {
    int     peer_idx;       /* index into peers[], or -1 if disconnected */
    time_t  last_attempt;
};
static struct upstream_state upstream_state[MAX_UPSTREAMS];

/* -------------------------------------------------------------------
 *  Utility
 * ------------------------------------------------------------------- */
static void set_nonblock(int fd)
{
    int flags = fcntl(fd, F_GETFL, 0);
    if (flags >= 0) {
        fcntl(fd, F_SETFL, flags | O_NONBLOCK);
    }
}

static void set_tcp_nodelay(int fd)
{
    int val = 1;
    setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &val, sizeof(val));
}

/* Read a little-endian uint16 from a buffer */
static inline uint16_t read_le16(const uint8_t *p)
{
    return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}

/* -------------------------------------------------------------------
 *  Peer management
 * ------------------------------------------------------------------- */
static int add_peer(int fd, int is_bridge, const char *label)
{
    if (num_peers >= MAX_PEERS) {
        fprintf(stderr, "hub: max peers (%d) reached, rejecting\n",
                MAX_PEERS);
        close(fd);
        return -1;
    }
    set_nonblock(fd);
    peers[num_peers].fd = fd;
    peers[num_peers].buf_used = 0;
    peers[num_peers].is_bridge = is_bridge;
    peers[num_peers].channel_freq = 0;
    peers[num_peers].channel_bond_freq = 0;
    snprintf(peers[num_peers].label, sizeof(peers[num_peers].label),
             "%s", label);
    fprintf(stderr, "hub: peer %d connected: %s (fd=%d, %s)\n",
            num_peers, label, fd, is_bridge ? "bridge" : "local");
    num_peers++;
    return num_peers - 1;
}

static void remove_peer(int idx)
{
    int i;

    fprintf(stderr, "hub: peer %d disconnected: %s (fd=%d)\n",
            idx, peers[idx].label, peers[idx].fd);
    close(peers[idx].fd);

    /* Update upstream_state references before the swap */
    for (i = 0; i < num_upstreams; i++) {
        if (upstream_state[i].peer_idx == idx) {
            upstream_state[i].peer_idx = -1;
        } else if (upstream_state[i].peer_idx == num_peers - 1) {
            upstream_state[i].peer_idx = idx;
        }
    }

    /* Swap with last */
    if (idx < num_peers - 1) {
        peers[idx] = peers[num_peers - 1];
    }
    num_peers--;
}

/* -------------------------------------------------------------------
 *  Channel matching logic
 *
 *  Determines whether a frame on (tx_freq, tx_bond_freq) should be
 *  delivered to a peer whose last-known channel is (rx_freq, rx_bond_freq).
 *
 *  Rules:
 *   1) tx_freq == 0 → broadcast (v1 compat, unknown channel): deliver to all
 *   2) rx_freq == 0 → peer has unknown channel: deliver (conservative)
 *   3) If tx has bonding (tx_bond_freq != 0):
 *      - Exact match: rx must have same (freq, bond_freq) pair
 *      - OR rx is on the primary or secondary channel alone (HT20 on
 *        one of the bonded channels can still receive)
 *   4) If tx has no bonding (HT20/legacy):
 *      - rx_freq must match tx_freq (ignoring rx's bonding status)
 *      - OR if rx is bonded, tx_freq must match rx's primary or secondary
 * ------------------------------------------------------------------- */
static int channel_match(uint16_t tx_freq, uint16_t tx_bond_freq,
                         uint16_t rx_freq, uint16_t rx_bond_freq)
{
    /* Rule 1: unknown sender channel → broadcast */
    if (tx_freq == 0)
        return 1;

    /* Rule 2: unknown receiver channel → deliver (conservative) */
    if (rx_freq == 0)
        return 1;

    /* Rule 3: TX is bonded (HT40) */
    if (tx_bond_freq != 0) {
        /* Exact bonded match */
        if (rx_freq == tx_freq && rx_bond_freq == tx_bond_freq)
            return 1;
        /* RX is on primary channel only (HT20 device on primary) */
        if (rx_freq == tx_freq && rx_bond_freq == 0)
            return 1;
        /* RX is on secondary channel only (HT20 device on secondary) */
        if (rx_freq == tx_bond_freq && rx_bond_freq == 0)
            return 1;
        return 0;
    }

    /* Rule 4: TX is not bonded (HT20 / legacy) */
    /* Direct frequency match */
    if (rx_freq == tx_freq)
        return 1;
    /* RX is bonded and TX is on one of its channels */
    if (rx_bond_freq != 0 &&
        (tx_freq == rx_freq || tx_freq == rx_bond_freq))
        return 1;

    return 0;
}

/* -------------------------------------------------------------------
 *  Extract channel info from a v2 frame header
 * ------------------------------------------------------------------- */
static void extract_channel_info(const uint8_t *payload, uint32_t payload_len,
                                 uint16_t *freq, uint16_t *bond_freq)
{
    uint16_t version;

    *freq = 0;
    *bond_freq = 0;

    if (payload_len < MIN_HDR_SIZE_V2)
        return;

    version = read_le16(payload + OFF_VERSION);
    if (version >= MEDIUM_VERSION_V2) {
        *freq = read_le16(payload + OFF_CHAN_FREQ);
        *bond_freq = read_le16(payload + OFF_CHAN_BOND_FREQ);
    }
}

/* -------------------------------------------------------------------
 *  Frame forwarding with TTL + channel filtering
 *
 *  - Frames from local QEMU clients have TTL=0 (flags field unused).
 *    We stamp DEFAULT_TTL before forwarding to bridge peers.
 *  - Frames from bridge peers carry a TTL.  We decrement it.
 *    If TTL hits 0, we only forward to local clients, not other bridges.
 *  - Local clients always receive frames with TTL=0 (they ignore it).
 *  - Channel filtering: only deliver to peers on the same channel.
 * ------------------------------------------------------------------- */
static void forward_message(int sender_idx, const uint8_t *msg,
                            uint32_t total_len)
{
    uint8_t tmp[4 + MAX_MSG_SIZE];
    uint32_t payload_len;
    int sender_is_bridge;
    uint8_t ttl;
    int forward_to_bridges;
    int i;
    ssize_t sent;
    uint32_t offset;
    uint16_t tx_freq, tx_bond_freq;

    if (total_len < 4 || total_len > sizeof(tmp)) {
        return;
    }
    payload_len = total_len - 4;
    if (payload_len < MIN_HDR_SIZE_V1) {
        return;
    }

    sender_is_bridge = peers[sender_idx].is_bridge;

    /* Work on a copy so we can modify TTL without corrupting source */
    memcpy(tmp, msg, total_len);

    /* Extract channel info from the frame */
    extract_channel_info(tmp + 4, payload_len, &tx_freq, &tx_bond_freq);

    /* Update sender's channel state from their transmitted frame */
    if (!sender_is_bridge && tx_freq != 0) {
        if (peers[sender_idx].channel_freq != tx_freq ||
            peers[sender_idx].channel_bond_freq != tx_bond_freq) {
            if (verbose) {
                fprintf(stderr, "hub: peer %d (%s) channel: %u MHz%s\n",
                        sender_idx, peers[sender_idx].label, tx_freq,
                        tx_bond_freq ? " (bonded)" : "");
            }
            peers[sender_idx].channel_freq = tx_freq;
            peers[sender_idx].channel_bond_freq = tx_bond_freq;
        }
    }

    ttl = tmp[4 + TTL_BYTE_OFFSET];

    if (sender_is_bridge) {
        if (ttl == 0) {
            forward_to_bridges = 0;
        } else {
            ttl--;
            tmp[4 + TTL_BYTE_OFFSET] = ttl;
            forward_to_bridges = (ttl > 0);
        }
    } else {
        forward_to_bridges = 1;
        tmp[4 + TTL_BYTE_OFFSET] = DEFAULT_TTL;
    }

    for (i = 0; i < num_peers; i++) {
        if (i == sender_idx) {
            continue;
        }

        if (peers[i].is_bridge && !forward_to_bridges) {
            continue;
        }

        /* Channel filtering for local QEMU peers.
         * Bridge peers always get the frame (the remote hub will filter). */
        if (!peers[i].is_bridge) {
            if (!channel_match(tx_freq, tx_bond_freq,
                               peers[i].channel_freq,
                               peers[i].channel_bond_freq)) {
                continue;
            }
        }

        /*
         * For local QEMU clients, clear TTL back to 0 so it doesn't
         * confuse the device (it ignores flags, but be clean).
         */
        if (!peers[i].is_bridge) {
            uint8_t saved = tmp[4 + TTL_BYTE_OFFSET];
            tmp[4 + TTL_BYTE_OFFSET] = 0;

            offset = 0;
            while (offset < total_len) {
                sent = write(peers[i].fd, tmp + offset,
                             total_len - offset);
                if (sent <= 0) {
                    if (sent < 0 && (errno == EAGAIN || errno == EINTR)) {
                        continue;
                    }
                    break;
                }
                offset += (uint32_t)sent;
            }
            tmp[4 + TTL_BYTE_OFFSET] = saved;
        } else {
            offset = 0;
            while (offset < total_len) {
                sent = write(peers[i].fd, tmp + offset,
                             total_len - offset);
                if (sent <= 0) {
                    if (sent < 0 && (errno == EAGAIN || errno == EINTR)) {
                        continue;
                    }
                    break;
                }
                offset += (uint32_t)sent;
            }
        }
    }
}

/* -------------------------------------------------------------------
 *  Peer data handling
 * ------------------------------------------------------------------- */
static void handle_peer_data(int idx)
{
    struct peer *p = &peers[idx];
    ssize_t n;
    uint32_t net_len, msg_len, consumed;

    n = read(p->fd, p->buf + p->buf_used,
             RECV_BUF_SIZE - p->buf_used);
    if (n <= 0) {
        if (n < 0 && (errno == EAGAIN || errno == EINTR)) {
            return;
        }
        remove_peer(idx);
        return;
    }
    p->buf_used += (uint32_t)n;

    while (p->buf_used >= 4) {
        memcpy(&net_len, p->buf, 4);
        msg_len = ntohl(net_len);

        if (msg_len > MAX_MSG_SIZE) {
            fprintf(stderr, "hub: peer %d (%s) oversized msg (%u), "
                    "disconnecting\n", idx, p->label, msg_len);
            remove_peer(idx);
            return;
        }

        if (p->buf_used < 4 + msg_len) {
            break;
        }

        forward_message(idx, p->buf, 4 + msg_len);

        consumed = 4 + msg_len;
        if (consumed < p->buf_used) {
            memmove(p->buf, p->buf + consumed, p->buf_used - consumed);
        }
        p->buf_used -= consumed;
    }
}

/* -------------------------------------------------------------------
 *  TCP upstream connection + auto-reconnect
 * ------------------------------------------------------------------- */
static int connect_upstream(int upstream_idx)
{
    struct addrinfo hints, *res, *rp;
    int fd = -1;
    char label[80];
    int pidx;

    memset(&hints, 0, sizeof(hints));
    hints.ai_family = AF_UNSPEC;
    hints.ai_socktype = SOCK_STREAM;

    if (getaddrinfo(upstreams[upstream_idx].host,
                    upstreams[upstream_idx].port,
                    &hints, &res) != 0) {
        return -1;
    }

    for (rp = res; rp != NULL; rp = rp->ai_next) {
        fd = socket(rp->ai_family, rp->ai_socktype, rp->ai_protocol);
        if (fd < 0) {
            continue;
        }
        if (connect(fd, rp->ai_addr, rp->ai_addrlen) == 0) {
            break;
        }
        close(fd);
        fd = -1;
    }
    freeaddrinfo(res);

    if (fd < 0) {
        return -1;
    }

    set_tcp_nodelay(fd);
    snprintf(label, sizeof(label), "upstream[%s:%s]",
             upstreams[upstream_idx].host,
             upstreams[upstream_idx].port);

    pidx = add_peer(fd, 1, label);
    if (pidx < 0) {
        return -1;
    }
    upstream_state[upstream_idx].peer_idx = pidx;
    return 0;
}

static void try_reconnect_upstreams(void)
{
    int i;
    time_t now = time(NULL);

    for (i = 0; i < num_upstreams; i++) {
        if (upstream_state[i].peer_idx >= 0) {
            continue;
        }
        if (now - upstream_state[i].last_attempt < UPSTREAM_RETRY_SEC) {
            continue;
        }
        upstream_state[i].last_attempt = now;

        if (connect_upstream(i) == 0) {
            fprintf(stderr, "hub: reconnected to upstream %s:%s\n",
                    upstreams[i].host, upstreams[i].port);
        }
    }
}

/* -------------------------------------------------------------------
 *  Socket setup
 * ------------------------------------------------------------------- */
static int setup_tcp_listen(const char *port_str)
{
    struct addrinfo hints, *res;
    int fd, val = 1;

    memset(&hints, 0, sizeof(hints));
    hints.ai_family = AF_INET6;
    hints.ai_socktype = SOCK_STREAM;
    hints.ai_flags = AI_PASSIVE | AI_V4MAPPED;

    if (getaddrinfo(NULL, port_str, &hints, &res) != 0) {
        hints.ai_family = AF_INET;
        if (getaddrinfo(NULL, port_str, &hints, &res) != 0) {
            fprintf(stderr, "hub: cannot resolve TCP port %s\n", port_str);
            return -1;
        }
    }

    fd = socket(res->ai_family, res->ai_socktype, res->ai_protocol);
    if (fd < 0) {
        freeaddrinfo(res);
        return -1;
    }

    setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &val, sizeof(val));
    if (res->ai_family == AF_INET6) {
        val = 0;
        setsockopt(fd, IPPROTO_IPV6, IPV6_V6ONLY, &val, sizeof(val));
    }

    if (bind(fd, res->ai_addr, res->ai_addrlen) < 0) {
        perror("hub: TCP bind");
        close(fd);
        freeaddrinfo(res);
        return -1;
    }
    freeaddrinfo(res);

    if (listen(fd, 16) < 0) {
        perror("hub: TCP listen");
        close(fd);
        return -1;
    }

    set_nonblock(fd);
    return fd;
}

static int setup_unix_listen(const char *path)
{
    struct sockaddr_un addr;
    int fd;

    unlink(path);

    fd = socket(AF_UNIX, SOCK_STREAM, 0);
    if (fd < 0) {
        perror("hub: Unix socket");
        return -1;
    }

    memset(&addr, 0, sizeof(addr));
    addr.sun_family = AF_UNIX;
    strncpy(addr.sun_path, path, sizeof(addr.sun_path) - 1);

    if (bind(fd, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        perror("hub: Unix bind");
        close(fd);
        return -1;
    }

    if (listen(fd, 16) < 0) {
        perror("hub: Unix listen");
        close(fd);
        return -1;
    }

    set_nonblock(fd);
    return fd;
}

/* -------------------------------------------------------------------
 *  Cleanup and signal handling
 * ------------------------------------------------------------------- */
static void cleanup(void)
{
    int i;
    if (unix_listen_fd >= 0) {
        close(unix_listen_fd);
    }
    if (tcp_listen_fd >= 0) {
        close(tcp_listen_fd);
    }
    for (i = 0; i < num_peers; i++) {
        if (peers[i].fd >= 0) {
            close(peers[i].fd);
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

/* -------------------------------------------------------------------
 *  Usage
 * ------------------------------------------------------------------- */
static void usage(const char *prog)
{
    fprintf(stderr,
        "Usage: %s <unix-socket-path> [options]\n"
        "\n"
        "Options:\n"
        "  -t <port>        TCP listen port for incoming bridge connections\n"
        "  -u <host:port>   Connect to upstream hub (repeatable, max %d)\n"
        "  -v               Verbose logging (channel changes, frame routing)\n"
        "  -h               Show this help\n"
        "\n"
        "Channel filtering:\n"
        "  The hub inspects the v2 protocol header's channel_freq field.\n"
        "  Frames are only delivered to peers on the same channel.\n"
        "  Peers with channel_freq=0 (v1 clients, or before first TX)\n"
        "  receive all frames.\n"
        "\n"
        "  HT40 channel bonding is supported: if a frame carries a\n"
        "  non-zero channel_bond_freq, it is only delivered to peers\n"
        "  with matching primary+secondary channel pairs (or HT20 peers\n"
        "  on either the primary or secondary frequency).\n"
        "\n"
        "Examples:\n"
        "  # Local-only hub:\n"
        "  %s /tmp/ath9k.sock\n"
        "\n"
        "  # Hub with TCP bridge port:\n"
        "  %s /tmp/ath9k.sock -t 5550\n"
        "\n"
        "  # Hub bridged to a remote hub:\n"
        "  %s /tmp/ath9k.sock -u 192.168.1.10:5550\n"
        "\n",
        prog, MAX_UPSTREAMS, prog, prog, prog);
}

/* -------------------------------------------------------------------
 *  Main loop
 * ------------------------------------------------------------------- */
int main(int argc, char **argv)
{
    const char *tcp_port = NULL;
    struct pollfd *pfds = NULL;
    int nfds, i, opt;
    int tcp_listen_pfd, peer_base;

    if (argc < 2 || argv[1][0] == '-') {
        usage(argv[0]);
        return 1;
    }
    socket_path = argv[1];

    /* Parse options (appear after the positional socket path) */
    optind = 2;
    while ((opt = getopt(argc, argv, "t:u:vh")) != -1) {
        switch (opt) {
        case 't':
            tcp_port = optarg;
            break;
        case 'u':
            if (num_upstreams >= MAX_UPSTREAMS) {
                fprintf(stderr, "hub: too many -u (max %d)\n",
                        MAX_UPSTREAMS);
                return 1;
            }
            {
                char *colon = strrchr(optarg, ':');
                size_t hlen;
                if (!colon || colon == optarg) {
                    fprintf(stderr, "hub: bad -u format '%s' "
                            "(need host:port)\n", optarg);
                    return 1;
                }
                hlen = (size_t)(colon - optarg);
                if (hlen >= sizeof(upstreams[0].host)) {
                    hlen = sizeof(upstreams[0].host) - 1;
                }
                memcpy(upstreams[num_upstreams].host, optarg, hlen);
                upstreams[num_upstreams].host[hlen] = '\0';
                snprintf(upstreams[num_upstreams].port,
                         sizeof(upstreams[0].port), "%s", colon + 1);
                num_upstreams++;
            }
            break;
        case 'v':
            verbose = 1;
            break;
        case 'h':
            usage(argv[0]);
            return 0;
        default:
            usage(argv[0]);
            return 1;
        }
    }

    signal(SIGINT, sighandler);
    signal(SIGTERM, sighandler);
    signal(SIGPIPE, SIG_IGN);

    /* Setup listen sockets */
    unix_listen_fd = setup_unix_listen(socket_path);
    if (unix_listen_fd < 0) {
        return 1;
    }
    fprintf(stderr, "hub: Unix socket %s\n", socket_path);

    if (tcp_port) {
        tcp_listen_fd = setup_tcp_listen(tcp_port);
        if (tcp_listen_fd < 0) {
            cleanup();
            return 1;
        }
        fprintf(stderr, "hub: TCP port %s\n", tcp_port);
    }

    /* Initialize upstream state */
    for (i = 0; i < MAX_UPSTREAMS; i++) {
        upstream_state[i].peer_idx = -1;
        upstream_state[i].last_attempt = 0;
    }

    /* Connect to upstreams */
    for (i = 0; i < num_upstreams; i++) {
        fprintf(stderr, "hub: connecting upstream %s:%s...\n",
                upstreams[i].host, upstreams[i].port);
        upstream_state[i].last_attempt = time(NULL);
        if (connect_upstream(i) != 0) {
            fprintf(stderr, "hub: upstream %s:%s unavailable (will retry)\n",
                    upstreams[i].host, upstreams[i].port);
        }
    }

    fprintf(stderr, "hub: ready (max %d peers, TTL %d, %d upstreams, "
            "channel filtering ON, protocol v1+v2)\n",
            MAX_PEERS, DEFAULT_TTL, num_upstreams);

    pfds = calloc(2 + MAX_PEERS, sizeof(struct pollfd));
    if (!pfds) {
        perror("calloc");
        cleanup();
        return 1;
    }

    while (running) {
        nfds = 0;

        pfds[nfds].fd = unix_listen_fd;
        pfds[nfds].events = POLLIN;
        pfds[nfds].revents = 0;
        nfds++;

        tcp_listen_pfd = -1;
        if (tcp_listen_fd >= 0) {
            tcp_listen_pfd = nfds;
            pfds[nfds].fd = tcp_listen_fd;
            pfds[nfds].events = POLLIN;
            pfds[nfds].revents = 0;
            nfds++;
        }

        peer_base = nfds;
        for (i = 0; i < num_peers; i++) {
            pfds[nfds].fd = peers[i].fd;
            pfds[nfds].events = POLLIN;
            pfds[nfds].revents = 0;
            nfds++;
        }

        if (poll(pfds, (nfds_t)nfds, 1000) < 0) {
            if (errno == EINTR) {
                continue;
            }
            perror("poll");
            break;
        }

        /* Reconnect dropped upstreams */
        if (num_upstreams > 0) {
            try_reconnect_upstreams();
        }

        /* Accept Unix clients */
        if (pfds[0].revents & POLLIN) {
            int cfd = accept(unix_listen_fd, NULL, NULL);
            if (cfd >= 0) {
                add_peer(cfd, 0, "local-qemu");
            }
        }

        /* Accept TCP bridge clients */
        if (tcp_listen_pfd >= 0 &&
            (pfds[tcp_listen_pfd].revents & POLLIN)) {
            struct sockaddr_storage sa;
            socklen_t salen = sizeof(sa);
            int cfd = accept(tcp_listen_fd, (struct sockaddr *)&sa, &salen);
            if (cfd >= 0) {
                char lbl[64];
                char hbuf[48], sbuf[8];
                if (getnameinfo((struct sockaddr *)&sa, salen,
                                hbuf, sizeof(hbuf), sbuf, sizeof(sbuf),
                                NI_NUMERICHOST | NI_NUMERICSERV) == 0) {
                    snprintf(lbl, sizeof(lbl), "tcp[%.40s:%s]", hbuf, sbuf);
                } else {
                    snprintf(lbl, sizeof(lbl), "tcp-in[?]");
                }
                set_tcp_nodelay(cfd);
                add_peer(cfd, 1, lbl);
            }
        }

        /* Handle peer data (backwards for safe removal) */
        for (i = num_peers - 1; i >= 0; i--) {
            int pfd_idx = peer_base + i;
            if (pfd_idx < nfds &&
                (pfds[pfd_idx].revents & (POLLIN | POLLHUP | POLLERR))) {
                handle_peer_data(i);
            }
        }
    }

    fprintf(stderr, "hub: shutting down (%d peers)\n", num_peers);
    free(pfds);
    cleanup();
    return 0;
}
