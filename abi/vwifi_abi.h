/*
 * vwifi-virt — paravirtual virtual-Wi-Fi PCI device
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Shared ABI between the QEMU device model (host) and the Windows
 * WDI miniport driver (guest). Every structure in this file is part
 * of the device contract and must be kept binary-compatible once
 * external drivers ship.
 *
 * Endianness: all multi-byte fields are little-endian (guest native on
 * x86/x64/ARM64). This is a paravirtual device — we do not pretend to
 * be network-byte-order hardware.
 *
 * Layout of guest-visible state:
 *   BAR0 MMIO   — control registers (see "MMIO register map" below)
 *   BAR1 MSI-X  — table + PBA, one vector per ring
 *   Guest RAM   — four descriptor rings (see "Descriptor rings" below),
 *                 allocated by the driver, addresses programmed into
 *                 VWIFI_REG_*_RING_BASE.
 *
 * The underlying medium protocol (frames exchanged with the hub) is
 * defined in ath9k_medium.h from qemu-vwifi. This file does NOT
 * redefine those; the device translates between this ABI and the
 * medium protocol internally.
 */

#ifndef VWIFI_ABI_H
#define VWIFI_ABI_H

/*
 * Consumers of this header now include a Linux kernel module, where
 * <stdint.h> does not exist. The kernel's <linux/types.h> supplies the
 * whole fixed-width set this header uses -- signed included -- so the
 * struct definitions below need no other conditionals.
 *
 * The third consumer is the Windows WDI miniport, built with MSVC in
 * kernel mode. There <stdint.h> lives in the UCRT, which is not on the
 * kernel-mode include path, so the compiler's own sized types stand in.
 * Define VWIFI_ABI_HAVE_STDINT to skip the shim if some future MSVC
 * consumer does have <stdint.h> in scope.
 */
#ifdef __KERNEL__
#include <linux/types.h>
#elif defined(_MSC_VER) && !defined(VWIFI_ABI_HAVE_STDINT)
typedef unsigned __int8  uint8_t;
typedef unsigned __int16 uint16_t;
typedef unsigned __int32 uint32_t;
typedef unsigned __int64 uint64_t;
typedef signed   __int8  int8_t;
typedef signed   __int16 int16_t;
typedef signed   __int32 int32_t;
typedef signed   __int64 int64_t;
#else
#include <stdint.h>
#endif

/*
 * Structure packing. Every struct in this file is wire format shared
 * verbatim with the QEMU device, so all of them are byte-packed --
 * there is no unpacked struct here for a blanket #pragma pack to
 * silently reinterpret. GCC/Clang keep the per-struct attribute so
 * their layout is bit-identical to what it has always been; MSVC has
 * no such attribute and takes the pragma instead.
 */
#if defined(_MSC_VER)
#define VWIFI_PACKED
#pragma pack(push, 1)
#else
#define VWIFI_PACKED __attribute__((packed))
#endif

/* ================================================================
 * Identification
 * ================================================================ */

#define VWIFI_PCI_VENDOR_ID     0x1AF4  /* Red Hat (paravirt range) */
#define VWIFI_PCI_DEVICE_ID     0x0E00  /* vwifi-virt — not upstream, reserve! */
#define VWIFI_PCI_REVISION      0x01
#define VWIFI_PCI_CLASS         0x0280  /* Other network controller */

/* ABI version bumped whenever any structure in this file changes.
 * The driver reads VWIFI_REG_ABI_VERSION and refuses to bind on mismatch. */
#define VWIFI_ABI_VERSION       1

/* Signature the driver can read to detect a genuine vwifi-virt device
 * even if PCI IDs got reassigned. Spells "VWIF" in little-endian. */
#define VWIFI_SIGNATURE         0x46495756u

/* ================================================================
 * MMIO register map (BAR0, 4 KiB)
 *
 * All registers are 32-bit little-endian. RO = read-only from guest,
 * RW = read/write, W1C = write-one-to-clear, W = write-only.
 * ================================================================ */

/* Identification & capabilities (RO) */
#define VWIFI_REG_SIGNATURE         0x000   /* RO: VWIFI_SIGNATURE */
#define VWIFI_REG_ABI_VERSION       0x004   /* RO: VWIFI_ABI_VERSION */
#define VWIFI_REG_CAPS              0x008   /* RO: VWIFI_CAP_* bits */
#define VWIFI_REG_NUM_VECTORS       0x00C   /* RO: number of MSI-X vectors */

/* Global control (RW) */
#define VWIFI_REG_CTRL              0x010   /* RW: VWIFI_CTRL_* bits */
#define VWIFI_REG_STATUS            0x014   /* RO: VWIFI_STATUS_* bits */
#define VWIFI_REG_RESET             0x018   /* W:  write 1 to reset device */

/* Interrupt summary — bit N set means ring N has events.
 * Reading does not clear; driver acks by writing to RING_HEAD. */
#define VWIFI_REG_IRQ_STATUS        0x020   /* RO: per-ring event bits */
#define VWIFI_REG_IRQ_MASK          0x024   /* RW: 1 = mask (disable) vector */

/* Ring base addresses — driver sets each as guest physical address of
 * a contiguous ring buffer. All four must be programmed before the
 * driver writes VWIFI_CTRL_ENABLE. Rings are power-of-two sized; the
 * size is selected by the driver via the SIZE registers. */
#define VWIFI_REG_CTRL_REQ_RING_BASE_LO   0x040
#define VWIFI_REG_CTRL_REQ_RING_BASE_HI   0x044
#define VWIFI_REG_CTRL_REQ_RING_SIZE      0x048   /* entries, power of two */
#define VWIFI_REG_CTRL_REQ_RING_DOORBELL  0x04C   /* W: new producer index */

#define VWIFI_REG_CTRL_RSP_RING_BASE_LO   0x050
#define VWIFI_REG_CTRL_RSP_RING_BASE_HI   0x054
#define VWIFI_REG_CTRL_RSP_RING_SIZE      0x058
#define VWIFI_REG_CTRL_RSP_RING_HEAD      0x05C   /* RW: consumer index (ack) */

#define VWIFI_REG_TX_RING_BASE_LO         0x060
#define VWIFI_REG_TX_RING_BASE_HI         0x064
#define VWIFI_REG_TX_RING_SIZE            0x068
#define VWIFI_REG_TX_RING_DOORBELL        0x06C

#define VWIFI_REG_RX_RING_BASE_LO         0x070
#define VWIFI_REG_RX_RING_BASE_HI         0x074
#define VWIFI_REG_RX_RING_SIZE            0x078
#define VWIFI_REG_RX_RING_HEAD            0x07C   /* consumer index */

/* Producer indices the device advertises to the driver (RO). The
 * driver also maintains its own tail counts in the ring descriptor
 * "device_tail" fields; these are a convenience for quick polling. */
#define VWIFI_REG_CTRL_RSP_RING_TAIL      0x080   /* RO */
#define VWIFI_REG_RX_RING_TAIL            0x084   /* RO */

/* Diagnostic (RO) */
#define VWIFI_REG_DIAG_FRAMES_TX          0x0C0   /* RO: frames device->medium */
#define VWIFI_REG_DIAG_FRAMES_RX          0x0C4   /* RO: frames medium->device */
#define VWIFI_REG_DIAG_DROPS              0x0C8   /* RO: drop counter */

#define VWIFI_MMIO_SIZE             0x1000   /* 4 KiB BAR0 */

/* VWIFI_REG_CAPS bits */
#define VWIFI_CAP_MONITOR       (1u << 0)  /* Network Monitor op mode */
#define VWIFI_CAP_INJECT        (1u << 1)  /* raw TX injection */
#define VWIFI_CAP_STA           (1u << 2)  /* station (infra) op mode */
#define VWIFI_CAP_SOFTAP        (1u << 3)  /* SoftAP op mode */
#define VWIFI_CAP_WPA2          (1u << 4)  /* CCMP-128 offload */
#define VWIFI_CAP_WPA3          (1u << 5)  /* GCMP-256 + SAE offload */
#define VWIFI_CAP_HT            (1u << 8)  /* 802.11n */
#define VWIFI_CAP_VHT           (1u << 9)  /* 802.11ac */

/* VWIFI_REG_CTRL bits */
#define VWIFI_CTRL_ENABLE       (1u << 0)  /* start ring processing */
#define VWIFI_CTRL_IRQ_ENABLE   (1u << 1)  /* master IRQ enable */
/* Deliver STA data frames as 802.11 MPDUs instead of converting them to
 * 802.3. The frame arrives with its 802.11 header and LLC/SNAP intact
 * and VWIFI_RX_F_RAW set, decrypted if it was protected.
 *
 * Opt-in, and off by default, because it is the mirror of
 * VWIFI_TX_F_80211 and exists for the same reason: Windows' WDI stack
 * works in MPDUs in both directions. wdiwifi hands the miniport a
 * complete 802.11 frame to transmit, and on receive it expects one back
 * and does the 802.3 conversion itself. Handing it Ethernet makes it
 * read the destination MAC as frame control, find a frame that is not
 * data, and discard it without a word.
 *
 * The Linux driver wants 802.3 and never sets this bit, so its RX path
 * is unchanged. */
#define VWIFI_CTRL_RX_80211     (1u << 2)  /* RX stays 802.11, no re-encap */
#define VWIFI_CTRL_RESET        (1u << 31) /* same effect as REG_RESET */

/* VWIFI_REG_STATUS bits */
#define VWIFI_STATUS_READY      (1u << 0)  /* device ready for commands */
#define VWIFI_STATUS_LINK_UP    (1u << 1)  /* hub connection healthy */
#define VWIFI_STATUS_RESETTING  (1u << 2)

/* MSI-X vector assignments — index into the MSI-X table */
#define VWIFI_VEC_CTRL_RSP      0   /* control response ring has events */
#define VWIFI_VEC_RX            1   /* RX ring has frames */
#define VWIFI_VEC_TX_COMPLETE   2   /* TX completions (optional) */
#define VWIFI_VEC_EVENT         3   /* async events / link state */
#define VWIFI_NUM_VECTORS       4

/* ================================================================
 * Descriptor rings
 *
 * Each ring is a contiguous array of fixed-size descriptors in guest
 * RAM. The driver allocates the memory and programs the physical
 * address + size into MMIO.
 *
 * Producer/consumer semantics:
 *
 *   Control request (guest -> device):
 *     producer = driver  (driver writes entries, rings doorbell)
 *     consumer = device
 *
 *   Control response (device -> guest):
 *     producer = device
 *     consumer = driver  (driver writes head back to ack)
 *
 *   TX data (guest -> device):
 *     producer = driver
 *     consumer = device
 *
 *   RX data (device -> guest):
 *     producer = device
 *     consumer = driver
 *
 * All rings use 8-byte-aligned descriptors with an "owner" bit in
 * flags that toggles per wrap to allow lockless producer/consumer
 * progress without shared head pointers in memory. (Classic virtio-
 * style packed-ring idea, simplified.)
 * ================================================================ */

/* ---- Control request descriptor ---- */

struct vwifi_ctrl_req_desc {
    uint16_t opcode;         /* VWIFI_OP_* */
    uint16_t flags;          /* VWIFI_DESC_F_* */
    uint32_t req_id;         /* echoed back in response */
    uint64_t payload_addr;   /* guest physical addr of payload buffer */
    uint32_t payload_len;    /* payload length in bytes */
    uint32_t _reserved;
} VWIFI_PACKED;

/* ---- Control response descriptor ---- */

struct vwifi_ctrl_rsp_desc {
    uint16_t opcode;         /* matches request opcode, or event code */
    uint16_t flags;          /* VWIFI_DESC_F_* | VWIFI_RSP_F_* */
    uint32_t req_id;         /* zero if unsolicited event */
    int32_t  status;         /* 0 = OK, negative = errno-like */
    uint32_t payload_len;    /* bytes written to payload_addr */
    uint64_t payload_addr;   /* same buffer the request used (for sync)
                              * OR device-owned for async events */
    uint64_t _reserved;
} VWIFI_PACKED;

/* ---- TX data descriptor ---- */

struct vwifi_tx_desc {
    uint16_t flags;          /* VWIFI_DESC_F_* | VWIFI_TX_F_* */
    uint16_t frame_len;      /* 802.11 frame length in bytes */
    uint32_t _reserved0;
    uint64_t frame_addr;     /* guest physical addr of 802.11 frame */
    uint16_t channel_freq;   /* transmit channel (MHz), 0 = device default */
    uint8_t  rate_code;      /* ath9k rate code, 0 = device default */
    uint8_t  tid;            /* 802.11 TID 0-15, 0 for non-QoS */
    uint32_t _reserved1;
} VWIFI_PACKED;

/* ---- RX data descriptor ---- */

struct vwifi_rx_desc {
    uint16_t flags;          /* VWIFI_DESC_F_* | VWIFI_RX_F_* */
    uint16_t frame_len;      /* bytes written at frame_addr */
    int8_t   rssi;           /* dBm */
    uint8_t  rate_code;      /* ath9k rate code */
    uint16_t channel_freq;   /* RX channel (MHz) */
    uint64_t frame_addr;     /* guest physical addr of RX buffer (set by driver
                              * when posting a free buffer; device writes frame) */
    uint32_t buffer_len;     /* size of the RX buffer (set by driver) */
    uint64_t tsf;            /* 64-bit TSF timestamp at reception */
    uint32_t _reserved;
} VWIFI_PACKED;

/* Common descriptor flag bits (all rings) */
#define VWIFI_DESC_F_OWN        (1u << 0)  /* owner bit; producer sets, consumer clears */

/* Control response flags */
#define VWIFI_RSP_F_EVENT       (1u << 1)  /* unsolicited event (no matching req) */

/* TX descriptor flags */
#define VWIFI_TX_F_INJECT       (1u << 1)  /* raw injection (monitor-mode or mgmt) */
#define VWIFI_TX_F_NO_ACK       (1u << 2)  /* do not expect ACK from peer */
#define VWIFI_TX_F_ENCRYPTED    (1u << 3)  /* frame pre-encrypted by driver */
/* Payload is already a complete 802.11 frame: do NOT encapsulate it.
 *
 * The STA TX path's default contract is that the driver hands over
 * 802.3 and the device builds the 802.11 header. That is not something
 * every driver can honour. Windows WDI hands the miniport fully-formed
 * MPDUs -- wdiwifi owns the 802.11 MAC state, so it has already set
 * frame control, duration, the three addresses and the sequence number
 * by the time the miniport sees the frame -- and encapsulating one of
 * those a second time produces a frame whose "destination MAC" is the
 * first six bytes of the original 802.11 header. It goes out, the AP
 * drops it, and nothing anywhere reports an error.
 *
 * Distinct from VWIFI_TX_F_INJECT, which also skips the association
 * check and the cipher. This flag changes the FORMAT of the payload and
 * nothing else: the frame is still refused when not associated and
 * still encrypted when a key is installed.
 */
#define VWIFI_TX_F_80211        (1u << 4)  /* payload is 802.11, not 802.3 */

/* RX descriptor flags */
#define VWIFI_RX_F_FCS_OK       (1u << 1)
#define VWIFI_RX_F_DECRYPTED    (1u << 2)  /* device decrypted this frame */
/* The frame above the descriptor is an 802.11 MPDU, not 802.3. Set for
 * every monitor-mode capture, and — since VWIFI_CTRL_RX_80211 — for STA
 * data frames too when the driver asked for them unconverted. It says
 * what SHAPE the bytes are, not which mode produced them; a receiver
 * that reads it as "this is monitor mode" is reading more into it than
 * it carries. */
#define VWIFI_RX_F_RAW          (1u << 3)  /* payload is 802.11, not 802.3 */

/* ================================================================
 * Control opcodes
 *
 * Payload layouts for each opcode are defined below. Opcodes are
 * stable once assigned.
 * ================================================================ */

enum vwifi_opcode {
    VWIFI_OP_NOP             = 0x0000,
    VWIFI_OP_GET_CAPS        = 0x0001,  /* -> struct vwifi_caps */
    VWIFI_OP_SET_STA_MAC     = 0x0002,  /* <- 6 bytes */
    VWIFI_OP_SET_OP_MODE     = 0x0010,  /* <- struct vwifi_op_mode */
    VWIFI_OP_SET_CHANNEL     = 0x0011,  /* <- struct vwifi_channel */
    VWIFI_OP_SET_RAW_FILTER  = 0x0012,  /* <- struct vwifi_raw_filter */
    VWIFI_OP_SCAN            = 0x0020,  /* <- struct vwifi_scan_req */
    VWIFI_OP_SCAN_ABORT      = 0x0021,
    VWIFI_OP_CONNECT         = 0x0030,  /* <- struct vwifi_connect_req */
    VWIFI_OP_DISCONNECT      = 0x0031,  /* <- struct vwifi_disconnect_req */
    VWIFI_OP_SET_KEY         = 0x0040,  /* <- struct vwifi_key */
    VWIFI_OP_DEL_KEY         = 0x0041,  /* <- struct vwifi_key_id */
    VWIFI_OP_START_AP        = 0x0050,  /* <- struct vwifi_ap_config */
    VWIFI_OP_STOP_AP         = 0x0051,
};

/* Async event codes (in vwifi_ctrl_rsp_desc.opcode when _F_EVENT set) */
enum vwifi_event {
    VWIFI_EV_BSS_FOUND       = 0x8001,  /* -> struct vwifi_bss_entry */
    VWIFI_EV_SCAN_COMPLETE   = 0x8002,  /* -> int32 status */
    VWIFI_EV_ASSOC_RESULT    = 0x8003,  /* -> struct vwifi_assoc_result */
    VWIFI_EV_DISCONNECTED    = 0x8004,  /* -> struct vwifi_disconnect_ev */
    VWIFI_EV_MGMT_RX         = 0x8005,  /* -> struct vwifi_mgmt_rx (mgmt frames in STA mode) */
    VWIFI_EV_KEY_INSTALLED   = 0x8006,  /* -> struct vwifi_key_id */
    VWIFI_EV_LINK_STATE      = 0x8007,  /* -> uint32 link_up */
};

/* ================================================================
 * Command payload structures
 * ================================================================ */

struct vwifi_caps {
    uint32_t abi_version;
    uint32_t caps;                  /* VWIFI_CAP_* */
    uint8_t  default_mac[6];        /* device-suggested MAC */
    uint8_t  _reserved0[2];
    uint32_t supported_channels_24; /* bitmask: bit N = channel N (1..14) */
    uint64_t supported_channels_5;  /* bitmask of 5 GHz channels */
    uint16_t max_scan_ssids;
    uint16_t max_bss_entries;
    uint32_t _reserved1;
} VWIFI_PACKED;

enum vwifi_mode {
    VWIFI_MODE_IDLE     = 0,
    VWIFI_MODE_STA      = 1,
    VWIFI_MODE_MONITOR  = 2,
    VWIFI_MODE_SOFTAP   = 3,
};

struct vwifi_op_mode {
    uint32_t mode;                  /* enum vwifi_mode */
} VWIFI_PACKED;

struct vwifi_channel {
    uint16_t primary_freq;          /* MHz */
    uint16_t flags;                 /* ATH9K_CHAN_FLAG_* from ath9k_medium.h */
    uint16_t bond_freq;             /* HT40 secondary, 0 if none */
    uint16_t center_freq1;          /* VHT center freq of primary segment */
    uint16_t center_freq2;          /* VHT80+80 center of secondary, 0 if none */
    uint16_t _reserved;
} VWIFI_PACKED;

struct vwifi_raw_filter {
    uint32_t mask;                  /* VWIFI_RAW_F_* */
} VWIFI_PACKED;

#define VWIFI_RAW_F_DATA         (1u << 0)  /* raw data MPDUs */
#define VWIFI_RAW_F_MGMT         (1u << 1)  /* raw mgmt MPDUs */
#define VWIFI_RAW_F_CTRL         (1u << 2)  /* raw control frames */
#define VWIFI_RAW_F_FCS_FAIL     (1u << 3)  /* include FCS failures */

struct vwifi_scan_req {
    uint32_t flags;                 /* reserved for active/passive */
    uint32_t channel_mask_24;       /* bit N = scan channel N (1..14); 0 = all */
    uint64_t channel_mask_5;
    uint16_t dwell_ms;              /* per-channel dwell time */
    uint16_t num_ssids;             /* 0 = broadcast probe */
    /*
     * followed by num_ssids × { uint8 len; uint8 ssid[33]; }
     *
     * An entry with len == 0 is the 802.11 wildcard SSID, not a literal
     * empty name: it makes the device probe with a zero-length SSID IE
     * AND report every BSS it hears, exactly as num_ssids == 0 does.
     * This is not a nicety -- cfg80211 puts one zero-length SSID in
     * essentially every scan request, so a device that compares it
     * literally reports only hidden APs and looks like it is sitting on
     * a dead medium.
     */
} VWIFI_PACKED;

/* Set in vwifi_bss_entry.capability_info bit 16 to tell the driver
 * whether the attached frame is a Beacon or a Probe Response — WDI has
 * a separate TLV for each (WDI_TLV_BEACON_FRAME vs
 * WDI_TLV_PROBE_RESPONSE_FRAME). Low 16 bits stay the 802.11 capability
 * field.
 *
 * One event carries one frame, so a BSS the device has heard both a
 * beacon and a probe response from produces TWO VWIFI_EV_BSS_FOUND
 * events with the same BSSID, emitted back to back and distinguished
 * only by this bit. Drivers merge them by BSSID: cfg80211 does it for
 * free in cfg80211_inform_bss_frame_data(), and the WDI driver folds
 * them into one WDI_TLV_BSS_ENTRY carrying both blobs.
 *
 * That is the point of the split. The two frames are separate facts
 * about a BSS and the host consumes them separately, so a single frame
 * slot per BSS -- which is what this ABI used to imply -- silently
 * threw one of them away. */
#define VWIFI_BSS_F_BEACON   (1u << 16)

struct vwifi_bss_entry {
    uint8_t  bssid[6];
    uint8_t  _reserved0[2];
    uint16_t channel_freq;
    uint16_t channel_flags;
    int8_t   rssi;
    uint8_t  _reserved1[3];
    uint32_t beacon_period_tu;
    uint32_t capability_info;       /* low 16 bits = 802.11 capability field;
                                     * bit 16 = VWIFI_BSS_F_BEACON */
    uint64_t tsf;
    uint16_t ie_len;                /* length of the raw frame that follows */
    uint16_t ssid_len;
    uint8_t  ssid[33];
    uint8_t  _reserved2[3];
    /* Followed by ie_len bytes: the WHOLE beacon / probe-response frame,
     * starting at the 802.11 MAC header, not just the IE tail. The name
     * is historical and misleading -- it is a frame length, not an IE
     * length.
     *
     * The full frame is what Linux wants:
     * cfg80211_inform_bss_frame_data() takes a struct ieee80211_mgmt,
     * MAC header included.
     *
     * Windows wants the opposite, and each consumer is responsible for
     * its own trimming. WDI's beacon/probe-response byte blobs are
     * documented as NOT including the 802.11 MAC header, so the Windows
     * driver skips VWIFI_80211_MGMT_HDR_LEN bytes before handing the
     * frame over. Getting that wrong is not a parse error: the OS reads
     * the first eight bytes of the MAC header as the beacon timestamp
     * and starts looking for information elements inside addr2, finds no
     * SSID, and reports the network as hidden. */
} VWIFI_PACKED;

/* 802.11 management frame header: frame control, duration, three
 * addresses, sequence control. The frame body -- fixed parameters then
 * information elements -- starts here. */
#define VWIFI_80211_MGMT_HDR_LEN  24

struct vwifi_connect_req {
    uint8_t  bssid[6];              /* specific AP to target */
    uint16_t ssid_len;
    uint8_t  ssid[33];
    uint8_t  _reserved0[1];
    uint16_t channel_freq;
    uint16_t auth_algo;             /* VWIFI_AUTH_* */
    uint16_t cipher_pairwise;       /* VWIFI_CIPHER_* */
    uint16_t cipher_group;
    uint16_t akm_suite;             /* VWIFI_AKM_* */
    uint16_t assoc_ie_len;
    uint16_t _reserved1;
    /* followed by assoc_ie_len bytes of IEs to include in assoc request */
} VWIFI_PACKED;

enum {
    VWIFI_AUTH_OPEN          = 0,
    VWIFI_AUTH_SHARED        = 1,
    VWIFI_AUTH_SAE           = 2,
};

enum {
    VWIFI_CIPHER_NONE        = 0,
    VWIFI_CIPHER_WEP40       = 1,
    VWIFI_CIPHER_WEP104      = 2,
    VWIFI_CIPHER_TKIP        = 3,
    VWIFI_CIPHER_CCMP128     = 4,
    VWIFI_CIPHER_GCMP256     = 5,
};

enum {
    VWIFI_AKM_NONE           = 0,
    VWIFI_AKM_PSK            = 1,   /* WPA2-PSK */
    VWIFI_AKM_SAE            = 2,   /* WPA3-SAE */
    VWIFI_AKM_8021X          = 3,
};

struct vwifi_disconnect_req {
    uint16_t reason_code;           /* 802.11 reason code */
    uint16_t _reserved;
} VWIFI_PACKED;

struct vwifi_assoc_result {
    uint8_t  bssid[6];
    uint8_t  _reserved0[2];
    uint16_t status_code;           /* 0 = success, 802.11 status code otherwise */
    uint16_t aid;
    uint16_t ie_len;                /* assoc response FRAME, whole */
    /* The association REQUEST frame this station sent, whole.
     *
     * Frames, not IE blocks, on both of these. WDI defines
     * WDI_TLV_ASSOCIATION_REQUEST_FRAME and
     * WDI_TLV_ASSOCIATION_REQUEST_IES separately, and the driver
     * reports these as the frames; handing an IE block to the frame TLV
     * made Windows reject an otherwise perfect association in
     * thirty-three milliseconds.
     *
     * The four-way handshake is authenticated over the two RSN elements
     * that were exchanged, so a supplicant needs the one the station
     * sent as well as the one the AP answered with. The device builds
     * the station's RSN element itself -- WDI never supplies one -- so
     * without this the OS has no way to know what it associated with.
     *
     * This was _reserved1, and takes its place rather than extending
     * the structure: the size is unchanged, every existing field stays
     * where it was, and a reader that does not know about this simply
     * reads ie_len bytes and stops, as it always did. */
    uint16_t req_ie_len;
    /* followed by ie_len bytes of the AP's assoc response frame,
     * then req_ie_len bytes of our own assoc request frame */
} VWIFI_PACKED;

struct vwifi_disconnect_ev {
    uint16_t reason_code;
    uint8_t  local;                 /* 1 = locally initiated */
    uint8_t  _reserved;
} VWIFI_PACKED;

struct vwifi_mgmt_rx {
    uint16_t frame_len;
    int8_t   rssi;
    uint8_t  _reserved0;
    uint16_t channel_freq;
    uint16_t _reserved1;
    /* followed by frame_len bytes of raw 802.11 management frame */
} VWIFI_PACKED;

struct vwifi_key_id {
    uint8_t  mac[6];                /* peer MAC; all-zero = group key */
    uint8_t  key_idx;               /* 0..3 */
    uint8_t  pairwise;              /* 1 = pairwise, 0 = group */
} VWIFI_PACKED;

struct vwifi_key {
    struct vwifi_key_id id;
    uint16_t cipher;                /* VWIFI_CIPHER_* */
    uint16_t key_len;               /* 16 for CCMP128, 32 for GCMP256 */
    uint8_t  key[32];
} VWIFI_PACKED;

struct vwifi_ap_config {
    uint16_t ssid_len;
    uint8_t  ssid[33];
    uint8_t  _reserved0[1];
    uint16_t channel_freq;
    uint16_t beacon_interval_tu;
    uint16_t dtim_period;
    uint16_t ie_len;                /* beacon/probe-resp IEs */
    uint16_t _reserved1;
    /* followed by ie_len bytes of IEs */
} VWIFI_PACKED;

#if defined(_MSC_VER)
#pragma pack(pop)
#endif

#endif /* VWIFI_ABI_H */
