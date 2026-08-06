/*
 * vwifi — Windows WDI miniport for the vwifi-virt QEMU device
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Driver-private header. Defines the per-adapter context and
 * the internal API between driver.c / hardware.c / rings.c /
 * wdi_ops.c.
 */

#pragma once

#include <ntifs.h>
#include <ndis.h>
#include <dot11wdi.h>
#include <windot11.h>
/* RtlStringCch* -- the safe-string routines. Shared rather than
 * per-file: the port-0xE9 logger needs them, and so does anything that
 * assembles a trace line, which is the one thing every file here ends
 * up doing. */
#include <ntstrsafe.h>

#include "vwifi_abi.h"
/* NOTE: the WDI TLV library is C++ (TlvGeneratorParser.hpp, plus C++
 * new/delete). It is deliberately NOT included here — this header is
 * pulled in by every .c file, and a C compiler cannot parse a C++
 * header. All C++ is contained in tlv_shim.cpp / tlv_mem.cpp and
 * reaches the rest of the driver through this plain-C boundary. */
#include "tlv_shim.h"

/* ============================================================
 * Build-time configuration
 * ============================================================ */

#define VWIFI_POOL_TAG          'fiWv'   /* 'vWif' when viewed in WinDbg */
#define VWIFI_DRIVER_NAME       L"vwifi"

#define VWIFI_NDIS_MAJOR_VERSION  6
#define VWIFI_NDIS_MINOR_VERSION  50   /* Windows 10 1507+ — supports WDI */

/* Default ring sizes. Must be powers of two and agreed with QEMU
 * by writing them into the corresponding *_RING_SIZE MMIO regs. */
#define VWIFI_CTRL_REQ_RING_SIZE   64
#define VWIFI_CTRL_RSP_RING_SIZE   64
#define VWIFI_TX_RING_SIZE         256
#define VWIFI_RX_RING_SIZE         256

/* RX frame buffer size — 802.11 maximum MSDU is 2346 bytes but A-MSDU
 * aggregation can go larger. Match the medium protocol's ceiling. */
#define VWIFI_RX_BUFFER_SIZE       4096

/* Ctrl request/response payload scratch buffer size. Every opcode's
 * payload today fits well under this. */
#define VWIFI_CTRL_PAYLOAD_SIZE    2048

/* Scan limits. MAX_FRAME must match the device's VWIFI_BSS_FRAME_MAX —
 * we stage whole beacon frames because WDI's BSS entry TLV wants the
 * raw frame, not parsed IEs. */
#define VWIFI_SCAN_MAX_FRAME     768
#define VWIFI_SCAN_MAX_SSIDS       4
#define VWIFI_SCAN_PENDING_MAX    16
/* How many BSSes the driver remembers between scans, to answer
 * OID_WDI_GET_BSS_ENTRY_LIST. */
#define VWIFI_SCAN_CACHE_MAX      32

/* Stamped in by the build; see the PreprocessorDefinitions in
 * vwifi.vcxproj. Two levels of macro so the argument is expanded before
 * it is stringized. */
#ifndef VWIFI_BUILD_VERSION
#define VWIFI_BUILD_VERSION 0.0.0.0
#endif
#define VWIFI_STR2(x) #x
#define VWIFI_STR(x)  VWIFI_STR2(x)

/* ============================================================
 * Logging
 *
 * Two sinks, because DbgPrintEx on its own loses exactly the failures
 * that are worth having. Its output goes to an attached kernel debugger
 * -- DebugView is one -- and a guest that hard-locks takes DebugView's
 * buffer down with it. Boot-time output is lost the same way: nothing
 * is running yet to catch it. Every silent freeze in this driver's
 * bring-up has been invisible for that reason and no other.
 *
 * VWIFI_E9 mirrors each line to I/O port 0xE9, QEMU's debug console.
 * QEMU writes every byte to its -debugcon file the moment the guest
 * emits it, on the host, outside the VM. A trace collected that way
 * survives a hard lock, a triple fault and a reset alike, and it is the
 * only thing that can say what the driver was doing at the instant a
 * machine froze at the Windows logo.
 *
 * Port 0xE9 is unassigned on real hardware and writes to it are
 * discarded, so this is inert outside a VM. It is Debug-only anyway: a
 * release driver has no business writing to arbitrary I/O ports.
 *
 * Host side:
 *     -debugcon file:/tmp/vwifi-boot.log
 * ============================================================ */

/* The "vwifi: " prefix and the timestamp are added by VwifiE9Printf,
 * not here: the time has to be read when the line is emitted, and a
 * format string cannot do that. */
#if DBG
VOID VwifiE9Printf(_In_z_ _Printf_format_string_ PCSTR Format, ...);
#define VWIFI_E9(fmt, ...) VwifiE9Printf(fmt "\n", ##__VA_ARGS__)
#else
#define VWIFI_E9(fmt, ...) ((VOID)0)
#endif

#define VWIFI_DBG(level, fmt, ...)                                  \
    do {                                                            \
        DbgPrintEx(DPFLTR_IHVNETWORK_ID, level,                     \
                   "vwifi: " fmt "\n", ##__VA_ARGS__);              \
        VWIFI_E9(fmt, ##__VA_ARGS__);                               \
    } while (0)

#define VWIFI_INFO(fmt, ...)  VWIFI_DBG(DPFLTR_INFO_LEVEL,    fmt, ##__VA_ARGS__)
#define VWIFI_WARN(fmt, ...)  VWIFI_DBG(DPFLTR_WARNING_LEVEL, fmt, ##__VA_ARGS__)
#define VWIFI_ERR(fmt, ...)   VWIFI_DBG(DPFLTR_ERROR_LEVEL,   fmt, ##__VA_ARGS__)

/* ============================================================
 * Ring state (driver side)
 *
 * For producer rings (ctrl-req, tx): the driver owns `next_free`,
 * which is the index where it will write the next descriptor. It
 * sets OWN=1 on that descriptor and advances. QEMU clears OWN when
 * done.
 *
 * For consumer rings (ctrl-rsp, rx): the driver owns `next_head`,
 * which is the index it will read next. QEMU sets OWN=1 when it
 * writes a descriptor. The driver processes, clears OWN, advances,
 * and writes `next_head` back to the RING_HEAD MMIO register so
 * QEMU can deassert the corresponding MSI-X vector.
 * ============================================================ */

typedef struct _VWIFI_RING
{
    /* DMA-coherent allocation covering the whole ring. */
    NDIS_HANDLE        NblPool;           /* unused, reserved for future */
    PVOID              VirtualAddress;    /* CPU-mapped ring base */
    NDIS_PHYSICAL_ADDRESS PhysicalAddress;/* guest-physical ring base */
    ULONG              SizeBytes;         /* total allocation size */
    ULONG              NumDescs;          /* descriptor count */
    ULONG              DescSize;          /* bytes per descriptor */
    ULONG              Mask;              /* NumDescs - 1 */

    /* Driver-owned index (next to produce or consume). */
    volatile ULONG     NextIndex;

    /* MMIO register offsets used to program this ring. */
    ULONG              RegBaseLo;
    ULONG              RegBaseHi;
    ULONG              RegSize;
    ULONG              RegDoorbell;       /* producer rings only */
    ULONG              RegHead;           /* consumer rings only */

    PCSTR              Name;              /* "ctrl-req" etc. */
} VWIFI_RING, *PVWIFI_RING;

/* ============================================================
 * RX NBL context
 *
 * Both the monitor drain (monitor.c) and the STA drain (data.c)
 * produce RX NBLs, and both are reclaimed by the single
 * VwifiMiniportReturnNetBufferLists in monitor.c. The context
 * layout must therefore be shared, not duplicated per-file.
 * ============================================================ */

typedef struct _VWIFI_RX_NBL_CONTEXT
{
    ULONG SlotIndex;   /* which RX ring slot backs this NBL */
} VWIFI_RX_NBL_CONTEXT, *PVWIFI_RX_NBL_CONTEXT;

/* The context size actually asked of NDIS.
 *
 * Not sizeof(VWIFI_RX_NBL_CONTEXT). That is four bytes, and an NBL
 * context is required to be a multiple of MEMORY_ALLOCATION_ALIGNMENT
 * -- sixteen on x64. Requesting four got NULL back from
 * NdisAllocateNetBufferAndNetBufferList for every RX frame the device
 * ever delivered, which read in the trace as "rx(sta): NBL alloc
 * failed" and, one layer up, as an associated link that could not
 * complete DHCP.
 *
 * The pool is created with this size so every NBL it hands out already
 * carries the context, and the allocation calls ask for none. */
#define VWIFI_RX_NBL_CONTEXT_SIZE                                     \
    ((ULONG)(((sizeof(VWIFI_RX_NBL_CONTEXT) +                         \
               MEMORY_ALLOCATION_ALIGNMENT - 1) /                     \
              MEMORY_ALLOCATION_ALIGNMENT) * MEMORY_ALLOCATION_ALIGNMENT))

/* ============================================================
 * Pending control-request tracking
 *
 * When the OID layer wants to send a synchronous command, it
 * allocates a VWIFI_PENDING_REQ, enqueues it, and blocks on
 * the CompletionEvent until the ctrl-rsp ring worker wakes it.
 * ============================================================ */

typedef struct _VWIFI_PENDING_REQ
{
    LIST_ENTRY       Link;
    ULONG            ReqId;
    USHORT           Opcode;
    KEVENT           CompletionEvent;
    NTSTATUS         Status;
    ULONG            ReplyPayloadLen;
    PVOID            ReplyPayload;       /* driver-allocated, filled by rsp worker */
} VWIFI_PENDING_REQ, *PVWIFI_PENDING_REQ;

/* ============================================================
 * WDI peers
 *
 * One entry per peer the WLAN component has been told about. See
 * wdi_peer.c for what a peer is for and why the id is ours to pick.
 *
 * Eight because that is the MaxNumPeers this device's component
 * reports in TalTxRxStart ("TAL start: max 5 ports, 8 peers"), and a
 * station needs exactly one. The array is sized to the ceiling and the
 * runtime value clamps allocation, so a component that reports fewer
 * cannot be handed an id it has no room for.
 * ============================================================ */
#define VWIFI_MAX_PEERS 8

typedef struct _VWIFI_PEER
{
    BOOLEAN     InUse;
    /* Told to the component, not yet confirmed gone. The slot cannot be
     * reused in this state: the component still has state for it. */
    BOOLEAN     DeletePending;
    /* TalTxRxPeerConfigHandler has run, so the component has finished
     * setting the peer up and data may flow to it. */
    BOOLEAN     Configured;
    WDI_PEER_ID PeerId;
    WDI_PORT_ID WdiPortId;
    UCHAR       Mac[6];
} VWIFI_PEER, *PVWIFI_PEER;

/* ============================================================
 * Adapter context
 *
 * Allocated in MiniportWdiAllocateAdapter, freed in
 * MiniportWdiFreeAdapter. Lives for the duration of one PnP
 * device lifetime.
 * ============================================================ */

typedef struct _VWIFI_ADAPTER
{
    /* NDIS handles. */
    NDIS_HANDLE         MiniportAdapterHandle;
    NDIS_HANDLE         DriverContext;
    PDEVICE_OBJECT      PhysicalDeviceObject;

    /* Completion routines the OS handed us in NDIS_WDI_INIT_PARAMETERS.
     * OpenAdapter and CloseAdapter are asynchronous in WDI: the handler
     * returns NDIS_STATUS_PENDING (or SUCCESS) and the operation is only
     * finished when we call the matching complete routine. Dropping
     * these on the floor wedges the WLAN component at start-up with no
     * error anywhere. */
    NDIS_WDI_OPEN_ADAPTER_COMPLETE_HANDLER  OpenAdapterCompleteHandler;
    NDIS_WDI_CLOSE_ADAPTER_COMPLETE_HANDLER CloseAdapterCompleteHandler;

    /* The WDI version the running OS gave us at init. Threaded into
     * every TLV_CONTEXT so the parser/generator emits and consumes a
     * byte stream matching this peer. This is the mechanism that lets
     * one binary serve Windows 10 (WDI 1.1.9) and Windows 11 — see
     * "WDI TLV versioning". Never hardcode this. */
    ULONG               WdiPeerVersion;

    /* DMA.
     *
     * The rings are common buffers obtained straight from WDM via
     * IoGetDmaAdapter, not from NdisMAllocateSharedMemory. That NDIS
     * routine needs NDIS_MINIPORT_ATTRIBUTES_BUS_MASTER to be in effect,
     * and in the WDI model the attributes we fill in during
     * AllocateAdapter are applied by the WLAN component, which does not
     * carry our AttributeFlags through -- so it fails with
     * NDIS_STATUS_RESOURCES no matter when it is called. The DMA adapter
     * belongs to the PDO and does not care about any of that. */
    PDMA_ADAPTER        DmaAdapter;
    ULONG               DmaMapRegisters;

    /* The host's port, from OID_WDI_TASK_CREATE_PORT. WDI ports are
     * the component's abstraction over one virtual interface; the host
     * assigns the id and a single-radio device has exactly one, so
     * there is no table here -- only which id is live. */
    WDI_PORT_ID         WdiPortId;

    /* The NDIS port number the host assigned this port, out of the
     * CREATE_PORT request's WDI_TLV_PORT_ATTRIBUTES.
     *
     * NOT NDIS_OID_REQUEST.PortNumber, which is the port the CREATE_PORT
     * request itself arrived on -- an adapter-scoped request, so zero.
     * This is the port the OS will address the station by, and every
     * NDIS status indication about that port has to carry it or the
     * indication lands on the default port and the station port is
     * never told anything. */
    ULONG               NdisPortNumber;

    /* The component's receive filter for this port, from
     * OID_WDI_SET_RECEIVE_PACKET_FILTER. NDIS packet-type bits, not the
     * device's raw-capture mask -- recorded so the trace can show what
     * was asked for, not consulted by the RX path. */
    ULONG               WdiPacketFilter;
    BOOLEAN             PortCreated;

    /* The WDI data path, captured in TalTxRxInitialize. The TAL
     * callbacks are handed a TAL_TXRX_HANDLE rather than the adapter,
     * so anything that has to call back into the WLAN component needs
     * these reachable from here. */
    NDIS_HANDLE         DataPathHandle;
    PNDIS_WDI_DATA_API  DataPathApi;

    /* PCI resources (discovered in hardware.c). */
    PHYSICAL_ADDRESS    MmioPhysicalAddress;
    PVOID               MmioVirtualAddress;
    ULONG               MmioLength;

    /* Interrupt. */
    NDIS_HANDLE         InterruptHandle;
    PIO_INTERRUPT_MESSAGE_INFO MessageInfo;

    /* Rings. */
    VWIFI_RING          CtrlReqRing;
    VWIFI_RING          CtrlRspRing;
    VWIFI_RING          TxRing;
    VWIFI_RING          RxRing;

    /* RX buffer pool — each RX descriptor points here. */
    PVOID               RxBufferPoolVa;
    NDIS_PHYSICAL_ADDRESS RxBufferPoolPa;
    ULONG               RxBufferPoolSize;

    /* TX buffer pool — per-slot staging for injected frames. */
    PVOID               TxBufferPoolVa;
    NDIS_PHYSICAL_ADDRESS TxBufferPoolPa;
    ULONG               TxBufferPoolSize;

    /* Phase 1.5 monitor-mode receive. */
    NDIS_HANDLE         RxNblPool;

    /* RX ring slots handed to an NBL and not yet given back.
     *
     * The RX drain decides "is there a frame here?" from the
     * descriptor's OWN bit alone, and that is only sound while every
     * consumed slot is re-armed before the consumer index laps it. It
     * is not: a slot stays un-armed until its NBL comes back through
     * VwifiMiniportReturnNetBufferLists, which can be arbitrarily
     * later.
     *
     * Let enough frames arrive before the returns catch up and the
     * consumer index wraps onto a slot it already consumed -- OWN
     * clear, stale frame_len -- which reads exactly like a fresh
     * frame. It is consumed again, and so is the next, all the way
     * round a ring in which nothing is armed: an unbounded loop
     * allocating NBLs at DISPATCH_LEVEL, which takes the machine with
     * it and leaves no trace, because the loop never returns to
     * anything that could log.
     *
     * Counted so the drain can stop one slot short of exhausting the
     * ring, which keeps the lap from ever happening. The drain is also
     * bounded by the descriptor count, so the loop terminates even if
     * this reasoning is wrong. */
    volatile LONG       RxOutstanding;
    /* One DOT11_EXTSTA_RECV_CONTEXT per RX slot, referenced by the
     * NBL's MediaSpecificInformation OOB pointer while in flight. */
    struct DOT11_EXTSTA_RECV_CONTEXT *RxRecvContext;

    /* Set once VwifiHwStart has run. Adapter creation and adapter
     * start are separate in the WDI model — see hardware.c. */
    BOOLEAN             Started;

    /* Current operating mode + channel, mirrored to the device. */
    ULONG               OpMode;         /* enum vwifi_mode */

    /* An operation mode the TAL asked for above PASSIVE_LEVEL, waiting
     * for the work item that can actually send it. -1 (or any negative)
     * means nothing is pending. See VwifiTalApplyOpMode. */
    volatile LONG       PendingOpMode;
    ULONG               RawFilter;      /* VWIFI_RAW_F_* */
    USHORT              CurrentFreq;    /* MHz */

    /* Phase 2 scan task state (opaque; see wdi_scan.c). */
    struct _VWIFI_SCAN_TASK *ScanTask;

    /* Phase 3 connect task state (opaque; see wdi_connect.c). */
    struct _VWIFI_CONNECT_TASK *ConnectTask;

    /* Association state, mirrored from the device's ASSOC_RESULT /
     * DISCONNECTED events. Gates the STA data path. */
    BOOLEAN             Associated;
    UCHAR               Bssid[6];

    /* WDI peers. See wdi_peer.c.
     *
     * The WDI data path is addressed by (port, peer, TID), never by
     * port alone, and until the component is told a peer exists its TX
     * queues stay paused on WDI_TX_PAUSE_REASON_PEER_CREATE. A station
     * has exactly one peer -- the AP -- but the table is a table
     * because the ids are ours to assign and the delete is
     * asynchronous, so a slot outlives the association that made it.
     *
     * MaxPeers/MaxPorts come from the component in TalTxRxStart and
     * bound what may be handed back to it. */
    VWIFI_PEER          Peers[VWIFI_MAX_PEERS];
    UINT8               MaxPeers;
    UINT8               MaxPorts;

    /* RX frames indicated to the component and not yet collected.
     *
     * WDI's RX is two calls, not one: RxInorderDataIndication says
     * frames exist for a (peer, TID) and the component then calls back
     * through RxGetMpdusHandler to take them. They have to be parked
     * somewhere in between, and that is here. */
    PNET_BUFFER_LIST    RxQueueHead;
    PNET_BUFFER_LIST    RxQueueTail;
    ULONG               RxQueueCount;
    KSPIN_LOCK          RxQueueLock;

    /* Non-zero once RxInorderDataIndication has answered
     * NDIS_STATUS_PAUSED.
     *
     * That status is the component saying "stop indicating"; the
     * matching RxResumeHandler is how it says the other thing. This
     * driver logged the pause and carried on indicating into it, which
     * is a contract violation whatever else is true. Frames are still
     * queued while paused -- nothing is dropped -- they are simply not
     * announced until the resume arrives. */
    volatile LONG       RxPaused;

    /* Re-entrancy guard for the TX pump, and a note that it was needed
     * while it was running.
     *
     * Three TAL handlers -- TxDataSend, TxTalSend, TxPeerBacklog -- all
     * call VwifiTalTxPump, and the pump re-enters the component through
     * TxDequeueIndication, TxReleaseFrameIndication,
     * TxTransferCompleteIndication and TxSendCompleteIndication. The
     * component is entitled to call those handlers back from inside any
     * of them, and the trace shows it doing exactly that: a
     * TxPeerBacklog arrives between a pump's first log line and its
     * last, at the same timestamp, so the notification came from inside
     * the pump.
     *
     * Nothing bounded that. Each re-entry is another pump frame on the
     * kernel stack, and a kernel stack is 24 KB. A guest hang was
     * caught with gdb showing CR2 pointing into the stack region -- a
     * guard-page fault, which is stack overflow -- and both processors
     * inside ntoskrnl at IRQL 15, which is KeBugCheckEx running with
     * the other CPU halted by IPI. That is the hang: not a livelock, a
     * bugcheck that never reached the screen.
     *
     * Deferring rather than recursing loses nothing. The running pump
     * loops again and takes whatever the re-entrant call was going to
     * ask for, because the component is asked what it has rather than
     * told what to send. */
    volatile LONG       TxPumpActive;
    volatile LONG       TxPumpAgain;

    /* The periodic heartbeat: a timer this driver owns, and the count
     * of beats it has printed. See VwifiHeartbeatStart in driver.c for
     * why the beat is not driven by MiniportCheckForHangEx. */
    NDIS_HANDLE         HeartbeatTimer;
    ULONG               HangChecks;

    /* Ctrl-request payload scratch buffers (per-slot). */
    PVOID               CtrlReqPayloadVa;
    NDIS_PHYSICAL_ADDRESS CtrlReqPayloadPa;
    PVOID               CtrlRspPayloadVa;
    NDIS_PHYSICAL_ADDRESS CtrlRspPayloadPa;

    /* Pending requests awaiting responses. */
    LIST_ENTRY          PendingReqList;
    KSPIN_LOCK          PendingReqLock;
    volatile LONG       NextReqId;

    /* Discovered capabilities (from OP_GET_CAPS reply). */
    struct vwifi_caps   Caps;
    BOOLEAN             CapsValid;

    /* Adapter MAC. */
    UCHAR               PermanentMac[6];
    UCHAR               CurrentMac[6];

    /* State flags. */
    volatile LONG       DataPathRunning;
    BOOLEAN             MediumLinkUp;
} VWIFI_ADAPTER, *PVWIFI_ADAPTER;

/* ============================================================
 * MMIO register helpers (memory-mapped, little-endian)
 * ============================================================ */

FORCEINLINE ULONG
VwifiRead32(_In_ PVWIFI_ADAPTER Adapter, _In_ ULONG Offset)
{
    return READ_REGISTER_ULONG(
        (PULONG)((PUCHAR)Adapter->MmioVirtualAddress + Offset));
}

FORCEINLINE VOID
VwifiWrite32(_In_ PVWIFI_ADAPTER Adapter, _In_ ULONG Offset, _In_ ULONG Value)
{
    WRITE_REGISTER_ULONG(
        (PULONG)((PUCHAR)Adapter->MmioVirtualAddress + Offset), Value);
}

/* ============================================================
 * Function prototypes
 * ============================================================ */

/* driver.c
 *
 * MINIPORT_UNLOAD is the function type; MINIPORT_DRIVER_UNLOAD is the
 * *pointer* typedef that NDIS_MINIPORT_DRIVER_CHARACTERISTICS holds.
 * Using the pointer here declares a data variable, which then collides
 * with the definition in driver.c. Same trap for every NDIS role type. */
DRIVER_INITIALIZE DriverEntry;
MINIPORT_UNLOAD VwifiDriverUnload;
MINIPORT_OID_REQUEST VwifiOidRequest;

/* Adapter lifetime. In the WDI model these are driven by the WDI
 * control-path handlers below, not by MiniportInitializeEx: the WLAN
 * component owns MiniportInitializeEx and calls AllocateAdapter in its
 * place, handing us the registration attributes to fill in. */
NDIS_STATUS VwifiAdapterCreate(
    _In_ NDIS_HANDLE NdisMiniportHandle,
    _In_ PNDIS_MINIPORT_INIT_PARAMETERS InitParameters,
    _Inout_ PNDIS_MINIPORT_ADAPTER_REGISTRATION_ATTRIBUTES RegistrationAttributes);
VOID VwifiAdapterDestroy(_In_ NDIS_HANDLE MiniportAdapterContext);

/* wdi_ops.c — WDI handler table.
 *
 * Note the role-type names: the handler for StartOperationHandler is
 * MINIPORT_WDI_START_ADAPTER_OPERATION, not MINIPORT_WDI_START_OPERATION
 * (that name exists only with a _HANDLER suffix, as the pointer). Four
 * others follow the same "_ADAPTER_" pattern. */
MINIPORT_WDI_ALLOCATE_ADAPTER          VwifiWdiAllocateAdapter;
MINIPORT_WDI_FREE_ADAPTER              VwifiWdiFreeAdapter;
MINIPORT_WDI_OPEN_ADAPTER              VwifiWdiOpenAdapter;
MINIPORT_WDI_CLOSE_ADAPTER             VwifiWdiCloseAdapter;
MINIPORT_WDI_START_ADAPTER_OPERATION   VwifiWdiStartOperation;
MINIPORT_WDI_STOP_ADAPTER_OPERATION    VwifiWdiStopOperation;
MINIPORT_WDI_POST_ADAPTER_PAUSE        VwifiWdiPostPause;
MINIPORT_WDI_POST_ADAPTER_RESTART      VwifiWdiPostRestart;
MINIPORT_WDI_ADAPTER_HANG_DIAGNOSE     VwifiWdiHangDiagnose;
MINIPORT_WDI_TAL_TXRX_INITIALIZE       VwifiWdiTalTxRxInitialize;
MINIPORT_WDI_TAL_TXRX_DEINITIALIZE     VwifiWdiTalTxRxDeinitialize;
MINIPORT_WDI_IDLE_NOTIFICATION         VwifiWdiLeIdleNotification;
MINIPORT_WDI_CANCEL_IDLE_NOTIFICATION  VwifiWdiLeCancelIdleNotification;

/* Set from NDIS_WDI_INIT_PARAMETERS.WdiVersion in AllocateAdapter,
 * which runs before there is an adapter context to store it in. */
extern ULONG g_WdiPeerVersion;

/* hardware.c */
NDIS_STATUS VwifiHwInitialize(_Inout_ PVWIFI_ADAPTER Adapter,
                              _In_ PNDIS_MINIPORT_INIT_PARAMETERS InitParams);
/* Everything that needs a *registered* miniport — DMA rings, NBL pools,
 * interrupts — happens here, from OpenAdapter, not in Initialize. */
NDIS_STATUS VwifiHwStart(_Inout_ PVWIFI_ADAPTER Adapter);
VOID        VwifiHwStop(_Inout_ PVWIFI_ADAPTER Adapter);

/* driver.c — the periodic "alive" trace. */
NDIS_STATUS VwifiHeartbeatStart(_Inout_ PVWIFI_ADAPTER Adapter);
VOID        VwifiHeartbeatStop(_Inout_ PVWIFI_ADAPTER Adapter);
VOID        VwifiHwShutdown(_Inout_ PVWIFI_ADAPTER Adapter);
NDIS_STATUS VwifiHwReset(_Inout_ PVWIFI_ADAPTER Adapter);
BOOLEAN     VwifiHwInterruptDpc(_In_ PVWIFI_ADAPTER Adapter);

/* wdi_tal.c */
/* Populates the WDI data-path handler table. Every entry is filled: a
 * NULL in that table is a hole, not a default. */
VOID VwifiTalFillDataHandlers(_Inout_ PNDIS_MINIPORT_WDI_DATA_HANDLERS H);

/* rings.c */
/* Common-buffer helpers over the PDO's DMA adapter. Both are no-ops
 * when Adapter->DmaAdapter is NULL. */
NDIS_STATUS VwifiDmaAlloc(_Inout_ PVWIFI_ADAPTER Adapter,
                          _In_ ULONG Length,
                          _Outptr_result_maybenull_ PVOID *Va,
                          _Out_ PHYSICAL_ADDRESS *Pa);
VOID        VwifiDmaFree(_Inout_ PVWIFI_ADAPTER Adapter,
                         _In_ ULONG Length,
                         _In_opt_ PVOID Va,
                         _In_ PHYSICAL_ADDRESS Pa);

NDIS_STATUS VwifiRingsAllocate(_Inout_ PVWIFI_ADAPTER Adapter);
VOID        VwifiRingsFree(_Inout_ PVWIFI_ADAPTER Adapter);
VOID        VwifiRingsProgramMmio(_In_ PVWIFI_ADAPTER Adapter);
VOID        VwifiRingsArmCtrlRsp(_Inout_ PVWIFI_ADAPTER Adapter);
VOID        VwifiRingsPostRxBuffers(_Inout_ PVWIFI_ADAPTER Adapter);

/* Synchronous control command. Blocks until response arrives or
 * the adapter is halted. Returns NDIS_STATUS_* codes. */
NDIS_STATUS VwifiCtrlSendSync(_Inout_ PVWIFI_ADAPTER Adapter,
                              _In_ USHORT Opcode,
                              _In_reads_bytes_opt_(InLen) const VOID *InPayload,
                              _In_ ULONG InLen,
                              _Out_writes_bytes_to_opt_(*OutLen, *OutLen)
                                  VOID *OutPayload,
                              _Inout_ ULONG *OutLen);

/* DPC-context consumer: drain all response descriptors, wake
 * any pending requesters, dispatch events. */
VOID VwifiCtrlRspDrain(_Inout_ PVWIFI_ADAPTER Adapter);

/* DPC-context consumer for RX ring (Phase 1 stub — logs & re-arms). */
VOID VwifiRxDrain(_Inout_ PVWIFI_ADAPTER Adapter);

/* monitor.c — Phase 1.5 monitor-mode data path. */
NDIS_STATUS VwifiRxNblPoolCreate(_Inout_ PVWIFI_ADAPTER Adapter);
VOID        VwifiRxNblPoolDestroy(_Inout_ PVWIFI_ADAPTER Adapter);
VOID        VwifiRxDrainMonitor(_Inout_ PVWIFI_ADAPTER Adapter);
NDIS_STATUS VwifiInjectFrame(_Inout_ PVWIFI_ADAPTER Adapter,
                             _In_reads_bytes_(FrameLen) PUCHAR Frame,
                             _In_ ULONG FrameLen);

/* Control helpers to push mode/channel/filter to the device. */
NDIS_STATUS VwifiSetOpMode(_Inout_ PVWIFI_ADAPTER Adapter, ULONG Mode);
NDIS_STATUS VwifiSetChannel(_Inout_ PVWIFI_ADAPTER Adapter, USHORT FreqMhz);
NDIS_STATUS VwifiSetRawFilter(_Inout_ PVWIFI_ADAPTER Adapter, ULONG Mask);

/* wdi_common.c — shared WDI plumbing. */
NDIS_STATUS VwifiGetTlvPayload(_In_ PNDIS_OID_REQUEST Req,
                               _Outptr_ PVOID *TlvBuffer,
                               _Out_ PULONG TlvLength);
UINT32      VwifiGetWdiTransactionId(_In_ PNDIS_OID_REQUEST Req);
/* TransactionId must echo the originating OID request's transaction id
 * for a task-completion indication; WDI_TRANSACTION_ID_UNSOLICIT is the
 * right value for an unsolicited event. The OS matches completions by
 * this field, so a wrong one looks like a task that never finished.
 *
 * MessageStatus is the operation's result, carried in the message
 * header rather than in a TLV. For the empty messages — SCAN_COMPLETE,
 * CONNECT_COMPLETE — it is the only place the outcome appears at all;
 * everything else passes NDIS_STATUS_SUCCESS. */
WDI_PORT_ID VwifiGetWdiPortId(_In_ PNDIS_OID_REQUEST Req);
/* WdiPortId scopes the WDI message header; NdisPortNumber scopes the
 * NDIS status indication. Different namespaces -- see VwifiGetWdiPortId. */
VOID        VwifiSendWdiIndication(_Inout_ PVWIFI_ADAPTER Adapter,
                                   _In_ WDI_PORT_ID WdiPortId,
                                   _In_ ULONG NdisPortNumber,
                                   _In_ NDIS_STATUS StatusCode,
                                   _In_ NDIS_STATUS MessageStatus,
                                   _In_ UINT32 TransactionId,
                                   _In_reads_bytes_opt_(TlvLength) PVOID TlvBuffer,
                                   _In_ ULONG TlvLength);
/* oids.c — the NDIS statuses this driver actually sees, by name. For
 * logs: a bare 0xc0010015 costs a trip to the headers at exactly the
 * moment the log is meant to be answering a question. */
PCSTR       VwifiNdisStatusName(_In_ NDIS_STATUS Status);
ULONG       VwifiRssiToLinkQuality(_In_ CHAR Rssi);
ULONGLONG   VwifiGetTickCountMs(VOID);

/* oids.c */
NDIS_STATUS VwifiHandleTaskChangeOpMode(_Inout_ PVWIFI_ADAPTER Adapter,
                                        _In_ PNDIS_OID_REQUEST Req);

/* Accept a WDI task: write the M2 into the OID's output buffer with a
 * Status of SUCCESS, and return NDIS_STATUS_INDICATION_REQUIRED as the
 * OID's own status.
 *
 * Those are two different fields and both matter. Every task handler
 * must return through this rather than a status of its own. Completing
 * the OID with SUCCESS instead -- tried for one build -- makes wdiwifi
 * treat the scan as already finished, so it polls an empty BSS cache
 * ten times and stops merging scan tasks. See oids.c for the
 * measurement, and for why NDIS_STATUS_PENDING is the next thing to
 * try. */
NDIS_STATUS VwifiWdiTaskAccepted(_In_ PNDIS_OID_REQUEST Req);

/* Accept a task and leave its OID outstanding: write the M2, return
 * NDIS_STATUS_PENDING, and complete it later with VwifiWdiTaskComplete.
 *
 * Used only by OID_WDI_TASK_SCAN. A breakpoint trace over a whole
 * failed connect showed every scan job finishing with status
 * 0x40230001 -- the INDICATION_REQUIRED this driver returns -- and
 * never with 0, which is the one value that lets FinishJob record
 * WfcPortPropertyGoodScanStartTime. PENDING keeps the job open the way
 * INDICATION_REQUIRED does and still completes with zero. See oids.c. */
NDIS_STATUS VwifiWdiTaskPending(_In_ PNDIS_OID_REQUEST Req);

/* Answer a task whose work is already done: write the M2 and complete
 * the OID with SUCCESS. For the one path where that is true -- a scan
 * merged into a sweep that finished underneath it -- and not a general
 * task return; see oids.c. */
NDIS_STATUS VwifiWdiTaskAnsweredInline(_In_ PNDIS_OID_REQUEST Req);

/* Complete an OID left outstanding by VwifiWdiTaskPending. Exactly once
 * per request: NDIS frees it here, and never calling it leaves the WLAN
 * component waiting forever. */
VOID        VwifiWdiTaskComplete(_Inout_ PVWIFI_ADAPTER Adapter,
                                 _In_ PNDIS_OID_REQUEST Req,
                                 _In_ NDIS_STATUS Status);

/* wdi_scan.c — Phase 2 scan task. */
/* Re-indicate everything the scan cache holds, for
 * OID_WDI_GET_BSS_ENTRY_LIST. */
VOID        VwifiScanFlushCache(_Inout_ PVWIFI_ADAPTER Adapter);
VOID        VwifiScanIndicateCachedBss(_Inout_ PVWIFI_ADAPTER Adapter,
                                       _In_ WDI_PORT_ID WdiPortId,
                                       _In_ ULONG NdisPortNumber);
NDIS_STATUS VwifiScanTaskCreate(_Inout_ PVWIFI_ADAPTER Adapter);
VOID        VwifiScanTaskDestroy(_Inout_ PVWIFI_ADAPTER Adapter);
NDIS_STATUS VwifiHandleTaskScan(_Inout_ PVWIFI_ADAPTER Adapter,
                                _In_ PNDIS_OID_REQUEST Req);
VOID        VwifiIndicateLinkState(_Inout_ PVWIFI_ADAPTER Adapter,
                                   _In_ BOOLEAN Up);
NDIS_STATUS VwifiHandleTaskScanAbort(_Inout_ PVWIFI_ADAPTER Adapter);
/* 1 if a scan is awaiting its SCAN_COMPLETE. For the heartbeat. */
ULONG       VwifiScanTaskState(_In_ PVWIFI_ADAPTER Adapter);
/* Complete any scan held because the device would not sweep while it
 * was associating. Called from the connect task's completion; a no-op
 * when nothing is held. */
VOID        VwifiScanReleaseDeferred(_Inout_ PVWIFI_ADAPTER Adapter);
VOID        VwifiScanOnBssFound(_Inout_ PVWIFI_ADAPTER Adapter,
                                _In_reads_bytes_(PayloadLen) const VOID *Payload,
                                _In_ ULONG PayloadLen);
VOID        VwifiScanOnComplete(_Inout_ PVWIFI_ADAPTER Adapter,
                                _In_reads_bytes_(PayloadLen) const VOID *Payload,
                                _In_ ULONG PayloadLen);
/* Called once at the end of every ctrl-rsp drain, after all of that
 * drain's BSS_FOUND events have been staged. Applies the documented
 * batch throttle. */
VOID        VwifiScanFlushBatch(_Inout_ PVWIFI_ADAPTER Adapter);

/* wdi_connect.c — Phase 3 connect task. */
NDIS_STATUS VwifiConnectTaskCreate(_Inout_ PVWIFI_ADAPTER Adapter);
VOID        VwifiConnectTaskDestroy(_Inout_ PVWIFI_ADAPTER Adapter);
NDIS_STATUS VwifiHandleTaskConnect(_Inout_ PVWIFI_ADAPTER Adapter,
                                   _In_ PNDIS_OID_REQUEST Req);
NDIS_STATUS VwifiHandleTaskDisconnect(_Inout_ PVWIFI_ADAPTER Adapter,
                                      _In_ PNDIS_OID_REQUEST Req);
/* Bit 0: a connect awaits CONNECT_COMPLETE. Bit 1: a disconnect awaits
 * DISCONNECT_COMPLETE. For the heartbeat. */
#define VWIFI_TASK_CONNECT_PENDING     0x1
#define VWIFI_TASK_DISCONNECT_PENDING  0x2
ULONG       VwifiConnectTaskState(_In_ PVWIFI_ADAPTER Adapter);

/* Rate-limited tracing for the per-frame handlers.
 *
 * Shared between wdi_tal.c and wdi_data.c: both sit on paths that can
 * run at DISPATCH_LEVEL and at whatever rate the component likes, and
 * logging every call would drown the trace that made either file
 * necessary. ONCE announces a path exists; FIRST(n) shows the first n
 * calls in full, for the handlers whose SHAPE is the open question. */
#define VWIFI_TAL_ONCE(fmt, ...)                                    \
    do {                                                            \
        static LONG _vwifi_once = 0;                                \
        if (InterlockedCompareExchange(&_vwifi_once, 1, 0) == 0) {  \
            VWIFI_INFO(fmt, ##__VA_ARGS__);                         \
        }                                                           \
    } while (0)

#define VWIFI_TAL_FIRST(n, fmt, ...)                                \
    do {                                                            \
        static LONG _vwifi_n = 0;                                   \
        if (InterlockedIncrement(&_vwifi_n) <= (n)) {               \
            VWIFI_INFO(fmt, ##__VA_ARGS__);                         \
        }                                                           \
    } while (0)

/* How many transfers we tell the component it may have outstanding.
 *
 * ONE. Raising it to 64 broke peer creation, and the log isolates that
 * cleanly enough to be worth writing down.
 *
 * The build that raised it changed five things: this number, the TX
 * pump wired to the send notifications, the RX drain routed through the
 * TAL, the RxGetMpdus/RxReturnFrames handoff, and a log line. In the
 * failing trace the component sent no TX notification and the device
 * received no data frame, so the pump never ran, the RX routing never
 * ran and the RX queue was never non-empty. Four of the five changes
 * did not execute. This one did -- TalTxRxStart runs at adapter init,
 * long before a peer exists -- and TalTxRxPeerConfigHandler, which had
 * arrived within the same millisecond as the connect completion on the
 * previous build, stopped arriving at all.
 *
 * That is the whole argument for 1 over 64: not that 1 is right, but
 * that 64 is the only candidate the evidence leaves. Raise it again
 * only with a trace that shows TX actually starved.
 *
 * One is also not obviously a throttle any more. A transfer here is a
 * memcpy into a ring slot and is complete before the handler returns,
 * so the component can never find more than one outstanding -- and
 * TxPeerBacklog, which the previous build did receive, now drives the
 * pump directly. The depth that matters is the component's willingness
 * to hand frames over, not our ability to hold them. */
#define VWIFI_TAL_MAX_OUTSTANDING_TRANSFERS 1

/* monitor.c — NDIS's return path for RX NBLs, and the only code that
 * frees an RX NBL and re-arms the ring slot behind it. Declared here
 * because wdi_data.c needs it too: a frame handed back through the TAL
 * has to go through the same reclaim, and a second copy of it is how
 * the two drift apart. (Third time a helper has had to come out of the
 * file that happened to own it -- see VwifiWdiAckHeaderOnly and
 * VwifiNdisStatusName.) */
MINIPORT_RETURN_NET_BUFFER_LISTS VwifiMiniportReturnNetBufferLists;

/* wdi_data.c — the WDI per-frame data path.
 *
 * TX is a pull: the component queues frames and the miniport comes and
 * takes them. RX is a two-step push: indicate that frames exist, then
 * hand them over when the component asks. */
/* Every TID at once. The bitmask form the pause/restart and
 * release-frames indications take; this device does no QoS, so there is
 * never a reason to name a subset. */
#define VWIFI_TAL_ALL_TIDS 0xFFFFFFFFu

VOID        VwifiTalTxPump(_Inout_ PVWIFI_ADAPTER Adapter,
                           _In_ WDI_PORT_ID PortId,
                           _In_ WDI_PEER_ID PeerId,
                           _In_ UINT32 ExTidBitmask);
/* Clear WDI_TX_PAUSE_REASON_PEER_CREATE for a peer the component has
 * finished configuring, then drain whatever was waiting behind it. */
VOID        VwifiTalTxRestartPeer(_Inout_ PVWIFI_ADAPTER Adapter,
                                  _In_ WDI_PORT_ID PortId,
                                  _In_ WDI_PEER_ID PeerId);
/* Clear the RX pause latch and announce anything held behind it. */
VOID        VwifiTalRxOnResume(_Inout_ PVWIFI_ADAPTER Adapter);
VOID        VwifiTalRxIndicate(_Inout_ PVWIFI_ADAPTER Adapter,
                               _In_ PNET_BUFFER_LIST Nbl,
                               _In_ WDI_PEER_ID PeerId,
                               _In_ WDI_EXTENDED_TID ExTid);
VOID        VwifiTalRxTakeQueued(_Inout_ PVWIFI_ADAPTER Adapter,
                                 _Outptr_result_maybenull_ PNET_BUFFER_LIST *Out);
VOID        VwifiTalRxReturn(_Inout_ PVWIFI_ADAPTER Adapter,
                             _In_ PNET_BUFFER_LIST Nbl);
VOID        VwifiTalRxFlushQueued(_Inout_ PVWIFI_ADAPTER Adapter);

/* wdi_peer.c — the WDI peer table.
 *
 * VwifiPeerCreate assigns the id and tells the component. Delete is
 * two-step: VwifiPeerDelete indicates it, and the slot is only
 * released when the component calls back through
 * VwifiPeerOnDeleteConfirm. VwifiPeerForgetAll skips both, for
 * teardown once the data-path API is no longer callable. */
NDIS_STATUS VwifiPeerCreate(_Inout_ PVWIFI_ADAPTER Adapter,
                            _In_ WDI_PORT_ID WdiPortId,
                            _In_reads_bytes_(6) const UCHAR *Mac,
                            _Out_ WDI_PEER_ID *PeerIdOut);
VOID        VwifiPeerDelete(_Inout_ PVWIFI_ADAPTER Adapter,
                            _In_ WDI_PEER_ID PeerId);
VOID        VwifiPeerDeleteAll(_Inout_ PVWIFI_ADAPTER Adapter);
VOID        VwifiPeerForgetAll(_Inout_ PVWIFI_ADAPTER Adapter);
VOID        VwifiPeerOnConfig(_Inout_ PVWIFI_ADAPTER Adapter,
                              _In_ WDI_PORT_ID PortId,
                              _In_ WDI_PEER_ID PeerId,
                              _In_opt_ const WDI_TXRX_PEER_CFG *Cfg);
VOID        VwifiPeerOnDeleteConfirm(_Inout_ PVWIFI_ADAPTER Adapter,
                                     _In_ WDI_PORT_ID PortId,
                                     _In_ WDI_PEER_ID PeerId);
PVWIFI_PEER VwifiPeerFind(_In_ PVWIFI_ADAPTER Adapter,
                          _In_ WDI_PEER_ID PeerId);
PVWIFI_PEER VwifiPeerFirstActive(_In_ PVWIFI_ADAPTER Adapter);
VOID        VwifiConnectOnAssocResult(_Inout_ PVWIFI_ADAPTER Adapter,
                                      _In_reads_bytes_(PayloadLen) const VOID *Payload,
                                      _In_ ULONG PayloadLen);
VOID        VwifiConnectOnDisconnected(_Inout_ PVWIFI_ADAPTER Adapter,
                                       _In_reads_bytes_(PayloadLen) const VOID *Payload,
                                       _In_ ULONG PayloadLen);

/* wdi_keys.c — Phase 4 cipher keys. */
NDIS_STATUS VwifiHandleAddCipherKeys(_Inout_ PVWIFI_ADAPTER Adapter,
                                     _In_ PNDIS_OID_REQUEST Req);
NDIS_STATUS VwifiHandleDeleteCipherKeys(_Inout_ PVWIFI_ADAPTER Adapter,
                                        _In_ PNDIS_OID_REQUEST Req);
VOID        VwifiKeysOnInstalled(_Inout_ PVWIFI_ADAPTER Adapter,
                                 _In_reads_bytes_(PayloadLen) const VOID *Payload,
                                 _In_ ULONG PayloadLen);

/* data.c — Phase 3 STA data path. */
/* ExtraFlags is OR'd into the TX descriptor alongside VWIFI_DESC_F_OWN.
 * Pass 0 for an 802.3 frame the device should encapsulate, or
 * VWIFI_TX_F_80211 for a frame that already carries its own 802.11
 * header -- which is what the WDI data path hands down. */
NDIS_STATUS VwifiTxDataFrame(_Inout_ PVWIFI_ADAPTER Adapter,
                             _In_reads_bytes_(FrameLen) PUCHAR Frame,
                             _In_ ULONG FrameLen,
                             _In_ USHORT ExtraFlags);
VOID        VwifiRxDrainSta(_Inout_ PVWIFI_ADAPTER Adapter);
