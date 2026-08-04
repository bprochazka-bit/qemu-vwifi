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
ULONG       VwifiRssiToLinkQuality(_In_ CHAR Rssi);
ULONGLONG   VwifiGetTickCountMs(VOID);

/* oids.c */
NDIS_STATUS VwifiHandleTaskChangeOpMode(_Inout_ PVWIFI_ADAPTER Adapter,
                                        _In_ PNDIS_OID_REQUEST Req);

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
VOID        VwifiScanOnBssFound(_Inout_ PVWIFI_ADAPTER Adapter,
                                _In_reads_bytes_(PayloadLen) const VOID *Payload,
                                _In_ ULONG PayloadLen);
VOID        VwifiScanOnComplete(_Inout_ PVWIFI_ADAPTER Adapter,
                                _In_reads_bytes_(PayloadLen) const VOID *Payload,
                                _In_ ULONG PayloadLen);

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
NDIS_STATUS VwifiTxDataFrame(_Inout_ PVWIFI_ADAPTER Adapter,
                             _In_reads_bytes_(FrameLen) PUCHAR Frame,
                             _In_ ULONG FrameLen);
VOID        VwifiRxDrainSta(_Inout_ PVWIFI_ADAPTER Adapter);
