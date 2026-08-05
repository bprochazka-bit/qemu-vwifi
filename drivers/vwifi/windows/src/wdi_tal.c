/*
 * vwifi — wdi_tal.c
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * The WDI Target Abstraction Layer: the data-path handler table the
 * WLAN component gets from MiniportWdiTalTxRxInitialize.
 *
 * Why this file exists at all
 * ---------------------------
 * It used to not. TalTxRxInitialize left the whole
 * NDIS_MINIPORT_WDI_DATA_HANDLERS table zeroed, on the reasoning that
 * "Phase 1 doesn't route data through the TAL, so the component's
 * defaults are fine". There are no defaults. The table is twenty-five
 * function pointers and the component calls them; a NULL entry is not a
 * fallback, it is a hole.
 *
 * The trace showed what that cost:
 *
 *     WdiTalTxRxInitialize: handler table supplied (left empty)
 *     OID: WDI_GET_ADAPTER_CAPABILITIES -> 744 bytes
 *     OID: WDI_SET_ADAPTER_CONFIGURATION -> accepted
 *     WdiTalTxRxDeinitialize
 *     WdiCloseAdapter
 *
 * The component initialised the data path, read the capabilities,
 * applied the configuration, then unwound without asking for anything
 * else. Creating a port goes through TalTxRxAddPortHandler. That
 * pointer was NULL, so there was no way to create one, and an adapter
 * with no ports has nothing to do.
 *
 * What is implemented here
 * ------------------------
 * The lifecycle: start, stop, ports, peers. Those are what the
 * component needs in order to bring an adapter up, and they are cheap
 * for a device whose queueing lives on the other side of a ring.
 *
 * Peers are no longer a stub. The peer callbacks -- PeerConfig and
 * PeerDeleteConfirm -- are the component's half of a handshake whose
 * other half is NdisWdiPeerCreateIndication/PeerDeleteIndication, and
 * both halves now run. See wdi_peer.c for what a peer is and why
 * nothing can be sent without one.
 *
 * The per-frame TX and RX paths are still honest stubs. Every stub is
 * written so the component cannot be misled by it: no NBL is accepted
 * and silently dropped, no output parameter is left uninitialised, and
 * RxGetMpdus hands back NULL rather than whatever was on the stack.
 *
 * That means TX from the WDI data path does not work yet. It is a real
 * limitation, not a temporary shim that happens to work — anything sent
 * this way stays queued in the component. Scanning and connecting are
 * control-path operations and do not go through here.
 *
 * What is missing to finish it
 * ----------------------------
 * One contract, and it is not in this header's kit. A TX NBL arrives
 * from NdisWdiTxDequeueIndication carrying a WDI_FRAME_METADATA -- the
 * struct is defined in dot11wdi.h, has a pNBL back-pointer, and holds
 * the WDI_FRAME_ID that NdisWdiTxSendCompleteIndication is fed -- but
 * nothing in dot11wdi.h says WHERE on the NBL to find it. Guessing at a
 * kernel ABI is how this project has lost rounds before, so it is being
 * read out of wdiwifi.sys instead: see the TxMgr disassembly pass in
 * dump-wdi-state.ps1.
 *
 * The TxDataSend and RxGetMpdus stubs log their first calls in full
 * rather than announcing themselves once, because whether they fire at
 * all is the measurement that says the peer indication took.
 *
 * Every handler traces
 * --------------------
 * Including the ones that do nothing. Ten of them used to be silent,
 * and that silence cost a diagnosis: an ETW trace showed the WLAN
 * component take OID_DOT11_RESET_REQUEST, never turn it into
 * OID_WDI_TASK_DOT11_RESET for this driver, and never complete it. What
 * the component is documented to do first is "abort any task in
 * progress on the port. It also flushes its Rx and TX queues" -- which
 * runs through TxAbort, RxStop and RxFlush, all three of which said
 * nothing. So the driver's trace could not distinguish "the component
 * never called us" from "the component called us and we are where it
 * stopped".
 *
 * The per-frame handlers still announce themselves once rather than per
 * call, because at full rate they would bury everything else. The
 * control and flush handlers log every time: they are rare, and each
 * one is a step in a sequence worth seeing in order.
 *
 * That tracing paid for itself immediately: it showed TxAbort, RxStop
 * and RxFlush arriving together at connect time and nothing after, and
 * RxFlush turned out to be the one handler here that cannot finish by
 * returning. See VwifiTalRxFlush.
 *
 * Which handlers must call back
 * -----------------------------
 * A handler with an _Out_ NDIS_STATUS can finish synchronously by
 * reporting success -- TxAbort and RxStop both do. A handler returning
 * void cannot, and must confirm through NDIS_WDI_DATA_API. Read the
 * return type before assuming a stub is free.
 */

#include "vwifi_drv.h"

/* Per-frame handlers can be called at DISPATCH_LEVEL and at whatever
 * rate the component likes. Logging every call would drown the trace
 * that made this file necessary, so they announce themselves once. */
#define VWIFI_TAL_ONCE(fmt, ...)                                    \
    do {                                                            \
        static LONG _vwifi_once = 0;                                \
        if (InterlockedCompareExchange(&_vwifi_once, 1, 0) == 0) {  \
            VWIFI_INFO(fmt, ##__VA_ARGS__);                         \
        }                                                           \
    } while (0)

/* The first few calls in full, then silence.
 *
 * For the handlers whose SHAPE is the open question rather than their
 * existence: one line says they started firing, and a handful says what
 * the component is actually asking for -- which peer, which TID, how
 * many frames -- without turning the trace into a firehose at rate.
 * Eight is enough to see a pattern and few enough to read. */
#define VWIFI_TAL_FIRST(n, fmt, ...)                                \
    do {                                                            \
        static LONG _vwifi_n = 0;                                   \
        if (InterlockedIncrement(&_vwifi_n) <= (n)) {               \
            VWIFI_INFO(fmt, ##__VA_ARGS__);                         \
        }                                                           \
    } while (0)

/* ============================================================
 * TAL lifecycle
 * ============================================================ */

static NDIS_STATUS
VwifiTalStart(
    _In_ TAL_TXRX_HANDLE MiniportTalTxRxContext,
    _In_ PWDI_TXRX_TARGET_CONFIGURATION pWifiTxRxConfiguration,
    _Out_ PTAL_TXRX_PARAMETERS pTalTxRxParameters)
{
    PVWIFI_ADAPTER adapter = (PVWIFI_ADAPTER)MiniportTalTxRxContext;

    VWIFI_INFO("TAL start: max %u ports, %u peers, params %u bytes",
               pWifiTxRxConfiguration ? pWifiTxRxConfiguration->MaxNumPorts : 0,
               pWifiTxRxConfiguration ? pWifiTxRxConfiguration->MaxNumPeers : 0,
               (ULONG)sizeof(*pTalTxRxParameters));

    /* Kept, because peer ids are the miniport's to assign and the
     * component sizes its own per-peer state from this number. Handing
     * back an id at or past it is handing it an index into an array it
     * did not allocate. See VwifiPeerCreate. */
    if (adapter != NULL && pWifiTxRxConfiguration != NULL) {
        adapter->MaxPeers = pWifiTxRxConfiguration->MaxNumPeers;
        adapter->MaxPorts = pWifiTxRxConfiguration->MaxNumPorts;
    }

    /* Zero the whole block before writing the one field we know about.
     *
     * pTalTxRxParameters is _Out_, which makes initialising every byte
     * of it this function's job, and this function was setting exactly
     * one member. Whatever else TAL_TXRX_PARAMETERS holds was left as
     * whatever the component's caller had on its stack -- and the
     * component configures its data path from this block.
     *
     * Zero is not known to be a good value for the fields we are not
     * naming; it is only known to be a defined one. That is the whole
     * claim. Stack residue is worse in every case and unreproducible in
     * the bargain, which is exactly the shape of failure being chased
     * here. The size is logged so a trace says how much of this struct
     * the kit actually has. */
    RtlZeroMemory(pTalTxRxParameters, sizeof(*pTalTxRxParameters));

    /* One outstanding transfer: the TAL TX path is not implemented, so
     * there is no depth to offer and claiming otherwise would invite
     * the component to queue work that never completes. */
    pTalTxRxParameters->MaxOutstandingTransfers = 1;
    return NDIS_STATUS_SUCCESS;
}

static VOID
VwifiTalStop(_In_ TAL_TXRX_HANDLE MiniportTalTxRxContext)
{
    PVWIFI_ADAPTER adapter = (PVWIFI_ADAPTER)MiniportTalTxRxContext;

    VWIFI_INFO("TAL stop");

    /* Forget rather than delete. The data path is being torn down, so
     * indicating a delete into it is a call through an API table that
     * is on its way out -- and there is nothing left to tell anyway. */
    VwifiPeerForgetAll(adapter);
}

/* ============================================================
 * Put the device into the mode the host just gave the port.
 *
 * AddPort and SetPortOpMode are the only places the component states
 * what a port is actually FOR. Nothing else does: OID_WDI_TASK_CHANGE_
 * OPERATION_MODE is how a *running* adapter is switched, and it is
 * never sent during bring-up, so a driver that only listens there never
 * learns its own mode.
 *
 * Logging it and moving on -- which is what this used to do -- left
 * Adapter->OpMode at VWIFI_MODE_IDLE and the device likewise. In that
 * state VwifiMiniportSendNetBufferLists completes every frame with
 * NDIS_STATUS_PAUSED, VwifiRxDrain re-arms slots and drops what it
 * finds, and the device has never been told to behave as a station. An
 * adapter that scans but cannot associate is what that looks like from
 * the outside.
 * ============================================================ */
/* The passive-level half of VwifiTalApplyOpMode.
 *
 * NDIS runs this at PASSIVE_LEVEL, which is what VwifiSetOpMode needs
 * and what the TAL callback could not promise. */
static NDIS_IO_WORKITEM_FUNCTION VwifiTalOpModeWorker;

_Use_decl_annotations_
static VOID
VwifiTalOpModeWorker(PVOID Context, NDIS_HANDLE WorkItem)
{
    PVWIFI_ADAPTER adapter = (PVWIFI_ADAPTER)Context;
    LONG mode = InterlockedExchange(&adapter->PendingOpMode, -1);

    /* Negative means another worker already took it. Two TAL callbacks
     * in quick succession can queue two items; the first one to run
     * does the work and the second finds nothing to do. */
    if (mode >= 0) {
        VWIFI_INFO("TAL: applying deferred opmode %d at IRQL %u",
                   mode, KeGetCurrentIrql());
        (VOID)VwifiSetOpMode(adapter, (ULONG)mode);
    }

    NdisFreeIoWorkItem(WorkItem);
}

static VOID
VwifiTalApplyOpMode(_Inout_ PVWIFI_ADAPTER Adapter,
                    _In_ WDI_OPERATION_MODE OpMode)
{
    ULONG mode;

    if (OpMode & WDI_OPERATION_MODE_STA) {
        mode = VWIFI_MODE_STA;
    } else {
        /* The P2P roles are the rest of the enum and this device
         * implements none of them. Saying so beats silently sitting in
         * idle and looking like a hang. */
        VWIFI_WARN("TAL: opmode 0x%x is not a mode this device has; "
                   "staying idle", OpMode);
        return;
    }

    if (Adapter->OpMode == mode) {
        return;
    }

    /* VwifiSetOpMode goes to the device over the control ring and waits
     * for the reply, so it needs PASSIVE_LEVEL. The TAL callbacks can
     * arrive above it, and blocking there is a bugcheck.
     *
     * This used to log an error and return, which meant the mode change
     * was silently dropped whenever the component happened to call at
     * DISPATCH_LEVEL. The consequence is the one described at the top of
     * this section: the adapter stays in VWIFI_MODE_IDLE, every send is
     * completed with NDIS_STATUS_PAUSED, every receive is dropped, and
     * the device is never told to behave as a station -- an adapter
     * that scans but cannot associate. Worse, it depends on the IRQL
     * the component happens to pick, so it works until it doesn't.
     *
     * Deferred to a work item instead. NDIS runs those at PASSIVE. */
    if (KeGetCurrentIrql() == PASSIVE_LEVEL) {
        (VOID)VwifiSetOpMode(Adapter, mode);
        return;
    }

    {
        NDIS_HANDLE wi;

        /* Published before the item is queued: the worker reads it and
         * the worker cannot run until NdisQueueIoWorkItem returns. */
        InterlockedExchange(&Adapter->PendingOpMode, (LONG)mode);

        wi = NdisAllocateIoWorkItem(Adapter->MiniportAdapterHandle);
        if (wi == NULL) {
            VWIFI_ERR("TAL: opmode change to %u wanted at IRQL %u and no "
                      "work item to defer it to", mode, KeGetCurrentIrql());
            InterlockedExchange(&Adapter->PendingOpMode, -1);
            return;
        }

        VWIFI_INFO("TAL: opmode change to %u at IRQL %u -- deferring to a "
                   "work item", mode, KeGetCurrentIrql());
        NdisQueueIoWorkItem(wi, VwifiTalOpModeWorker, Adapter);
    }
}

static VOID
VwifiTalAddPort(
    _In_ TAL_TXRX_HANDLE MiniportTalTxRxContext,
    _In_ WDI_PORT_ID PortId,
    _In_ WDI_OPERATION_MODE OpMode)
{
    PVWIFI_ADAPTER adapter = (PVWIFI_ADAPTER)MiniportTalTxRxContext;

    /* The call that could not happen before this file existed. */
    VWIFI_INFO("TAL add port %u, opmode 0x%x", PortId, OpMode);

    /* The authoritative WDI port id, and the first place it is stated.
     *
     * Adapter->WdiPortId has existed since the adapter struct did and
     * has never been assigned -- it worked only because this device has
     * one port and its id happens to be 0, so the zeroed field read
     * correctly by accident. Peers are addressed by port, which makes
     * that accident load-bearing, so it is written down here where the
     * component actually says what the id is. */
    adapter->WdiPortId = PortId;

    VwifiTalApplyOpMode(adapter, OpMode);
}

static VOID
VwifiTalDeletePort(
    _In_ TAL_TXRX_HANDLE MiniportTalTxRxContext,
    _In_ WDI_PORT_ID PortId)
{
    PVWIFI_ADAPTER adapter = (PVWIFI_ADAPTER)MiniportTalTxRxContext;

    VWIFI_INFO("TAL delete port %u", PortId);

    /* A port's peers cannot outlive it. In practice the disconnect that
     * precedes a port delete has already taken them, and this is the
     * backstop for the paths that do not go through one -- an adapter
     * disabled or removed while associated. */
    VwifiPeerDeleteAll(adapter);
}

static VOID
VwifiTalSetPortOpMode(
    _In_ TAL_TXRX_HANDLE MiniportTalTxRxContext,
    _In_ WDI_PORT_ID PortId,
    _In_ WDI_OPERATION_MODE Opmode)
{
    PVWIFI_ADAPTER adapter = (PVWIFI_ADAPTER)MiniportTalTxRxContext;

    VWIFI_INFO("TAL port %u opmode -> 0x%x", PortId, Opmode);
    VwifiTalApplyOpMode(adapter, Opmode);
}

static VOID
VwifiTalResetPort(
    _In_ TAL_TXRX_HANDLE MiniportTalTxRxContext,
    _In_ WDI_PORT_ID PortId)
{
    PVWIFI_ADAPTER adapter = (PVWIFI_ADAPTER)MiniportTalTxRxContext;

    VWIFI_INFO("TAL reset port %u", PortId);

    /* A dot11 reset puts the port back to disconnected, so any peer it
     * had is gone. Usually a no-op: this fires at the START of a connect
     * too, when there is nothing to delete. */
    VwifiPeerDeleteAll(adapter);
}

/* The component's half of peer creation: it has finished setting up
 * whatever per-peer state it keeps, and data may now flow to this peer.
 * Both of these used to log and drop the fact on the floor, which was
 * harmless only because no peer was ever created. */
static VOID
VwifiTalPeerConfig(
    _In_ TAL_TXRX_HANDLE MiniportTalTxRxContext,
    _In_ WDI_PORT_ID PortId,
    _In_ WDI_PEER_ID PeerId,
    _In_ PWDI_TXRX_PEER_CFG pPeerCfg)
{
    PVWIFI_ADAPTER adapter = (PVWIFI_ADAPTER)MiniportTalTxRxContext;

    VWIFI_INFO("TAL peer config: port %u peer %u", PortId, PeerId);
    VwifiPeerOnConfig(adapter, PortId, PeerId, pPeerCfg);
}

/* The other half of a delete. VwifiPeerDelete only indicates it; the
 * slot is not reusable until this arrives. */
static VOID
VwifiTalPeerDeleteConfirm(
    _In_ TAL_TXRX_HANDLE MiniportTalTxRxContext,
    _In_ WDI_PORT_ID PortId,
    _In_ WDI_PEER_ID PeerId)
{
    PVWIFI_ADAPTER adapter = (PVWIFI_ADAPTER)MiniportTalTxRxContext;

    VWIFI_INFO("TAL peer delete confirm: port %u peer %u", PortId, PeerId);
    VwifiPeerOnDeleteConfirm(adapter, PortId, PeerId);
}

/* ============================================================
 * TX — not implemented, but never silently so
 * ============================================================ */

static VOID
VwifiTalTxAbort(
    _In_ TAL_TXRX_HANDLE MiniportTalTxRxContext,
    _In_ WDI_PORT_ID PortId,
    _In_ WDI_PEER_ID PeerId,
    _Out_ NDIS_STATUS *pWifiStatus)
{
    UNREFERENCED_PARAMETER(MiniportTalTxRxContext);

    /* Nothing was ever accepted, so there is nothing outstanding to
     * abort and success is the truthful answer. */
    VWIFI_INFO("TAL TxAbort: port %u peer %u", PortId, PeerId);
    *pWifiStatus = NDIS_STATUS_SUCCESS;
}

static VOID
VwifiTalTxTargetDescInit(
    _In_ TAL_TXRX_HANDLE MiniportTalTxRxContext,
    _In_ PNET_BUFFER_LIST pNBL,
    _Out_ NDIS_STATUS *pWifiStatus)
{
    UNREFERENCED_PARAMETER(MiniportTalTxRxContext);
    UNREFERENCED_PARAMETER(pNBL);

    /* No per-frame target descriptor to build: this device takes whole
     * frames through the TX ring, not a descriptor the TAL prepares. */
    VWIFI_TAL_ONCE("TAL TxTargetDescInit (first call)");
    *pWifiStatus = NDIS_STATUS_SUCCESS;
}

static VOID
VwifiTalTxTargetDescDeInit(
    _In_ TAL_TXRX_HANDLE MiniportTalTxRxContext,
    _In_ PNET_BUFFER_LIST pNBL)
{
    UNREFERENCED_PARAMETER(MiniportTalTxRxContext);
    UNREFERENCED_PARAMETER(pNBL);

    VWIFI_TAL_ONCE("TAL TxTargetDescDeInit (first call)");
}

static VOID
VwifiTalTxDataSend(
    _In_ TAL_TXRX_HANDLE MiniportTalTxRxContext,
    _In_ WDI_PORT_ID PortId,
    _In_ WDI_PEER_ID PeerId,
    _In_ WDI_EXTENDED_TID ExTid,
    _In_ UINT16 NumQueueFrames,
    _In_ UINT32 NumActiveFrames,
    _In_ BOOLEAN bRobustnessFlag)
{
    UNREFERENCED_PARAMETER(MiniportTalTxRxContext);
    UNREFERENCED_PARAMETER(bRobustnessFlag);

    /* The component is telling us frames are queued and we should come
     * and get them. We do not, so they stay queued. Said out loud,
     * because a silently stalled TX path looks exactly like a working
     * one until something tries to use it.
     *
     * This handler had never fired in any trace before peers existed,
     * and that was not a coincidence: WDI addresses data by (port,
     * peer, TID) and holds TX paused on WDI_TX_PAUSE_REASON_PEER_CREATE
     * until a peer exists. Whether it fires NOW is the measurement that
     * says the peer indication took, so the first calls are logged in
     * full rather than announced once. */
    VWIFI_TAL_FIRST(8, "TAL TxDataSend: port %u peer %u tid %u -- %u queued, "
                       "%u active. The TAL TX path is not implemented yet, "
                       "so these stay queued.",
                    PortId, PeerId, ExTid, NumQueueFrames, NumActiveFrames);
}

static VOID
VwifiTalTxTalSend(
    _In_ TAL_TXRX_HANDLE MiniportTalTxRxContext,
    _In_ WDI_PORT_ID PortId,
    _In_ WDI_PEER_ID PeerId,
    _In_ WDI_EXTENDED_TID ExTid,
    _In_ UINT16 NumQueueFrames,
    _In_ UINT32 NumActiveFrames,
    _In_ BOOLEAN bRobustnessFlag)
{
    UNREFERENCED_PARAMETER(MiniportTalTxRxContext);
    UNREFERENCED_PARAMETER(bRobustnessFlag);

    VWIFI_TAL_FIRST(8, "TAL TxTalSend: port %u peer %u tid %u -- %u queued, "
                       "%u active. Not implemented.",
                    PortId, PeerId, ExTid, NumQueueFrames, NumActiveFrames);
}

static VOID
VwifiTalTxTalSendComplete(
    _In_ TAL_TXRX_HANDLE MiniportTalTxRxContext,
    _In_ PNET_BUFFER_LIST pNBL,
    _In_ WDI_TX_FRAME_STATUS TxFrameStatus)
{
    UNREFERENCED_PARAMETER(MiniportTalTxRxContext);
    UNREFERENCED_PARAMETER(pNBL);
    UNREFERENCED_PARAMETER(TxFrameStatus);

    VWIFI_TAL_ONCE("TAL TxTalSendComplete (first call)");
}

static VOID
VwifiTalTxQueueInOrder(
    _In_ TAL_TXRX_HANDLE MiniportTalTxRxContext,
    _In_ WDI_PEER_ID PeerId,
    _In_ UINT32 ExTidBitmask)
{
    UNREFERENCED_PARAMETER(MiniportTalTxRxContext);

    VWIFI_INFO("TAL TxQueueInOrder: peer %u tids 0x%x", PeerId, ExTidBitmask);
}

static VOID
VwifiTalTxPeerBacklog(
    _In_ TAL_TXRX_HANDLE MiniportTalTxRxContext,
    _In_ WDI_PORT_ID PortId,
    _In_ WDI_PEER_ID PeerId,
    _In_ BOOLEAN bBacklogged)
{
    UNREFERENCED_PARAMETER(MiniportTalTxRxContext);

    VWIFI_INFO("TAL TxPeerBacklog: port %u peer %u backlogged %u",
               PortId, PeerId, bBacklogged ? 1u : 0u);
}

static VOID
VwifiTalTxSuspectFrameAbort(
    _In_ TAL_TXRX_HANDLE MiniportTalTxRxContext,
    _In_ UINT64 SuspectFrameContext,
    _In_ UINT16 NumSuspectFrames,
    _In_reads_(NumSuspectFrames) PNET_BUFFER_LIST *SuspectFrameList)
{
    UNREFERENCED_PARAMETER(MiniportTalTxRxContext);
    UNREFERENCED_PARAMETER(SuspectFrameContext);
    UNREFERENCED_PARAMETER(SuspectFrameList);

    VWIFI_INFO("TAL TxSuspectFrameAbort: %u frames", NumSuspectFrames);
}

/* ============================================================
 * RX — likewise
 * ============================================================ */

static VOID
VwifiTalRxStop(
    _In_ TAL_TXRX_HANDLE MiniportTalTxRxContext,
    _In_ WDI_PORT_ID PortId,
    _Out_ NDIS_STATUS *pWifiStatus)
{
    UNREFERENCED_PARAMETER(MiniportTalTxRxContext);

    /* Nothing is running, so it is already stopped. */
    VWIFI_INFO("TAL RxStop: port %u", PortId);
    *pWifiStatus = NDIS_STATUS_SUCCESS;
}

static VOID
VwifiTalRxFlush(
    _In_ TAL_TXRX_HANDLE MiniportTalTxRxContext,
    _In_ WDI_PORT_ID PortId)
{
    PVWIFI_ADAPTER adapter = (PVWIFI_ADAPTER)MiniportTalTxRxContext;

    VWIFI_INFO("TAL RxFlush: port %u", PortId);

    /* RxFlush is the one handler in this file that CANNOT finish by
     * returning.
     *
     * TxAbort and RxStop each take an _Out_ NDIS_STATUS: a success
     * status there means "done, synchronously", and that is what they
     * report. MiniportWdiRxFlush returns void. It has no way to say
     * "done", so it is asynchronous by construction, and the TAL
     * finishes it by calling NdisWdiRxFlushConfirm -- "the RxEngine
     * must have finished discarding all matching RX data frames before
     * invoking this function".
     *
     * This handler discarded nothing and confirmed nothing, and that
     * one missing call is the whole of the connect hang.
     *
     * The sequence, from two logs that interlock to the millisecond:
     * the MSM takes Cmd_Connect, calls MSMSecStopSecurity, and NWiFi
     * issues OID_DOT11_RESET_REQUEST. The WLAN component then does what
     * it is documented to do before a dot11 reset -- "aborts any task
     * in progress on the port. It also flushes its Rx and TX queues" --
     * which arrives here as TxAbort, RxStop, RxFlush, in that order, in
     * the same millisecond. The first two completed. The third did not,
     * so the flush never finished, so the component never issued
     * OID_WDI_TASK_DOT11_RESET, so the reset never completed, so
     * StopSecurity never returned. Everything downstream follows from
     * there: no connect task, no more scans, wlansvc holding its
     * interface lock, and netsh and Device Manager blocking behind it.
     *
     * Nothing to discard before confirming: no frame has ever been
     * indicated to the component through the TAL. The RX path runs over
     * the vwifi rings, and there is no RxEngine here holding anything.
     * So the confirmation is immediate and it is honest.
     *
     * The API table comes from MiniportWdiTalTxRxInitialize and has been
     * captured and unused since it was first stored. This is the first
     * call this driver makes into it. */
    if (adapter == NULL || adapter->DataPathApi == NULL ||
        adapter->DataPathApi->RxFlushConfirm == NULL) {
        VWIFI_ERR("TAL RxFlush: no RxFlushConfirm available -- the "
                  "component will wait for this flush forever");
        return;
    }

    adapter->DataPathApi->RxFlushConfirm(adapter->DataPathHandle);
    VWIFI_INFO("TAL RxFlush: confirmed");
}

static VOID
VwifiTalRxRestart(
    _In_ TAL_TXRX_HANDLE MiniportTalTxRxContext,
    _In_ WDI_PORT_ID PortId)
{
    UNREFERENCED_PARAMETER(MiniportTalTxRxContext);

    VWIFI_INFO("TAL RxRestart: port %u", PortId);
}

static VOID
VwifiTalRxGetMpdus(
    _In_ TAL_TXRX_HANDLE MiniportTalTxRxContext,
    _In_ WDI_PEER_ID PeerId,
    _In_ WDI_EXTENDED_TID ExTid,
    _Out_ PNET_BUFFER_LIST *ppNBL)
{
    UNREFERENCED_PARAMETER(MiniportTalTxRxContext);

    /* NULL, explicitly. This is an _Out_ pointer the component will
     * walk; leaving it as whatever was on the stack is the difference
     * between "no frames" and a bugcheck. */
    VWIFI_TAL_FIRST(8, "TAL RxGetMpdus: peer %u tid %u -- no frames "
                       "(the TAL RX path is not implemented yet)",
                    PeerId, ExTid);
    *ppNBL = NULL;
}

static VOID
VwifiTalRxReturnFrames(
    _In_ TAL_TXRX_HANDLE MiniportTalTxRxContext,
    _In_ PNET_BUFFER_LIST pNBL)
{
    UNREFERENCED_PARAMETER(MiniportTalTxRxContext);
    UNREFERENCED_PARAMETER(pNBL);

    /* Nothing was ever indicated through the TAL, so nothing can come
     * back through here. */
    VWIFI_TAL_ONCE("TAL RxReturnFrames (first call) -- unexpected, nothing "
                   "was indicated through the TAL");
}

static VOID
VwifiTalRxResume(_In_ TAL_TXRX_HANDLE MiniportTalTxRxContext)
{
    UNREFERENCED_PARAMETER(MiniportTalTxRxContext);

    VWIFI_INFO("TAL RxResume");
}

static VOID
VwifiTalRxThrottle(
    _In_ TAL_TXRX_HANDLE MiniportTalTxRxContext,
    _In_ WDI_RX_THROTTLE_LEVEL RxThrottleLevel)
{
    UNREFERENCED_PARAMETER(MiniportTalTxRxContext);

    VWIFI_INFO("TAL RxThrottle: level %u", RxThrottleLevel);
}

static VOID
VwifiTalRxPpduRssi(
    _In_ TAL_TXRX_HANDLE MiniportTalTxRxContext,
    _In_ PNET_BUFFER_LIST pNBL,
    _Out_ UINT8 *pRssi)
{
    UNREFERENCED_PARAMETER(MiniportTalTxRxContext);
    UNREFERENCED_PARAMETER(pNBL);

    /* Another _Out_ that must be written, whatever it means. dot11wdi.h
     * gives the scale as a bare UINT8 with no units, and nothing asks
     * for it in Phase 1 because no frame reaches the component through
     * the TAL -- the real per-frame RSSI travels with the RX descriptor
     * and monitor.c puts it straight into the radiotap header. A fixed
     * mid-scale value is honest here in a way an invented dBm reading
     * would not be. */
    VWIFI_TAL_ONCE("TAL RxPpduRssi (first call)");
    *pRssi = 60;
}

/* ============================================================
 * Table fill
 *
 * pMiniportDataHandlers is _Inout_, not _Out_: the component allocates
 * it and may have declared, in the header, which revision it expects.
 * Revision 1 stops at TalTxRxPeerDeleteConfirmHandler; revision 2 adds
 * TxSuspectFrameAbortHandler. Writing the revision-2 field into a
 * revision-1 buffer would be an overrun, so the incoming header decides
 * how far we write.
 * ============================================================ */
VOID
VwifiTalFillDataHandlers(_Inout_ PNDIS_MINIPORT_WDI_DATA_HANDLERS H)
{
    BOOLEAN rev2;

    /* Trust the caller's revision only if it looks like it really set
     * one; a zeroed header means it left the choice to us, and
     * revision 1 is the choice that cannot overrun. */
    rev2 = (H->Header.Type == NDIS_OBJECT_TYPE_MINIPORT_WDI_DATA_HANDLERS &&
            H->Header.Revision >= NDIS_OBJECT_TYPE_MINIPORT_WDI_DATA_HANDLERS_REVISION_2 &&
            H->Header.Size >= NDIS_SIZEOF_MINIPORT_WDI_DATA_HANDLERS_REVISION_2);

    VWIFI_INFO("TAL: incoming handler header type 0x%x rev %u size %u -> "
               "filling revision %u",
               H->Header.Type, H->Header.Revision, H->Header.Size,
               rev2 ? 2 : 1);

    H->Header.Type = NDIS_OBJECT_TYPE_MINIPORT_WDI_DATA_HANDLERS;
    if (rev2) {
        H->Header.Revision = NDIS_OBJECT_TYPE_MINIPORT_WDI_DATA_HANDLERS_REVISION_2;
        H->Header.Size     = NDIS_SIZEOF_MINIPORT_WDI_DATA_HANDLERS_REVISION_2;
    } else {
        H->Header.Revision = NDIS_OBJECT_TYPE_MINIPORT_WDI_DATA_HANDLERS_REVISION_1;
        H->Header.Size     = NDIS_SIZEOF_MINIPORT_WDI_DATA_HANDLERS_REVISION_1;
    }

    H->TxAbortHandler              = VwifiTalTxAbort;
    H->TxTargetDescInitHandler     = VwifiTalTxTargetDescInit;
    H->TxTargetDescDeInitHandler   = VwifiTalTxTargetDescDeInit;
    H->TxDataSendHandler           = VwifiTalTxDataSend;
    H->TxTalSendHandler            = VwifiTalTxTalSend;
    H->TxTalSendCompleteHandler    = VwifiTalTxTalSendComplete;
    H->TxTalQueueInOrderHandler    = VwifiTalTxQueueInOrder;
    H->TxPeerBacklogHandler        = VwifiTalTxPeerBacklog;

    H->RxStopHandler               = VwifiTalRxStop;
    H->RxFlushHandler              = VwifiTalRxFlush;
    H->RxRestartHandler            = VwifiTalRxRestart;
    H->RxGetMpdusHandler           = VwifiTalRxGetMpdus;
    H->RxReturnFramesHandler       = VwifiTalRxReturnFrames;
    H->RxResumeHandler             = VwifiTalRxResume;
    H->RxThrottleHandler           = VwifiTalRxThrottle;
    H->RxPpduRssiHandler           = VwifiTalRxPpduRssi;

    H->TalTxRxStartHandler             = VwifiTalStart;
    H->TalTxRxStopHandler              = VwifiTalStop;
    H->TalTxRxAddPortHandler           = VwifiTalAddPort;
    H->TalTxRxDeletePortHandler        = VwifiTalDeletePort;
    H->TalTxRxSetPortOpModeHandler     = VwifiTalSetPortOpMode;
    H->TalTxRxResetPortHandler         = VwifiTalResetPort;
    H->TalTxRxPeerConfigHandler        = VwifiTalPeerConfig;
    H->TalTxRxPeerDeleteConfirmHandler = VwifiTalPeerDeleteConfirm;

    if (rev2) {
        H->TxSuspectFrameAbortHandler = VwifiTalTxSuspectFrameAbort;
    }
}
