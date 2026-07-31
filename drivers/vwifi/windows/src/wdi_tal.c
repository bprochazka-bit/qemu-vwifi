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
 * The per-frame TX and RX paths are honest stubs. This driver's data
 * path runs over the vwifi rings (see rings.c and data.c), not the TAL,
 * so nothing here moves a frame. Every stub is written so the component
 * cannot be misled by it: no NBL is accepted and silently dropped, no
 * output parameter is left uninitialised, and RxGetMpdus hands back
 * NULL rather than whatever was on the stack.
 *
 * That means TX from the WDI data path does not work yet. It is a real
 * limitation, not a temporary shim that happens to work — anything sent
 * this way stays queued in the component. Scanning and connecting are
 * control-path operations and do not go through here.
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

/* ============================================================
 * TAL lifecycle
 * ============================================================ */

static NDIS_STATUS
VwifiTalStart(
    _In_ TAL_TXRX_HANDLE MiniportTalTxRxContext,
    _In_ PWDI_TXRX_TARGET_CONFIGURATION pWifiTxRxConfiguration,
    _Out_ PTAL_TXRX_PARAMETERS pTalTxRxParameters)
{
    UNREFERENCED_PARAMETER(MiniportTalTxRxContext);

    VWIFI_INFO("TAL start: max %u ports, %u peers",
               pWifiTxRxConfiguration ? pWifiTxRxConfiguration->MaxNumPorts : 0,
               pWifiTxRxConfiguration ? pWifiTxRxConfiguration->MaxNumPeers : 0);

    /* The only thing we get to say here. One outstanding transfer: the
     * TAL TX path is not implemented, so there is no depth to offer and
     * claiming otherwise would invite the component to queue work that
     * never completes. */
    pTalTxRxParameters->MaxOutstandingTransfers = 1;
    return NDIS_STATUS_SUCCESS;
}

static VOID
VwifiTalStop(_In_ TAL_TXRX_HANDLE MiniportTalTxRxContext)
{
    UNREFERENCED_PARAMETER(MiniportTalTxRxContext);
    VWIFI_INFO("TAL stop");
}

static VOID
VwifiTalAddPort(
    _In_ TAL_TXRX_HANDLE MiniportTalTxRxContext,
    _In_ WDI_PORT_ID PortId,
    _In_ WDI_OPERATION_MODE OpMode)
{
    UNREFERENCED_PARAMETER(MiniportTalTxRxContext);

    /* The call that could not happen before this file existed. */
    VWIFI_INFO("TAL add port %u, opmode 0x%x", PortId, OpMode);
}

static VOID
VwifiTalDeletePort(
    _In_ TAL_TXRX_HANDLE MiniportTalTxRxContext,
    _In_ WDI_PORT_ID PortId)
{
    UNREFERENCED_PARAMETER(MiniportTalTxRxContext);
    VWIFI_INFO("TAL delete port %u", PortId);
}

static VOID
VwifiTalSetPortOpMode(
    _In_ TAL_TXRX_HANDLE MiniportTalTxRxContext,
    _In_ WDI_PORT_ID PortId,
    _In_ WDI_OPERATION_MODE Opmode)
{
    UNREFERENCED_PARAMETER(MiniportTalTxRxContext);
    VWIFI_INFO("TAL port %u opmode -> 0x%x", PortId, Opmode);
}

static VOID
VwifiTalResetPort(
    _In_ TAL_TXRX_HANDLE MiniportTalTxRxContext,
    _In_ WDI_PORT_ID PortId)
{
    UNREFERENCED_PARAMETER(MiniportTalTxRxContext);
    VWIFI_INFO("TAL reset port %u", PortId);
}

static VOID
VwifiTalPeerConfig(
    _In_ TAL_TXRX_HANDLE MiniportTalTxRxContext,
    _In_ WDI_PORT_ID PortId,
    _In_ WDI_PEER_ID PeerId,
    _In_ PWDI_TXRX_PEER_CFG pPeerCfg)
{
    UNREFERENCED_PARAMETER(MiniportTalTxRxContext);
    UNREFERENCED_PARAMETER(pPeerCfg);
    VWIFI_INFO("TAL peer config: port %u peer %u", PortId, PeerId);
}

static VOID
VwifiTalPeerDeleteConfirm(
    _In_ TAL_TXRX_HANDLE MiniportTalTxRxContext,
    _In_ WDI_PORT_ID PortId,
    _In_ WDI_PEER_ID PeerId)
{
    UNREFERENCED_PARAMETER(MiniportTalTxRxContext);
    VWIFI_INFO("TAL peer delete confirm: port %u peer %u", PortId, PeerId);
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
    UNREFERENCED_PARAMETER(PortId);
    UNREFERENCED_PARAMETER(PeerId);

    /* Nothing was ever accepted, so there is nothing outstanding to
     * abort and success is the truthful answer. */
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
    *pWifiStatus = NDIS_STATUS_SUCCESS;
}

static VOID
VwifiTalTxTargetDescDeInit(
    _In_ TAL_TXRX_HANDLE MiniportTalTxRxContext,
    _In_ PNET_BUFFER_LIST pNBL)
{
    UNREFERENCED_PARAMETER(MiniportTalTxRxContext);
    UNREFERENCED_PARAMETER(pNBL);
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
    UNREFERENCED_PARAMETER(PortId);
    UNREFERENCED_PARAMETER(PeerId);
    UNREFERENCED_PARAMETER(ExTid);
    UNREFERENCED_PARAMETER(NumActiveFrames);
    UNREFERENCED_PARAMETER(bRobustnessFlag);

    /* The component is telling us frames are queued and we should come
     * and get them. We do not, so they stay queued. Said out loud once,
     * because a silently stalled TX path looks exactly like a working
     * one until something tries to use it. */
    VWIFI_TAL_ONCE("TAL TxDataSend: %u frames queued -- the TAL TX path "
                   "is not implemented, frames will not be sent",
                   NumQueueFrames);
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
    UNREFERENCED_PARAMETER(PortId);
    UNREFERENCED_PARAMETER(PeerId);
    UNREFERENCED_PARAMETER(ExTid);
    UNREFERENCED_PARAMETER(NumActiveFrames);
    UNREFERENCED_PARAMETER(bRobustnessFlag);

    VWIFI_TAL_ONCE("TAL TxTalSend: %u frames queued -- not implemented",
                   NumQueueFrames);
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
}

static VOID
VwifiTalTxQueueInOrder(
    _In_ TAL_TXRX_HANDLE MiniportTalTxRxContext,
    _In_ WDI_PEER_ID PeerId,
    _In_ UINT32 ExTidBitmask)
{
    UNREFERENCED_PARAMETER(MiniportTalTxRxContext);
    UNREFERENCED_PARAMETER(PeerId);
    UNREFERENCED_PARAMETER(ExTidBitmask);
}

static VOID
VwifiTalTxPeerBacklog(
    _In_ TAL_TXRX_HANDLE MiniportTalTxRxContext,
    _In_ WDI_PORT_ID PortId,
    _In_ WDI_PEER_ID PeerId,
    _In_ BOOLEAN bBacklogged)
{
    UNREFERENCED_PARAMETER(MiniportTalTxRxContext);
    UNREFERENCED_PARAMETER(PortId);
    UNREFERENCED_PARAMETER(PeerId);
    UNREFERENCED_PARAMETER(bBacklogged);
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
    UNREFERENCED_PARAMETER(NumSuspectFrames);
    UNREFERENCED_PARAMETER(SuspectFrameList);
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
    UNREFERENCED_PARAMETER(PortId);

    /* Nothing is running, so it is already stopped. */
    *pWifiStatus = NDIS_STATUS_SUCCESS;
}

static VOID
VwifiTalRxFlush(
    _In_ TAL_TXRX_HANDLE MiniportTalTxRxContext,
    _In_ WDI_PORT_ID PortId)
{
    UNREFERENCED_PARAMETER(MiniportTalTxRxContext);
    UNREFERENCED_PARAMETER(PortId);
}

static VOID
VwifiTalRxRestart(
    _In_ TAL_TXRX_HANDLE MiniportTalTxRxContext,
    _In_ WDI_PORT_ID PortId)
{
    UNREFERENCED_PARAMETER(MiniportTalTxRxContext);
    UNREFERENCED_PARAMETER(PortId);
}

static VOID
VwifiTalRxGetMpdus(
    _In_ TAL_TXRX_HANDLE MiniportTalTxRxContext,
    _In_ WDI_PEER_ID PeerId,
    _In_ WDI_EXTENDED_TID ExTid,
    _Out_ PNET_BUFFER_LIST *ppNBL)
{
    UNREFERENCED_PARAMETER(MiniportTalTxRxContext);
    UNREFERENCED_PARAMETER(PeerId);
    UNREFERENCED_PARAMETER(ExTid);

    /* NULL, explicitly. This is an _Out_ pointer the component will
     * walk; leaving it as whatever was on the stack is the difference
     * between "no frames" and a bugcheck. */
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
}

static VOID
VwifiTalRxResume(_In_ TAL_TXRX_HANDLE MiniportTalTxRxContext)
{
    UNREFERENCED_PARAMETER(MiniportTalTxRxContext);
}

static VOID
VwifiTalRxThrottle(
    _In_ TAL_TXRX_HANDLE MiniportTalTxRxContext,
    _In_ WDI_RX_THROTTLE_LEVEL RxThrottleLevel)
{
    UNREFERENCED_PARAMETER(MiniportTalTxRxContext);
    UNREFERENCED_PARAMETER(RxThrottleLevel);
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
