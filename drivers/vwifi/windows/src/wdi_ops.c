/*
 * vwifi — wdi_ops.c
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * WDI control-path callback implementations.
 *
 * These are the handlers listed in NDIS_MINIPORT_DRIVER_WDI_CHARACTERISTICS.
 * Their signatures come from dot11wdi.h and are not what the WDI
 * documentation's prose suggests — in particular AllocateAdapter is the
 * WDI model's MiniportInitializeEx. It receives the NDIS miniport
 * handle, the PnP init parameters and an _Inout_ registration
 * attributes block to fill in; the WLAN component owns the real
 * MiniportInitializeEx and calls us from inside it. That is why adapter
 * creation lives here and not in driver.c's NDIS handler table.
 */

#include "vwifi_drv.h"

/* AllocateAdapter runs before there is an adapter context to store the
 * peer version in. Park it here and have VwifiAdapterCreate copy it into
 * the adapter. Single-threaded per PnP start, so no locking needed. */
ULONG g_WdiPeerVersion = 0;

/* ============================================================
 * AllocateAdapter / FreeAdapter
 *
 * Allocate is called once per adapter, in place of
 * MiniportInitializeEx. Free is its mirror, in place of
 * MiniportHaltEx.
 * ============================================================ */
_Use_decl_annotations_
NDIS_STATUS
VwifiWdiAllocateAdapter(
    NDIS_HANDLE NdisMiniportHandle,
    NDIS_HANDLE MiniportDriverContext,
    PNDIS_MINIPORT_INIT_PARAMETERS MiniportInitParameters,
    PNDIS_WDI_INIT_PARAMETERS NdisWdiInitParameters,
    PNDIS_MINIPORT_ADAPTER_REGISTRATION_ATTRIBUTES RegistrationAttributes)
{
    NDIS_STATUS status;

    UNREFERENCED_PARAMETER(MiniportDriverContext);

    VWIFI_INFO("WdiAllocateAdapter");

    /* ============================================================
     * Capture the peer's WDI version.
     *
     * We register with WDI_VERSION_LATEST (see DriverEntry) because
     * Microsoft is explicit that pinning a specific version "will
     * become out of date and cause problems with the TLV parser
     * generator because the other end might send a byte stream that
     * is unexpected."
     *
     * But *generating* must respect what the peer actually speaks.
     * The OS hands us its version here; we stash it and feed it into
     * every TLV_CONTEXT.PeerVersion. The library then emits an older
     * byte stream when running on older WDI, and consumes older
     * streams on parse. That's the whole single-binary story.
     * ============================================================ */
    if (NdisWdiInitParameters != NULL) {
        g_WdiPeerVersion = NdisWdiInitParameters->WdiVersion;
        VWIFI_INFO("WDI peer version: 0x%08x", g_WdiPeerVersion);
    } else {
        /* Defensive: assume the version we compiled against rather
         * than 0, which the library rejects with
         * NDIS_STATUS_NOT_SUPPORTED_REVISION. */
        g_WdiPeerVersion = WDI_VERSION_LATEST;
        VWIFI_WARN("no WDI init params; assuming WDI_VERSION_LATEST");
    }

    status = VwifiAdapterCreate(NdisMiniportHandle, MiniportInitParameters,
                                RegistrationAttributes);
    if (status != NDIS_STATUS_SUCCESS) {
        /* This is what PnP reports as Code 10, "device cannot start".
         * Log it at the boundary so the failing status is visible even
         * when the specific step below it did not log one. */
        VWIFI_ERR("WdiAllocateAdapter failing with 0x%08x — "
                  "device will not start", status);
        return status;
    }

    /* Stash the OS completion routines now that the adapter exists.
     * OpenAdapter/CloseAdapter are asynchronous and are only finished
     * when we call these back. */
    if (NdisWdiInitParameters != NULL) {
        PVWIFI_ADAPTER adapter =
            (PVWIFI_ADAPTER)RegistrationAttributes->MiniportAdapterContext;
        adapter->OpenAdapterCompleteHandler =
            NdisWdiInitParameters->OpenAdapterCompleteHandler;
        adapter->CloseAdapterCompleteHandler =
            NdisWdiInitParameters->CloseAdapterCompleteHandler;
    }

    return NDIS_STATUS_SUCCESS;
}

_Use_decl_annotations_
VOID
VwifiWdiFreeAdapter(NDIS_HANDLE MiniportAdapterContext)
{
    VWIFI_INFO("WdiFreeAdapter");
    VwifiAdapterDestroy(MiniportAdapterContext);
}

/* ============================================================
 * OpenAdapter / CloseAdapter
 *
 * OpenAdapter is where the WLAN component declares "I'm ready to
 * start issuing tasks." Real drivers use this to download firmware
 * and wire up data-path resources; for us it is where the rings and
 * the interrupt come up.
 *
 * Both are asynchronous, so the completion routine from
 * NDIS_WDI_INIT_PARAMETERS must be called: returning success without
 * completing leaves the WLAN component waiting forever with nothing
 * logged anywhere.
 *
 * On the return value, DO NOT "fix" this to NDIS_STATUS_PENDING
 * without evidence. That looks like the tidier async contract -- the
 * callback says the work is done, so the return value saying it
 * finished synchronously is arguably a double completion -- and it was
 * tried, on exactly that reasoning and no data. The build that
 * followed locked the guest hard during install. That is not proof
 * this was the cause, since another change shipped alongside it, but
 * the shape here (complete, then return the status) is the one that
 * has been observed to open the adapter cleanly, and reasoning about
 * an undocumented state machine is worth less than that.
 * ============================================================ */
_Use_decl_annotations_
NDIS_STATUS
VwifiWdiOpenAdapter(
    NDIS_HANDLE MiniportAdapterContext,
    PNDIS_MINIPORT_INIT_PARAMETERS MiniportInitParameters)
{
    PVWIFI_ADAPTER adapter = (PVWIFI_ADAPTER)MiniportAdapterContext;
    NDIS_STATUS status;

    UNREFERENCED_PARAMETER(MiniportInitParameters);
    VWIFI_INFO("WdiOpenAdapter");

    /* The rings, the NBL pool and the interrupt are allocated here
     * rather than in AllocateAdapter because they need the registration
     * attributes to be in effect, and those are only applied after
     * AllocateAdapter returns. */
    status = VwifiHwStart(adapter);
    if (status != NDIS_STATUS_SUCCESS) {
        VWIFI_ERR("WdiOpenAdapter: start failed 0x%08x", status);
    }

    if (adapter->OpenAdapterCompleteHandler != NULL) {
        adapter->OpenAdapterCompleteHandler(adapter->MiniportAdapterHandle,
                                            status);
    } else {
        VWIFI_WARN("no OpenAdapterCompleteHandler; completing inline");
    }
    return status;
}

_Use_decl_annotations_
NDIS_STATUS
VwifiWdiCloseAdapter(NDIS_HANDLE MiniportAdapterContext)
{
    PVWIFI_ADAPTER adapter = (PVWIFI_ADAPTER)MiniportAdapterContext;

    VWIFI_INFO("WdiCloseAdapter");

    VwifiHwStop(adapter);

    if (adapter->CloseAdapterCompleteHandler != NULL) {
        adapter->CloseAdapterCompleteHandler(adapter->MiniportAdapterHandle,
                                             NDIS_STATUS_SUCCESS);
    } else {
        VWIFI_WARN("no CloseAdapterCompleteHandler; completing inline");
    }
    return NDIS_STATUS_SUCCESS;
}

/* ============================================================
 * Start / Stop operation
 *
 * These bracket one or more WDI tasks (scan, connect, etc.).
 * Placeholders in Phase 1. Note Stop returns void, not a status.
 * ============================================================ */
_Use_decl_annotations_
NDIS_STATUS
VwifiWdiStartOperation(NDIS_HANDLE MiniportAdapterContext)
{
    UNREFERENCED_PARAMETER(MiniportAdapterContext);
    VWIFI_INFO("WdiStartOperation");
    return NDIS_STATUS_SUCCESS;
}

_Use_decl_annotations_
VOID
VwifiWdiStopOperation(NDIS_HANDLE MiniportAdapterContext)
{
    UNREFERENCED_PARAMETER(MiniportAdapterContext);
    VWIFI_INFO("WdiStopOperation");
}

/* ============================================================
 * PostPause / PostRestart
 *
 * Called after the WLAN component has finished its own
 * pause/restart work on the data path. The IHV can use these to
 * reclaim per-peer resources or re-prime queues. Both take the
 * corresponding NDIS parameters block and return a status.
 * ============================================================ */
_Use_decl_annotations_
NDIS_STATUS
VwifiWdiPostPause(
    NDIS_HANDLE MiniportAdapterContext,
    PNDIS_MINIPORT_PAUSE_PARAMETERS PauseParameters)
{
    UNREFERENCED_PARAMETER(MiniportAdapterContext);
    UNREFERENCED_PARAMETER(PauseParameters);
    VWIFI_INFO("WdiPostPause");
    return NDIS_STATUS_SUCCESS;
}

_Use_decl_annotations_
NDIS_STATUS
VwifiWdiPostRestart(
    NDIS_HANDLE MiniportAdapterContext,
    PNDIS_MINIPORT_RESTART_PARAMETERS RestartParameters)
{
    UNREFERENCED_PARAMETER(MiniportAdapterContext);
    UNREFERENCED_PARAMETER(RestartParameters);
    VWIFI_INFO("WdiPostRestart");
    return NDIS_STATUS_SUCCESS;
}

/* ============================================================
 * HangDiagnose — WDI's cousin of MiniportCheckForHang.
 *
 * Note the first argument is the *driver* context, not the adapter
 * context, and the contract is to fill a caller-supplied blob that
 * ends up in a LiveKD dump. We have no firmware to dump; write the
 * two device registers that say whether the device is alive at all,
 * which is the only thing worth having post-mortem here.
 * ============================================================ */
_Use_decl_annotations_
NDIS_STATUS
VwifiWdiHangDiagnose(
    NDIS_HANDLE MiniportDriverContext,
    eDiagnoseLevel DiagnoseLevel,
    UINT32 BufferSize,
    UINT8 *FirmwareBlob,
    UINT32 *pOutputSize)
{
    UNREFERENCED_PARAMETER(MiniportDriverContext);
    UNREFERENCED_PARAMETER(DiagnoseLevel);

    VWIFI_INFO("WdiHangDiagnose");
    *pOutputSize = 0;

    if (FirmwareBlob == NULL || BufferSize == 0) {
        return NDIS_STATUS_SUCCESS;
    }

    /* Phase 1: nothing to dump. A later phase can record the last-seen
     * ring indices and pending-request stats here — that is the state
     * that actually explains a wedge. */
    return NDIS_STATUS_SUCCESS;
}

/* ============================================================
 * TAL (Target Abstraction Layer) TxRx lifecycle
 *
 * In real WDI drivers the TAL callbacks are where the IHV wires
 * up its hardware-specific TX/RX handlers. In Phase 1 we don't
 * route data through the TAL — the Microsoft WLAN component's
 * defaults are fine because we don't accept sends anyway.
 * ============================================================ */
_Use_decl_annotations_
NDIS_STATUS
VwifiWdiTalTxRxInitialize(
    NDIS_HANDLE MiniportAdapterContext,
    NDIS_HANDLE NdisMiniportDataPathHandle,
    PNDIS_WDI_DATA_API NdisWdiDataPathApi,
    PTAL_TXRX_HANDLE pMiniportTalTxRxContext,
    PNDIS_MINIPORT_WDI_DATA_HANDLERS pMiniportDataHandlers,
    UINT32 *pMiniportWdiFrameMetadataExtraSpace)
{
    PVWIFI_ADAPTER adapter = (PVWIFI_ADAPTER)MiniportAdapterContext;

    VWIFI_INFO("WdiTalTxRxInitialize");

    if (pMiniportDataHandlers == NULL) {
        VWIFI_ERR("no data handler table supplied");
        return NDIS_STATUS_INVALID_PARAMETER;
    }

    /* Keep the data-path handle and the component's own API table. The
     * TAL callbacks get a TAL_TXRX_HANDLE, not the adapter, so anything
     * that has to call back into the component later needs these
     * reachable from the adapter. */
    adapter->DataPathHandle = NdisMiniportDataPathHandle;
    adapter->DataPathApi    = NdisWdiDataPathApi;

    /* Fill in all twenty-five handlers. Leaving them NULL is what kept
     * the adapter from ever getting a port -- see the header comment in
     * wdi_tal.c. */
    VwifiTalFillDataHandlers(pMiniportDataHandlers);

    /* Return our adapter pointer as the TAL handle so later callbacks
     * can find us. */
    *pMiniportTalTxRxContext = (TAL_TXRX_HANDLE)MiniportAdapterContext;
    *pMiniportWdiFrameMetadataExtraSpace = 0;
    return NDIS_STATUS_SUCCESS;
}

_Use_decl_annotations_
VOID
VwifiWdiTalTxRxDeinitialize(TAL_TXRX_HANDLE MiniportTalTxRxContext)
{
    UNREFERENCED_PARAMETER(MiniportTalTxRxContext);
    VWIFI_INFO("WdiTalTxRxDeinitialize");
}

/* ============================================================
 * Low-energy idle notification (NDIS selective suspend).
 *
 * INCOMPLETE, DELIBERATELY, AND NOT SAFE TO CALL COMPLETE.
 *
 * The comment here used to read "not relevant for a virtual PCI
 * device; just succeed so the WLAN component's power management
 * doesn't get wedged." That is the same reasoning that produced the
 * RxFlush bug -- succeeding is exactly what wedges a caller whose
 * contract expects a callback -- so it is written out rather than left
 * to mislead the next reader.
 *
 * What the contract actually says: NDIS calls MiniportWdiIdleNotification
 * to START a selective-suspend operation. The driver then calls
 * NdisWdiIdleNotificationConfirm to say the adapter can safely be
 * suspended, or NdisWdiIdleNotificationComplete to complete a pending
 * notification. Both live in NDIS_WDI_INIT_PARAMETERS. Returning
 * success and calling neither does not decline the suspend; it starts
 * one and never finishes it.
 *
 * This has never been observed to fire, and should not: selective
 * suspend is a capability the adapter has to advertise, and this driver
 * advertises none of it. That is the only thing making the gap
 * harmless, and it is a property of the capabilities message rather
 * than of this code -- so if selective suspend is ever advertised,
 * these two handlers must be finished at the same time.
 * ============================================================ */
_Use_decl_annotations_
NDIS_STATUS
VwifiWdiLeIdleNotification(
    NDIS_HANDLE MiniportAdapterContext,
    BOOLEAN ForceIdle)
{
    UNREFERENCED_PARAMETER(MiniportAdapterContext);
    VWIFI_INFO("WdiLeIdleNotification force=%u", ForceIdle);
    return NDIS_STATUS_SUCCESS;
}

_Use_decl_annotations_
VOID
VwifiWdiLeCancelIdleNotification(NDIS_HANDLE MiniportAdapterContext)
{
    UNREFERENCED_PARAMETER(MiniportAdapterContext);
    VWIFI_INFO("WdiLeCancelIdleNotification");
}
