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
 * and wire up data-path resources. We have none of that, so we
 * complete immediately — but we must still *call* the completion
 * routine, because these are asynchronous operations. Returning
 * success without completing leaves the WLAN component waiting
 * forever, with nothing logged anywhere.
 * ============================================================ */
_Use_decl_annotations_
NDIS_STATUS
VwifiWdiOpenAdapter(
    NDIS_HANDLE MiniportAdapterContext,
    PNDIS_MINIPORT_INIT_PARAMETERS MiniportInitParameters)
{
    PVWIFI_ADAPTER adapter = (PVWIFI_ADAPTER)MiniportAdapterContext;

    UNREFERENCED_PARAMETER(MiniportInitParameters);
    VWIFI_INFO("WdiOpenAdapter");

    if (adapter->OpenAdapterCompleteHandler != NULL) {
        adapter->OpenAdapterCompleteHandler(adapter->MiniportAdapterHandle,
                                            NDIS_STATUS_SUCCESS);
    }
    return NDIS_STATUS_SUCCESS;
}

_Use_decl_annotations_
NDIS_STATUS
VwifiWdiCloseAdapter(NDIS_HANDLE MiniportAdapterContext)
{
    PVWIFI_ADAPTER adapter = (PVWIFI_ADAPTER)MiniportAdapterContext;

    VWIFI_INFO("WdiCloseAdapter");

    if (adapter->CloseAdapterCompleteHandler != NULL) {
        adapter->CloseAdapterCompleteHandler(adapter->MiniportAdapterHandle,
                                             NDIS_STATUS_SUCCESS);
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
    return NDIS_STATUS_SUCCESS;
}

_Use_decl_annotations_
VOID
VwifiWdiStopOperation(NDIS_HANDLE MiniportAdapterContext)
{
    UNREFERENCED_PARAMETER(MiniportAdapterContext);
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
    UNREFERENCED_PARAMETER(NdisMiniportDataPathHandle);
    UNREFERENCED_PARAMETER(NdisWdiDataPathApi);
    UNREFERENCED_PARAMETER(pMiniportDataHandlers);

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
}

/* ============================================================
 * Low-energy idle notification (USB selective suspend). Not
 * relevant for a virtual PCI device; just succeed so the WLAN
 * component's power management doesn't get wedged.
 * ============================================================ */
_Use_decl_annotations_
NDIS_STATUS
VwifiWdiLeIdleNotification(
    NDIS_HANDLE MiniportAdapterContext,
    BOOLEAN ForceIdle)
{
    UNREFERENCED_PARAMETER(MiniportAdapterContext);
    UNREFERENCED_PARAMETER(ForceIdle);
    return NDIS_STATUS_SUCCESS;
}

_Use_decl_annotations_
VOID
VwifiWdiLeCancelIdleNotification(NDIS_HANDLE MiniportAdapterContext)
{
    UNREFERENCED_PARAMETER(MiniportAdapterContext);
}
