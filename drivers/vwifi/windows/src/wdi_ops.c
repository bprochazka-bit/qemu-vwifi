/*
 * vwifi — wdi_ops.c
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * WDI control-path callback implementations. In Phase 1 these
 * are minimal: we accept the Microsoft WLAN component's lifecycle
 * calls and plumb them to our own adapter state without actually
 * doing per-port work (ports, peers, roaming, scans come in later
 * phases).
 */

#include "vwifi_drv.h"

/* AllocateAdapter runs before MiniportInitializeEx, so there's no
 * adapter context to store the peer version in yet. Park it here and
 * have MiniportInitializeEx copy it into the adapter. Single-threaded
 * per PnP start, so no locking needed. */
ULONG g_WdiPeerVersion = 0;

/* ============================================================
 * AllocateAdapter / FreeAdapter
 *
 * Invoked once per adapter, BEFORE MiniportInitializeEx on the
 * same device. This is the WDI model's split: Allocate creates
 * the context; Initialize binds it to NDIS; OpenAdapter starts it.
 *
 * Because our driver's MiniportInitializeEx does all the hard
 * work, these handlers are effectively no-ops that just pass the
 * context through.
 * ============================================================ */
_Use_decl_annotations_
NDIS_STATUS
VwifiWdiAllocateAdapter(
    NDIS_HANDLE MiniportDriverContext,
    PNDIS_WDI_INIT_PARAMETERS InitParams,
    NDIS_HANDLE *MiniportWdiAdapterContext,
    PWDI_ADAPTER_ATTRIBUTES *AdapterAttributes)
{
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
     *
     * [VERIFY] The exact field name on NDIS_WDI_INIT_PARAMETERS needs
     * checking against dot11wdi.h — it's the WDI version the OS
     * reports. If the struct doesn't carry it, the fallback is to
     * take it from the WdiVersion echoed back after
     * NdisMRegisterWdiMiniportDriver returns.
     * ============================================================ */
    if (InitParams != NULL) {
        /* [VERIFY] field name */
        g_WdiPeerVersion = InitParams->WdiVersion;
        VWIFI_INFO("WDI peer version: 0x%08x", g_WdiPeerVersion);
    } else {
        /* Defensive: assume the version we compiled against rather
         * than 0, which the library rejects with
         * NDIS_STATUS_NOT_SUPPORTED_REVISION. */
        g_WdiPeerVersion = WDI_VERSION_LATEST;
        VWIFI_WARN("no init params; assuming WDI_VERSION_LATEST");
    }

    /* We defer all real work to MiniportInitializeEx, which runs
     * right after this call. Just hand back a NULL context for now;
     * the real PVWIFI_ADAPTER gets installed via
     * NdisMSetMiniportAttributes during Initialize. */
    *MiniportWdiAdapterContext = NULL;

    /* Fill in minimal adapter attributes for the WLAN component.
     * Detailed capabilities come through OID handlers (later). */
    static WDI_ADAPTER_ATTRIBUTES attrs = { 0 };
    *AdapterAttributes = &attrs;

    return NDIS_STATUS_SUCCESS;
}

_Use_decl_annotations_
VOID
VwifiWdiFreeAdapter(NDIS_HANDLE MiniportWdiAdapterContext)
{
    UNREFERENCED_PARAMETER(MiniportWdiAdapterContext);
    VWIFI_INFO("WdiFreeAdapter");
}

/* ============================================================
 * OpenAdapter / CloseAdapter
 *
 * OpenAdapter is where the WLAN component declares "I'm ready to
 * start issuing tasks." Real drivers use this to download firmware
 * and wire up data-path resources. We have none of that, so we
 * just succeed.
 * ============================================================ */
_Use_decl_annotations_
NDIS_STATUS
VwifiWdiOpenAdapter(
    NDIS_HANDLE MiniportAdapterContext,
    PWDI_OPEN_ADAPTER_PARAMETERS OpenParameters)
{
    UNREFERENCED_PARAMETER(MiniportAdapterContext);
    UNREFERENCED_PARAMETER(OpenParameters);
    VWIFI_INFO("WdiOpenAdapter");
    return NDIS_STATUS_SUCCESS;
}

_Use_decl_annotations_
VOID
VwifiWdiCloseAdapter(NDIS_HANDLE MiniportAdapterContext)
{
    UNREFERENCED_PARAMETER(MiniportAdapterContext);
    VWIFI_INFO("WdiCloseAdapter");
}

/* ============================================================
 * Start / Stop operation
 *
 * These bracket one or more WDI tasks (scan, connect, etc.).
 * Placeholders in Phase 1.
 * ============================================================ */
_Use_decl_annotations_
NDIS_STATUS
VwifiWdiStartOperation(NDIS_HANDLE MiniportAdapterContext)
{
    UNREFERENCED_PARAMETER(MiniportAdapterContext);
    return NDIS_STATUS_SUCCESS;
}

_Use_decl_annotations_
NDIS_STATUS
VwifiWdiStopOperation(NDIS_HANDLE MiniportAdapterContext)
{
    UNREFERENCED_PARAMETER(MiniportAdapterContext);
    return NDIS_STATUS_SUCCESS;
}

/* ============================================================
 * PostPause / PostRestart
 *
 * Called after the WLAN component has finished its own
 * pause/restart work on the data path. The IHV can use these to
 * reclaim per-peer resources or re-prime queues.
 * ============================================================ */
_Use_decl_annotations_
VOID
VwifiWdiPostPause(NDIS_HANDLE MiniportAdapterContext)
{
    UNREFERENCED_PARAMETER(MiniportAdapterContext);
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
 * Return a simple failure code; NDIS will convert this into a
 * reset request if the WLAN component agrees we're hung.
 * ============================================================ */
_Use_decl_annotations_
VOID
VwifiWdiHangDiagnose(
    NDIS_HANDLE MiniportAdapterContext,
    PWDI_HANG_DIAGNOSTICS_BUFFER DiagnosticsBuffer)
{
    PVWIFI_ADAPTER adapter = (PVWIFI_ADAPTER)MiniportAdapterContext;
    UNREFERENCED_PARAMETER(DiagnosticsBuffer);

    /* Phase 1: nothing meaningful to diagnose. A later phase can
     * record last-seen ring indices and pending-request stats. */
    if (adapter && adapter->MmioVirtualAddress) {
        ULONG sig = VwifiRead32(adapter, VWIFI_REG_SIGNATURE);
        ULONG sts = VwifiRead32(adapter, VWIFI_REG_STATUS);
        VWIFI_WARN("HangDiagnose: sig=0x%08x status=0x%08x", sig, sts);
    }
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
    TAL_TXRX_HANDLE *TalTxRxHandle)
{
    UNREFERENCED_PARAMETER(MiniportAdapterContext);
    /* Return our adapter pointer as the TAL handle so later callbacks
     * can find us. */
    *TalTxRxHandle = (TAL_TXRX_HANDLE)MiniportAdapterContext;
    return NDIS_STATUS_SUCCESS;
}

_Use_decl_annotations_
VOID
VwifiWdiTalTxRxDeinitialize(TAL_TXRX_HANDLE TalTxRxHandle)
{
    UNREFERENCED_PARAMETER(TalTxRxHandle);
}

/* ============================================================
 * Low-energy idle notification (runtime D3 power hint). Not
 * relevant for a virtual device; just succeed so the WLAN
 * component's power management doesn't get wedged.
 * ============================================================ */
_Use_decl_annotations_
NDIS_STATUS
VwifiWdiLeIdleNotification(
    NDIS_HANDLE MiniportAdapterContext,
    PNDIS_WDI_IDLE_NOTIFICATION_INFO IdleNotification)
{
    UNREFERENCED_PARAMETER(MiniportAdapterContext);
    UNREFERENCED_PARAMETER(IdleNotification);
    return NDIS_STATUS_SUCCESS;
}

_Use_decl_annotations_
VOID
VwifiWdiLeCancelIdleNotification(NDIS_HANDLE MiniportAdapterContext)
{
    UNREFERENCED_PARAMETER(MiniportAdapterContext);
}
