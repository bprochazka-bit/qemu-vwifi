/*
 * vwifi — WDI miniport driver entry point and NDIS registration.
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * This file wires up DriverEntry, populates the two characteristics
 * structures (NDIS miniport + WDI), and calls NdisMRegisterWdiMiniport
 * Driver. Per the WDI IHV driver interfaces doc, most NDIS miniport
 * handlers are optional for WDI because the Microsoft WLAN component
 * provides them; only OidRequestHandler and DriverUnload are required.
 * We additionally hook MiniportInitializeEx/HaltEx for PCI resource
 * setup, since those aren't provided by the WLAN component.
 */

#include "vwifi_drv.h"

/* Registered by NdisMRegisterWdiMiniportDriver; saved for unload. */
static NDIS_HANDLE g_DriverHandle = NULL;

/* ====================================================================
 * NDIS miniport handlers required beyond what WLAN component provides
 * ==================================================================== */

MINIPORT_INITIALIZE VwifiMiniportInitializeEx;
MINIPORT_HALT       VwifiMiniportHaltEx;
MINIPORT_PAUSE      VwifiMiniportPause;
MINIPORT_RESTART    VwifiMiniportRestart;
MINIPORT_RETURN_NET_BUFFER_LISTS VwifiMiniportReturnNetBufferLists;
MINIPORT_SEND_NET_BUFFER_LISTS   VwifiMiniportSendNetBufferLists;
MINIPORT_CANCEL_SEND VwifiMiniportCancelSend;
MINIPORT_CHECK_FOR_HANG VwifiMiniportCheckForHangEx;
MINIPORT_RESET      VwifiMiniportReset;
MINIPORT_DEVICE_PNP_EVENT_NOTIFY VwifiMiniportDevicePnPEventNotify;
MINIPORT_SHUTDOWN   VwifiMiniportShutdownEx;

/* ====================================================================
 * MiniportInitializeEx
 *
 * Called by NDIS after PnP has claimed the PCI device. Here we:
 *   1. Allocate the adapter context
 *   2. Parse assigned PCI resources
 *   3. Map BAR0 (MMIO) into kernel VA
 *   4. Allocate and program the four rings
 *   5. Connect the interrupt
 *   6. Issue GET_CAPS to discover the device's default MAC & features
 *   7. Register our registration & general attributes with NDIS
 * ==================================================================== */

static NDIS_STATUS
VwifiSetRegistrationAttributes(_Inout_ PVWIFI_ADAPTER Adapter)
{
    NDIS_MINIPORT_ADAPTER_REGISTRATION_ATTRIBUTES reg = { 0 };
    reg.Header.Type = NDIS_OBJECT_TYPE_MINIPORT_ADAPTER_REGISTRATION_ATTRIBUTES;
    reg.Header.Revision = NDIS_MINIPORT_ADAPTER_REGISTRATION_ATTRIBUTES_REVISION_2;
    reg.Header.Size = NDIS_SIZEOF_MINIPORT_ADAPTER_REGISTRATION_ATTRIBUTES_REVISION_2;
    reg.MiniportAdapterContext = (NDIS_HANDLE)Adapter;
    reg.AttributeFlags = NDIS_MINIPORT_ATTRIBUTES_BUS_MASTER |
                         NDIS_MINIPORT_ATTRIBUTES_HARDWARE_DEVICE;
    reg.CheckForHangTimeInSeconds = 4;
    reg.InterfaceType = NdisInterfacePci;

    return NdisMSetMiniportAttributes(
        Adapter->MiniportAdapterHandle,
        (PNDIS_MINIPORT_ADAPTER_ATTRIBUTES)&reg);
}

_Use_decl_annotations_
NDIS_STATUS
VwifiMiniportInitializeEx(
    NDIS_HANDLE MiniportAdapterHandle,
    NDIS_HANDLE MiniportDriverContext,
    PNDIS_MINIPORT_INIT_PARAMETERS MiniportInitParameters)
{
    PVWIFI_ADAPTER adapter;
    NDIS_STATUS status;

    UNREFERENCED_PARAMETER(MiniportDriverContext);

    VWIFI_INFO("MiniportInitializeEx entry");

    adapter = NdisAllocateMemoryWithTagPriority(
        MiniportAdapterHandle, sizeof(*adapter),
        VWIFI_POOL_TAG, NormalPoolPriority);
    if (!adapter) {
        return NDIS_STATUS_RESOURCES;
    }
    RtlZeroMemory(adapter, sizeof(*adapter));
    adapter->MiniportAdapterHandle = MiniportAdapterHandle;
    InitializeListHead(&adapter->PendingReqList);
    KeInitializeSpinLock(&adapter->PendingReqLock);

    /* Inherit the WDI version the OS reported to AllocateAdapter, which
     * ran just before us. Every TLV_CONTEXT carries this. Getting it
     * wrong means the parser returns NDIS_STATUS_UNSUPPORTED_REVISION
     * (PeerVersion below WDI_VERSION_MIN_SUPPORTED) or, worse, silently
     * mis-encodes. */
    adapter->WdiPeerVersion = VwifiGetAllocatedWdiVersion();
    VWIFI_INFO("adapter peer WDI version = 0x%08x", adapter->WdiPeerVersion);

    /* Pick up the WDI version the OS reported in AllocateAdapter.
     * Every TLV_CONTEXT we build uses this. */
    adapter->WdiPeerVersion = g_WdiPeerVersion;
    VWIFI_INFO("adapter WDI peer version 0x%08x", adapter->WdiPeerVersion);

    status = VwifiSetRegistrationAttributes(adapter);
    if (status != NDIS_STATUS_SUCCESS) {
        goto fail;
    }

    status = VwifiHwInitialize(adapter, MiniportInitParameters);
    if (status != NDIS_STATUS_SUCCESS) {
        VWIFI_ERR("VwifiHwInitialize failed 0x%x", status);
        goto fail;
    }

    VWIFI_INFO("adapter %p initialized successfully", adapter);
    return NDIS_STATUS_SUCCESS;

fail:
    VwifiHwShutdown(adapter);
    NdisFreeMemoryWithTagPriority(MiniportAdapterHandle, adapter,
                                  VWIFI_POOL_TAG);
    return status;
}

/* ====================================================================
 * MiniportHaltEx
 * ==================================================================== */
_Use_decl_annotations_
VOID
VwifiMiniportHaltEx(
    NDIS_HANDLE MiniportAdapterContext,
    NDIS_HALT_ACTION HaltAction)
{
    PVWIFI_ADAPTER adapter = (PVWIFI_ADAPTER)MiniportAdapterContext;
    UNREFERENCED_PARAMETER(HaltAction);

    VWIFI_INFO("MiniportHaltEx");
    VwifiHwShutdown(adapter);
    NdisFreeMemoryWithTagPriority(adapter->MiniportAdapterHandle, adapter,
                                  VWIFI_POOL_TAG);
}

/* ====================================================================
 * Pause/Restart — data path control. Phase 1: just flip a flag; the
 * WDI component will sequence PostPause/PostRestart callbacks for us.
 * ==================================================================== */
_Use_decl_annotations_
NDIS_STATUS
VwifiMiniportPause(
    NDIS_HANDLE MiniportAdapterContext,
    PNDIS_MINIPORT_PAUSE_PARAMETERS PauseParameters)
{
    PVWIFI_ADAPTER adapter = (PVWIFI_ADAPTER)MiniportAdapterContext;
    UNREFERENCED_PARAMETER(PauseParameters);
    InterlockedExchange(&adapter->DataPathRunning, 0);
    return NDIS_STATUS_SUCCESS;
}

_Use_decl_annotations_
NDIS_STATUS
VwifiMiniportRestart(
    NDIS_HANDLE MiniportAdapterContext,
    PNDIS_MINIPORT_RESTART_PARAMETERS RestartParameters)
{
    PVWIFI_ADAPTER adapter = (PVWIFI_ADAPTER)MiniportAdapterContext;
    UNREFERENCED_PARAMETER(RestartParameters);
    InterlockedExchange(&adapter->DataPathRunning, 1);
    return NDIS_STATUS_SUCCESS;
}

/* ====================================================================
 * Send / Return / Cancel — data path stubs for Phase 1.
 *
 * In a WDI driver the real TX path goes through the WDI TAL
 * callbacks (MiniportWdiTxTalSend). These NDIS-level handlers
 * are a fallback NDIS requires us to provide. In Phase 1 we
 * reject sends cleanly.
 * ==================================================================== */
_Use_decl_annotations_
VOID
VwifiMiniportSendNetBufferLists(
    NDIS_HANDLE MiniportAdapterContext,
    PNET_BUFFER_LIST NetBufferLists,
    NDIS_PORT_NUMBER PortNumber,
    ULONG SendFlags)
{
    PVWIFI_ADAPTER adapter = (PVWIFI_ADAPTER)MiniportAdapterContext;
    PNET_BUFFER_LIST nbl, next;
    UNREFERENCED_PARAMETER(PortNumber);

    for (nbl = NetBufferLists; nbl; nbl = next) {
        next = NET_BUFFER_LIST_NEXT_NBL(nbl);
        NET_BUFFER_LIST_NEXT_NBL(nbl) = NULL;

        if (adapter->OpMode == VWIFI_MODE_MONITOR) {
            /* Injection: each NB is a raw 802.11 frame (maybe with a
             * radiotap prefix). Flatten and inject. */
            PNET_BUFFER nb = NET_BUFFER_LIST_FIRST_NB(nbl);
            NDIS_STATUS st = NDIS_STATUS_SUCCESS;

            for (; nb; nb = NET_BUFFER_NEXT_NB(nb)) {
                ULONG len = NET_BUFFER_DATA_LENGTH(nb);
                PUCHAR flat;
                PVOID alloc = NULL;

                /* Try to get a contiguous pointer; if the NB spans
                 * MDLs, copy into a temporary buffer. */
                flat = NdisGetDataBuffer(nb, len, NULL, 1, 0);
                if (!flat) {
                    alloc = NdisAllocateMemoryWithTagPriority(
                        adapter->MiniportAdapterHandle, len,
                        VWIFI_POOL_TAG, NormalPoolPriority);
                    if (!alloc) { st = NDIS_STATUS_RESOURCES; break; }
                    flat = NdisGetDataBuffer(nb, len, alloc, 1, 0);
                }
                if (flat) {
                    st = VwifiInjectFrame(adapter, flat, len);
                }
                if (alloc) {
                    NdisFreeMemoryWithTagPriority(
                        adapter->MiniportAdapterHandle, alloc,
                        VWIFI_POOL_TAG);
                }
                if (st != NDIS_STATUS_SUCCESS) break;
            }
            NET_BUFFER_LIST_STATUS(nbl) = st;
        } else if (adapter->OpMode == VWIFI_MODE_STA && adapter->Associated) {
            /* STA data path. The device converts 802.3 -> 802.11 using
             * its association state, so we just hand it the 802.3
             * frame the WLAN component gave us. */
            PNET_BUFFER nb = NET_BUFFER_LIST_FIRST_NB(nbl);
            NDIS_STATUS st = NDIS_STATUS_SUCCESS;

            for (; nb; nb = NET_BUFFER_NEXT_NB(nb)) {
                ULONG len = NET_BUFFER_DATA_LENGTH(nb);
                PUCHAR flat;
                PVOID alloc = NULL;

                flat = NdisGetDataBuffer(nb, len, NULL, 1, 0);
                if (!flat) {
                    alloc = NdisAllocateMemoryWithTagPriority(
                        adapter->MiniportAdapterHandle, len,
                        VWIFI_POOL_TAG, NormalPoolPriority);
                    if (!alloc) { st = NDIS_STATUS_RESOURCES; break; }
                    flat = NdisGetDataBuffer(nb, len, alloc, 1, 0);
                }
                if (flat) {
                    st = VwifiTxDataFrame(adapter, flat, len);
                }
                if (alloc) {
                    NdisFreeMemoryWithTagPriority(
                        adapter->MiniportAdapterHandle, alloc,
                        VWIFI_POOL_TAG);
                }
                if (st != NDIS_STATUS_SUCCESS) break;
            }
            NET_BUFFER_LIST_STATUS(nbl) = st;
        } else {
            /* Not associated and not monitoring — nothing to do. */
            NET_BUFFER_LIST_STATUS(nbl) = NDIS_STATUS_PAUSED;
        }
    }

    NdisMSendNetBufferListsComplete(
        adapter->MiniportAdapterHandle,
        NetBufferLists,
        (SendFlags & NDIS_SEND_FLAGS_DISPATCH_LEVEL)
            ? NDIS_SEND_COMPLETE_FLAGS_DISPATCH_LEVEL : 0);
}

/* VwifiMiniportReturnNetBufferLists now lives in monitor.c — it
 * reclaims RX NBLs indicated by the monitor-mode drain and re-arms
 * their backing ring slots. */

_Use_decl_annotations_
VOID
VwifiMiniportCancelSend(
    NDIS_HANDLE MiniportAdapterContext,
    PVOID CancelId)
{
    UNREFERENCED_PARAMETER(MiniportAdapterContext);
    UNREFERENCED_PARAMETER(CancelId);
}

/* ====================================================================
 * Check-for-hang / Reset / PnP / Shutdown
 * ==================================================================== */
_Use_decl_annotations_
BOOLEAN
VwifiMiniportCheckForHangEx(NDIS_HANDLE MiniportAdapterContext)
{
    PVWIFI_ADAPTER adapter = (PVWIFI_ADAPTER)MiniportAdapterContext;
    ULONG sig;

    /* Quick sanity check: signature register must read back. If the
     * device has disappeared or gone insane, the read will likely
     * return 0xFFFFFFFF and we report a hang. */
    sig = VwifiRead32(adapter, VWIFI_REG_SIGNATURE);
    if (sig != VWIFI_SIGNATURE) {
        VWIFI_WARN("check-for-hang: signature 0x%08x != expected 0x%08x",
                   sig, VWIFI_SIGNATURE);
        return TRUE;
    }
    return FALSE;
}

_Use_decl_annotations_
NDIS_STATUS
VwifiMiniportReset(
    NDIS_HANDLE MiniportAdapterContext,
    PBOOLEAN AddressingReset)
{
    PVWIFI_ADAPTER adapter = (PVWIFI_ADAPTER)MiniportAdapterContext;
    *AddressingReset = FALSE;
    return VwifiHwReset(adapter);
}

_Use_decl_annotations_
VOID
VwifiMiniportDevicePnPEventNotify(
    NDIS_HANDLE MiniportAdapterContext,
    PNET_DEVICE_PNP_EVENT NetDevicePnPEvent)
{
    UNREFERENCED_PARAMETER(MiniportAdapterContext);
    UNREFERENCED_PARAMETER(NetDevicePnPEvent);
}

_Use_decl_annotations_
VOID
VwifiMiniportShutdownEx(
    NDIS_HANDLE MiniportAdapterContext,
    NDIS_SHUTDOWN_ACTION ShutdownAction)
{
    PVWIFI_ADAPTER adapter = (PVWIFI_ADAPTER)MiniportAdapterContext;
    UNREFERENCED_PARAMETER(ShutdownAction);

    /* Quiesce the device quickly — can be called at high IRQL
     * during bugcheck. Just assert reset and return. */
    VwifiWrite32(adapter, VWIFI_REG_RESET, 1);
}

/* MiniportOidRequest (VwifiOidRequest) now lives in oids.c — it
 * intercepts the Native 802.11 OIDs Npcap uses for monitor mode and
 * falls through to NOT_SUPPORTED for everything else. */

/* ====================================================================
 * DriverUnload
 * ==================================================================== */
_Use_decl_annotations_
VOID
VwifiDriverUnload(PDRIVER_OBJECT DriverObject)
{
    UNREFERENCED_PARAMETER(DriverObject);
    VWIFI_INFO("DriverUnload");
    if (g_DriverHandle) {
        NdisMDeregisterMiniportDriver(g_DriverHandle);
        g_DriverHandle = NULL;
    }
}

/* ====================================================================
 * DriverEntry
 * ==================================================================== */
_Use_decl_annotations_
NTSTATUS
DriverEntry(
    PDRIVER_OBJECT DriverObject,
    PUNICODE_STRING RegistryPath)
{
    NDIS_MINIPORT_DRIVER_CHARACTERISTICS          m = { 0 };
    NDIS_MINIPORT_DRIVER_WDI_CHARACTERISTICS      w = { 0 };
    NDIS_STATUS status;

    VWIFI_INFO("DriverEntry");

    /* Base NDIS miniport characteristics. Per WDI IHV doc, most of
     * these are optional when the WLAN component is in play; OID and
     * Unload are the minimum. We fill in more to handle PCI resource
     * setup and pause/restart cleanly. */
    m.Header.Type         = NDIS_OBJECT_TYPE_MINIPORT_DRIVER_CHARACTERISTICS;
    m.Header.Revision     = NDIS_MINIPORT_DRIVER_CHARACTERISTICS_REVISION_2;
    m.Header.Size         = NDIS_SIZEOF_MINIPORT_DRIVER_CHARACTERISTICS_REVISION_2;
    m.MajorNdisVersion    = VWIFI_NDIS_MAJOR_VERSION;
    m.MinorNdisVersion    = VWIFI_NDIS_MINOR_VERSION;
    m.MajorDriverVersion  = 1;
    m.MinorDriverVersion  = 0;
    m.InitializeHandlerEx          = VwifiMiniportInitializeEx;
    m.HaltHandlerEx                = VwifiMiniportHaltEx;
    m.PauseHandler                 = VwifiMiniportPause;
    m.RestartHandler               = VwifiMiniportRestart;
    m.OidRequestHandler            = VwifiOidRequest;
    m.SendNetBufferListsHandler    = VwifiMiniportSendNetBufferLists;
    m.ReturnNetBufferListsHandler  = VwifiMiniportReturnNetBufferLists;
    m.CancelSendHandler            = VwifiMiniportCancelSend;
    m.CheckForHangHandlerEx        = VwifiMiniportCheckForHangEx;
    m.ResetHandlerEx               = VwifiMiniportReset;
    m.DevicePnPEventNotifyHandler  = VwifiMiniportDevicePnPEventNotify;
    m.ShutdownHandlerEx            = VwifiMiniportShutdownEx;
    m.UnloadHandler                = VwifiDriverUnload;

    /* WDI characteristics — the control-path callbacks the IHV
     * miniport must register. Data-path handlers come separately
     * later (in OpenAdapter or via TalTxRxInitialize). */
    w.Header.Type     = NDIS_OBJECT_TYPE_MINIPORT_WDI_CHARACTERISTICS;
    w.Header.Revision = NDIS_MINIPORT_WDI_CHARACTERISTICS_REVISION_1;
    w.Header.Size     = NDIS_SIZEOF_MINIPORT_WDI_CHARACTERISTICS_REVISION_1;
    w.WdiVersion      = WDI_VERSION_LATEST;
    w.AllocateAdapterHandler         = VwifiWdiAllocateAdapter;
    w.FreeAdapterHandler             = VwifiWdiFreeAdapter;
    w.OpenAdapterHandler             = VwifiWdiOpenAdapter;
    w.CloseAdapterHandler            = VwifiWdiCloseAdapter;
    w.StartOperationHandler          = VwifiWdiStartOperation;
    w.StopOperationHandler           = VwifiWdiStopOperation;
    w.PostPauseHandler               = VwifiWdiPostPause;
    w.PostRestartHandler             = VwifiWdiPostRestart;
    w.HangDiagnoseHandler            = VwifiWdiHangDiagnose;
    w.TalTxRxInitializeHandler       = VwifiWdiTalTxRxInitialize;
    w.TalTxRxDeinitializeHandler     = VwifiWdiTalTxRxDeinitialize;
    w.LeIdleNotificationHandler      = VwifiWdiLeIdleNotification;
    w.LeCancelIdleNotificationHandler = VwifiWdiLeCancelIdleNotification;

    status = NdisMRegisterWdiMiniportDriver(
        DriverObject, RegistryPath, NULL,
        &m, &w, &g_DriverHandle);
    if (status != NDIS_STATUS_SUCCESS) {
        VWIFI_ERR("NdisMRegisterWdiMiniportDriver failed 0x%x", status);
        return status;
    }

    return STATUS_SUCCESS;
}
