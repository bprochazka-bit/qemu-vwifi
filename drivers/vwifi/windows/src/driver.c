/*
 * vwifi — WDI miniport driver entry point and NDIS registration.
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * This file wires up DriverEntry, populates the two characteristics
 * structures (NDIS miniport + WDI), and calls NdisMRegisterWdiMiniport
 * Driver. Per the WDI IHV driver interfaces doc, most NDIS miniport
 * handlers are optional for WDI because the Microsoft WLAN component
 * provides them; only OidRequestHandler and DriverUnload are required.
 *
 * MiniportInitializeEx/HaltEx are among the ones the WLAN component
 * provides. Adapter creation happens in MiniportWdiAllocateAdapter
 * instead — see its signature in dot11wdi.h, which takes the NDIS
 * miniport handle, the PnP init parameters and the registration
 * attributes to fill in. Those are exactly MiniportInitializeEx's
 * inputs, because AllocateAdapter is what the component calls from
 * inside its own MiniportInitializeEx.
 */

#include "vwifi_drv.h"
#include <ntstrsafe.h>

/* Registered by NdisMRegisterWdiMiniportDriver; saved for unload. */
static NDIS_HANDLE g_DriverHandle = NULL;

#if DBG
/* ====================================================================
 * QEMU debug console (I/O port 0xE9)
 *
 * The sink behind VWIFI_E9. See the logging block in vwifi_drv.h for
 * why this exists: it is the only output path that survives the guest
 * dying, because the bytes leave the VM one at a time as they are
 * written and QEMU appends them to a host file immediately.
 *
 * Unbuffered on purpose. Buffering would be faster and would lose the
 * last few lines -- which are the only ones that matter when the
 * machine stops mid-line.
 *
 * The port IS shared state, which an earlier version of this comment
 * denied. Two CPUs writing a line each byte at a time interleave them,
 * and the result is unreadable exactly when it is most needed:
 *
 *   vwifiv:w iTfAiL:  rrexs: e14t5  pboyrttes  f0
 *
 * So the write is serialised. Not with a KSPIN_LOCK: this is called
 * from the ISR at DIRQL, above the DISPATCH_LEVEL ceiling those
 * require. Raising to HIGH_LEVEL first means nothing on this CPU can
 * interrupt us mid-line -- which is what makes the interlocked spin
 * below deadlock-free, since the holder can never be preempted by
 * something on its own CPU that also wants the lock.
 * ==================================================================== */
#define VWIFI_E9_PORT ((PUCHAR)0x00E9)

static volatile LONG g_E9Lock = 0;

VOID
VwifiE9Printf(PCSTR Format, ...)
{
    CHAR    buf[256];
    va_list ap;
    PCSTR   p;
    KIRQL   oldIrql;

    va_start(ap, Format);
    /* Return value ignored deliberately: on truncation this still
     * NUL-terminates, and a truncated line is worth infinitely more
     * than no line when it is the last thing before a freeze. */
    (VOID)RtlStringCchVPrintfA(buf, RTL_NUMBER_OF(buf), Format, ap);
    va_end(ap);

    KeRaiseIrql(HIGH_LEVEL, &oldIrql);
    while (InterlockedCompareExchange(&g_E9Lock, 1, 0) != 0) {
        YieldProcessor();
    }

    for (p = buf; *p != '\0'; p++) {
        WRITE_PORT_UCHAR(VWIFI_E9_PORT, (UCHAR)*p);
    }

    InterlockedExchange(&g_E9Lock, 0);
    KeLowerIrql(oldIrql);
}
#endif /* DBG */

/* ====================================================================
 * NDIS miniport handlers required beyond what WLAN component provides
 * ==================================================================== */

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
 * VwifiAdapterCreate — the WDI model's MiniportInitializeEx
 *
 * Called from MiniportWdiAllocateAdapter after PnP has claimed the
 * PCI device. Here we:
 *   1. Allocate the adapter context
 *   2. Parse assigned PCI resources
 *   3. Map BAR0 (MMIO) into kernel VA and check signature/ABI
 *   4. Fill in the caller's registration attributes
 *
 * Step 4 is a fill-in, not a call: WDI hands us the attributes block
 * to populate, where a plain NDIS miniport would call
 * NdisMSetMiniportAttributes itself. Which is precisely why the rings,
 * the interrupt and GET_CAPS are NOT here: none of the attributes are
 * in effect until this function returns, so any NDIS allocation made
 * here is made by an unregistered miniport and fails. They live in
 * VwifiHwStart, called from OpenAdapter.
 * ==================================================================== */

static VOID
VwifiFillRegistrationAttributes(
    _In_ PVWIFI_ADAPTER Adapter,
    _Inout_ PNDIS_MINIPORT_ADAPTER_REGISTRATION_ATTRIBUTES Reg)
{
    Reg->Header.Type = NDIS_OBJECT_TYPE_MINIPORT_ADAPTER_REGISTRATION_ATTRIBUTES;
    Reg->Header.Revision = NDIS_MINIPORT_ADAPTER_REGISTRATION_ATTRIBUTES_REVISION_2;
    Reg->Header.Size = NDIS_SIZEOF_MINIPORT_ADAPTER_REGISTRATION_ATTRIBUTES_REVISION_2;
    Reg->MiniportAdapterContext = (NDIS_HANDLE)Adapter;
    Reg->AttributeFlags = NDIS_MINIPORT_ATTRIBUTES_BUS_MASTER |
                          NDIS_MINIPORT_ATTRIBUTES_HARDWARE_DEVICE;
    Reg->CheckForHangTimeInSeconds = 4;
    Reg->InterfaceType = NdisInterfacePci;
}

_Use_decl_annotations_
NDIS_STATUS
VwifiAdapterCreate(
    NDIS_HANDLE MiniportAdapterHandle,
    PNDIS_MINIPORT_INIT_PARAMETERS MiniportInitParameters,
    PNDIS_MINIPORT_ADAPTER_REGISTRATION_ATTRIBUTES RegistrationAttributes)
{
    PVWIFI_ADAPTER adapter;
    NDIS_STATUS status;

    VWIFI_INFO("VwifiAdapterCreate entry");

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

    /* Pick up the WDI version the OS reported in AllocateAdapter, which
     * ran immediately before us. Every TLV_CONTEXT carries this. Getting
     * it wrong means the parser returns NDIS_STATUS_UNSUPPORTED_REVISION
     * (PeerVersion below WDI_VERSION_MIN_SUPPORTED) or, worse, silently
     * mis-encodes. */
    adapter->WdiPeerVersion = g_WdiPeerVersion;
    VWIFI_INFO("adapter WDI peer version 0x%08x", adapter->WdiPeerVersion);

    VwifiFillRegistrationAttributes(adapter, RegistrationAttributes);

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
 * VwifiAdapterDestroy — the WDI model's MiniportHaltEx, called from
 * MiniportWdiFreeAdapter.
 * ==================================================================== */
_Use_decl_annotations_
VOID
VwifiAdapterDestroy(NDIS_HANDLE MiniportAdapterContext)
{
    PVWIFI_ADAPTER adapter = (PVWIFI_ADAPTER)MiniportAdapterContext;

    if (adapter == NULL) {
        return;
    }

    VWIFI_INFO("VwifiAdapterDestroy");
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
    VWIFI_INFO("MiniportPause");
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
    VWIFI_INFO("MiniportRestart");
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
    PNET_BUFFER_LIST nbl;
    UNREFERENCED_PARAMETER(PortNumber);

    /* Walk the chain without unlinking it.
     *
     * This used to save NEXT_NBL and then null it, so that after the
     * loop NetBufferLists was a one-element list -- and the single
     * NdisMSendNetBufferListsComplete below returned exactly one NBL to
     * the stack. Every other NBL in the chain was never completed and
     * never freed.
     *
     * NDIS counts outstanding sends and MiniportPause does not finish
     * until the count reaches zero, so one multi-NBL send is enough to
     * make pausing the miniport wait forever: disabling the adapter
     * hangs, uninstalling it hangs, and shutdown hangs behind those. */
    for (nbl = NetBufferLists; nbl; nbl = NET_BUFFER_LIST_NEXT_NBL(nbl)) {
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

    /* The whole chain, in one call, exactly once. */
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

    /* The heartbeat.
     *
     * NDIS calls this handler on its own timer -- every
     * CheckForHangTimeInSeconds, set to 4 in the registration
     * attributes -- whether or not anything else is happening. That
     * makes it the one place in this driver that can answer a question
     * every trace so far has left open: when the guest stops responding
     * and nothing appears in the log, is the miniport not being called,
     * or is it being called and unable to say so?
     *
     * Silence has been read as the first of those, here and in what I
     * told you about it. That reading is only sound if the log itself
     * is known to be working, and after a hang it is not: a wedged
     * kernel, a stalled port-0xE9 write, or a lock held by a spinning
     * CPU all produce exactly the same empty file as a miniport that
     * was never called.
     *
     * A line every 8 seconds settles it. If the heartbeat keeps ticking
     * after a connect stalls, the driver and NDIS are both alive and
     * the block is above them. If it stops at the moment of the click,
     * the block is at or below NDIS and the earlier conclusion was
     * wrong.
     *
     * The task state rides along because it is free here and it says
     * whether anything is outstanding on our side. */
    if ((adapter->HangChecks++ % 2) == 0) {
        VWIFI_INFO("alive: beat %u, scan %u, conn 0x%x, assoc %u, port %u",
                   adapter->HangChecks,
                   VwifiScanTaskState(adapter),
                   VwifiConnectTaskState(adapter),
                   adapter->Associated ? 1u : 0u,
                   adapter->PortCreated ? 1u : 0u);
    }

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
    VWIFI_INFO("MiniportReset");
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

    /* The PnP event codes are the OS explaining itself -- a surprise
     * removal, a power state, a "your device is being reset". Worth
     * having in the trace when an adapter goes away unbidden. */
    VWIFI_INFO("MiniportDevicePnPEventNotify: event %u",
               NetDevicePnPEvent ? NetDevicePnPEvent->DevicePnPEvent : 0);
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
        /* Registered with NdisMRegisterWdiMiniportDriver, so it must be
         * torn down with the WDI counterpart, not the plain one. */
        NdisMDeregisterWdiMiniportDriver(g_DriverHandle);
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

    /* Which build this is.
     *
     * Not decoration. The install script spent several rounds staging
     * packages that were never bound, so every trace collected in that
     * time came from whatever .sys happened to be loaded already --
     * indistinguishable, from the log, from one where the change under
     * test was present and did nothing. A version in the first line
     * settles that in one glance.
     *
     * VWIFI_BUILD_VERSION arrives from the build as bare tokens
     * (1.0.MMdd.HHmm), which lex as a single preprocessing number, so
     * stringizing it gives the text back intact. */
    VWIFI_INFO("DriverEntry  build " VWIFI_STR(VWIFI_BUILD_VERSION));

    /* Where we are loaded, so an instruction pointer can be turned back
     * into a source line without a Windows debugger.
     *
     * This matters when the guest dies in a way Windows never gets to
     * report -- a triple fault or a hard lock leaves no bugcheck and no
     * dump, and the only surviving record of where the CPU was is
     * whatever QEMU logged on the host. That is an absolute RIP. With
     * the image base printed here, RIP - base is an RVA, and rip2sym.py
     * resolves it against the linker map. See "Debugging from a Linux
     * host" in the README. */
    VWIFI_INFO("image base %p size 0x%x  (RVA = RIP - base)",
               DriverObject->DriverStart, DriverObject->DriverSize);

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
    /* No InitializeHandlerEx/HaltEx: the WLAN component supplies those
     * and routes them to our AllocateAdapter/FreeAdapter handlers. */
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
    w.Header.Revision = NDIS_MINIPORT_DRIVER_WDI_CHARACTERISTICS_REVISION_1;
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
