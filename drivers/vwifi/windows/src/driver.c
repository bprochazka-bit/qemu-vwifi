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
    CHAR      buf[288];
    va_list   ap;
    PCSTR     p;
    KIRQL     oldIrql;
    size_t    used = 0;
    ULONGLONG ms;
    BOOLEAN   held = FALSE;

    /* Timestamped, because a trace without one cannot be lined up
     * against anything that happened outside the guest. Reading this
     * log has meant marking it by hand -- pressing Enter between a scan,
     * a netsh command and a connect click -- to know which lines came
     * from which. That is a real cost on every round trip and it is one
     * line of code to remove.
     *
     * Seconds since boot, to the millisecond. KeQueryInterruptTime is
     * callable at any IRQL, which matters because this runs from the
     * ISR, and it is monotonic across the sleeps and clock adjustments
     * that would make wall time useless here. */
    ms = KeQueryInterruptTime() / 10000ULL;

    (VOID)RtlStringCchPrintfA(buf, RTL_NUMBER_OF(buf), "vwifi: [%5llu.%03llu] ",
                              ms / 1000ULL, ms % 1000ULL);
    (VOID)RtlStringCchLengthA(buf, RTL_NUMBER_OF(buf), &used);

    va_start(ap, Format);
    /* Return value ignored deliberately: on truncation this still
     * NUL-terminates, and a truncated line is worth infinitely more
     * than no line when it is the last thing before a freeze. */
    (VOID)RtlStringCchVPrintfA(buf + used, RTL_NUMBER_OF(buf) - used,
                               Format, ap);
    va_end(ap);

    KeRaiseIrql(HIGH_LEVEL, &oldIrql);

    /* Bounded, not infinite.
     *
     * This is also called from the bugcheck callback below, and there
     * every other processor is already frozen exactly where it stood.
     * One of them frozen holding this lock would hang the crash path
     * forever -- and the crash path is the one place whose output
     * cannot be collected any other way. A garbled last line beats no
     * last line, so give up on the lock and write anyway. */
    {
        ULONG spins;
        for (spins = 0; spins < 1000000; spins++) {
            if (InterlockedCompareExchange(&g_E9Lock, 1, 0) == 0) {
                held = TRUE;
                break;
            }
            YieldProcessor();
        }
    }

    for (p = buf; *p != '\0'; p++) {
        WRITE_PORT_UCHAR(VWIFI_E9_PORT, (UCHAR)*p);
    }

    /* Only if we actually took it. Releasing a lock whose holder is
     * frozen would just let the next line collide with it too. */
    if (held) {
        InterlockedExchange(&g_E9Lock, 0);
    }
    KeLowerIrql(oldIrql);
}

/* ====================================================================
 * Bugcheck callback
 *
 * Every hang in this bring-up has looked identical from outside: the
 * desktop stops repainting, the 0xE9 trace stops mid-flow, and a gdb
 * attach finds both processors in ntoskrnl at IRQL 15. That is what a
 * bugcheck's processor freeze looks like -- and it is also what a
 * genuine hard lock looks like from the same distance, which is why
 * three separate "fixes" have been aimed at mechanisms that turned out
 * not to be the hang.
 *
 * This tells the two apart, and does it in the one log that survives
 * the machine dying. If these lines appear, Windows bugchecked: the
 * callback runs from KeBugCheck2 with every other processor already
 * frozen. If they never appear, it was not a bugcheck and the driver
 * state below is not where to look.
 *
 * The callback cannot see the bugcheck code -- KBUGCHECK_CALLBACK_ROUTINE
 * is handed only its own buffer -- so it reports what the OS will not:
 * where this driver's data path was standing when the machine stopped.
 * The code itself comes from the minidump, which is why writing one
 * should stay enabled.
 *
 * Runs at HIGH_LEVEL. Nothing here may allocate, take a lock that a
 * frozen processor could hold, or touch pageable memory: it reads
 * already-resident adapter fields and writes bytes to an I/O port.
 * ==================================================================== */

static KBUGCHECK_CALLBACK_RECORD g_BugCheckRecord;
static BOOLEAN                   g_BugCheckRegistered = FALSE;

/* The adapter, for the crash path only. Single-instance by design: this
 * device is not enumerated more than once, and a wrong guess here costs
 * nothing but a misleading log line on a machine that is already dead. */
static PVWIFI_ADAPTER volatile   g_BugCheckAdapter = NULL;

static KBUGCHECK_CALLBACK_ROUTINE VwifiBugCheckCallback;
static VOID VwifiBugCheckDumpStack(VOID);

static VOID
VwifiBugCheckCallback(_In_ PVOID Buffer, _In_ ULONG Length)
{
    PVWIFI_ADAPTER a = g_BugCheckAdapter;

    UNREFERENCED_PARAMETER(Buffer);
    UNREFERENCED_PARAMETER(Length);

    VwifiE9Printf("BUGCHECK: the machine stopped in a bugcheck, not a "
                  "hard lock -- read the code from the minidump\n");

    if (a == NULL) {
        VwifiE9Printf("BUGCHECK: no adapter registered\n");
        return;
    }

    VwifiE9Printf("BUGCHECK: adapter %p opmode %u assoc %u datapath %d\n",
                  a, a->OpMode, (ULONG)a->Associated, a->DataPathRunning);
    VwifiE9Printf("BUGCHECK: tx pump active %d again %d\n",
                  a->TxPumpActive, a->TxPumpAgain);
    VwifiE9Printf("BUGCHECK: rx paused %d outstanding %d\n",
                  a->RxPaused, a->RxOutstanding);
    VwifiE9Printf("BUGCHECK: rings tx@%u rx@%u\n",
                  a->TxRing.NextIndex, a->RxRing.NextIndex);

    VwifiBugCheckDumpStack();
}

/* Raw stack scan, not an unwind.
 *
 * The minidump from the first captured bugcheck did not contain the
 * stack the machine died on: the fault was in DPC context, and a triage
 * dump records the interrupted thread's stack instead -- which was a
 * completely unrelated NTFS write. So the crash path has to record its
 * own.
 *
 * A scan rather than RtlCaptureStackBackTrace because this runs at
 * HIGH_LEVEL with every other processor frozen. The unwinder consults
 * the loaded-module list and the exception directories and is not
 * something to invoke in that state; reading words off the stack needs
 * nothing but the stack. It costs false positives -- data that happens
 * to look like a code address -- which is a fair trade for a routine
 * that cannot itself become the reason the crash log is empty.
 *
 * IoGetStackLimits bounds it, and it is the reason this is safe: it
 * reports the limits of the stack actually in use, DPC stack included,
 * so the walk cannot step off the end into unmapped memory.
 *
 * The anchor line is what makes the rest readable: every address is
 * raw, and one known address inside this driver is enough to tell which
 * of them are ours.
 */
static VOID
VwifiBugCheckDumpStack(VOID)
{
    ULONG_PTR  lo = 0, hi = 0;
    ULONG_PTR  probe;
    ULONG_PTR *p;
    ULONG      shown = 0;

    IoGetStackLimits(&lo, &hi);

    VwifiE9Printf("BUGCHECK: stack %p..%p, this callback is at %p\n",
                  (PVOID)lo, (PVOID)hi, (PVOID)(ULONG_PTR)VwifiBugCheckCallback);

    if (lo == 0 || hi <= lo) {
        VwifiE9Printf("BUGCHECK: no usable stack limits -- not scanning\n");
        return;
    }

    /* From here up. &probe is inside the current frame, so everything
     * above it is the callers we want and everything below is ours. */
    probe = (ULONG_PTR)&probe;
    if (probe < lo || probe >= hi) probe = lo;

    for (p = (ULONG_PTR *)(probe & ~(ULONG_PTR)7);
         (ULONG_PTR)p + sizeof(*p) <= hi;
         p++) {
        ULONG_PTR v = *p;

        /* Kernel image range. Everything loaded in the captured dump --
         * ntoskrnl, ndis, wdiwifi, this driver -- was in 0xfffff8xx. */
        if (v < 0xfffff80000000000ULL || v > 0xfffff8ffffffffffULL) continue;

        VwifiE9Printf("BUGCHECK: stack+0x%04x %p\n",
                      (ULONG)((ULONG_PTR)p - lo), (PVOID)v);

        /* Enough to see who called whom, few enough that emitting it
         * one byte at a time through an I/O port still finishes. */
        if (++shown >= 64) {
            VwifiE9Printf("BUGCHECK: (stopping after %u addresses)\n", shown);
            break;
        }
    }
}

VOID
VwifiBugCheckRegister(_In_ PVWIFI_ADAPTER Adapter)
{
    g_BugCheckAdapter = Adapter;

    if (g_BugCheckRegistered) {
        return;
    }
    KeInitializeCallbackRecord(&g_BugCheckRecord);
    g_BugCheckRegistered = KeRegisterBugCheckCallback(
        &g_BugCheckRecord, VwifiBugCheckCallback, NULL, 0, (PUCHAR)"vwifi");
    VWIFI_INFO("bugcheck callback %s",
               g_BugCheckRegistered ? "registered" : "NOT registered");
}

VOID
VwifiBugCheckDeregister(VOID)
{
    g_BugCheckAdapter = NULL;
    if (g_BugCheckRegistered) {
        (VOID)KeDeregisterBugCheckCallback(&g_BugCheckRecord);
        g_BugCheckRegistered = FALSE;
    }
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
    /* Not zero: zero is a valid operation mode, and this field means
     * "nothing pending". */
    adapter->PendingOpMode = -1;
    InitializeListHead(&adapter->PendingReqList);
    KeInitializeSpinLock(&adapter->PendingReqLock);
    /* Guards the frames indicated to the WLAN component but not yet
     * collected through RxGetMpdusHandler. See wdi_data.c. */
    KeInitializeSpinLock(&adapter->RxQueueLock);

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

    /* Last, so the crash path only ever sees a fully built adapter. */
    VwifiBugCheckRegister(adapter);

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
    /* Before the memory goes back, or the crash path would read a freed
     * adapter -- which is a bugcheck of its own. */
    VwifiBugCheckDeregister();
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
            /* STA data path -- which in WDI mode is not supposed to
             * run at all. wdiwifi.sys owns the send path for station
             * data: NDIS hands data NBLs to IT, and the miniport takes
             * them back through NdisWdiTxDequeueIndication (see
             * wdi_data.c). This handler stays registered and stays
             * unused.
             *
             * It used to pass flags of 0, i.e. "this is 802.3", under a
             * comment saying the device would encapsulate it. That was
             * wrong on both counts and only harmless because nothing
             * reached it: anything arriving here came down through the
             * WDI stack and is therefore already an MPDU, per the
             * frame-format contract in vwifi_drv.h.
             *
             * Logged once because it firing at all would mean something
             * about the send path is not what this driver believes. */
            PNET_BUFFER nb = NET_BUFFER_LIST_FIRST_NB(nbl);
            NDIS_STATUS st = NDIS_STATUS_SUCCESS;

            VWIFI_ONCE("send: MiniportSendNetBufferLists took the STA "
                       "data path -- unexpected in WDI mode; treating "
                       "the frame as an 802.11 MPDU");

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
                    st = VwifiTxDataFrame(adapter, flat, len,
                                          VWIFI_TX_F_80211);
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
 * Heartbeat
 *
 * A line every VWIFI_HEARTBEAT_MS on a timer this driver owns, saying
 * the driver is still being scheduled and what it still has
 * outstanding.
 *
 * It exists because silence in the trace has been ambiguous every time
 * the guest wedged, and I read it as "the miniport was never called"
 * without being able to show that. That reading is only sound if the
 * log is known to be working. A stalled port-0xE9 write, a lock held
 * by a spinning CPU, or a block below NDIS all produce the same empty
 * file as a miniport nothing ever reached.
 *
 * The first attempt put this in MiniportCheckForHangEx, on the
 * reasoning that NDIS polls it on its own timer. It printed nothing at
 * all -- NDIS does not poll a WDI miniport that way. Hence a timer of
 * our own, which depends on nothing but KeSetTimer underneath.
 *
 * If the beat keeps ticking while a connect is stalled, the driver is
 * alive, nothing is outstanding on our side, and the block is above
 * us. If it stops, the block is at or below NDIS.
 * ==================================================================== */
#define VWIFI_HEARTBEAT_MS 8000

static VOID
VwifiHeartbeat(_In_ PVOID SystemSpecific1,
               _In_ PVOID FunctionContext,
               _In_ PVOID SystemSpecific2,
               _In_ PVOID SystemSpecific3)
{
    PVWIFI_ADAPTER adapter = (PVWIFI_ADAPTER)FunctionContext;

    UNREFERENCED_PARAMETER(SystemSpecific1);
    UNREFERENCED_PARAMETER(SystemSpecific2);
    UNREFERENCED_PARAMETER(SystemSpecific3);

    if (adapter == NULL) return;

    adapter->HangChecks++;

    /* opmode is here because it is the field most likely to be quietly
     * wrong: it is pushed to the device from a TAL callback that may
     * have to defer to a work item, so "did the adapter actually become
     * a station" is a question the trace should answer without anyone
     * having to reason about IRQL. pend is the deferred request, -1
     * when there is none. */
    VWIFI_INFO("alive: beat %u, scan %u, conn 0x%x, assoc %u, port %u, "
               "opmode %u, pend %d",
               adapter->HangChecks,
               VwifiScanTaskState(adapter),
               VwifiConnectTaskState(adapter),
               adapter->Associated ? 1u : 0u,
               adapter->PortCreated ? 1u : 0u,
               adapter->OpMode,
               adapter->PendingOpMode);

    /* The RX path on its own line, because the state above says nothing
     * about it: every trace so far has shown assoc 1 on a link that
     * carried no received traffic at all, and the difference between
     * "no frames arrive" and "frames arrive and the component will not
     * take them" was not visible without rebuilding. */
    VWIFI_INFO("alive: rx ind ok %d, not-ok %d, held %u, slots out %d, "
               "paused %d",
               adapter->RxIndOk, adapter->RxIndNotOk,
               adapter->RxQueueCount, adapter->RxOutstanding,
               adapter->RxPaused);

    /* The insurance on the RX pause latch.
     *
     * Holding frames until RxResumeHandler arrives is right when it
     * arrives, and it does -- but a version of that latch has already
     * stalled this driver's receive path permanently once, when it did
     * not. Nothing about the component guarantees it, so do not build
     * on the assumption: if frames sit held across two heartbeats, give
     * up on the resume, clear the latch and announce them anyway.
     *
     * Loud, because reaching this means the pause contract is not what
     * this driver believes and the next reader needs to know that
     * before anything else here is trusted. */
    if (adapter->RxPaused && adapter->RxQueueCount > 0) {
        if (++adapter->RxPausedBeats >= 2) {
            VWIFI_WARN("alive: %u frame(s) held across %u heartbeats waiting "
                       "for an RX resume that has not come -- clearing the "
                       "pause and indicating anyway",
                       adapter->RxQueueCount, adapter->RxPausedBeats);
            adapter->RxPausedBeats = 0;
            VwifiTalRxOnResume(adapter);
        }
    } else {
        adapter->RxPausedBeats = 0;
    }
}

NDIS_STATUS
VwifiHeartbeatStart(_Inout_ PVWIFI_ADAPTER Adapter)
{
    NDIS_TIMER_CHARACTERISTICS tc = { 0 };
    LARGE_INTEGER due;
    NDIS_STATUS status;

    tc.Header.Type     = NDIS_OBJECT_TYPE_TIMER_CHARACTERISTICS;
    tc.Header.Revision = NDIS_TIMER_CHARACTERISTICS_REVISION_1;
    tc.Header.Size     = NDIS_SIZEOF_TIMER_CHARACTERISTICS_REVISION_1;
    tc.AllocationTag   = VWIFI_POOL_TAG;
    tc.TimerFunction   = VwifiHeartbeat;
    tc.FunctionContext = Adapter;

    status = NdisAllocateTimerObject(Adapter->MiniportAdapterHandle, &tc,
                                     &Adapter->HeartbeatTimer);
    if (status != NDIS_STATUS_SUCCESS) {
        VWIFI_WARN("heartbeat timer unavailable (0x%x)", status);
        Adapter->HeartbeatTimer = NULL;
        return status;
    }

    /* Periodic: the third argument is the repeat interval, so this
     * re-arms itself and no callback has to. */
    due.QuadPart = -((LONGLONG)VWIFI_HEARTBEAT_MS * 10000LL);
    NdisSetTimerObject(Adapter->HeartbeatTimer, due, VWIFI_HEARTBEAT_MS, NULL);
    return NDIS_STATUS_SUCCESS;
}

VOID
VwifiHeartbeatStop(_Inout_ PVWIFI_ADAPTER Adapter)
{
    if (Adapter->HeartbeatTimer == NULL) return;

    (VOID)NdisCancelTimerObject(Adapter->HeartbeatTimer);
    NdisFreeTimerObject(Adapter->HeartbeatTimer);
    Adapter->HeartbeatTimer = NULL;
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

    /* No heartbeat here.
     *
     * This handler was where the heartbeat went first, on the reasoning
     * that NDIS calls it every CheckForHangTimeInSeconds regardless of
     * what else is happening. It printed nothing, ever -- not during a
     * hang, not during normal operation. NDIS does not poll a WDI
     * miniport this way; hang detection for WDI goes through the
     * WdiHangDiagnose handler and the component's own watchdogs
     * instead, and the registration attribute is ignored.
     *
     * The heartbeat is now on a timer this driver owns. See
     * VwifiHeartbeat below. */

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
