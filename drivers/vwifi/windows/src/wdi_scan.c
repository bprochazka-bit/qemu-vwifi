/*
 * vwifi — wdi_scan.c
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Phase 2: the scan task.
 *
 * This file is plain C and never touches the TLV library directly —
 * that library is C++ (TlvGeneratorParser.hpp + C++ new/delete), so it
 * lives behind tlv_shim.h's C boundary. See tlv_shim.h for why.
 *
 * Message shapes come from the WDK's WABIModel.xml, the XML that
 * generates the parser/generator, so they are authoritative:
 *
 *   WDI_TASK_SCAN / WDI_SCAN_PARAMETERS               (ToIhv)
 *     WDI_TLV_BSSID              BSSID        MacAddressContainer  req
 *     WDI_TLV_SSID               SSIDList     WiFiSSID      multi  req
 *     WDI_TLV_VENDOR_SPECIFIC_IE VendorIEs    ByteBlob             opt
 *     WDI_TLV_SCAN_MODE          ScanModeParameters   ScanMode
 *     WDI_TLV_SCAN_DWELL_TIME    DwellTime    ScanDwellTimeContainer
 *     WDI_TLV_BAND_CHANNEL       BandChannelList ...    multi      opt
 *
 *   WDI_TASK_SCAN / WDI_SCAN_RESULTS                  (FromIhv)
 *     "No TLV data needed, header is sufficient"
 *
 *   WDI_INDICATION_BSS_ENTRY_LIST                     (FromIhv)
 *     WDI_TLV_BSS_ENTRY  DeviceDescriptor  BSSEntryContainer multi opt
 *
 *   WDI_INDICATION_SCAN_COMPLETE                      (FromIhv)
 *     "No TLV data needed, header is sufficient"
 *
 * The load-bearing detail: WDI's BSS entry wants the RAW BEACON FRAME
 * as a byte blob. The OS parses the IEs itself. That is why the device
 * keeps whole frames in its BSS table and why this file stages frames
 * rather than IE tails.
 */

#include "vwifi_drv.h"

/* One staged BSS: the device's own record plus the frames it was seen
 * in. Shared by the pending batch and the cache below, which hold
 * exactly the same thing for different lengths of time.
 *
 * TWO frames, not one, and they are not interchangeable.
 *
 * WDI_TLV_BSS_ENTRY carries WDI_TLV_BEACON_FRAME and
 * WDI_TLV_PROBE_RESPONSE_FRAME as separate optional sub-TLVs, and the
 * host treats them as separate facts about the BSS -- a probe response
 * is a reply to us, a beacon is what the AP broadcasts to everyone.
 * This stage used to hold a single frame with a flag saying which kind
 * it was, so whichever arrived second overwrote the first. In practice
 * that was always the probe response (the probe request goes out at the
 * start of the dwell and the reply is immediate; the beacon lands up to
 * a beacon interval later), so BEACON_FRAME was never once populated in
 * any scan this driver has ever reported.
 *
 * Keeping both means an entry can carry both, which is what real
 * hardware reports and what the host expects to be able to ask for. */
typedef struct _VWIFI_BSS_STAGE
{
    struct vwifi_bss_entry Entry;

    /* Whole frames, MAC header included -- the shim trims the header
     * off when it builds the blob, because WDI's beacon and probe
     * blobs are bodies. Zero length means "not seen in that form". */
    USHORT                 BeaconLen;
    USHORT                 ProbeLen;
    UCHAR                  Beacon[VWIFI_SCAN_MAX_FRAME];
    UCHAR                  Probe[VWIFI_SCAN_MAX_FRAME];

    /* System time when the frame arrived, for the entry's age info.
     * Recorded here rather than at indication time because the cache
     * below replays entries long after the scan that found them, and
     * an entry that keeps reporting "discovered just now" would never
     * age out of the OS's list at all. */
    ULONGLONG              SeenSystemTime;
} VWIFI_BSS_STAGE, *PVWIFI_BSS_STAGE;

/* How many scan tasks one device sweep may be answering at once. The
 * host asks for a second scan milliseconds into the first as part of a
 * connect; four is room to spare for that. */
#define VWIFI_SCAN_MAX_PENDING_TASKS 4

typedef struct _VWIFI_SCAN_TASK
{
    /* 1 while a scan awaits its SCAN_COMPLETE.
     *
     * Interlocked and claimed rather than tested: the device's response
     * drain and the watchdog below are independent DPCs and can be on
     * different processors at once, and whichever swaps this back to 0
     * owns the completion. */
    volatile LONG Active;

    /* 1 when this "sweep" has no device sweep behind it: the scan was
     * refused because a connect is in flight, and its completion is
     * being held until the connect finishes.
     *
     * The task is left Active anyway, so everything downstream -- the
     * merge path, the watchdog, VwifiIndicateScanComplete -- works
     * unchanged and a second refused scan merges into this one instead
     * of needing a parallel queue of its own. This flag exists only so
     * the two paths that would otherwise complete it with a FAILURE
     * know that "no results" is the expected answer here rather than a
     * sweep that went missing. */
    volatile LONG DeferredForConnect;

    /* Fires if SCAN_COMPLETE never arrives. See VwifiScanWatchdog --
     * the lazy check this replaces could not fire in the one case it
     * existed for. */
    NDIS_HANDLE Watchdog;

    /* Both port namespaces, from the request that STARTED the sweep.
     * WdiPortId scopes the WDI message header; PortId is the NDIS port
     * the request arrived on. See VwifiGetWdiPortId.
     *
     * Used for BSS_ENTRY_LIST, which is an event about the sweep rather
     * than a completion for any one requester. Task completions do NOT
     * use these -- see Pending[] below. */
    WDI_PORT_ID     WdiPortId;
    ULONG     PortId;

    /* Every scan task the device's current sweep is going to answer.
     *
     * Usually one. The host can ask for a second scan a few
     * milliseconds into the first -- it does exactly that as part of a
     * connect -- and one sweep satisfies both, so all of them are held
     * here and each gets a SCAN_COMPLETE when the sweep really
     * finishes. Completing one of them early, before the device has
     * reported anything, tells the host the scan found nothing.
     *
     * All three fields per entry, not just the transaction id. A task
     * completion is matched by the port driver on BOTH the transaction
     * id and the port id -- wdiwifi's Task::OnDeviceIndicationArrived
     * compares the indication against DeviceCommand::get_CommandToken
     * and DeviceCommand::get_PortId and returns silently if either
     * differs, leaving the task with no output at all. Sending every
     * merged completion with the first requester's port id was
     * therefore a real bug, invisible only because every scan so far
     * has arrived on port 0.
     *
     * Request is the OID itself, still outstanding. VwifiHandleTaskScan
     * returns NDIS_STATUS_PENDING, so NDIS keeps each request alive and
     * owned by us until VwifiWdiTaskComplete is called on it -- which
     * is what puts a zero status where CScanJob::FinishJob reads the
     * job's outcome. Every path that abandons a sweep has to complete
     * these; a scan task that is dropped instead leaves the WLAN
     * component waiting on a request that never returns. */
    struct {
        UINT32            TransactionId;
        WDI_PORT_ID       WdiPortId;
        ULONG             PortId;
        PNDIS_OID_REQUEST Request;
    }         Requesters[VWIFI_SCAN_MAX_PENDING_TASKS];
    ULONG     TransactionCount;

    ULONG     PendingCount;

    /* BSS entries awaiting indication. We stage the device's own
     * representation and convert to TLV only at indication time: the
     * DPC path stays cheap and TLV generation gets batched. */
    PVWIFI_BSS_STAGE Pending;
    ULONG     PendingCapacity;

    /* Everything seen recently, kept past the scan that found it.
     *
     * OID_WDI_GET_BSS_ENTRY_LIST is "get cached BSS entry list from
     * adapter", and a WDI miniport is expected to have one. Refusing it
     * does not merely return nothing: asking for the list and being
     * told the adapter has none is what made the network vanish from
     * `netsh wlan show networks` and from the UI, and stopped the
     * component scanning.
     *
     * Updated whether or not a scan is running, though the device only
     * emits BSS_FOUND during one -- see the note in VwifiScanOnBssFound.
     *
     * Keyed by BSSID, newest write wins, oldest evicted when full. No
     * ageing here -- each entry carries the time it was last seen and
     * the OS ages it out on that. */
    PVWIFI_BSS_STAGE Cache;
    ULONG     CacheCount;
    ULONG     CacheCapacity;

    ULONGLONG LastIndicationTimeMs;
    /* When this scan was started, for the stale-task backstop below. */
    ULONGLONG StartedTimeMs;
} VWIFI_SCAN_TASK, *PVWIFI_SCAN_TASK;

/* From OID_WDI_TASK_SCAN: "the port should throttle indications and
 * send updates to the host only when it has discovered 3 or more, or
 * when it has discovered less than 3 entries but has not reported them
 * to the host for more than 500 milliseconds." */
#define VWIFI_SCAN_BATCH_THRESHOLD   3
#define VWIFI_SCAN_FLUSH_INTERVAL_MS 500

/* How long a scan may stay active before the next scan request treats
 * it as lost. A full sweep is 16 channels x 100 ms dwell; ten seconds
 * is far enough past that to mean the completion is never coming. */
#define VWIFI_SCAN_STALE_MS          10000

/* ============================================================
 * Indicate the accumulated BSS entries
 * ============================================================ */
/* One line per BSS entry we are about to hand the host.
 *
 * Per entry rather than an aggregate count, because the questions this
 * answers are per entry: did the beacon go out, and -- since the host
 * now asks us for the cached list -- which list did this entry come
 * from. `Which` is "live" for a running scan's results and "cached" for
 * a reply to OID_WDI_GET_BSS_ENTRY_LIST. */
static VOID
VwifiTraceBssItem(_In_z_ const CHAR *Which,
                  _In_ ULONG Index,
                  _In_ const VWIFI_BSS_STAGE *Stage)
{
    VWIFI_INFO("BSS entry %u (%s): %02x:%02x:%02x:%02x:%02x:%02x "
               "beacon=%u probe=%u",
               Index, Which,
               Stage->Entry.bssid[0], Stage->Entry.bssid[1],
               Stage->Entry.bssid[2], Stage->Entry.bssid[3],
               Stage->Entry.bssid[4], Stage->Entry.bssid[5],
               Stage->BeaconLen, Stage->ProbeLen);
}

static VOID
VwifiIndicateBssEntryList(_Inout_ PVWIFI_ADAPTER Adapter)
{
    PVWIFI_SCAN_TASK task = Adapter->ScanTask;
    VWIFI_TLV_BSS_ITEM items[VWIFI_SCAN_PENDING_MAX];
    PVOID  generated = NULL;
    ULONG  generatedLen = 0;
    NDIS_STATUS status;
    ULONG n;

    if (!task || task->PendingCount == 0) return;
    n = task->PendingCount;
    if (n > VWIFI_SCAN_PENDING_MAX) n = VWIFI_SCAN_PENDING_MAX;

    for (ULONG i = 0; i < n; i++) {
        items[i].Entry         = &task->Pending[i].Entry;
        items[i].Beacon        = task->Pending[i].Beacon;
        items[i].BeaconLen     = task->Pending[i].BeaconLen;
        items[i].Probe         = task->Pending[i].Probe;
        items[i].ProbeLen      = task->Pending[i].ProbeLen;
        items[i].HostTimeStamp = task->Pending[i].SeenSystemTime;
        items[i].Cached        = FALSE;   /* live: a scan is running */

        VwifiTraceBssItem("live", i, &task->Pending[i]);
    }

    status = VwifiTlvGenerateBssEntryList(Adapter->WdiPeerVersion,
                                          items, n,
                                          &generated, &generatedLen);
    if (status != NDIS_STATUS_SUCCESS) {
        /* NDIS_STATUS_BUFFER_OVERFLOW here means the entries overflow
         * the TLV header's 2-byte length field — indicate fewer. */
        VWIFI_ERR("BSS entry list generation failed 0x%x (%u entries)",
                  status, n);
        goto done;
    }

    /* BSS_ENTRY_LIST is an event, not the scan task's completion, so it
     * carries the unsolicited transaction id even mid-scan. */
    VwifiSendWdiIndication(Adapter, task->WdiPortId, task->PortId,
                           NDIS_STATUS_WDI_INDICATION_BSS_ENTRY_LIST,
                           NDIS_STATUS_SUCCESS,
                           WDI_TRANSACTION_ID_UNSOLICIT,
                           generated, generatedLen);

    VWIFI_INFO("indicated BSS_ENTRY_LIST: %u entries, %u bytes",
               n, generatedLen);

done:
    /* MANDATORY: the docs warn that skipping the library's cleanup
     * routines "will lead to hard-to-diagnose memory leaks". */
    VwifiTlvFreeGenerated(generated);
    task->PendingCount         = 0;
    task->LastIndicationTimeMs = VwifiGetTickCountMs();
}

/* ============================================================
 * Scan complete — WABIModel: "No TLV data needed, header is
 * sufficient". Nothing to generate; the shim emits a bare header.
 * ============================================================ */
/* Take ownership of one outstanding scan OID, or get NULL if someone
 * else already has.
 *
 * Interlocked rather than a plain read-and-clear because completing a
 * request twice is a use after free and never completing it hangs the
 * WLAN component: those are the two outcomes worth spending an atomic
 * on. The completion path, the teardown path and the late-merge
 * recovery below all claim through here, so exactly one of them wins
 * each slot. */
static PNDIS_OID_REQUEST
VwifiScanClaimRequest(_Inout_ PVWIFI_SCAN_TASK Task, _In_ ULONG Index)
{
    return (PNDIS_OID_REQUEST)InterlockedExchangePointer(
        (PVOID volatile *)&Task->Requesters[Index].Request, NULL);
}

static VOID
VwifiIndicateScanComplete(_Inout_ PVWIFI_ADAPTER Adapter, _In_ NDIS_STATUS Status)
{
    PVWIFI_SCAN_TASK task = Adapter->ScanTask;

    if (!task) return;

    /* Claim the completion. The device's SCAN_COMPLETE and the watchdog
     * both arrive here and only one of them may indicate. */
    if (InterlockedCompareExchange(&task->Active, 0, 1) != 1) return;

    /* Cleared by whoever claims the task, so the next sweep starts
     * without inheriting this one's deferral. */
    InterlockedExchange(&task->DeferredForConnect, 0);

    /* Flush anything still staged before completing the task. */
    VwifiIndicateBssEntryList(Adapter);

    /* One SCAN_COMPLETE per task the sweep was answering, each echoing
     * its own transaction id. SCAN_COMPLETE carries no TLVs at all --
     * the outcome rides in the message header's Status field.
     *
     * The OID is completed FIRST and the M3 sent second, per requester.
     * CScanJob::FinishJob's status argument is not our M3's -- that has
     * been 0 in every build, including the ones where FinishJob was
     * measured receiving 0x40230001 -- so it comes from the request, and
     * it has to already be 0 when the M3 drives the job to finish. The
     * BSS entries were flushed just above, so by this point the results
     * are in: completing here is nothing like the NDIS_STATUS_SUCCESS
     * build, which completed before the sweep had found anything. */
    {
        ULONG n = task->TransactionCount;

        if (n > VWIFI_SCAN_MAX_PENDING_TASKS) n = VWIFI_SCAN_MAX_PENDING_TASKS;
        for (ULONG i = 0; i < n; i++) {
            /* Claimed before the call. NdisMOidRequestComplete can run
             * the next scan request on another processor before it
             * returns, and that request writes this same slot. */
            PNDIS_OID_REQUEST req = VwifiScanClaimRequest(task, i);

            VwifiWdiTaskComplete(Adapter, req, Status);

            /* Each requester's OWN port ids, not the sweep's. */
            VwifiSendWdiIndication(Adapter,
                                   task->Requesters[i].WdiPortId,
                                   task->Requesters[i].PortId,
                                   NDIS_STATUS_WDI_INDICATION_SCAN_COMPLETE,
                                   Status,
                                   task->Requesters[i].TransactionId,
                                   NULL, 0);
        }
        VWIFI_INFO("indicated SCAN_COMPLETE (0x%x) for %u task(s)",
                   Status, n);
        task->TransactionCount = 0;
    }
}

/* Complete every scan OID still outstanding, without indicating
 * anything.
 *
 * For teardown only. VwifiIndicateScanComplete is the normal path and
 * completes each request itself; this exists so that halting with a
 * scan in flight does not leave NDIS holding requests forever, which is
 * the failure mode that made adapter removal hang before the disconnect
 * task learned to send its completion. */
static VOID
VwifiScanAbandonRequests(_Inout_ PVWIFI_ADAPTER Adapter,
                         _Inout_ PVWIFI_SCAN_TASK Task,
                         _In_ NDIS_STATUS Status)
{
    ULONG n = Task->TransactionCount;

    if (n > VWIFI_SCAN_MAX_PENDING_TASKS) n = VWIFI_SCAN_MAX_PENDING_TASKS;
    for (ULONG i = 0; i < n; i++) {
        PNDIS_OID_REQUEST req = VwifiScanClaimRequest(Task, i);

        if (req) {
            VWIFI_WARN("abandoning outstanding scan OID txn %u with 0x%08x",
                       Task->Requesters[i].TransactionId, Status);
            VwifiWdiTaskComplete(Adapter, req, Status);
        }
    }
    Task->TransactionCount = 0;
}

/* ============================================================
 * Scan watchdog
 *
 * A scan whose SCAN_COMPLETE never arrives -- the device drops the
 * event when the response ring has no free slot -- used to be caught
 * lazily, by the next scan request noticing the task had been active
 * too long. That check cannot fire in the one situation it exists for.
 * WDI runs one scan at a time per port, so an outstanding scan is
 * exactly why the OS does not ask for the next one, and the thing that
 * was supposed to notice only ran when it did. Scanning stopped, the
 * OS's network list emptied itself as the entries aged out, and a
 * connect then had no BSS to be built from.
 *
 * An armed timer notices without needing anyone to ask.
 * ============================================================ */
static VOID
VwifiScanWatchdog(_In_ PVOID SystemSpecific1,
                  _In_ PVOID FunctionContext,
                  _In_ PVOID SystemSpecific2,
                  _In_ PVOID SystemSpecific3)
{
    PVWIFI_ADAPTER adapter = (PVWIFI_ADAPTER)FunctionContext;
    PVWIFI_SCAN_TASK task = adapter ? adapter->ScanTask : NULL;

    UNREFERENCED_PARAMETER(SystemSpecific1);
    UNREFERENCED_PARAMETER(SystemSpecific2);
    UNREFERENCED_PARAMETER(SystemSpecific3);

    if (!task || !task->Active) return;

    /* A scan deferred behind a connect has no sweep to have gone
     * missing, so its backstop is not a failure. Failing it here would
     * re-create the exact bug the deferral exists to avoid: a failed
     * scan task aborts the connect job's post-connect sequence and the
     * peer is never configured. The connect must have died without
     * completing for this to fire at all -- rare, and still no reason
     * to report a scan as broken. */
    if (task->DeferredForConnect) {
        VWIFI_WARN("scan deferred behind a connect never drained after "
                   "%u ms -- the connect did not complete; answering it "
                   "as a no-op scan anyway", VWIFI_SCAN_STALE_MS);
        VwifiIndicateScanComplete(adapter, NDIS_STATUS_SUCCESS);
        return;
    }

    VWIFI_WARN("no SCAN_COMPLETE after %u ms -- completing the scan so the "
               "next one can be asked for", VWIFI_SCAN_STALE_MS);

    /* Failure, not success: the entries found so far have already gone
     * up as BSS_ENTRY_LIST indications, and what is unknown is whether
     * the sweep finished. Claiming it did would be inventing a result.
     * The flush inside this call still sends anything staged. */
    VwifiIndicateScanComplete(adapter, NDIS_STATUS_FAILURE);
}

/* ============================================================
 * Device events — DPC context
 * ============================================================ */

/* The same 2.4/5 GHz mapping tlv_shim.cpp uses for
 * WDI_TLV_BSS_ENTRY_CHANNEL_INFO. Duplicated rather than shared because
 * that one lives on the C++ side of the shim; the point here is to
 * compare what we tell the host against what the AP says in its own
 * DS Parameter Set, so it has to be the same arithmetic. */
static ULONG
VwifiFreqToChannelNumber(_In_ USHORT FreqMhz)
{
    if (FreqMhz == 2484)                    return 14;
    if (FreqMhz >= 2412 && FreqMhz <= 2472) return (ULONG)((FreqMhz - 2407) / 5);
    if (FreqMhz >= 5160 && FreqMhz <= 5885) return (ULONG)((FreqMhz - 5000) / 5);
    return 0;
}

/* Decode the management-frame body exactly as the OS will.
 *
 * WDI hands the raw beacon or probe-response body to the host and the
 * host parses it -- BSS type, SSID, rates, channel, security all come
 * from these bytes and from nothing this driver says. That makes the
 * body the one thing worth reading byte for byte when the host lists a
 * network happily and then declines to connect to it: whatever it
 * objects to is in here.
 *
 * Body layout after the 24-byte MAC header:
 *   0..7   timestamp
 *   8..9   beacon interval (TU)
 *   10..11 capability information  -- bit 0 ESS, bit 1 IBSS, bit 4 Privacy
 *   12..   information elements, each id(1) len(1) data(len)
 *
 * Logged as "cap=0x.... bi=... ies: id(len) id(len) ..." so a trace can
 * be decoded without the frame itself. */
static VOID
VwifiScanTraceFrameBody(_In_reads_bytes_(BodyLen) const UCHAR *Body,
                        _In_ ULONG BodyLen,
                        _In_ ULONG ReportedChannel)
{
    CHAR  ies[160];
    ULONG w = 0;
    ULONG off;
    USHORT cap, bi;
    ULONG dsChannel = 0;      /* element 3, the channel the AP claims */

    if (BodyLen < 12) {
        VWIFI_WARN("BSS body only %u bytes -- no fixed fields", BodyLen);
        return;
    }

    bi  = (USHORT)(Body[8]  | (Body[9]  << 8));
    cap = (USHORT)(Body[10] | (Body[11] << 8));

    ies[0] = '\0';
    for (off = 12; off + 2 <= BodyLen; ) {
        UCHAR id  = Body[off];
        UCHAR len = Body[off + 1];

        /* Element 3 is the DS Parameter Set: one byte, the channel the
         * AP says it is on. The host has two channel numbers for this
         * BSS -- this one and the one in WDI_TLV_BSS_ENTRY_CHANNEL_INFO
         * -- and they had better agree. */
        if (id == 3 && len == 1) dsChannel = Body[off + 2];

        if (off + 2 + len > BodyLen) {
            (VOID)RtlStringCchPrintfA(ies + w, RTL_NUMBER_OF(ies) - w,
                                      " !truncated@%u", off);
            break;
        }
        if (w + 12 >= RTL_NUMBER_OF(ies)) {
            (VOID)RtlStringCchPrintfA(ies + w, RTL_NUMBER_OF(ies) - w, " ...");
            break;
        }
        (VOID)RtlStringCchPrintfA(ies + w, RTL_NUMBER_OF(ies) - w,
                                  " %u(%u)", id, len);
        {
            size_t used = 0;
            (VOID)RtlStringCchLengthA(ies, RTL_NUMBER_OF(ies), &used);
            w = (ULONG)used;
        }
        off += 2u + len;
    }

    /* ESS and IBSS are what the host turns into "infrastructure" or
     * "ad hoc", and it will not connect an infrastructure request to a
     * BSS that does not claim ESS. */
    VWIFI_INFO("BSS body: %u bytes cap=0x%04x (ESS=%u IBSS=%u Privacy=%u) "
               "bi=%u ds-chan=%u (we report %u)%s ies:%s",
               BodyLen, cap,
               (cap & 0x0001) ? 1u : 0u,
               (cap & 0x0002) ? 1u : 0u,
               (cap & 0x0010) ? 1u : 0u,
               bi, dsChannel, ReportedChannel,
               (dsChannel != 0 && dsChannel != ReportedChannel)
                   ? " MISMATCH" : "",
               ies);
}

/* Fold one received frame into a stage slot.
 *
 * The frame goes in the slot for its own kind, so a probe response can
 * never displace a beacon or the other way round, and a slot that has
 * seen both keeps both. Everything else in the entry is overwritten
 * from the newest report: signal and channel are receive-side facts and
 * the newest one is the true one.
 *
 * Fresh slots must have both lengths cleared before the first call --
 * see the two callers below, which do it as part of claiming the
 * slot. */
static VOID
VwifiBssStageStore(_Inout_ PVWIFI_BSS_STAGE Stage,
                   _In_ const struct vwifi_bss_entry *Bss,
                   _In_reads_bytes_(FrameLen) const UCHAR *Frame,
                   _In_ USHORT FrameLen,
                   _In_ BOOLEAN IsBeacon,
                   _In_ ULONGLONG SeenSystemTime)
{
    Stage->Entry          = *Bss;
    Stage->SeenSystemTime = SeenSystemTime;

    if (IsBeacon) {
        RtlCopyMemory(Stage->Beacon, Frame, FrameLen);
        Stage->BeaconLen = FrameLen;
    } else {
        RtlCopyMemory(Stage->Probe, Frame, FrameLen);
        Stage->ProbeLen = FrameLen;
    }

    /* The device ABI overloads ie_len as the length of the frame that
     * came with THIS event, and capability_info bit 16 as which kind it
     * was. Neither describes the merged slot -- which may now hold two
     * frames of two different lengths -- so both are stripped rather
     * than left to be read as if they still meant something. The frame
     * lengths above are the record; capability_info keeps its low 16
     * bits, which are the real 802.11 capability field. */
    Stage->Entry.ie_len = 0;
    Stage->Entry.capability_info &= ~VWIFI_BSS_F_BEACON;
}

VOID
VwifiScanOnBssFound(_Inout_ PVWIFI_ADAPTER Adapter,
                    _In_reads_bytes_(PayloadLen) const VOID *Payload,
                    _In_ ULONG PayloadLen)
{
    const struct vwifi_bss_entry *bss = Payload;
    PVWIFI_SCAN_TASK task = Adapter->ScanTask;
    const UCHAR *frame;
    USHORT frameLen;
    BOOLEAN isBeacon;
    LARGE_INTEGER now;

    if (!task) return;

    if (PayloadLen < sizeof(*bss)) {
        VWIFI_WARN("BSS_FOUND payload too short: %u", PayloadLen);
        return;
    }

    /* In our device ABI, ie_len carries the WHOLE frame length. */
    frameLen = bss->ie_len;
    if (PayloadLen < sizeof(*bss) + frameLen) {
        VWIFI_WARN("BSS_FOUND frame len %u exceeds payload %u",
                   frameLen, PayloadLen);
        return;
    }
    if (frameLen > VWIFI_SCAN_MAX_FRAME) {
        VWIFI_WARN("BSS_FOUND frame %u > %u, truncating",
                   frameLen, VWIFI_SCAN_MAX_FRAME);
        frameLen = VWIFI_SCAN_MAX_FRAME;
    }
    frame = (const UCHAR *)Payload + sizeof(*bss);

    /* The device sets this bit in capability_info on the way out; it is
     * the only thing distinguishing the two events it now emits for the
     * same BSS. */
    isBeacon = (bss->capability_info & VWIFI_BSS_F_BEACON) ? TRUE : FALSE;

    /* Stamped once, here, and carried by both copies below. This is the
     * value WDI_TLV_BSS_ENTRY_AGE_INFO is specified to want -- system
     * time, from NdisGetCurrentSystemTime -- and it is what decides how
     * long the OS keeps the network in its list. Callable at
     * DISPATCH_LEVEL, which is where this DPC runs. */
    NdisGetCurrentSystemTime(&now);

    /* The cache, first and unconditionally.
     *
     * This used to return early unless a scan was running. Updating
     * regardless costs nothing and means a report that arrives outside
     * a scan is not thrown away.
     *
     * It does not currently make the cache any fresher, and it is worth
     * saying so rather than leaving the impression it does. The device
     * observes every beacon and probe response continuously -- see
     * bss_observe() in vwifi_device.c -- but it emits VWIFI_EV_BSS_FOUND
     * only while a scan is running, once per BSS per scan. So between
     * scans nothing arrives here to cache, and the cached entries age
     * exactly as far as the gap between scans. Closing that would mean
     * either a device event stream outside scans or an ABI operation to
     * read the device's table on demand; neither exists yet.
     *
     * What keeps the list alive today is scans continuing to happen,
     * which is what the overlap handling and the watchdog in this file
     * are for.
     *
     * Replace by BSSID if we already know this one; otherwise append,
     * evicting the oldest when full. */
    {
        ULONG slot = task->CacheCount;

        for (ULONG i = 0; i < task->CacheCount; i++) {
            if (RtlCompareMemory(task->Cache[i].Entry.bssid,
                                 bss->bssid, 6) == 6) {
                slot = i;
                break;
            }
        }
        if (slot == task->CacheCount) {
            if (task->CacheCount < task->CacheCapacity) {
                task->CacheCount++;
            } else {
                /* Full: drop the oldest and shuffle down. The cache is
                 * small and this runs once per newly seen BSS, so the
                 * copy is cheaper than threading a ring index through
                 * every reader. */
                RtlMoveMemory(&task->Cache[0], &task->Cache[1],
                              (task->CacheCapacity - 1) *
                                  sizeof(task->Cache[0]));
                slot = task->CacheCapacity - 1;
            }
            /* Claiming a slot: clear both frames, so a new BSS cannot
             * inherit the beacon of whatever used to live here. A slot
             * we matched by BSSID keeps what it has -- that is the
             * merge. */
            task->Cache[slot].BeaconLen = 0;
            task->Cache[slot].ProbeLen  = 0;
        }

        VwifiBssStageStore(&task->Cache[slot], bss, frame, frameLen,
                           isBeacon, (ULONGLONG)now.QuadPart);
    }

    /* The device parses the SSID out of the beacon for its own BSS
     * table, so it is here for free -- and it is the reference for what
     * the OS ought to end up displaying. WDI has no SSID field: the OS
     * reads it out of the frame body we hand over, so when this line
     * shows a name and the UI still says "Hidden Network", the frame
     * blob is what to look at, not the scan. */
    VWIFI_INFO("BSS %02x:%02x:%02x:%02x:%02x:%02x ssid='%.*s' "
               "freq=%u rssi=%d frame=%u (%s)",
               bss->bssid[0], bss->bssid[1], bss->bssid[2],
               bss->bssid[3], bss->bssid[4], bss->bssid[5],
               bss->ssid_len > 32 ? 32 : bss->ssid_len, bss->ssid,
               bss->channel_freq, bss->rssi, frameLen,
               isBeacon ? "beacon" : "probe-resp");

    /* And the body the host actually parses, which is the only thing
     * that decides whether it will treat this BSS as connectable. */
    if (frameLen > VWIFI_80211_MGMT_HDR_LEN) {
        VwifiScanTraceFrameBody(frame + VWIFI_80211_MGMT_HDR_LEN,
                                (ULONG)(frameLen - VWIFI_80211_MGMT_HDR_LEN),
                                VwifiFreqToChannelNumber(bss->channel_freq));
    }

    /* Everything past here is the running scan's business. Outside a
     * scan the cache above is the whole job: there is no scan for these
     * entries to be results of, and BSS_ENTRY_LIST is how a scan
     * reports, so an unsolicited stream of them between scans would be
     * reporting a scan that is not happening. The OS gets them when it
     * asks, through OID_WDI_GET_BSS_ENTRY_LIST. */
    if (!task->Active) return;

    /* Merge into the batch by BSSID, the same way the cache does.
     *
     * This used to append unconditionally, which was harmless while the
     * device emitted one event per BSS per scan. It emits two now --
     * beacon and probe response -- and appending would put the same
     * BSSID in the batch twice, as two entries each carrying half of
     * what we know.
     *
     * Whether the host would merge those two entries or let the second
     * supersede the first is not documented, and guessing is what this
     * whole line of work has been paying for. What IS documented is
     * that one WDI_TLV_BSS_ENTRY has room for both blobs -- so the
     * entry that can carry both is the one to send. */
    {
        ULONG slot = task->PendingCount;

        for (ULONG i = 0; i < task->PendingCount; i++) {
            if (RtlCompareMemory(task->Pending[i].Entry.bssid,
                                 bss->bssid, 6) == 6) {
                slot = i;
                break;
            }
        }

        if (slot == task->PendingCount) {
            /* Flush first if the accumulator is full -- which resets
             * PendingCount, so the new entry lands at 0. */
            if (task->PendingCount >= task->PendingCapacity) {
                VwifiIndicateBssEntryList(Adapter);
                slot = task->PendingCount;      /* now 0 */
            }
            task->Pending[slot].BeaconLen = 0;
            task->Pending[slot].ProbeLen  = 0;
            task->PendingCount++;
        }

        VwifiBssStageStore(&task->Pending[slot], bss, frame, frameLen,
                           isBeacon, (ULONGLONG)now.QuadPart);
    }

    /* No throttle check here. See VwifiScanFlushBatch: the decision to
     * indicate belongs at the end of the ring drain, not in the middle
     * of one. */
}

/* Apply the documented throttle, once, after a drain has delivered
 * everything the device had to say.
 *
 * "The port should throttle indications and send updates to the host
 * only when it has discovered 3 or more, or when it has discovered less
 * than 3 entries but has not reported them to the host for more than
 * 500 milliseconds."
 *
 * This used to run at the bottom of VwifiScanOnBssFound, per event,
 * which was correct while one BSS meant one event. It no longer does:
 * the device emits the beacon and the probe response for a BSS as two
 * consecutive events, and the 500 ms rule fires on essentially every
 * first event of a scan -- so the beacon would be indicated on its own
 * and the probe response would land in the *next* batch as a second
 * entry for the same BSSID. That is the split this change exists to
 * close, reappearing one layer up.
 *
 * Deferring to the end of the drain closes it, because the device posts
 * a BSS's two events back to back and a drain that sees the first
 * almost always sees the second. Almost: if the device posts them
 * either side of an interrupt the two frames still end up in separate
 * entries. That is a benign outcome -- two valid entries for one BSSID,
 * which is also what real hardware produces when a beacon arrives after
 * the probe response has already been reported -- and it is rare, not
 * the every-scan certainty it was before. */
VOID
VwifiScanFlushBatch(_Inout_ PVWIFI_ADAPTER Adapter)
{
    PVWIFI_SCAN_TASK task = Adapter->ScanTask;

    if (!task || !task->Active || task->PendingCount == 0) return;

    if (task->PendingCount >= VWIFI_SCAN_BATCH_THRESHOLD ||
        (VwifiGetTickCountMs() - task->LastIndicationTimeMs)
            >= VWIFI_SCAN_FLUSH_INTERVAL_MS) {
        VwifiIndicateBssEntryList(Adapter);
    }
}

VOID
VwifiScanOnComplete(_Inout_ PVWIFI_ADAPTER Adapter,
                    _In_reads_bytes_(PayloadLen) const VOID *Payload,
                    _In_ ULONG PayloadLen)
{
    INT32 devStatus = 0;

    if (PayloadLen >= sizeof(INT32)) {
        devStatus = *(const INT32 *)Payload;
    }
    VwifiIndicateScanComplete(
        Adapter,
        (devStatus == 0) ? NDIS_STATUS_SUCCESS : NDIS_STATUS_FAILURE);
}

/* ============================================================
 * OID_WDI_TASK_SCAN
 * ============================================================ */

NDIS_STATUS
VwifiHandleTaskScan(_Inout_ PVWIFI_ADAPTER Adapter,
                    _In_ PNDIS_OID_REQUEST Req)
{
    PVWIFI_SCAN_TASK task = Adapter->ScanTask;
    UCHAR reqbuf[sizeof(struct vwifi_scan_req) + VWIFI_SCAN_MAX_SSIDS * 34];
    struct vwifi_scan_req *scanReq = (struct vwifi_scan_req *)reqbuf;
    ULONG reqLen = 0;
    NDIS_STATUS status;
    ULONG outLen = 0;
    PVOID tlvBuf;
    ULONG tlvLen;
    ULONG slot;

    if (!task) return NDIS_STATUS_RESOURCES;
    if (task->Active) {
        /* A scan arriving while one is in flight is normal, and neither
         * refusing it nor cutting the running one short is right.
         *
         * It first returned NDIS_STATUS_REQUEST_ABORTED. A trace caught
         * what that costs: the OS asked again 1.3 seconds into the
         * first sweep, was refused, and never asked for another scan
         * for the rest of the session.
         *
         * The fix for that completed the outstanding task immediately
         * and adopted the new request, and a later trace caught what
         * THAT costs. The host issues a scan as part of a connect and
         * another a few milliseconds behind it; the second arrived 15 ms
         * into the first, so the first was completed 15 ms in, before
         * the device had reported a single BSS. A SCAN_COMPLETE with no
         * entries is not "the sweep is still going" -- it is "the scan
         * found nothing". The host read it exactly that way and failed
         * the association with STATUS_NETWORK_UNREACHABLE, two
         * milliseconds after we said it, and about a second before the
         * sweep actually found the AP.
         *
         * So: neither. The device is already sweeping every channel it
         * supports, and one sweep answers both requests. Remember the
         * new transaction id alongside the old and let the sweep finish
         * into both -- each task gets exactly one SCAN_COMPLETE, and
         * every one of them arrives after real results.
         *
         * The device is not asked to scan again. It answers -EBUSY
         * while its scan state machine is not idle, which would turn a
         * normal overlap into a device error. */
        if (task->TransactionCount >= VWIFI_SCAN_MAX_PENDING_TASKS) {
            VWIFI_WARN("scan requested with %u already pending on this "
                       "sweep -- refusing", task->TransactionCount);
            return NDIS_STATUS_REQUEST_ABORTED;
        }

        slot = task->TransactionCount++;
        task->Requesters[slot].TransactionId = VwifiGetWdiTransactionId(Req);
        task->Requesters[slot].WdiPortId     = VwifiGetWdiPortId(Req);
        task->Requesters[slot].PortId        = Req->PortNumber;
        task->Requesters[slot].Request       = Req;

        VWIFI_INFO("scan requested %llu ms into one already running -- "
                   "the running sweep will answer both (%u pending); "
                   "M1 txn %u wdiport 0x%04x ndisport %u",
                   VwifiGetTickCountMs() - task->StartedTimeMs,
                   task->TransactionCount,
                   task->Requesters[slot].TransactionId,
                   task->Requesters[slot].WdiPortId,
                   task->Requesters[slot].PortId);

        /* Re-arm: the watchdog now guards two tasks, not one. */
        if (task->Watchdog) {
            LARGE_INTEGER due;
            due.QuadPart = -((LONGLONG)VWIFI_SCAN_STALE_MS * 10000LL);
            NdisSetTimerObject(task->Watchdog, due, 0, NULL);
        }

        /* The sweep can finish between the `task->Active` test above and
         * the slot being filled in -- the device's completion is a DPC
         * and can be on another processor. If it did, its completion
         * loop has already walked past this slot, and a request nobody
         * completes hangs the caller for good.
         *
         * Re-reading Active after the store closes the window: the store
         * happens before this read and the loop's claim happens after it
         * clears Active, so whichever order the two take, exactly one of
         * them finds a request to complete.
         *
         * This one is then answered synchronously rather than left
         * pending. The sweep really has finished and its results are
         * already indicated, so there is nothing to wait for -- and
         * completing a request before returning PENDING from the handler
         * that owns it is a race not worth running for a path this
         * rare. */
        if (!task->Active && VwifiScanClaimRequest(task, slot) != NULL) {
            VWIFI_WARN("scan merged into a sweep that had just finished "
                       "-- answering txn %u synchronously",
                       task->Requesters[slot].TransactionId);
            VwifiSendWdiIndication(Adapter,
                                   task->Requesters[slot].WdiPortId,
                                   task->Requesters[slot].PortId,
                                   NDIS_STATUS_WDI_INDICATION_SCAN_COMPLETE,
                                   NDIS_STATUS_SUCCESS,
                                   task->Requesters[slot].TransactionId,
                                   NULL, 0);
            /* M3 first and the OID second here, the reverse of
             * VwifiIndicateScanComplete -- returning a status IS the
             * completion, so it cannot be moved earlier without
             * completing the request before the handler that owns it
             * has returned. The order does not matter on this path: the
             * sweep is over and its results are already indicated, so
             * there is no window in which the job could be finished
             * against a stale status. */
            return VwifiWdiTaskAnsweredInline(Req);
        }

        /* Outstanding until the running sweep finishes, like the request
         * that started it. */
        return VwifiWdiTaskPending(Req);
    }

    /* The OID buffer is [WDI_MESSAGE_HEADER][TLV blob]; the parser
     * wants the blob only. */
    status = VwifiGetTlvPayload(Req, &tlvBuf, &tlvLen);
    if (status != NDIS_STATUS_SUCCESS) return status;

    RtlZeroMemory(reqbuf, sizeof(reqbuf));
    status = VwifiTlvParseScanRequest(Adapter->WdiPeerVersion,
                                      tlvBuf, tlvLen,
                                      scanReq, sizeof(reqbuf), &reqLen);
    if (status != NDIS_STATUS_SUCCESS) {
        /* NDIS_STATUS_UNSUPPORTED_REVISION means PeerVersion is bogus —
         * usually it was never captured at AllocateAdapter. */
        VWIFI_ERR("scan param parse failed 0x%x (peer ver 0x%08x)",
                  status, Adapter->WdiPeerVersion);
        return status;
    }

    /* The OS may hand us a channel subset; scanning a superset is
     * always legal (a scan may report more than was asked for), so we
     * let the device sweep everything it supports and the OS filter.
     * Honouring the subset is a clean optimisation for later.
     *
     * BOTH masks, and only when BOTH are empty. The device reads two
     * zero masks as "everything you support" but a request naming only
     * 5 GHz leaves channel_mask_24 at zero legitimately, and widening
     * that one back to all of 2.4 GHz would silently change what was
     * asked for. */
    if (scanReq->channel_mask_24 == 0 && scanReq->channel_mask_5 == 0) {
        scanReq->channel_mask_24 = Adapter->CapsValid
                                 ? Adapter->Caps.supported_channels_24
                                 : 0x3FFE;
        scanReq->channel_mask_5  = Adapter->CapsValid
                                 ? Adapter->Caps.supported_channels_5
                                 : 0;
    }

    task->PortId               = Req->PortNumber;
    task->WdiPortId            = VwifiGetWdiPortId(Req);
    task->Requesters[0].TransactionId = VwifiGetWdiTransactionId(Req);
    task->Requesters[0].WdiPortId     = task->WdiPortId;
    task->Requesters[0].PortId        = task->PortId;
    /* Not stored yet. The device can still refuse the sweep below, and
     * that path returns a failure status synchronously -- a request that
     * is both failed by its return value and sitting in this slot would
     * be completed twice. It goes in only once the scan is running. */
    task->Requesters[0].Request       = NULL;
    task->TransactionCount     = 1;
    /* No M1 trace here: VwifiHandleOidRequest prints one for every WDI
     * method request, this one included. */
    task->PendingCount         = 0;
    task->LastIndicationTimeMs = VwifiGetTickCountMs();
    task->StartedTimeMs        = task->LastIndicationTimeMs;

    /* Marked active and the watchdog armed before the device is asked
     * to do anything: BSS_FOUND events can land on another processor
     * the instant the request is posted. */
    InterlockedExchange(&task->Active, 1);
    if (task->Watchdog) {
        LARGE_INTEGER due;
        due.QuadPart = -((LONGLONG)VWIFI_SCAN_STALE_MS * 10000LL);
        NdisSetTimerObject(task->Watchdog, due, 0, NULL);
    }

    status = VwifiCtrlSendSync(Adapter, VWIFI_OP_SCAN,
                               scanReq, reqLen, NULL, &outLen);
    if (status != NDIS_STATUS_SUCCESS) {
        /* Two reasons the device refuses a sweep, and both of them are
         * "not now", not "no".
         *
         * CONNECT_PENDING is the auth/assoc exchange. HandshakePending
         * is the 4-way handshake that follows it -- and that one is NOT
         * covered by the first, because CONNECT_COMPLETE is indicated
         * at association, before a single EAPOL frame moves. Keying the
         * deferral on the connect task alone was measured doing exactly
         * the damage the deferral exists to prevent: the device refused
         * the scan 0 ms after CONNECT_COMPLETE, this branch read the
         * connect as finished, failed the OID with 0xc0000001, and
         * wlansvc tore the association down 15 ms later. */
        BOOLEAN connecting = ((VwifiConnectTaskState(Adapter) &
                               VWIFI_TASK_CONNECT_PENDING) != 0 ||
                              Adapter->HandshakePending) ? TRUE : FALSE;

        if (!connecting) {
            /* Rejected before the scan ever started, so no SCAN_COMPLETE
             * is indicated on the failure path -- returning a failure
             * status and indicating a completion for the same task is a
             * double completion. Release the claim so the next scan is
             * not refused as an overlap. */
            if (InterlockedCompareExchange(&task->Active, 0, 1) == 1 &&
                task->Watchdog) {
                (VOID)NdisCancelTimerObject(task->Watchdog);
            }
            task->TransactionCount = 0;
            VWIFI_ERR("device rejected SCAN: 0x%x", status);
            return status;
        }

        /*
         * A connect is in flight, and this refusal is not a failure.
         *
         * The device will not hop channels while its auth/assoc
         * exchange is running -- it shares the radio and the one-shot
         * timer with it -- and wdiwifi issues a scan or two from inside
         * its own connect flow, so the two collide by design rather
         * than by accident.
         *
         * Neither failing it nor answering it succeeds. Both were
         * measured, and each breaks the connect a different way:
         *
         * FAILING it (the original behaviour) aborts the connect job.
         * A task OID's return value is the job's outcome, and a failed
         * scan job takes the post-connect sequence that ends in
         * TalTxRxPeerConfig down with it. The port still reaches
         * "connected" -- link state up, association result 0, AP happy
         * -- but the peer is never configured, so TX stays paused on
         * WDI_TX_PAUSE_REASON_PEER_CREATE for the life of the
         * association. Across five runs: whenever the association
         * happened to finish before wdiwifi got its scan in, the scan
         * was accepted and the peer config arrived; whenever it did
         * not, the scan was refused and it never came. Five for five.
         *
         * ANSWERING it immediately, as a completed no-op scan, fails
         * differently and worse. Finishing the scan job synchronously
         * is what drives CheckAndUpdateCandidates, and a refreshed
         * candidate list starts a NEW connect task while the first one
         * is still running. That second task is refused -- one connect
         * at a time -- with NDIS_STATUS_REQUEST_ABORTED, which fails
         * the job wlansvc is watching. The device associates anyway and
         * the link comes up, so the log looks like a success while the
         * OS puts up "Can't connect to this network". Measured too: a
         * second WDI_TASK_CONNECT 0 ms behind the first, and the line
         * "connect task already active", neither of which had ever
         * appeared in any earlier trace.
         *
         * So do what the accepted-sweep case does, which is the only
         * thing observed to work: hold the completion until the connect
         * is done. In the two runs that reached TalTxRxPeerConfig the
         * device took the scan, swept for ~1.6 s, and its SCAN_COMPLETE
         * landed AFTER the connect had completed. That ordering is the
         * whole difference, and deferring reproduces it deliberately
         * instead of leaving it to whether the AP happened to answer
         * inside one timer tick.
         *
         * The task stays Active with no sweep behind it, so a second
         * refused scan merges into it through the ordinary merge path
         * above and gets its own SCAN_COMPLETE from the same drain.
         * VwifiIndicateConnectComplete drains it; the watchdog is the
         * backstop if the connect never completes at all.
         */
        InterlockedExchange(&task->DeferredForConnect, 1);

        VWIFI_INFO("device refused SCAN while a connect is in flight; "
                   "holding txn %u until the connect completes (failing "
                   "it aborts the post-connect sequence; answering it "
                   "now starts a second connect task)",
                   task->Requesters[0].TransactionId);

        /* Adopt the request, then re-read Active.
         *
         * The connect can complete on another processor between the
         * flag above and this store, and its drain would then walk past
         * an empty slot and leave this request outstanding forever.
         * Re-reading after the store closes that window exactly as the
         * late-merge path does: the store happens before this read and
         * the drain's claim happens after it clears Active, so whichever
         * order the two take, exactly one of them finds a request to
         * complete. A claim that comes back NULL means the drain got
         * there first and has already completed it. */
        task->Requesters[0].Request = Req;

        if (!task->Active && VwifiScanClaimRequest(task, 0) != NULL) {
            VWIFI_WARN("connect completed while the refused scan was being "
                       "held -- answering txn %u synchronously",
                       task->Requesters[0].TransactionId);
            VwifiSendWdiIndication(Adapter,
                                   task->Requesters[0].WdiPortId,
                                   task->Requesters[0].PortId,
                                   NDIS_STATUS_WDI_INDICATION_SCAN_COMPLETE,
                                   NDIS_STATUS_SUCCESS,
                                   task->Requesters[0].TransactionId,
                                   NULL, 0);
            return VwifiWdiTaskAnsweredInline(Req);
        }

        return VwifiWdiTaskPending(Req);
    }

    VWIFI_INFO("scan started (mask24=0x%08x mask5=0x%llx dwell=%u ms ssids=%u)",
               scanReq->channel_mask_24,
               (unsigned long long)scanReq->channel_mask_5,
               scanReq->dwell_ms, scanReq->num_ssids);

    /* The sweep is running and cannot fail synchronously any more, so
     * the request can be adopted. */
    task->Requesters[0].Request = Req;

    /* WDI_SCAN_RESULTS is the M2: no TLV data, the header alone is the
     * "task accepted" acknowledgement, and it is written now. The OID
     * itself stays OUTSTANDING -- NDIS_STATUS_PENDING -- until
     * VwifiIndicateScanComplete completes it with the sweep's real
     * status, immediately before sending the SCAN_COMPLETE M3.
     *
     * Neither of the two values tried before works. INDICATION_REQUIRED
     * keeps the job open but is what CScanJob::FinishJob then reads as
     * the job's outcome -- measured, three times out of three, and it
     * suppresses WfcPortPropertyGoodScanStartTime, which is what makes
     * the connect job discard its candidates. SUCCESS zeroes that status
     * but ends the job the instant this returns, so wdiwifi polls the
     * BSS cache before a single frame has arrived and stops merging
     * scans. See VwifiWdiTaskPending. */
    return VwifiWdiTaskPending(Req);
}

/* Release a scan that was held because the device would not sweep
 * during an association.
 *
 * Called from VwifiIndicateConnectComplete on every outcome, success or
 * failure: what the held scan is waiting for is the connect being over,
 * not the connect having worked. The status is SUCCESS either way --
 * the scan did not fail, it was never run, and the BSS cache it would
 * have refreshed is still live and still served from
 * OID_WDI_GET_BSS_ENTRY_LIST.
 *
 * A no-op unless a scan is actually being held, so the connect path can
 * call it unconditionally. */
VOID
VwifiScanReleaseDeferred(_Inout_ PVWIFI_ADAPTER Adapter)
{
    PVWIFI_SCAN_TASK task = Adapter->ScanTask;

    if (!task || !task->Active || !task->DeferredForConnect) return;

    /* Not yet, if a 4-way handshake is still running.
     *
     * CONNECT_COMPLETE fires at association, which on a secure BSS is
     * the START of the handshake, not the end of the connect in any
     * sense the radio cares about. Releasing here would let the sweep
     * begin exactly when the EAPOL exchange needs the station to stay
     * on channel -- which is what was measured: M1 transmitted 60 ms
     * into a 13-channel sweep and never seen by the station at all.
     *
     * VwifiKeysOnInstalled releases it instead, and a disconnect
     * releases it too, so there is no path where the hold outlives the
     * thing it is waiting for. */
    if (Adapter->HandshakePending) {
        VWIFI_INFO("holding the %u scan task(s) past CONNECT_COMPLETE -- "
                   "the 4-way handshake still needs the channel",
                   task->TransactionCount);
        return;
    }

    VWIFI_INFO("connect finished -- releasing the %u scan task(s) held "
               "behind it", task->TransactionCount);

    if (task->Watchdog) (VOID)NdisCancelTimerObject(task->Watchdog);
    VwifiIndicateScanComplete(Adapter, NDIS_STATUS_SUCCESS);
}

/* For the heartbeat in MiniportCheckForHangEx, which lives in driver.c
 * and cannot see this file's private task structure. */
ULONG
VwifiScanTaskState(_In_ PVWIFI_ADAPTER Adapter)
{
    PVWIFI_SCAN_TASK task = Adapter->ScanTask;

    return (task && task->Active) ? 1u : 0u;
}

NDIS_STATUS
VwifiHandleTaskScanAbort(_Inout_ PVWIFI_ADAPTER Adapter)
{
    ULONG outLen = 0;
    PVWIFI_SCAN_TASK task = Adapter->ScanTask;

    if (!task || !task->Active) return NDIS_STATUS_SUCCESS;

    (VOID)VwifiCtrlSendSync(Adapter, VWIFI_OP_SCAN_ABORT,
                            NULL, 0, NULL, &outLen);
    /* The device emits SCAN_COMPLETE with a cancelled status, which
     * drives VwifiIndicateScanComplete. The OID docs require the port
     * be "in a clean state after the abort" — the device delivers that
     * by restoring the pre-scan channel. */
    return NDIS_STATUS_SUCCESS;
}

/* ============================================================
 * Task lifetime
 * ============================================================ */

NDIS_STATUS
VwifiScanTaskCreate(_Inout_ PVWIFI_ADAPTER Adapter)
{
    PVWIFI_SCAN_TASK task;

    task = NdisAllocateMemoryWithTagPriority(
        Adapter->MiniportAdapterHandle, sizeof(*task),
        VWIFI_POOL_TAG, NormalPoolPriority);
    if (!task) return NDIS_STATUS_RESOURCES;
    RtlZeroMemory(task, sizeof(*task));

    task->Pending = NdisAllocateMemoryWithTagPriority(
        Adapter->MiniportAdapterHandle,
        VWIFI_SCAN_PENDING_MAX * sizeof(task->Pending[0]),
        VWIFI_POOL_TAG, NormalPoolPriority);
    if (!task->Pending) {
        NdisFreeMemoryWithTagPriority(Adapter->MiniportAdapterHandle,
                                      task, VWIFI_POOL_TAG);
        return NDIS_STATUS_RESOURCES;
    }
    RtlZeroMemory(task->Pending,
                  VWIFI_SCAN_PENDING_MAX * sizeof(task->Pending[0]));
    task->PendingCapacity = VWIFI_SCAN_PENDING_MAX;

    task->Cache = NdisAllocateMemoryWithTagPriority(
        Adapter->MiniportAdapterHandle,
        VWIFI_SCAN_CACHE_MAX * sizeof(task->Cache[0]),
        VWIFI_POOL_TAG, NormalPoolPriority);
    if (!task->Cache) {
        NdisFreeMemoryWithTagPriority(Adapter->MiniportAdapterHandle,
                                      task->Pending, VWIFI_POOL_TAG);
        NdisFreeMemoryWithTagPriority(Adapter->MiniportAdapterHandle,
                                      task, VWIFI_POOL_TAG);
        return NDIS_STATUS_RESOURCES;
    }
    RtlZeroMemory(task->Cache,
                  VWIFI_SCAN_CACHE_MAX * sizeof(task->Cache[0]));
    task->CacheCapacity = VWIFI_SCAN_CACHE_MAX;

    {
        NDIS_TIMER_CHARACTERISTICS tc = { 0 };
        NDIS_STATUS st;

        tc.Header.Type     = NDIS_OBJECT_TYPE_TIMER_CHARACTERISTICS;
        tc.Header.Revision = NDIS_TIMER_CHARACTERISTICS_REVISION_1;
        tc.Header.Size     = NDIS_SIZEOF_TIMER_CHARACTERISTICS_REVISION_1;
        tc.AllocationTag   = VWIFI_POOL_TAG;
        tc.TimerFunction   = VwifiScanWatchdog;
        tc.FunctionContext = Adapter;

        st = NdisAllocateTimerObject(Adapter->MiniportAdapterHandle,
                                     &tc, &task->Watchdog);
        if (st != NDIS_STATUS_SUCCESS) {
            /* Not fatal. Scanning still works; what is lost is the
             * guarantee that a lost SCAN_COMPLETE costs one scan rather
             * than every scan from then on. */
            VWIFI_WARN("scan watchdog timer unavailable (0x%x)", st);
            task->Watchdog = NULL;
        }
    }

    Adapter->ScanTask = task;
    return NDIS_STATUS_SUCCESS;
}

VOID
VwifiScanTaskDestroy(_Inout_ PVWIFI_ADAPTER Adapter)
{
    PVWIFI_SCAN_TASK task = Adapter->ScanTask;
    if (!task) return;

    /* Cancel and free before the memory the callback reads goes away.
     * NdisFreeTimerObject waits for a callback already running, which
     * is the half NdisCancelTimerObject cannot promise. */
    if (task->Watchdog) {
        (VOID)NdisCancelTimerObject(task->Watchdog);
        NdisFreeTimerObject(task->Watchdog);
        task->Watchdog = NULL;
    }

    /* After the watchdog is gone, so nothing can hand out a completion
     * behind this, and before the task memory is freed, since the
     * requests live in it. A scan OID left outstanding here is not a
     * leak that shows up later -- it is the WLAN component blocked on a
     * request that can no longer be answered, which is what made
     * disabling the adapter hang before the disconnect task learned to
     * complete itself. */
    InterlockedExchange(&task->Active, 0);
    VwifiScanAbandonRequests(Adapter, task, NDIS_STATUS_REQUEST_ABORTED);

    if (task->Cache) {
        NdisFreeMemoryWithTagPriority(Adapter->MiniportAdapterHandle,
                                      task->Cache, VWIFI_POOL_TAG);
    }
    if (task->Pending) {
        NdisFreeMemoryWithTagPriority(Adapter->MiniportAdapterHandle,
                                      task->Pending, VWIFI_POOL_TAG);
    }
    NdisFreeMemoryWithTagPriority(Adapter->MiniportAdapterHandle,
                                  task, VWIFI_POOL_TAG);
    Adapter->ScanTask = NULL;
}

/* ============================================================
 * OID_WDI_GET_BSS_ENTRY_LIST
 *
 * "Get cached BSS entry list from adapter." The reply to the OID itself
 * is header-only; the entries come back the same way a scan reports
 * them, as an NDIS_STATUS_WDI_INDICATION_BSS_ENTRY_LIST.
 *
 * The request names an SSID, and this ignores it -- everything cached
 * is indicated and the component matches. A BSS_ENTRY_LIST indication
 * is the identical message the scan path already sends unsolicited, so
 * a superset is exactly what a scan would have produced anyway, and
 * filtering here would only risk hiding an entry the caller wanted.
 * ============================================================ */
/* Drop entries nothing has seen for a while.
 *
 * The cache had no expiry at all: an entry was replaced when the same
 * BSSID was seen again, evicted when the table filled, or cleared by an
 * explicit flush, and otherwise it lived forever. So an access point
 * that was reconfigured or switched off stayed in the list
 * indefinitely -- "netsh wlan show networks" kept listing an SSID that
 * had not been on the air for hours, the OS kept offering it, and every
 * connect to it failed because the network really was gone. Rebooting
 * the client did not help, because the first scan after boot re-cached
 * nothing and the stale entry was re-reported from a cache that had
 * never dropped it.
 *
 * The BSSID merge cannot cover this. It updates an entry when the same
 * BSS is seen again, which is the case where nothing is stale; the case
 * that needs handling is the BSS that is never seen again, and by
 * definition nothing arrives to trigger it. Only time can.
 *
 * VWIFI_SCAN_CACHE_TTL is a compromise between two real failures. Too
 * short and the cache is empty when the host asks for it -- it requests
 * the cached list a few milliseconds after a dot11 reset, with no time
 * to scan, and an empty answer is a connect that cannot succeed. Too
 * long is the bug above. Scans during any active use of the adapter
 * arrive far more often than this.
 */
static VOID
VwifiScanCacheExpire(_Inout_ PVWIFI_SCAN_TASK Task)
{
    LARGE_INTEGER now;
    ULONG kept = 0;

    if (Task->CacheCount == 0) return;

    NdisGetCurrentSystemTime(&now);

    for (ULONG i = 0; i < Task->CacheCount; i++) {
        ULONGLONG age;

        /* Unsigned, so a clock that stepped backwards would otherwise
         * make every entry look impossibly old and empty the cache. */
        if ((ULONGLONG)now.QuadPart < Task->Cache[i].SeenSystemTime) {
            age = 0;
        } else {
            age = (ULONGLONG)now.QuadPart - Task->Cache[i].SeenSystemTime;
        }

        if (age > VWIFI_SCAN_CACHE_TTL) {
            VWIFI_INFO("BSS cache: dropping %02x:%02x:%02x:%02x:%02x:%02x "
                       "-- not seen for %llu seconds",
                       Task->Cache[i].Entry.bssid[0], Task->Cache[i].Entry.bssid[1],
                       Task->Cache[i].Entry.bssid[2], Task->Cache[i].Entry.bssid[3],
                       Task->Cache[i].Entry.bssid[4], Task->Cache[i].Entry.bssid[5],
                       age / 10000000ULL);
            continue;
        }

        if (kept != i) Task->Cache[kept] = Task->Cache[i];
        kept++;
    }

    Task->CacheCount = kept;
}

VOID
VwifiScanIndicateCachedBss(_Inout_ PVWIFI_ADAPTER Adapter,
                           _In_ WDI_PORT_ID WdiPortId,
                           _In_ ULONG NdisPortNumber)
{
    PVWIFI_SCAN_TASK task = Adapter->ScanTask;
    VWIFI_TLV_BSS_ITEM items[VWIFI_SCAN_CACHE_MAX];
    PVOID  generated = NULL;
    ULONG  generatedLen = 0;
    NDIS_STATUS status;
    ULONG n;

    if (task) VwifiScanCacheExpire(task);

    if (!task || task->CacheCount == 0) {
        /* Loud, because this is no longer a harmless nothing-to-say.
         * We advertise BSSListCachemanagement = TRUE, so the host asks
         * this question when it needs a BSS to connect to -- typically
         * a few milliseconds after a dot11 reset, with no time to scan.
         * An empty answer here is a connect that will fail with "the
         * specific network is not available". */
        VWIFI_WARN("BSS cache is empty -- answering GET_BSS_ENTRY_LIST "
                   "with nothing; a connect issued now cannot succeed");
        return;
    }

    n = task->CacheCount;
    if (n > VWIFI_SCAN_CACHE_MAX) n = VWIFI_SCAN_CACHE_MAX;

    for (ULONG i = 0; i < n; i++) {
        items[i].Entry         = &task->Cache[i].Entry;
        items[i].Beacon        = task->Cache[i].Beacon;
        items[i].BeaconLen     = task->Cache[i].BeaconLen;
        items[i].Probe         = task->Cache[i].Probe;
        items[i].ProbeLen      = task->Cache[i].ProbeLen;
        items[i].HostTimeStamp = task->Cache[i].SeenSystemTime;
        /* 1, not 0: these come out of the adapter's own BSS list, which
         * is exactly what the TLV's second field distinguishes. */
        items[i].Cached        = TRUE;

        VwifiTraceBssItem("cached", i, &task->Cache[i]);
    }

    status = VwifiTlvGenerateBssEntryList(Adapter->WdiPeerVersion,
                                          items, n,
                                          &generated, &generatedLen);
    if (status != NDIS_STATUS_SUCCESS) {
        VWIFI_ERR("cached BSS list generation failed 0x%x (%u entries)",
                  status, n);
        return;
    }

    VwifiSendWdiIndication(Adapter, WdiPortId, NdisPortNumber,
                           NDIS_STATUS_WDI_INDICATION_BSS_ENTRY_LIST,
                           NDIS_STATUS_SUCCESS,
                           WDI_TRANSACTION_ID_UNSOLICIT,
                           generated, generatedLen);

    VWIFI_INFO("indicated cached BSS_ENTRY_LIST: %u entries, %u bytes",
               n, generatedLen);
    VwifiTlvFreeGenerated(generated);
}

/* OID_WDI_SET_FLUSH_BSS_ENTRY: drop what we remember. The host asks for
 * this when it wants a fresh view rather than a cached one. */
VOID
VwifiScanFlushCache(_Inout_ PVWIFI_ADAPTER Adapter)
{
    PVWIFI_SCAN_TASK task = Adapter->ScanTask;

    if (!task) return;
    task->CacheCount = 0;
    VWIFI_INFO("BSS cache flushed");
}
