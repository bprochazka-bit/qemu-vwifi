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

/* One staged BSS: the device's own record plus the frame it came in.
 * Shared by the pending batch and the cache below, which hold exactly
 * the same thing for different lengths of time. */
typedef struct _VWIFI_BSS_STAGE
{
    struct vwifi_bss_entry Entry;
    UCHAR                  Frame[VWIFI_SCAN_MAX_FRAME];

    /* System time when the frame arrived, for the entry's age info.
     * Recorded here rather than at indication time because the cache
     * below replays entries long after the scan that found them, and
     * an entry that keeps reporting "discovered just now" would never
     * age out of the OS's list at all. */
    ULONGLONG              SeenSystemTime;
} VWIFI_BSS_STAGE, *PVWIFI_BSS_STAGE;

typedef struct _VWIFI_SCAN_TASK
{
    /* 1 while a scan awaits its SCAN_COMPLETE.
     *
     * Interlocked and claimed rather than tested: the device's response
     * drain and the watchdog below are independent DPCs and can be on
     * different processors at once, and whichever swaps this back to 0
     * owns the completion. */
    volatile LONG Active;

    /* Fires if SCAN_COMPLETE never arrives. See VwifiScanWatchdog --
     * the lazy check this replaces could not fire in the one case it
     * existed for. */
    NDIS_HANDLE Watchdog;

    /* Both port namespaces. WdiPortId scopes the WDI message
     * header of every indication this task sends; PortId is the
     * NDIS port the request arrived on. See VwifiGetWdiPortId. */
    WDI_PORT_ID     WdiPortId;
    ULONG     PortId;
    UINT32    TransactionId;   /* echoed by SCAN_COMPLETE */
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
     * Kept up to date whether or not a scan is running. The device
     * reports every BSS it observes, continuously, which is what real
     * hardware does and what makes an adapter's cached list worth
     * having: entries stay fresh, so the OS's own list does not empty
     * itself between scans. Discarding those reports outside a scan
     * made the cache exactly as stale as the last scan, and a network
     * that had been visible went missing until the next one.
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
        items[i].Frame         = task->Pending[i].Frame;
        items[i].HostTimeStamp = task->Pending[i].SeenSystemTime;
        items[i].Cached        = FALSE;   /* live: a scan is running */
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
static VOID
VwifiIndicateScanComplete(_Inout_ PVWIFI_ADAPTER Adapter, _In_ NDIS_STATUS Status)
{
    PVWIFI_SCAN_TASK task = Adapter->ScanTask;

    if (!task) return;

    /* Claim the completion. The device's SCAN_COMPLETE and the watchdog
     * both arrive here and only one of them may indicate. */
    if (InterlockedCompareExchange(&task->Active, 0, 1) != 1) return;

    /* Flush anything still staged before completing the task. */
    VwifiIndicateBssEntryList(Adapter);

    /* SCAN_COMPLETE carries no TLVs at all — the scan's outcome rides in
     * the message header's Status field. */
    VwifiSendWdiIndication(Adapter, task->WdiPortId, task->PortId,
                           NDIS_STATUS_WDI_INDICATION_SCAN_COMPLETE,
                           Status, task->TransactionId, NULL, 0);

    VWIFI_INFO("indicated SCAN_COMPLETE (0x%x)", Status);
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

VOID
VwifiScanOnBssFound(_Inout_ PVWIFI_ADAPTER Adapter,
                    _In_reads_bytes_(PayloadLen) const VOID *Payload,
                    _In_ ULONG PayloadLen)
{
    const struct vwifi_bss_entry *bss = Payload;
    PVWIFI_SCAN_TASK task = Adapter->ScanTask;
    const UCHAR *frame;
    USHORT frameLen;
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

    /* Stamped once, here, and carried by both copies below. This is the
     * value WDI_TLV_BSS_ENTRY_AGE_INFO is specified to want -- system
     * time, from NdisGetCurrentSystemTime -- and it is what decides how
     * long the OS keeps the network in its list. Callable at
     * DISPATCH_LEVEL, which is where this DPC runs. */
    NdisGetCurrentSystemTime(&now);

    /* The cache, first and unconditionally.
     *
     * The device observes BSSes continuously, not only during scans,
     * and this used to return early when no scan was running -- so the
     * adapter's cached list was only ever as fresh as the last scan.
     * Between scans the entries aged past what the OS keeps and the
     * network disappeared from `netsh wlan show networks` and from the
     * flyout, with a connect attempt then having no BSS to build from.
     *
     * Real hardware maintains its BSS list continuously; so does this
     * now. Replace by BSSID if we already know this one; otherwise
     * append, evicting the oldest when full. */
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
        }

        task->Cache[slot].Entry = *bss;
        task->Cache[slot].Entry.ie_len = frameLen;
        task->Cache[slot].SeenSystemTime = (ULONGLONG)now.QuadPart;
        RtlCopyMemory(task->Cache[slot].Frame, frame, frameLen);
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
               (bss->capability_info & VWIFI_BSS_F_BEACON)
                   ? "beacon" : "probe-resp");

    /* Everything past here is the running scan's business. Outside a
     * scan the cache above is the whole job: there is no scan for these
     * entries to be results of, and BSS_ENTRY_LIST is how a scan
     * reports, so an unsolicited stream of them between scans would be
     * reporting a scan that is not happening. The OS gets them when it
     * asks, through OID_WDI_GET_BSS_ENTRY_LIST. */
    if (!task->Active) return;

    /* Flush first if the accumulator is full. */
    if (task->PendingCount >= task->PendingCapacity) {
        VwifiIndicateBssEntryList(Adapter);
    }

    task->Pending[task->PendingCount].Entry = *bss;
    task->Pending[task->PendingCount].Entry.ie_len = frameLen;
    task->Pending[task->PendingCount].SeenSystemTime = (ULONGLONG)now.QuadPart;
    RtlCopyMemory(task->Pending[task->PendingCount].Frame, frame, frameLen);
    task->PendingCount++;

    /* The documented throttle: 3+ staged, or 500ms since last update. */
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

    if (!task) return NDIS_STATUS_RESOURCES;
    if (task->Active) {
        /* A scan is genuinely in flight. WDI runs one at a time per
         * port, so refusing is the right answer -- and it is now only
         * ever a real overlap, because the watchdog has already
         * completed anything older than VWIFI_SCAN_STALE_MS.
         *
         * This used to be where a lost SCAN_COMPLETE was noticed, by
         * checking the task's age here. That check could not fire in
         * the case it existed for: an outstanding scan is exactly why
         * the OS does not ask for the next one, so the code meant to
         * notice only ran when there was nothing to notice. */
        VWIFI_WARN("scan task already active (%llu ms)",
                   VwifiGetTickCountMs() - task->StartedTimeMs);
        return NDIS_STATUS_REQUEST_ABORTED;
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
    task->TransactionId        = VwifiGetWdiTransactionId(Req);
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
        /* Rejected before the scan ever started, so the OID reports it
         * and no SCAN_COMPLETE is indicated -- returning a failure
         * status and indicating a completion for the same task is a
         * double completion. Release the claim so the next scan is not
         * refused as an overlap. */
        VWIFI_ERR("device rejected SCAN: 0x%x", status);
        if (InterlockedCompareExchange(&task->Active, 0, 1) == 1 &&
            task->Watchdog) {
            (VOID)NdisCancelTimerObject(task->Watchdog);
        }
        return status;
    }

    VWIFI_INFO("scan started (mask24=0x%08x mask5=0x%llx dwell=%u ms ssids=%u)",
               scanReq->channel_mask_24,
               (unsigned long long)scanReq->channel_mask_5,
               scanReq->dwell_ms, scanReq->num_ssids);

    /* WDI_SCAN_RESULTS (the M0) needs no TLV data — the header alone is
     * the "task started" acknowledgement. INDICATION_REQUIRED tells
     * NDIS the real completion arrives later as an indication. */
    return NDIS_STATUS_INDICATION_REQUIRED;
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

    if (!task || task->CacheCount == 0) {
        VWIFI_INFO("BSS cache is empty; nothing to report");
        return;
    }

    n = task->CacheCount;
    if (n > VWIFI_SCAN_CACHE_MAX) n = VWIFI_SCAN_CACHE_MAX;

    for (ULONG i = 0; i < n; i++) {
        items[i].Entry         = &task->Cache[i].Entry;
        items[i].Frame         = task->Cache[i].Frame;
        items[i].HostTimeStamp = task->Cache[i].SeenSystemTime;
        /* 1, not 0: these come out of the adapter's own BSS list, which
         * is exactly what the TLV's second field distinguishes. */
        items[i].Cached        = TRUE;
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
