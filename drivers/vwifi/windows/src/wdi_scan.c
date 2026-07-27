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

typedef struct _VWIFI_SCAN_TASK
{
    BOOLEAN   Active;
    ULONG     PortId;
    ULONG     PendingCount;

    /* BSS entries awaiting indication. We stage the device's own
     * representation and convert to TLV only at indication time: the
     * DPC path stays cheap and TLV generation gets batched. */
    struct {
        struct vwifi_bss_entry Entry;
        UCHAR                  Frame[VWIFI_SCAN_MAX_FRAME];
    } *Pending;
    ULONG     PendingCapacity;

    ULONGLONG LastIndicationTimeMs;
} VWIFI_SCAN_TASK, *PVWIFI_SCAN_TASK;

/* From OID_WDI_TASK_SCAN: "the port should throttle indications and
 * send updates to the host only when it has discovered 3 or more, or
 * when it has discovered less than 3 entries but has not reported them
 * to the host for more than 500 milliseconds." */
#define VWIFI_SCAN_BATCH_THRESHOLD   3
#define VWIFI_SCAN_FLUSH_INTERVAL_MS 500

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
        items[i].Entry = &task->Pending[i].Entry;
        items[i].Frame = task->Pending[i].Frame;
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

    VwifiSendWdiIndication(Adapter, task->PortId,
                           NDIS_STATUS_WDI_INDICATION_BSS_ENTRY_LIST,
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
    PVOID generated = NULL;
    ULONG generatedLen = 0;

    if (!task || !task->Active) return;

    /* Flush anything still staged before completing the task. */
    VwifiIndicateBssEntryList(Adapter);

    if (VwifiTlvGenerateScanComplete(Adapter->WdiPeerVersion, Status,
                                     &generated, &generatedLen)
            == NDIS_STATUS_SUCCESS) {
        VwifiSendWdiIndication(Adapter, task->PortId,
                               NDIS_STATUS_WDI_INDICATION_SCAN_COMPLETE,
                               generated, generatedLen);
        VwifiTlvFreeGenerated(generated);
    }

    VWIFI_INFO("indicated SCAN_COMPLETE (0x%x)", Status);
    task->Active = FALSE;
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

    /* The device observes BSSes continuously, not only during scans.
     * Outside a scan there is nothing to report. */
    if (!task || !task->Active) return;

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

    /* Flush first if the accumulator is full. */
    if (task->PendingCount >= task->PendingCapacity) {
        VwifiIndicateBssEntryList(Adapter);
    }

    task->Pending[task->PendingCount].Entry = *bss;
    task->Pending[task->PendingCount].Entry.ie_len = frameLen;
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
        VWIFI_WARN("scan task already active");
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
     * Honouring the subset is a clean optimisation for later. */
    if (scanReq->channel_mask_24 == 0) {
        scanReq->channel_mask_24 = Adapter->CapsValid
                                 ? Adapter->Caps.supported_channels_24
                                 : 0x3FFE;
    }

    task->Active               = TRUE;
    task->PortId               = Req->PortNumber;
    task->PendingCount         = 0;
    task->LastIndicationTimeMs = VwifiGetTickCountMs();

    status = VwifiCtrlSendSync(Adapter, VWIFI_OP_SCAN,
                               scanReq, reqLen, NULL, &outLen);
    if (status != NDIS_STATUS_SUCCESS) {
        VWIFI_ERR("device rejected SCAN: 0x%x", status);
        task->Active = FALSE;
        return status;
    }

    VWIFI_INFO("scan started (mask24=0x%08x dwell=%u ms ssids=%u)",
               scanReq->channel_mask_24, scanReq->dwell_ms,
               scanReq->num_ssids);

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

    Adapter->ScanTask = task;
    return NDIS_STATUS_SUCCESS;
}

VOID
VwifiScanTaskDestroy(_Inout_ PVWIFI_ADAPTER Adapter)
{
    PVWIFI_SCAN_TASK task = Adapter->ScanTask;
    if (!task) return;

    if (task->Pending) {
        NdisFreeMemoryWithTagPriority(Adapter->MiniportAdapterHandle,
                                      task->Pending, VWIFI_POOL_TAG);
    }
    NdisFreeMemoryWithTagPriority(Adapter->MiniportAdapterHandle,
                                  task, VWIFI_POOL_TAG);
    Adapter->ScanTask = NULL;
}
