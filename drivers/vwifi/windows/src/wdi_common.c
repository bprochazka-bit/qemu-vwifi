/*
 * vwifi — wdi_common.c
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Helpers shared by the WDI task files (wdi_scan.c, wdi_connect.c,
 * wdi_keys.c).
 *
 * The two that matter:
 *
 *   VwifiGetTlvPayload   — every WDI OID buffer is
 *                          [WDI_MESSAGE_HEADER][TLV blob]. The parser
 *                          wants the blob only, so every task handler
 *                          must advance past the header first. The docs
 *                          say this explicitly: "passes the TLV blob
 *                          (after advancing past the WDI_MESSAGE_HEADER)
 *                          to parse the TLVs into a C-structure."
 *
 *   VwifiSendWdiIndication — builds the reverse: a WDI_MESSAGE_HEADER
 *                          followed by the generated TLV blob, wrapped
 *                          in an NDIS_STATUS_INDICATION.
 */

#include "vwifi_drv.h"

/* ============================================================
 * TLV payload extraction
 * ============================================================ */

NDIS_STATUS
VwifiGetTlvPayload(_In_ PNDIS_OID_REQUEST Req,
                   _Outptr_ PVOID *TlvBuffer,
                   _Out_ PULONG TlvLength)
{
    PUCHAR buf;
    ULONG  len;

    *TlvBuffer = NULL;
    *TlvLength = 0;

    /* WDI tasks arrive as method requests. */
    buf = (PUCHAR)Req->DATA.METHOD_INFORMATION.InformationBuffer;
    len = Req->DATA.METHOD_INFORMATION.InputBufferLength;

    if (!buf || len < sizeof(WDI_MESSAGE_HEADER)) {
        VWIFI_WARN("OID buffer too short for a WDI header (%u)", len);
        return NDIS_STATUS_INVALID_LENGTH;
    }

    *TlvBuffer = buf + sizeof(WDI_MESSAGE_HEADER);
    *TlvLength = len - sizeof(WDI_MESSAGE_HEADER);
    return NDIS_STATUS_SUCCESS;
}

/* The transaction id a task-completion indication must echo back. Read
 * it when the task starts, not when it completes: by then the OID
 * request buffer is long gone. */
UINT32
VwifiGetWdiTransactionId(_In_ PNDIS_OID_REQUEST Req)
{
    const WDI_MESSAGE_HEADER *hdr;

    hdr = (const WDI_MESSAGE_HEADER *)
              Req->DATA.METHOD_INFORMATION.InformationBuffer;
    if (hdr == NULL ||
        Req->DATA.METHOD_INFORMATION.InputBufferLength < sizeof(*hdr)) {
        return WDI_TRANSACTION_ID_UNSOLICIT;
    }
    return hdr->TransactionId;
}

/* ============================================================
 * Indication sending
 *
 * Hands NDIS a [WDI_MESSAGE_HEADER][TLV blob] buffer as a status
 * indication.
 *
 * The header is written *into* the generated buffer, not prepended to
 * it. Every VwifiTlvGenerate* call passes kHeaderReserve
 * (== sizeof(WDI_MESSAGE_HEADER)) as the library's header-length
 * argument, so the blob already begins with that many bytes reserved
 * for us and the returned length counts them. Allocating a second
 * header and copying the blob behind it — which is what this used to do
 * — produces a message with two headers, where the OS reads the
 * reserved zeroes as the real one.
 *
 * TlvBuffer may be NULL for the several messages WABIModel marks "No
 * TLV data needed, header is sufficient" (CHANGE_OPERATION_MODE_
 * COMPLETE and friends); then there is nothing to reserve into and we
 * allocate a bare header.
 * ============================================================ */

VOID
VwifiSendWdiIndication(_Inout_ PVWIFI_ADAPTER Adapter,
                       _In_ ULONG PortId,
                       _In_ NDIS_STATUS StatusCode,
                       _In_ UINT32 TransactionId,
                       _In_reads_bytes_opt_(TlvLength) PVOID TlvBuffer,
                       _In_ ULONG TlvLength)
{
    NDIS_STATUS_INDICATION ind;
    WDI_MESSAGE_HEADER *hdr;
    PUCHAR msg;
    ULONG  msgLen;
    BOOLEAN ownBuffer = FALSE;

    if (TlvBuffer != NULL) {
        if (TlvLength < sizeof(WDI_MESSAGE_HEADER)) {
            VWIFI_ERR("generated blob %u bytes, shorter than the header "
                      "space it was asked to reserve", TlvLength);
            return;
        }
        msg    = (PUCHAR)TlvBuffer;
        msgLen = TlvLength;
    } else {
        msgLen = sizeof(WDI_MESSAGE_HEADER);
        msg = NdisAllocateMemoryWithTagPriority(
            Adapter->MiniportAdapterHandle, msgLen,
            VWIFI_POOL_TAG, NormalPoolPriority);
        if (!msg) {
            VWIFI_ERR("indication alloc failed (%u bytes)", msgLen);
            return;
        }
        ownBuffer = TRUE;
    }

    /* WDI_MESSAGE_HEADER carries no message id and no length: dot11wdi.h
     * is explicit that "MessageId, MessageLength come from the
     * NDIS_OID_REQUEST or NDIS_STATUS fields" — here, from StatusCode
     * and StatusBufferSize on the indication below. What the header does
     * carry is the port, the completion status, and the transaction id
     * the OS matches task completions on. */
    hdr = (WDI_MESSAGE_HEADER *)msg;
    RtlZeroMemory(hdr, sizeof(*hdr));
    hdr->PortId        = (WDI_PORT_ID)PortId;
    hdr->Status        = NDIS_STATUS_SUCCESS;
    hdr->TransactionId = TransactionId;
    hdr->IhvSpecificId = 0;

    RtlZeroMemory(&ind, sizeof(ind));
    ind.Header.Type       = NDIS_OBJECT_TYPE_STATUS_INDICATION;
    ind.Header.Revision   = NDIS_STATUS_INDICATION_REVISION_1;
    ind.Header.Size       = NDIS_SIZEOF_STATUS_INDICATION_REVISION_1;
    ind.SourceHandle      = Adapter->MiniportAdapterHandle;
    ind.PortNumber        = (NDIS_PORT_NUMBER)PortId;
    ind.StatusCode        = StatusCode;
    ind.StatusBuffer      = msg;
    ind.StatusBufferSize  = msgLen;

    NdisMIndicateStatusEx(Adapter->MiniportAdapterHandle, &ind);

    /* NdisMIndicateStatusEx copies the buffer, so it is ours to release
     * once it returns. When the blob came from the TLV generator the
     * caller frees it with VwifiTlvFreeGenerated — the library's
     * allocator owns it, not NDIS's. */
    if (ownBuffer) {
        NdisFreeMemoryWithTagPriority(Adapter->MiniportAdapterHandle,
                                      msg, VWIFI_POOL_TAG);
    }
}

/* ============================================================
 * Small utilities
 * ============================================================ */

/* Map dBm to WDI's 0..100 link quality. The mapping is a driver
 * policy decision, not a spec: -50 dBm or better reads as 100%,
 * -100 dBm or worse as 0%, linear between. Matches what most IHV
 * drivers do closely enough that the Wi-Fi UI's bars look sane. */
ULONG
VwifiRssiToLinkQuality(_In_ CHAR Rssi)
{
    LONG r = (LONG)Rssi;

    if (r >= -50)  return 100;
    if (r <= -100) return 0;
    return (ULONG)((r + 100) * 2);
}

ULONGLONG
VwifiGetTickCountMs(VOID)
{
    LARGE_INTEGER tick;
    KeQueryTickCount(&tick);
    /* KeQueryTimeIncrement returns 100ns units per tick. */
    return (ULONGLONG)tick.QuadPart * KeQueryTimeIncrement() / 10000ULL;
}
