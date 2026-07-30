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

/* ============================================================
 * Indication sending
 *
 * Builds [WDI_MESSAGE_HEADER][TLV blob] into one contiguous buffer and
 * hands it to NDIS as a status indication. TlvBuffer may be NULL for
 * the several messages WABIModel marks "No TLV data needed, header is
 * sufficient" (SCAN_COMPLETE, CONNECT_COMPLETE, ...).
 * ============================================================ */

VOID
VwifiSendWdiIndication(_Inout_ PVWIFI_ADAPTER Adapter,
                       _In_ ULONG PortId,
                       _In_ NDIS_STATUS StatusCode,
                       _In_reads_bytes_opt_(TlvLength) PVOID TlvBuffer,
                       _In_ ULONG TlvLength)
{
    NDIS_STATUS_INDICATION ind;
    WDI_MESSAGE_HEADER *hdr;
    PUCHAR msg;
    ULONG  msgLen = sizeof(WDI_MESSAGE_HEADER) + TlvLength;

    msg = NdisAllocateMemoryWithTagPriority(
        Adapter->MiniportAdapterHandle, msgLen,
        VWIFI_POOL_TAG, NormalPoolPriority);
    if (!msg) {
        VWIFI_ERR("indication alloc failed (%u bytes)", msgLen);
        return;
    }
    RtlZeroMemory(msg, msgLen);

    hdr = (WDI_MESSAGE_HEADER *)msg;
    hdr->PortId  = PortId;
    hdr->Message = StatusCode;
    hdr->Length  = TlvLength;

    if (TlvBuffer && TlvLength) {
        RtlCopyMemory(msg + sizeof(*hdr), TlvBuffer, TlvLength);
    }

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

    /* NdisMIndicateStatusEx copies the buffer, so we own it still. */
    NdisFreeMemoryWithTagPriority(Adapter->MiniportAdapterHandle,
                                  msg, VWIFI_POOL_TAG);
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
