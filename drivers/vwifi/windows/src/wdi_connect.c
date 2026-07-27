/*
 * vwifi — wdi_connect.c
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Phase 3: connect / disconnect task.
 *
 * Flow:
 *   OS wants to join a network
 *     -> OID_WDI_TASK_CONNECT
 *     -> we indicate ASSOCIATION_START
 *     -> submit VWIFI_OP_CONNECT to the device
 *     -> device runs Auth/Assoc on the medium
 *     -> VWIFI_EV_ASSOC_RESULT arrives
 *     -> indicate ASSOCIATION_RESULT, then CONNECT_COMPLETE
 *
 * ============================================================
 * THE ORDERING RULE — read before touching this file
 * ============================================================
 * WDI expects ASSOCIATION_RESULT and CONNECT_COMPLETE to be
 * indicated as soon as the 802.11 association succeeds — BEFORE
 * the 4-way handshake runs, not after. The connection isn't
 * really usable until the handshake completes and keys are
 * installed, but WDI models it the other way round:
 *
 *   association succeeds
 *     -> ASSOCIATION_RESULT (status success)
 *     -> CONNECT_COMPLETE   (status success)
 *     -> ...only now does the OS start pumping EAPOL frames
 *        through the normal TX/RX data path
 *     -> OID_WDI_SET_ADD_CIPHER_KEYS installs the PTK/GTK
 *
 * If you withhold CONNECT_COMPLETE waiting for the handshake,
 * the OS never sends the EAPOL frames, the handshake never
 * happens, and WDI eventually issues OID_WDI_TASK_DISCONNECT.
 * This is the single most common way WDI drivers fail on WPA2.
 *
 * That's why this file completes the task on ASSOC_RESULT alone
 * and does not wait for anything security-related. Phase 4 adds
 * key installation but does NOT change this ordering.
 * ============================================================
 *
 * TLV handling lives in tlv_shim.cpp.
 */

#include "vwifi_drv.h"
#include "tlv_shim.h"
#include <dot11wdi.h>

typedef struct _VWIFI_CONNECT_TASK
{
    BOOLEAN Active;
    ULONG   PortId;
    UCHAR   TargetBssid[6];
    UCHAR   TargetSsid[33];
    USHORT  TargetSsidLen;
    BOOLEAN Associated;
} VWIFI_CONNECT_TASK, *PVWIFI_CONNECT_TASK;

/* ============================================================
 * Parse the WDI connect request.
 * ============================================================ */
static NDIS_STATUS
VwifiParseConnectParameters(
    _In_ PVWIFI_ADAPTER Adapter,
    _In_reads_bytes_(BufferLen) PVOID Buffer,
    _In_ ULONG BufferLen,
    _Out_writes_bytes_to_(ReqCap, *ReqLen) struct vwifi_connect_req *Req,
    _In_ ULONG ReqCap,
    _Out_ PULONG ReqLen)
{
    NDIS_STATUS status;

    status = VwifiTlvParseConnectRequest(Adapter->WdiPeerVersion,
                                         Buffer, BufferLen,
                                         Req, ReqCap, ReqLen);
    if (status != NDIS_STATUS_SUCCESS) {
        VWIFI_ERR("connect TLV parse failed: 0x%x", status);
        return status;
    }

    /* Reject ciphers the device can't do, rather than associating and
     * then failing every data frame. */
    if (Req->cipher_pairwise != VWIFI_CIPHER_NONE &&
        Req->cipher_pairwise != VWIFI_CIPHER_CCMP128) {
        VWIFI_ERR("unsupported pairwise cipher %u", Req->cipher_pairwise);
        return NDIS_STATUS_NOT_SUPPORTED;
    }

    VWIFI_INFO("connect parsed: bssid %02x:%02x:%02x:%02x:%02x:%02x "
               "ssid='%s' auth=%u akm=%u cipher=%u assoc_ies=%u",
               Req->bssid[0], Req->bssid[1], Req->bssid[2],
               Req->bssid[3], Req->bssid[4], Req->bssid[5],
               Req->ssid, Req->auth_algo, Req->akm_suite,
               Req->cipher_pairwise, Req->assoc_ie_len);
    return NDIS_STATUS_SUCCESS;
}

/* ============================================================
 * WDI indications
 * ============================================================ */

static VOID
VwifiIndicateAssociationStart(_Inout_ PVWIFI_ADAPTER Adapter)
{
    PVWIFI_CONNECT_TASK task = Adapter->ConnectTask;
    NDIS_STATUS_INDICATION ind = { 0 };
    PVOID tlv = NULL;
    ULONG tlvLen = 0;

    if (VwifiTlvGenerateAssociationStart(Adapter->WdiPeerVersion,
                                         task->TargetBssid,
                                         task->TargetSsid,
                                         task->TargetSsidLen,
                                         &tlv, &tlvLen)
            != NDIS_STATUS_SUCCESS) {
        VWIFI_ERR("ASSOCIATION_START TLV generate failed");
        return;
    }

    ind.Header.Type      = NDIS_OBJECT_TYPE_STATUS_INDICATION;
    ind.Header.Revision  = NDIS_STATUS_INDICATION_REVISION_1;
    ind.Header.Size      = NDIS_SIZEOF_STATUS_INDICATION_REVISION_1;
    ind.SourceHandle     = Adapter->MiniportAdapterHandle;
    ind.PortNumber       = (NDIS_PORT_NUMBER)task->PortId;
    ind.StatusCode       = NDIS_STATUS_WDI_INDICATION_ASSOCIATION_START;
    ind.StatusBuffer     = tlv;
    ind.StatusBufferSize = tlvLen;

    VWIFI_INFO("indicating ASSOCIATION_START");
    NdisMIndicateStatusEx(Adapter->MiniportAdapterHandle, &ind);
    VwifiTlvFreeGenerated(tlv);
}

static VOID
VwifiIndicateAssociationResult(_Inout_ PVWIFI_ADAPTER Adapter,
                               _In_ const struct vwifi_assoc_result *Result,
                               _In_reads_bytes_(Result->ie_len) const UCHAR *Ies)
{
    PVWIFI_CONNECT_TASK task = Adapter->ConnectTask;
    NDIS_STATUS_INDICATION ind = { 0 };
    PVOID tlv = NULL;
    ULONG tlvLen = 0;

    if (VwifiTlvGenerateAssociationResult(Adapter->WdiPeerVersion,
                                          Result, Ies, &tlv, &tlvLen)
            != NDIS_STATUS_SUCCESS) {
        VWIFI_ERR("ASSOCIATION_RESULT TLV generate failed");
        return;
    }

    ind.Header.Type      = NDIS_OBJECT_TYPE_STATUS_INDICATION;
    ind.Header.Revision  = NDIS_STATUS_INDICATION_REVISION_1;
    ind.Header.Size      = NDIS_SIZEOF_STATUS_INDICATION_REVISION_1;
    ind.SourceHandle     = Adapter->MiniportAdapterHandle;
    ind.PortNumber       = (NDIS_PORT_NUMBER)task->PortId;
    ind.StatusCode       = NDIS_STATUS_WDI_INDICATION_ASSOCIATION_RESULT;
    ind.StatusBuffer     = tlv;
    ind.StatusBufferSize = tlvLen;

    VWIFI_INFO("indicating ASSOCIATION_RESULT status=%u aid=%u ies=%u",
               Result->status_code, Result->aid, Result->ie_len);
    NdisMIndicateStatusEx(Adapter->MiniportAdapterHandle, &ind);
    VwifiTlvFreeGenerated(tlv);
}

static VOID
VwifiIndicateConnectComplete(_Inout_ PVWIFI_ADAPTER Adapter,
                             _In_ NDIS_STATUS Status)
{
    PVWIFI_CONNECT_TASK task = Adapter->ConnectTask;
    NDIS_STATUS_INDICATION ind = { 0 };
    PVOID tlv = NULL;
    ULONG tlvLen = 0;

    if (VwifiTlvGenerateConnectComplete(Adapter->WdiPeerVersion,
                                        Status, task->TargetBssid,
                                        &tlv, &tlvLen)
            != NDIS_STATUS_SUCCESS) {
        VWIFI_ERR("CONNECT_COMPLETE TLV generate failed");
        task->Active = FALSE;
        return;
    }

    ind.Header.Type      = NDIS_OBJECT_TYPE_STATUS_INDICATION;
    ind.Header.Revision  = NDIS_STATUS_INDICATION_REVISION_1;
    ind.Header.Size      = NDIS_SIZEOF_STATUS_INDICATION_REVISION_1;
    ind.SourceHandle     = Adapter->MiniportAdapterHandle;
    ind.PortNumber       = (NDIS_PORT_NUMBER)task->PortId;
    ind.StatusCode       = NDIS_STATUS_WDI_INDICATION_CONNECT_COMPLETE;
    ind.StatusBuffer     = tlv;
    ind.StatusBufferSize = tlvLen;

    VWIFI_INFO("indicating CONNECT_COMPLETE (0x%x)", Status);
    NdisMIndicateStatusEx(Adapter->MiniportAdapterHandle, &ind);
    VwifiTlvFreeGenerated(tlv);

    task->Active = FALSE;
}

static VOID
VwifiIndicateDisassociation(_Inout_ PVWIFI_ADAPTER Adapter, _In_ USHORT Reason)
{
    PVWIFI_CONNECT_TASK task = Adapter->ConnectTask;
    NDIS_STATUS_INDICATION ind = { 0 };
    PVOID tlv = NULL;
    ULONG tlvLen = 0;

    if (VwifiTlvGenerateDisassociation(Adapter->WdiPeerVersion,
                                       Adapter->Bssid, Reason,
                                       &tlv, &tlvLen)
            != NDIS_STATUS_SUCCESS) {
        VWIFI_ERR("DISASSOCIATION TLV generate failed");
        return;
    }

    ind.Header.Type      = NDIS_OBJECT_TYPE_STATUS_INDICATION;
    ind.Header.Revision  = NDIS_STATUS_INDICATION_REVISION_1;
    ind.Header.Size      = NDIS_SIZEOF_STATUS_INDICATION_REVISION_1;
    ind.SourceHandle     = Adapter->MiniportAdapterHandle;
    ind.PortNumber       = task ? (NDIS_PORT_NUMBER)task->PortId
                                : NDIS_DEFAULT_PORT_NUMBER;
    ind.StatusCode       = NDIS_STATUS_WDI_INDICATION_DISASSOCIATION;
    ind.StatusBuffer     = tlv;
    ind.StatusBufferSize = tlvLen;

    VWIFI_INFO("indicating DISASSOCIATION reason=%u", Reason);
    NdisMIndicateStatusEx(Adapter->MiniportAdapterHandle, &ind);
    VwifiTlvFreeGenerated(tlv);
}

/* Report media-connect state to NDIS so the OS knows the link is up.
 * Distinct from the WDI indications above. */
static VOID
VwifiIndicateLinkState(_Inout_ PVWIFI_ADAPTER Adapter, _In_ BOOLEAN Up)
{
    NDIS_LINK_STATE linkState = { 0 };
    NDIS_STATUS_INDICATION ind = { 0 };

    linkState.Header.Type     = NDIS_OBJECT_TYPE_DEFAULT;
    linkState.Header.Revision = NDIS_LINK_STATE_REVISION_1;
    linkState.Header.Size     = NDIS_SIZEOF_LINK_STATE_REVISION_1;
    linkState.MediaConnectState =
        Up ? MediaConnectStateConnected : MediaConnectStateDisconnected;
    linkState.MediaDuplexState = MediaDuplexStateFull;
    linkState.XmitLinkSpeed    = Up ? 54000000ULL : NDIS_LINK_SPEED_UNKNOWN;
    linkState.RcvLinkSpeed     = Up ? 54000000ULL : NDIS_LINK_SPEED_UNKNOWN;

    ind.Header.Type       = NDIS_OBJECT_TYPE_STATUS_INDICATION;
    ind.Header.Revision   = NDIS_STATUS_INDICATION_REVISION_1;
    ind.Header.Size       = NDIS_SIZEOF_STATUS_INDICATION_REVISION_1;
    ind.SourceHandle      = Adapter->MiniportAdapterHandle;
    ind.StatusCode        = NDIS_STATUS_LINK_STATE;
    ind.StatusBuffer      = &linkState;
    ind.StatusBufferSize  = sizeof(linkState);

    NdisMIndicateStatusEx(Adapter->MiniportAdapterHandle, &ind);
}

/* ============================================================
 * Device event handlers — called from VwifiCtrlRspDrain at DPC.
 * ============================================================ */

VOID
VwifiConnectOnAssocResult(_Inout_ PVWIFI_ADAPTER Adapter,
                          _In_reads_bytes_(PayloadLen) const VOID *Payload,
                          _In_ ULONG PayloadLen)
{
    const struct vwifi_assoc_result *res = Payload;
    const UCHAR *ies;
    PVWIFI_CONNECT_TASK task = Adapter->ConnectTask;

    if (!task || !task->Active) return;
    if (PayloadLen < sizeof(*res)) {
        VWIFI_WARN("ASSOC_RESULT payload too short: %u", PayloadLen);
        return;
    }
    if (PayloadLen < sizeof(*res) + res->ie_len) {
        VWIFI_WARN("ASSOC_RESULT IE length %u exceeds payload %u",
                   res->ie_len, PayloadLen);
        return;
    }
    ies = (const UCHAR *)Payload + sizeof(*res);

    VwifiIndicateAssociationResult(Adapter, res, ies);

    if (res->status_code == 0) {
        task->Associated = TRUE;
        Adapter->Associated = TRUE;
        RtlCopyMemory(Adapter->Bssid, res->bssid, 6);

        /* ORDERING: complete the task now, before any EAPOL. See the
         * comment block at the top of this file. */
        VwifiIndicateConnectComplete(Adapter, NDIS_STATUS_SUCCESS);
        VwifiIndicateLinkState(Adapter, TRUE);
    } else {
        task->Associated = FALSE;
        Adapter->Associated = FALSE;
        VwifiIndicateConnectComplete(Adapter, NDIS_STATUS_FAILURE);
    }
}

VOID
VwifiConnectOnDisconnected(_Inout_ PVWIFI_ADAPTER Adapter,
                           _In_reads_bytes_(PayloadLen) const VOID *Payload,
                           _In_ ULONG PayloadLen)
{
    const struct vwifi_disconnect_ev *ev = Payload;
    USHORT reason = 0;
    PVWIFI_CONNECT_TASK task = Adapter->ConnectTask;

    if (PayloadLen >= sizeof(*ev)) reason = ev->reason_code;

    Adapter->Associated = FALSE;
    RtlZeroMemory(Adapter->Bssid, 6);

    if (task && task->Active) {
        /* Disconnected while a connect was still in flight. */
        task->Associated = FALSE;
        VwifiIndicateConnectComplete(Adapter, NDIS_STATUS_FAILURE);
    }

    VwifiIndicateDisassociation(Adapter, reason);
    VwifiIndicateLinkState(Adapter, FALSE);
}

/* ============================================================
 * OID entry points
 * ============================================================ */

NDIS_STATUS
VwifiHandleTaskConnect(_Inout_ PVWIFI_ADAPTER Adapter,
                       _In_ PNDIS_OID_REQUEST Req)
{
    PVWIFI_CONNECT_TASK task = Adapter->ConnectTask;
    UCHAR reqbuf[sizeof(struct vwifi_connect_req) + 256];
    struct vwifi_connect_req *creq = (struct vwifi_connect_req *)reqbuf;
    ULONG reqLen = 0;
    NDIS_STATUS status;
    ULONG outLen = 0;

    if (!task) return NDIS_STATUS_RESOURCES;
    if (task->Active) {
        VWIFI_WARN("connect task already active");
        return NDIS_STATUS_REQUEST_ABORTED;
    }

    status = VwifiParseConnectParameters(
        Adapter,
        Req->DATA.METHOD_INFORMATION.InformationBuffer,
        Req->DATA.METHOD_INFORMATION.InputBufferLength,
        creq, sizeof(reqbuf), &reqLen);
    if (status != NDIS_STATUS_SUCCESS) return status;

    task->Active     = TRUE;
    task->Associated = FALSE;
    task->PortId     = Req->PortNumber;

    /* Stash the target so the indications can report it. */
    RtlCopyMemory(task->TargetBssid, creq->bssid, 6);
    task->TargetSsidLen = creq->ssid_len;
    RtlCopyMemory(task->TargetSsid, creq->ssid, sizeof(task->TargetSsid));

    /* Indicate the attempt is starting before we kick the device. */
    VwifiIndicateAssociationStart(Adapter);

    status = VwifiCtrlSendSync(Adapter, VWIFI_OP_CONNECT,
                              creq, reqLen, NULL, &outLen);
    if (status != NDIS_STATUS_SUCCESS) {
        VWIFI_ERR("device rejected CONNECT: 0x%x", status);
        task->Active = FALSE;
        VwifiIndicateConnectComplete(Adapter, NDIS_STATUS_FAILURE);
        return status;
    }

    /* The device runs Auth/Assoc; ASSOC_RESULT completes the task. */
    return NDIS_STATUS_INDICATION_REQUIRED;
}

NDIS_STATUS
VwifiHandleTaskDisconnect(_Inout_ PVWIFI_ADAPTER Adapter,
                          _In_ PNDIS_OID_REQUEST Req)
{
    struct vwifi_disconnect_req dreq = { 0 };
    ULONG outLen = 0;

    UNREFERENCED_PARAMETER(Req);

    dreq.reason_code = 3;   /* STA is leaving */

    (VOID)VwifiCtrlSendSync(Adapter, VWIFI_OP_DISCONNECT,
                            &dreq, sizeof(dreq), NULL, &outLen);
    /* The device emits DISCONNECTED, which drives the indications. */
    return NDIS_STATUS_INDICATION_REQUIRED;
}

/* ============================================================
 * Task lifetime
 * ============================================================ */

NDIS_STATUS
VwifiConnectTaskCreate(_Inout_ PVWIFI_ADAPTER Adapter)
{
    PVWIFI_CONNECT_TASK task = NdisAllocateMemoryWithTagPriority(
        Adapter->MiniportAdapterHandle, sizeof(*task),
        VWIFI_POOL_TAG, NormalPoolPriority);
    if (!task) return NDIS_STATUS_RESOURCES;
    RtlZeroMemory(task, sizeof(*task));
    Adapter->ConnectTask = task;
    return NDIS_STATUS_SUCCESS;
}

VOID
VwifiConnectTaskDestroy(_Inout_ PVWIFI_ADAPTER Adapter)
{
    if (!Adapter->ConnectTask) return;
    NdisFreeMemoryWithTagPriority(Adapter->MiniportAdapterHandle,
                                  Adapter->ConnectTask, VWIFI_POOL_TAG);
    Adapter->ConnectTask = NULL;
}
