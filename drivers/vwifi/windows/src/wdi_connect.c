/*
 * vwifi — wdi_connect.c
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Phase 3: connect / disconnect task.
 *
 * Flow:
 *   OS wants to join a network
 *     -> OID_WDI_TASK_CONNECT
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

/* ============================================================
 * THE COMPLETION RULE — the other thing to read before touching
 * this file
 * ============================================================
 * Every task in here is completed by an indication that arrives
 * later, driven by a device event. Two consequences, and both were
 * learned the hard way:
 *
 *   A task that is never completed does not fail. It hangs, and it
 *   hangs the whole WLAN service with it: NDIS serialises OID
 *   requests per adapter and wlansvc serialises its own calls per
 *   interface, so one outstanding task stops every query behind it.
 *   The visible symptom is not "couldn't connect" -- it is `netsh
 *   wlan show drivers` stopping mid-output, the flyout sitting on
 *   "Checking network requirements" forever, and a VM that cannot be
 *   shut down cleanly. So every path out of here completes exactly
 *   once, and a watchdog completes it if the device never answers.
 *
 *   A task must be completed by ITS OWN indication. A disconnect is
 *   not finished by CONNECT_COMPLETE, however much the two look
 *   alike from in here -- see VwifiIndicateDisconnectComplete.
 * ============================================================ */

/* How long to wait for the device before giving up on a task.
 *
 * OID_WDI_TASK_CONNECT's documented "normal execution time" is 10
 * seconds and OID_WDI_TASK_DISCONNECT's is 1. These are the outer
 * bounds past which the event is not late, it is not coming. */
#define VWIFI_CONNECT_WATCHDOG_MS      15000
#define VWIFI_DISCONNECT_WATCHDOG_MS    5000

typedef struct _VWIFI_CONNECT_TASK
{
    /* Both port namespaces. WdiPortId scopes the WDI message
     * header of every indication this task sends; PortId is the
     * NDIS port the request arrived on. See VwifiGetWdiPortId. */
    WDI_PORT_ID   WdiPortId;
    ULONG   PortId;
    UINT32  TransactionId;     /* echoed by CONNECT_COMPLETE */
    UCHAR   TargetBssid[6];
    UCHAR   TargetSsid[33];
    USHORT  TargetSsidLen;
    BOOLEAN Associated;

    /* Band and algorithms, kept from the connect request so the
     * association result can state them.
     *
     * The device's vwifi_assoc_result reports the BSSID, the status
     * code and the response IEs and nothing else -- it has no reason
     * to echo back what it was just told. WDI's association result
     * wants the band and the negotiated algorithms named explicitly,
     * so they are remembered here rather than reconstructed. */
    VWIFI_ASSOC_PARAMS AssocParams;

    /* The disconnect's own port and transaction id. Separate fields,
     * not the connect's: a disconnect can arrive with no connect ever
     * having run, and its completion has to echo the transaction id it
     * came in with. */
    WDI_PORT_ID   DiscWdiPortId;
    ULONG   DiscPortId;
    UINT32  DiscTransactionId;

    /* 1 while the task awaits its completion indication.
     *
     * Interlocked, and claimed rather than tested, because the two
     * paths that can complete a task run as independent DPCs -- the
     * device's response drain and the watchdog below -- and may be on
     * different processors at the same moment. Whoever swaps the flag
     * back to 0 owns the completion; the loser does nothing. A plain
     * BOOLEAN here is a double completion under exactly the timing
     * that makes it hardest to reproduce. */
    volatile LONG ConnectPending;
    volatile LONG DisconnectPending;

    /* Fires when the device has not answered in time, so that a lost
     * event costs one failed connect instead of a wedged Wi-Fi stack.
     * The same idea as the stale-scan backstop in wdi_scan.c, armed
     * rather than lazy because nothing else comes along afterwards to
     * notice on our behalf. */
    NDIS_HANDLE Watchdog;
} VWIFI_CONNECT_TASK, *PVWIFI_CONNECT_TASK;

static VOID VwifiConnectArmWatchdog(_In_ PVWIFI_CONNECT_TASK Task,
                                    _In_ ULONG Milliseconds);

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

    /* freq is in here for a reason, and it was missing.
     *
     * It is the one connect parameter whose being wrong looks exactly
     * like an association timeout: the device tunes to it to
     * authenticate, and a zero or wrong channel means the auth frames
     * go somewhere the AP is not. A run that came back
     * ASSOCIATION_RESULT status=16 -- "authentication rejected due to
     * timeout waiting for the next frame in sequence", one clean second
     * after the request -- could not be told apart from a medium
     * problem, because the log said everything about that connect
     * except where it was aimed.
     *
     * Zero is legal and is not automatically the fault: it means the
     * connect request carried no preferred BSS entry with channel
     * information, and VwifiTlvParseConnectRequest leaves it for the
     * device to resolve from its own BSS table. It is only a problem if
     * the device's table has nothing to resolve it against. Either way
     * the log now says which case it was. */
    VWIFI_INFO("connect parsed: bssid %02x:%02x:%02x:%02x:%02x:%02x "
               "ssid='%s' freq=%u auth=%u akm=%u cipher=%u/%u "
               "assoc_ies=%u%s",
               Req->bssid[0], Req->bssid[1], Req->bssid[2],
               Req->bssid[3], Req->bssid[4], Req->bssid[5],
               Req->ssid, Req->channel_freq, Req->auth_algo,
               Req->akm_suite, Req->cipher_pairwise, Req->cipher_group,
               Req->assoc_ie_len,
               (Req->channel_freq == 0)
                   ? " -- no channel in the request; the device must "
                     "resolve it from its own BSS table"
                   : "");
    return NDIS_STATUS_SUCCESS;
}

/* ============================================================
 * WDI indications
 * ============================================================ */

/* An 802.11 status code by name, for the association result.
 *
 * The device reports these verbatim from the AP and they are the one
 * place in this driver's log where a bare number is genuinely someone
 * else's verdict rather than ours. "status=16" cost a round of looking
 * in the wrong place; "status=16 (AUTH_TIMEOUT)" would not have -- it
 * says immediately that the AP never answered the authentication
 * handshake, which is a question for the device and the host-side AP,
 * not for anything above this line.
 *
 * Only the codes this stack can actually produce. IEEE 802.11-2016
 * Table 9-46 has a hundred more and listing them would bury the six
 * that matter. */
static PCSTR
VwifiDot11StatusName(_In_ USHORT Status)
{
    switch (Status) {
    case 0:  return "SUCCESS";
    case 1:  return "UNSPECIFIED_FAILURE";
    case 10: return "CAPABILITIES_MISMATCH";
    case 12: return "DENIED_OTHER_REASON";
    case 13: return "UNSUPPORTED_AUTH_ALGORITHM";
    case 14: return "AUTH_SEQUENCE_ERROR";
    case 15: return "AUTH_CHALLENGE_FAILURE";
    case 16: return "AUTH_TIMEOUT -- the AP never answered";
    case 17: return "DENIED_NO_MORE_STAS";
    case 18: return "BASIC_RATES_MISMATCH";
    default: return "?";
    }
}

/* WDI has no ASSOCIATION_START indication -- dot11wdi.h defines
 * ASSOCIATION_RESULT (76) and CONNECT_COMPLETE (64) for this flow, and
 * ASSOCIATION_PARAMETERS_REQUEST (98) for the unrelated case where the
 * OS supplies association parameters. The driver used to indicate a
 * NDIS_STATUS_WDI_INDICATION_ASSOCIATION_START that does not exist, so
 * the sequence is now just result-then-complete.
 * ============================================================ */

static VOID
VwifiIndicateAssociationResult(_Inout_ PVWIFI_ADAPTER Adapter,
                               _In_ const struct vwifi_assoc_result *Result,
                               _In_ const VWIFI_ASSOC_PARAMS *Params,
                               _In_reads_bytes_(Result->ie_len) const UCHAR *Ies)
{
    PVWIFI_CONNECT_TASK task = Adapter->ConnectTask;
    PVOID tlv = NULL;
    ULONG tlvLen = 0;

    if (VwifiTlvGenerateAssociationResult(Adapter->WdiPeerVersion,
                                          Result, Params, Ies, &tlv, &tlvLen)
            != NDIS_STATUS_SUCCESS) {
        /* The mandatory-container trap, twice over now: this failed for
         * the whole life of the project because ActivePhyTypeList was
         * left empty, and the message it produced was a bare
         * "generate failed" with no idea which field. If it fires
         * again, the fields are printed so the next reader starts
         * somewhere. */
        VWIFI_ERR("ASSOCIATION_RESULT TLV generate failed "
                  "(status %u freq %u auth %u akm %u cipher %u/%u ies %u)",
                  Result->status_code, Params->FreqMhz, Params->AuthAlgo,
                  Params->AkmSuite, Params->CipherPairwise,
                  Params->CipherGroup, Result->ie_len);
        return;
    }

    VWIFI_INFO("indicating ASSOCIATION_RESULT status=%u (%s) aid=%u ies=%u "
               "akm=%u -- PROBE BUILD: reporting the port AUTHORIZED even "
               "with an AKM, to find out whether that flag is what stops "
               "EAPOL crossing it",
               Result->status_code,
               VwifiDot11StatusName(Result->status_code),
               Result->aid, Result->ie_len, Params->AkmSuite);
    VwifiSendWdiIndication(Adapter, task->WdiPortId, task->PortId,
                           NDIS_STATUS_WDI_INDICATION_ASSOCIATION_RESULT,
                           NDIS_STATUS_SUCCESS,
                           WDI_TRANSACTION_ID_UNSOLICIT, tlv, tlvLen);
    VwifiTlvFreeGenerated(tlv);
}

static VOID
VwifiIndicateConnectComplete(_Inout_ PVWIFI_ADAPTER Adapter,
                             _In_ NDIS_STATUS Status)
{
    PVWIFI_CONNECT_TASK task = Adapter->ConnectTask;

    /* Claim the completion. Whoever gets here first sends it; anyone
     * arriving second -- the watchdog and a late ASSOC_RESULT racing,
     * say -- finds the flag already clear and returns. */
    if (InterlockedCompareExchange(&task->ConnectPending, 0, 1) != 1) {
        return;
    }

    /* CONNECT_COMPLETE carries no TLVs — not the BSSID, not the status.
     * The outcome rides in the message header's Status field, and the OS
     * already knows which BSS it asked us to join. */
    VWIFI_INFO("indicating CONNECT_COMPLETE (0x%x)", Status);
    VwifiSendWdiIndication(Adapter, task->WdiPortId, task->PortId,
                           NDIS_STATUS_WDI_INDICATION_CONNECT_COMPLETE,
                           Status, task->TransactionId, NULL, 0);

    /* The device refuses to sweep while it is associating, so any scan
     * that arrived during this connect was held rather than failed or
     * answered early -- both of those break the connect, in different
     * places. See the refusal path in VwifiHandleTaskScan. This is the
     * moment it is safe to let go: after CONNECT_COMPLETE, so finishing
     * the scan job cannot start a second connect task on top of this
     * one. No-op if nothing was held. */
    VwifiScanReleaseDeferred(Adapter);
}

/* The disconnect task's completion -- and it has to be this message.
 *
 * OID_WDI_TASK_DISCONNECT's documented completion indication is
 * NDIS_STATUS_WDI_INDICATION_DISCONNECT_COMPLETE, and nothing in this
 * driver ever sent it. The disconnect handler returned
 * INDICATION_REQUIRED and the device's DISCONNECTED event drove a
 * DISASSOCIATION indication and a link-state change -- both correct,
 * neither a completion -- so every OID_WDI_TASK_DISCONNECT ever issued
 * to this driver stayed outstanding forever.
 *
 * A disconnect is not a rare path. It is what the OS issues to take a
 * port down, so it runs when the adapter is disabled, when the device
 * is uninstalled, and when the machine shuts down. That matches the
 * symptom that has been present since the driver first loaded:
 * disabling or removing the adapter hangs, produces no further driver
 * output, and leaves the VM needing to be powered off.
 *
 * Header-only: "This indication contains no additional data. The data
 * in the header is sufficient." */
static VOID
VwifiIndicateDisconnectComplete(_Inout_ PVWIFI_ADAPTER Adapter,
                                _In_ NDIS_STATUS Status)
{
    PVWIFI_CONNECT_TASK task = Adapter->ConnectTask;

    if (InterlockedCompareExchange(&task->DisconnectPending, 0, 1) != 1) {
        return;
    }

    VWIFI_INFO("indicating DISCONNECT_COMPLETE (0x%x)", Status);
    VwifiSendWdiIndication(Adapter, task->DiscWdiPortId, task->DiscPortId,
                           NDIS_STATUS_WDI_INDICATION_DISCONNECT_COMPLETE,
                           Status, task->DiscTransactionId, NULL, 0);
}

static VOID
VwifiIndicateDisassociation(_Inout_ PVWIFI_ADAPTER Adapter, _In_ BOOLEAN Local)
{
    PVWIFI_CONNECT_TASK task = Adapter->ConnectTask;
    PVOID tlv = NULL;
    ULONG tlvLen = 0;

    if (VwifiTlvGenerateDisassociation(Adapter->WdiPeerVersion,
                                       Adapter->Bssid, Local,
                                       &tlv, &tlvLen)
            != NDIS_STATUS_SUCCESS) {
        VWIFI_ERR("DISASSOCIATION TLV generate failed");
        return;
    }

    VWIFI_INFO("indicating DISASSOCIATION (%s-initiated)",
               Local ? "locally" : "peer");
    VwifiSendWdiIndication(Adapter,
                           task ? task->WdiPortId : WDI_PORT_ID_ADAPTER,
                           task ? task->PortId : NDIS_DEFAULT_PORT_NUMBER,
                           NDIS_STATUS_WDI_INDICATION_DISASSOCIATION,
                           NDIS_STATUS_SUCCESS,
                           WDI_TRANSACTION_ID_UNSOLICIT, tlv, tlvLen);
    VwifiTlvFreeGenerated(tlv);
}

/* Report media-connect state to NDIS so the OS knows the link is up.
 * Distinct from the WDI indications above.
 *
 * Not static: the initial state has to be reported when the port is
 * created, before anything has associated. NDIS expects a miniport to
 * say where its link stands; saying nothing is not the same as saying
 * "down", and an interface whose media state was never stated is not
 * obviously one the OS should start a connection on. */
VOID
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

    /* The station port, not the default one.
     *
     * This field was never set, so every link-state indication this
     * driver has ever sent went to port 0. The station port the host
     * created in OID_WDI_TASK_CREATE_PORT is the one the OS connects
     * on, and it had heard nothing about its own media state -- not at
     * port creation, not on association, not on disconnect. An
     * interface whose media state was never stated is not one the OS
     * has any reason to believe it can start a connection on.
     *
     * Zero until a port exists, which is correct: before CREATE_PORT
     * the default port is all there is. */
    ind.PortNumber        = (NDIS_PORT_NUMBER)Adapter->NdisPortNumber;

    VWIFI_INFO("IND: LINK_STATE %s on ndisport %u",
               Up ? "connected" : "disconnected", Adapter->NdisPortNumber);

    NdisMIndicateStatusEx(Adapter->MiniportAdapterHandle, &ind);
}

/* ============================================================
 * Watchdog
 *
 * Runs at DISPATCH_LEVEL, which is where the indications below are
 * sent from anyway.
 * ============================================================ */

static VOID
VwifiConnectWatchdog(_In_ PVOID SystemSpecific1,
                     _In_ PVOID FunctionContext,
                     _In_ PVOID SystemSpecific2,
                     _In_ PVOID SystemSpecific3)
{
    PVWIFI_ADAPTER adapter = (PVWIFI_ADAPTER)FunctionContext;
    PVWIFI_CONNECT_TASK task = adapter ? adapter->ConnectTask : NULL;

    UNREFERENCED_PARAMETER(SystemSpecific1);
    UNREFERENCED_PARAMETER(SystemSpecific2);
    UNREFERENCED_PARAMETER(SystemSpecific3);

    if (!task) return;

    if (task->ConnectPending) {
        VWIFI_WARN("no ASSOC_RESULT from the device -- failing the connect "
                   "rather than leaving the task outstanding");
        VwifiIndicateConnectComplete(adapter, NDIS_STATUS_FAILURE);
    }

    if (task->DisconnectPending) {
        /* Completed as success, not failure. The local side of the
         * disconnect is done either way -- the association state is
         * cleared below in VwifiConnectOnDisconnected, and cleared here
         * if that event never came -- and reporting failure would leave
         * the OS believing the port is still connected to something it
         * is not. */
        VWIFI_WARN("no DISCONNECTED event from the device -- completing "
                   "the disconnect anyway");
        adapter->Associated = FALSE;
        RtlZeroMemory(adapter->Bssid, 6);
        VwifiIndicateDisconnectComplete(adapter, NDIS_STATUS_SUCCESS);
        VwifiIndicateLinkState(adapter, FALSE);
    }
}

static VOID
VwifiConnectArmWatchdog(_In_ PVWIFI_CONNECT_TASK Task,
                        _In_ ULONG Milliseconds)
{
    LARGE_INTEGER due;

    /* One timer for both tasks, and arming it moves the single
     * deadline. That is fine in both directions: a disconnect arriving
     * during a connect shortens the connect's deadline, which is what
     * the OS is asking for anyway, and a connect arriving during a
     * disconnect lengthens the disconnect's, which only means a lost
     * event is noticed later. The callback checks both, so neither can
     * be left pending with no timer armed. */
    if (!Task->Watchdog) return;

    /* Negative is relative, in 100 ns units. */
    due.QuadPart = -((LONGLONG)Milliseconds * 10000LL);
    NdisSetTimerObject(Task->Watchdog, due, 0, NULL);
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

    if (!task || !task->ConnectPending) return;
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

    VwifiIndicateAssociationResult(Adapter, res, &task->AssocParams, ies);

    if (res->status_code == 0) {
        WDI_PEER_ID peerId = WDI_PEER_ANY;

        task->Associated = TRUE;
        Adapter->Associated = TRUE;
        RtlCopyMemory(Adapter->Bssid, res->bssid, 6);

        /* The AP is now a peer, and this is the moment to say so.
         *
         * Before CONNECT_COMPLETE, deliberately. The WDI data path is
         * addressed by (port, peer, TID) and the component keeps its TX
         * queues paused on WDI_TX_PAUSE_REASON_PEER_CREATE until a peer
         * exists -- so a connect completed with no peer is a link the
         * OS believes is up and cannot send a byte over. Creating the
         * peer first means the component has somewhere to send to
         * before it is told it may. */
        (VOID)VwifiPeerCreate(Adapter, task->WdiPortId, res->bssid,
                              &peerId);

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
    BOOLEAN local = FALSE;
    PVWIFI_CONNECT_TASK task = Adapter->ConnectTask;

    if (PayloadLen >= sizeof(*ev)) local = ev->local ? TRUE : FALSE;

    Adapter->Associated = FALSE;
    RtlZeroMemory(Adapter->Bssid, 6);

    /* The peer goes with the association. Indicated before the
     * disassociation and the link-state change for the same reason the
     * create runs before CONNECT_COMPLETE: the component should stop
     * having somewhere to send before it is told the link is down. */
    VwifiPeerDeleteAll(Adapter);

    if (task && task->ConnectPending) {
        /* Disconnected while a connect was still in flight. */
        task->Associated = FALSE;
        VwifiIndicateConnectComplete(Adapter, NDIS_STATUS_FAILURE);
    }

    /* And if this event is the answer to an OID_WDI_TASK_DISCONNECT,
     * that task's own completion. Both can be outstanding at once --
     * the OS aborts a connect by disconnecting the port -- so this is
     * not an else. */
    if (task && task->DisconnectPending) {
        VwifiIndicateDisconnectComplete(Adapter, NDIS_STATUS_SUCCESS);
    }

    VwifiIndicateDisassociation(Adapter, local);
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
    if (task->ConnectPending) {
        VWIFI_WARN("connect task already active");
        return NDIS_STATUS_REQUEST_ABORTED;
    }

    status = VwifiParseConnectParameters(
        Adapter,
        Req->DATA.METHOD_INFORMATION.InformationBuffer,
        Req->DATA.METHOD_INFORMATION.InputBufferLength,
        creq, sizeof(reqbuf), &reqLen);
    if (status != NDIS_STATUS_SUCCESS) return status;

    task->Associated = FALSE;
    task->PortId     = Req->PortNumber;
    task->WdiPortId  = VwifiGetWdiPortId(Req);
    task->TransactionId = VwifiGetWdiTransactionId(Req);

    /* Stash the target so the indications can report it. */
    RtlCopyMemory(task->TargetBssid, creq->bssid, 6);
    task->TargetSsidLen = creq->ssid_len;
    RtlCopyMemory(task->TargetSsid, creq->ssid, sizeof(task->TargetSsid));

    /* And what the association is being made on, for the result. */
    task->AssocParams.FreqMhz        = creq->channel_freq;
    task->AssocParams.AuthAlgo       = creq->auth_algo;
    task->AssocParams.AkmSuite       = creq->akm_suite;
    task->AssocParams.CipherPairwise = creq->cipher_pairwise;
    task->AssocParams.CipherGroup    = creq->cipher_group;

    /* Marked pending, and the watchdog armed, before the device is
     * asked to do anything: the ASSOC_RESULT can land on another
     * processor the instant the request is posted, and it drops the
     * event if the task is not pending yet. */
    InterlockedExchange(&task->ConnectPending, 1);
    VwifiConnectArmWatchdog(task, VWIFI_CONNECT_WATCHDOG_MS);

    status = VwifiCtrlSendSync(Adapter, VWIFI_OP_CONNECT,
                              creq, reqLen, NULL, &outLen);
    if (status != NDIS_STATUS_SUCCESS) {
        /* Failed before the task ever started, so it is the OID that
         * reports it and no indication is sent -- returning a failure
         * status and indicating a completion for the same task is the
         * double completion this file's header warns about. Release the
         * claim so a later connect is not refused as already active.
         *
         * If the ASSOC_RESULT beat us here the claim is already gone
         * and the task is already completed; nothing to undo. */
        VWIFI_ERR("device rejected CONNECT: 0x%x", status);
        if (InterlockedCompareExchange(&task->ConnectPending, 0, 1) == 1) {
            if (task->Watchdog) {
                (VOID)NdisCancelTimerObject(task->Watchdog);
            }
            return status;
        }
        return VwifiWdiTaskAccepted(Req);
    }

    /* The device runs Auth/Assoc; ASSOC_RESULT completes the task. */
    return VwifiWdiTaskAccepted(Req);
}

NDIS_STATUS
VwifiHandleTaskDisconnect(_Inout_ PVWIFI_ADAPTER Adapter,
                          _In_ PNDIS_OID_REQUEST Req)
{
    PVWIFI_CONNECT_TASK task = Adapter->ConnectTask;
    struct vwifi_disconnect_req dreq = { 0 };
    NDIS_STATUS status;
    ULONG outLen = 0;

    if (!task) return NDIS_STATUS_RESOURCES;

    /* The port and transaction id DISCONNECT_COMPLETE must echo. Read
     * here, while the request buffer still exists. */
    task->DiscPortId        = Req->PortNumber;
    task->DiscWdiPortId     = VwifiGetWdiPortId(Req);
    task->DiscTransactionId = VwifiGetWdiTransactionId(Req);

    /* Why the host is tearing this down.
     *
     * A WPA2 association that succeeded -- status 0, aid 1 -- was
     * disconnected sixteen milliseconds later, before the component had
     * configured the peer for data and so before any EAPOL could be
     * sent. Nothing in the trace says what the OS objected to, and this
     * request is the only thing it sends that could carry the reason. */
    VwifiTraceOidInput("task disconnect", Req);

    dreq.reason_code = 3;   /* STA is leaving */

    InterlockedExchange(&task->DisconnectPending, 1);
    VwifiConnectArmWatchdog(task, VWIFI_DISCONNECT_WATCHDOG_MS);

    status = VwifiCtrlSendSync(Adapter, VWIFI_OP_DISCONNECT,
                               &dreq, sizeof(dreq), NULL, &outLen);
    if (status != NDIS_STATUS_SUCCESS) {
        /* Nothing is going to disconnect, so nothing will emit
         * DISCONNECTED. Complete it here rather than let the watchdog
         * discover it fifteen seconds from now -- and complete it as
         * success, because a port that could not even be asked to
         * disconnect is not one the OS should keep treating as
         * connected. */
        VWIFI_ERR("device rejected DISCONNECT: 0x%x", status);
        Adapter->Associated = FALSE;
        RtlZeroMemory(Adapter->Bssid, 6);
        VwifiIndicateDisconnectComplete(Adapter, NDIS_STATUS_SUCCESS);
        VwifiIndicateLinkState(Adapter, FALSE);
        return VwifiWdiTaskAccepted(Req);
    }

    /* The device emits DISCONNECTED, which drives the indications. */
    return VwifiWdiTaskAccepted(Req);
}

/* For the heartbeat in MiniportCheckForHangEx, which lives in driver.c
 * and cannot see this file's private task structure. */
ULONG
VwifiConnectTaskState(_In_ PVWIFI_ADAPTER Adapter)
{
    PVWIFI_CONNECT_TASK task = Adapter->ConnectTask;
    ULONG state = 0;

    if (!task) return 0;
    if (task->ConnectPending)    state |= VWIFI_TASK_CONNECT_PENDING;
    if (task->DisconnectPending) state |= VWIFI_TASK_DISCONNECT_PENDING;
    return state;
}

/* ============================================================
 * Task lifetime
 * ============================================================ */

NDIS_STATUS
VwifiConnectTaskCreate(_Inout_ PVWIFI_ADAPTER Adapter)
{
    NDIS_TIMER_CHARACTERISTICS timerChars = { 0 };
    NDIS_STATUS status;
    PVWIFI_CONNECT_TASK task = NdisAllocateMemoryWithTagPriority(
        Adapter->MiniportAdapterHandle, sizeof(*task),
        VWIFI_POOL_TAG, NormalPoolPriority);
    if (!task) return NDIS_STATUS_RESOURCES;
    RtlZeroMemory(task, sizeof(*task));

    timerChars.Header.Type     = NDIS_OBJECT_TYPE_TIMER_CHARACTERISTICS;
    timerChars.Header.Revision = NDIS_TIMER_CHARACTERISTICS_REVISION_1;
    timerChars.Header.Size     = NDIS_SIZEOF_TIMER_CHARACTERISTICS_REVISION_1;
    timerChars.AllocationTag   = VWIFI_POOL_TAG;
    timerChars.TimerFunction   = VwifiConnectWatchdog;
    timerChars.FunctionContext = Adapter;

    status = NdisAllocateTimerObject(Adapter->MiniportAdapterHandle,
                                     &timerChars, &task->Watchdog);
    if (status != NDIS_STATUS_SUCCESS) {
        /* Not fatal. Connect and disconnect still work; what is lost is
         * the guarantee that a device that stops answering costs one
         * failed task rather than a wedged Wi-Fi stack. */
        VWIFI_WARN("connect watchdog timer unavailable (0x%x)", status);
        task->Watchdog = NULL;
    }

    Adapter->ConnectTask = task;
    return NDIS_STATUS_SUCCESS;
}

VOID
VwifiConnectTaskDestroy(_Inout_ PVWIFI_ADAPTER Adapter)
{
    PVWIFI_CONNECT_TASK task = Adapter->ConnectTask;

    if (!task) return;

    /* Cancel and free before the memory the callback reads goes away.
     * NdisFreeTimerObject waits for a callback already running, which
     * is the half NdisCancelTimerObject cannot promise. */
    if (task->Watchdog) {
        (VOID)NdisCancelTimerObject(task->Watchdog);
        NdisFreeTimerObject(task->Watchdog);
        task->Watchdog = NULL;
    }

    NdisFreeMemoryWithTagPriority(Adapter->MiniportAdapterHandle,
                                  task, VWIFI_POOL_TAG);
    Adapter->ConnectTask = NULL;
}
