/*
 * vwifi — oids.c
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Phase 1.5 OID handling. Npcap switches a Wi-Fi adapter into
 * monitor mode using the Native 802.11 OID surface, which the
 * Microsoft WLAN component surfaces to our miniport:
 *
 *   OID_DOT11_CURRENT_OPERATION_MODE  (set) -> op mode NETWORK_MONITOR
 *                                     (the only route to monitor mode:
 *                                      WDI itself has no such op mode)
 *   OID_DOT11_CURRENT_CHANNEL         (set) -> channel number (2.4 GHz)
 *   OID_DOT11_CURRENT_FREQUENCY       (set) -> channel for 5 GHz
 *   OID_GEN_CURRENT_PACKET_FILTER     (set) -> raw data/mgmt bits
 *
 * We intercept exactly these and translate to control-ring commands.
 * Everything else returns NOT_SUPPORTED so the WLAN component's
 * default handling applies.
 */

#include "vwifi_drv.h"
#include "tlv_shim.h"
#include <windot11.h>

/* ============================================================
 * NDIS_OID_REQUEST.DATA is a union.
 *
 * The same field name means a different offset in each arm, so the
 * buffer must always be read through the arm that matches RequestType.
 * WDI's own OIDs are method requests, but the Native 802.11 ones are
 * plain queries and sets, and the two are mixed in this file.
 * ============================================================ */

static VOID
VwifiOidOutBuffer(_In_ PNDIS_OID_REQUEST Req,
                  _Outptr_result_maybenull_ PVOID *Buf,
                  _Out_ PULONG Len)
{
    if (Req->RequestType == NdisRequestMethod) {
        *Buf = Req->DATA.METHOD_INFORMATION.InformationBuffer;
        *Len = Req->DATA.METHOD_INFORMATION.OutputBufferLength;
    } else {
        *Buf = Req->DATA.QUERY_INFORMATION.InformationBuffer;
        *Len = Req->DATA.QUERY_INFORMATION.InformationBufferLength;
    }
}

static VOID
VwifiOidSetWritten(_Inout_ PNDIS_OID_REQUEST Req, _In_ ULONG Written)
{
    if (Req->RequestType == NdisRequestMethod) {
        Req->DATA.METHOD_INFORMATION.BytesWritten = Written;
    } else {
        Req->DATA.QUERY_INFORMATION.BytesWritten = Written;
    }
}

static VOID
VwifiOidSetNeeded(_Inout_ PNDIS_OID_REQUEST Req, _In_ ULONG Needed)
{
    if (Req->RequestType == NdisRequestMethod) {
        Req->DATA.METHOD_INFORMATION.BytesNeeded = Needed;
    } else {
        Req->DATA.QUERY_INFORMATION.BytesNeeded = Needed;
    }
}

/* The statuses the TLV generator and parser actually return. A bare
 * 0xc0010015 costs a trip to the headers at exactly the moment the log
 * is meant to be answering a question. */
static PCSTR
VwifiNdisStatusName(_In_ NDIS_STATUS Status)
{
    switch (Status) {
    case NDIS_STATUS_SUCCESS:           return "SUCCESS";
    case NDIS_STATUS_PENDING:           return "PENDING";
    case NDIS_STATUS_FAILURE:           return "FAILURE";
    case NDIS_STATUS_RESOURCES:         return "RESOURCES";
    case NDIS_STATUS_NOT_SUPPORTED:     return "NOT_SUPPORTED";
    case NDIS_STATUS_BUFFER_TOO_SHORT:  return "BUFFER_TOO_SHORT";
    case NDIS_STATUS_INVALID_LENGTH:    return "INVALID_LENGTH";
    case NDIS_STATUS_INVALID_DATA:      return "INVALID_DATA";
    case NDIS_STATUS_INVALID_OID:       return "INVALID_OID";
    case NDIS_STATUS_INVALID_PARAMETER: return "INVALID_PARAMETER";
    case NDIS_STATUS_BAD_VERSION:       return "BAD_VERSION";
    default:                            return "?";
    }
}

static PCSTR
VwifiOidRequestTypeName(_In_ NDIS_REQUEST_TYPE Type)
{
    switch (Type) {
    case NdisRequestQueryInformation:  return "query";
    case NdisRequestQueryStatistics:   return "query-stats";
    case NdisRequestSetInformation:    return "set";
    case NdisRequestMethod:            return "method";
    default:                           return "?";
    }
}

/* The OIDs the WLAN component sends while bringing an adapter up. Named
 * so the trace is readable without a copy of dot11wdi.h to hand -- when
 * an adapter is closed seconds after it opens, the OID it gave up on is
 * the whole story, and a bare hex value buries it. */
static PCSTR
VwifiOidName(_In_ NDIS_OID Oid)
{
    switch (Oid) {
    case OID_WDI_GET_ADAPTER_CAPABILITIES:  return "WDI_GET_ADAPTER_CAPABILITIES";
    case OID_WDI_SET_ADAPTER_CONFIGURATION: return "WDI_SET_ADAPTER_CONFIGURATION";
    case OID_WDI_TASK_CREATE_PORT:          return "WDI_TASK_CREATE_PORT";
    case OID_WDI_TASK_DELETE_PORT:          return "WDI_TASK_DELETE_PORT";
    case OID_WDI_TASK_OPEN:                 return "WDI_TASK_OPEN";
    case OID_WDI_TASK_CLOSE:                return "WDI_TASK_CLOSE";
    case OID_WDI_TASK_DOT11_RESET:          return "WDI_TASK_DOT11_RESET";
    case OID_WDI_TASK_SET_RADIO_STATE:      return "WDI_TASK_SET_RADIO_STATE";
    case OID_WDI_TASK_SCAN:                 return "WDI_TASK_SCAN";
    case OID_WDI_TASK_CONNECT:              return "WDI_TASK_CONNECT";
    case OID_WDI_TASK_DISCONNECT:           return "WDI_TASK_DISCONNECT";
    case OID_WDI_TASK_CHANGE_OPERATION_MODE: return "WDI_TASK_CHANGE_OPERATION_MODE";
    case OID_WDI_SET_POWER_STATE:           return "WDI_SET_POWER_STATE";
    case OID_WDI_SET_OPERATION_MODE:        return "WDI_SET_OPERATION_MODE";
    case OID_WDI_SET_RECEIVE_PACKET_FILTER: return "WDI_SET_RECEIVE_PACKET_FILTER";
    case OID_WDI_SET_MULTICAST_LIST:        return "WDI_SET_MULTICAST_LIST";
    case OID_WDI_SET_ADD_CIPHER_KEYS:       return "WDI_SET_ADD_CIPHER_KEYS";
    case OID_WDI_SET_DELETE_CIPHER_KEYS:    return "WDI_SET_DELETE_CIPHER_KEYS";
    case OID_WDI_SET_DEFAULT_KEY_ID:        return "WDI_SET_DEFAULT_KEY_ID";
    case OID_WDI_GET_STATISTICS:            return "WDI_GET_STATISTICS";
    case OID_WDI_GET_BSS_ENTRY_LIST:        return "WDI_GET_BSS_ENTRY_LIST";
    case OID_WDI_SET_ADVERTISEMENT_INFORMATION: return "WDI_SET_ADVERTISEMENT_INFORMATION";
    case OID_WDI_SET_CONNECTION_QUALITY:    return "WDI_SET_CONNECTION_QUALITY";
    case OID_WDI_SET_PRIVACY_EXEMPTION_LIST: return "WDI_SET_PRIVACY_EXEMPTION_LIST";
    case OID_DOT11_CURRENT_OPERATION_MODE:  return "DOT11_CURRENT_OPERATION_MODE";
    case OID_DOT11_CURRENT_CHANNEL:         return "DOT11_CURRENT_CHANNEL";
    case OID_DOT11_CURRENT_FREQUENCY:       return "DOT11_CURRENT_FREQUENCY";
    case OID_GEN_CURRENT_PACKET_FILTER:     return "GEN_CURRENT_PACKET_FILTER";
    default:                                return "";
    }
}

/* 2.4 GHz channel number -> center frequency (MHz). */
static USHORT
VwifiChannelToFreq(ULONG channel)
{
    if (channel >= 1 && channel <= 13) {
        return (USHORT)(2407 + channel * 5);
    }
    if (channel == 14) {
        return 2484;
    }
    return 0;
}

/* Map the OID's requested operation mode to our VWIFI_MODE_*. */
static NDIS_STATUS
VwifiHandleSetOpMode(_Inout_ PVWIFI_ADAPTER Adapter,
                     _In_ PNDIS_OID_REQUEST Req)
{
    PDOT11_CURRENT_OPERATION_MODE mode;

    if (Req->DATA.SET_INFORMATION.InformationBufferLength < sizeof(*mode)) {
        return NDIS_STATUS_INVALID_LENGTH;
    }
    mode = (PDOT11_CURRENT_OPERATION_MODE)
        Req->DATA.SET_INFORMATION.InformationBuffer;

    switch (mode->uCurrentOpMode) {
    case DOT11_OPERATION_MODE_NETWORK_MONITOR:
        VWIFI_INFO("OID: enter NETWORK_MONITOR mode");
        return VwifiSetOpMode(Adapter, VWIFI_MODE_MONITOR);

    case DOT11_OPERATION_MODE_EXTENSIBLE_STATION:
        VWIFI_INFO("OID: enter ExtSTA mode");
        return VwifiSetOpMode(Adapter, VWIFI_MODE_STA);

    default:
        VWIFI_WARN("OID: unsupported op mode 0x%x", mode->uCurrentOpMode);
        return NDIS_STATUS_NOT_SUPPORTED;
    }
}

static NDIS_STATUS
VwifiHandleSetChannel(_Inout_ PVWIFI_ADAPTER Adapter,
                      _In_ PNDIS_OID_REQUEST Req)
{
    PULONG channel;
    USHORT freq;

    if (Req->DATA.SET_INFORMATION.InformationBufferLength < sizeof(ULONG)) {
        return NDIS_STATUS_INVALID_LENGTH;
    }
    channel = (PULONG)Req->DATA.SET_INFORMATION.InformationBuffer;

    freq = VwifiChannelToFreq(*channel);
    if (freq == 0) {
        VWIFI_WARN("OID: invalid channel %u", *channel);
        return NDIS_STATUS_INVALID_DATA;
    }
    return VwifiSetChannel(Adapter, freq);
}

static NDIS_STATUS
VwifiHandleSetFrequency(_Inout_ PVWIFI_ADAPTER Adapter,
                        _In_ PNDIS_OID_REQUEST Req)
{
    PULONG freq;

    if (Req->DATA.SET_INFORMATION.InformationBufferLength < sizeof(ULONG)) {
        return NDIS_STATUS_INVALID_LENGTH;
    }
    freq = (PULONG)Req->DATA.SET_INFORMATION.InformationBuffer;
    /* OID carries frequency in kHz for some drivers, MHz for others;
     * Native Wi-Fi uses MHz here. Clamp to USHORT. */
    return VwifiSetChannel(Adapter, (USHORT)*freq);
}

static NDIS_STATUS
VwifiHandleSetPacketFilter(_Inout_ PVWIFI_ADAPTER Adapter,
                           _In_ PNDIS_OID_REQUEST Req)
{
    ULONG filter;
    ULONG raw = 0;

    if (Req->DATA.SET_INFORMATION.InformationBufferLength < sizeof(ULONG)) {
        return NDIS_STATUS_INVALID_LENGTH;
    }
    filter = *(PULONG)Req->DATA.SET_INFORMATION.InformationBuffer;

    /* Translate the Native 802.11 raw packet-filter bits into our
     * device's raw filter mask. If neither raw bit is set, we're not
     * in a raw-capture configuration; leave the device filter at 0
     * (meaning "everything" for monitor, ignored for STA). */
    if (filter & NDIS_PACKET_TYPE_802_11_RAW_DATA) {
        raw |= VWIFI_RAW_F_DATA;
    }
    if (filter & NDIS_PACKET_TYPE_802_11_RAW_MGMT) {
        raw |= VWIFI_RAW_F_MGMT;
    }

    if (raw != 0) {
        return VwifiSetRawFilter(Adapter, raw);
    }
    /* Not a raw filter change we care about — let default handling
     * proceed by reporting success without touching the device. */
    return NDIS_STATUS_SUCCESS;
}

/* ============================================================
 * OID_WDI_GET_ADAPTER_CAPABILITIES
 *
 * The first thing the WLAN component asks for once OpenAdapter has
 * completed, and the answer decides whether the adapter is usable at
 * all. Returning NDIS_STATUS_NOT_SUPPORTED -- which is what the
 * dispatcher's default arm did, silently, because this OID had no case
 * and VwifiTlvGenerateAdapterCapabilities had no caller anywhere in the
 * tree -- tells it nothing about the radio, so it closes the adapter
 * again immediately. From the outside that looks like OpenAdapter
 * succeeding and CloseAdapter arriving a fraction of a millisecond
 * later with nothing in between.
 *
 * Unlike the WDI *tasks*, this one answers in the OID's own output
 * buffer rather than through an indication. The reply is the same
 * [WDI_MESSAGE_HEADER][TLV blob] shape as everything else, and the
 * generator has already reserved the header at the front of the blob,
 * so the header is filled in place -- see VwifiSendWdiIndication for
 * why prepending a second one is wrong.
 * ============================================================ */
static NDIS_STATUS
VwifiHandleGetAdapterCapabilities(_Inout_ PVWIFI_ADAPTER Adapter,
                                  _In_ PNDIS_OID_REQUEST Req)
{
    PVOID       blob    = NULL;
    ULONG       blobLen = 0;
    PVOID       out     = NULL;
    ULONG       outLen  = 0;
    NDIS_STATUS status;
    WDI_MESSAGE_HEADER *hdr;
    UINT32      transactionId = WDI_TRANSACTION_ID_UNSOLICIT;
    WDI_PORT_ID portId = 0;

    /* The capabilities we report are built from what the device told us
     * during GET_CAPS in VwifiHwStart. Without that there is nothing
     * honest to say. */
    if (!Adapter->CapsValid) {
        VWIFI_ERR("GET_ADAPTER_CAPABILITIES before GET_CAPS completed");
        return NDIS_STATUS_FAILURE;
    }

    /* Echo the request's port and transaction id when there is an input
     * buffer to read them from. A method request carries one; a plain
     * query does not. */
    if (Req->RequestType == NdisRequestMethod &&
        Req->DATA.METHOD_INFORMATION.InformationBuffer != NULL &&
        Req->DATA.METHOD_INFORMATION.InputBufferLength >=
            sizeof(WDI_MESSAGE_HEADER)) {
        const WDI_MESSAGE_HEADER *in = (const WDI_MESSAGE_HEADER *)
            Req->DATA.METHOD_INFORMATION.InformationBuffer;
        transactionId = in->TransactionId;
        portId        = in->PortId;
    }

    status = VwifiTlvGenerateAdapterCapabilities(
                 Adapter->WdiPeerVersion, &Adapter->Caps,
                 Adapter->CurrentMac, &blob, &blobLen);
    if (status != NDIS_STATUS_SUCCESS) {
        VWIFI_ERR("capabilities generate failed 0x%08x %s", status,
                  VwifiNdisStatusName(status));
        if (status == NDIS_STATUS_INVALID_DATA) {
            /* The generator says only that the message is bad, never
             * which field. Every previous instance has been a mandatory
             * container left as its zeroed self, so that is where to
             * look: WABIModel.xml, any containerRef without
             * optional="true". */
            VWIFI_ERR("  a mandatory container is empty -- check the "
                      "containerRefs without optional=\"true\" under "
                      "WDI_GET_ADAPTER_CAPABILITIES in WABIModel.xml");
        }
        return status;
    }

    if (blobLen < sizeof(WDI_MESSAGE_HEADER)) {
        VWIFI_ERR("capabilities blob %u bytes, shorter than the header "
                  "space it was asked to reserve", blobLen);
        VwifiTlvFreeGenerated(blob);
        return NDIS_STATUS_FAILURE;
    }

    hdr = (WDI_MESSAGE_HEADER *)blob;
    RtlZeroMemory(hdr, sizeof(*hdr));
    hdr->PortId        = portId;
    hdr->Status        = NDIS_STATUS_SUCCESS;
    hdr->TransactionId = transactionId;
    hdr->IhvSpecificId = 0;

    VwifiOidOutBuffer(Req, &out, &outLen);
    if (out == NULL || outLen < blobLen) {
        /* NDIS convention: say how much is needed and let the OS come
         * back with a buffer that size. Not an error worth logging as
         * one -- a first probe with a short buffer is normal. */
        VwifiOidSetNeeded(Req, blobLen);
        VwifiOidSetWritten(Req, 0);
        VwifiTlvFreeGenerated(blob);
        return NDIS_STATUS_BUFFER_TOO_SHORT;
    }

    RtlCopyMemory(out, blob, blobLen);
    VwifiOidSetWritten(Req, blobLen);
    VwifiTlvFreeGenerated(blob);

    VWIFI_INFO("OID: reported adapter capabilities (%u bytes)", blobLen);
    return NDIS_STATUS_SUCCESS;
}

/* ============================================================
 * "No TLV data needed, header is sufficient"
 *
 * WABIModel.xml uses that exact phrase for the FromIhv side of most of
 * the bring-up messages — SET_ADAPTER_CONFIGURATION, TASK_CREATE_PORT,
 * TASK_DELETE_PORT, TASK_SET_RADIO_STATE, TASK_OPEN, TASK_CLOSE. The
 * reply is a bare WDI_MESSAGE_HEADER echoing the request's port and
 * transaction id, with the outcome in its Status field.
 *
 * Tolerant about the output buffer on purpose: a set that is not
 * expected to return anything may well arrive with no output buffer at
 * all, and refusing that with BUFFER_TOO_SHORT would fail a request
 * that actually succeeded.
 * ============================================================ */
static NDIS_STATUS
VwifiWdiAckHeaderOnly(_In_ PNDIS_OID_REQUEST Req,
                      _In_ NDIS_STATUS MessageStatus)
{
    PVOID  out    = NULL;
    ULONG  outLen = 0;
    WDI_MESSAGE_HEADER *hdr;
    UINT32 transactionId = WDI_TRANSACTION_ID_UNSOLICIT;
    WDI_PORT_ID portId = 0;

    if (Req->RequestType == NdisRequestMethod &&
        Req->DATA.METHOD_INFORMATION.InformationBuffer != NULL &&
        Req->DATA.METHOD_INFORMATION.InputBufferLength >=
            sizeof(WDI_MESSAGE_HEADER)) {
        const WDI_MESSAGE_HEADER *in = (const WDI_MESSAGE_HEADER *)
            Req->DATA.METHOD_INFORMATION.InformationBuffer;
        transactionId = in->TransactionId;
        portId        = in->PortId;
    }

    VwifiOidOutBuffer(Req, &out, &outLen);
    if (out == NULL || outLen < sizeof(WDI_MESSAGE_HEADER)) {
        VwifiOidSetWritten(Req, 0);
        return NDIS_STATUS_SUCCESS;
    }

    hdr = (WDI_MESSAGE_HEADER *)out;
    RtlZeroMemory(hdr, sizeof(*hdr));
    hdr->PortId        = portId;
    hdr->Status        = MessageStatus;
    hdr->TransactionId = transactionId;
    hdr->IhvSpecificId = 0;

    VwifiOidSetWritten(Req, sizeof(WDI_MESSAGE_HEADER));
    return NDIS_STATUS_SUCCESS;
}

/* ============================================================
 * OID_WDI_SET_ADAPTER_CONFIGURATION
 *
 * Sent immediately after the capabilities are accepted, and returning
 * NOT_SUPPORTED for it closes the adapter just as surely as failing the
 * capabilities did.
 *
 * Of everything the message carries, only the configured MAC address
 * maps onto anything this device has; the rest is firmware policy —
 * P2P group-owner reset, unreachability detection, NLO scan mode,
 * PLDR — with no equivalent here. Silently ignoring the MAC would mean
 * frames going out with an address the OS did not ask for, so it is
 * pushed down to the device.
 *
 * A parse failure is logged and does not fail the OID. The message is
 * mostly settings we do not implement, and refusing the whole request
 * because an unrelated container did not decode would trade a cosmetic
 * problem for a dead adapter.
 * ============================================================ */
static NDIS_STATUS
VwifiHandleSetAdapterConfiguration(_Inout_ PVWIFI_ADAPTER Adapter,
                                   _In_ PNDIS_OID_REQUEST Req)
{
    PVOID       tlv    = NULL;
    ULONG       tlvLen = 0;
    UCHAR       mac[6];
    BOOLEAN     macPresent = FALSE;
    NDIS_STATUS status;

    status = VwifiGetTlvPayload(Req, &tlv, &tlvLen);
    if (status != NDIS_STATUS_SUCCESS) {
        VWIFI_WARN("SET_ADAPTER_CONFIGURATION: no TLV payload (0x%08x %s)",
                   status, VwifiNdisStatusName(status));
        return VwifiWdiAckHeaderOnly(Req, NDIS_STATUS_SUCCESS);
    }

    status = VwifiTlvParseAdapterConfiguration(Adapter->WdiPeerVersion,
                                               tlv, tlvLen,
                                               mac, &macPresent);
    if (status != NDIS_STATUS_SUCCESS) {
        VWIFI_WARN("SET_ADAPTER_CONFIGURATION: parse failed 0x%08x %s; "
                   "accepting anyway", status, VwifiNdisStatusName(status));
        return VwifiWdiAckHeaderOnly(Req, NDIS_STATUS_SUCCESS);
    }

    if (macPresent) {
        ULONG out_len = 0;

        VWIFI_INFO("OID: configured MAC %02x:%02x:%02x:%02x:%02x:%02x",
                   mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
        RtlCopyMemory(Adapter->CurrentMac, mac, 6);
        status = VwifiCtrlSendSync(Adapter, VWIFI_OP_SET_STA_MAC,
                                   mac, 6, NULL, &out_len);
        if (status != NDIS_STATUS_SUCCESS) {
            VWIFI_ERR("SET_STA_MAC failed 0x%08x %s", status,
                      VwifiNdisStatusName(status));
            return VwifiWdiAckHeaderOnly(Req, status);
        }
    } else {
        VWIFI_INFO("OID: adapter configuration accepted (no MAC change)");
    }

    return VwifiWdiAckHeaderOnly(Req, NDIS_STATUS_SUCCESS);
}

/* Tasks return NDIS_STATUS_INDICATION_REQUIRED, never SUCCESS.
 *
 * A WDI task is accepted by the OID and completed by an indication.
 * INDICATION_REQUIRED is how the miniport says exactly that: the
 * request is taken, watch for the completion. SUCCESS says the opposite
 * -- finished, nothing more coming -- and then an indication arrives
 * for a task the component has already closed out.
 *
 * wdi_scan.c and wdi_connect.c have always returned
 * INDICATION_REQUIRED. Every task handler added later returned SUCCESS,
 * which is a double completion: WDI runs one task at a time per port,
 * so a task the component has closed out early and then sees completed
 * again is a state machine being told something it cannot place.
 *
 * No observed failure is attributed to this -- it is being corrected
 * because it is wrong, not because a trace pointed at it.
 */

/* ============================================================
 * OID_WDI_TASK_CREATE_PORT / OID_WDI_TASK_DELETE_PORT
 *
 * The last step of adapter bring-up, and the first thing that is a
 * *task* rather than a get or a set: it is answered by an indication
 * (NDIS_STATUS_WDI_INDICATION_CREATE_PORT_COMPLETE), not by the OID's
 * output buffer, and the OID itself just returns success to say the
 * task was accepted.
 *
 * A port in WDI is the host's handle on one virtual interface. It
 * assigns the id -- it is in the request's WDI_MESSAGE_HEADER, not in
 * the TLV body -- so there is nothing for us to allocate or hand back.
 * The body says what the port is for: which operation modes the host
 * may configure on it later, and the NDIS port number it will appear
 * under.
 *
 * A single-radio virtual device has one port and no per-port state to
 * keep, so creating one is bookkeeping. The device is already running
 * by this point; the ports the host may go on to add are the
 * component's abstraction, not the device's.
 * ============================================================ */
static NDIS_STATUS
VwifiHandleTaskCreatePort(_Inout_ PVWIFI_ADAPTER Adapter,
                          _In_ PNDIS_OID_REQUEST Req)
{
    PVOID       tlv    = NULL;
    ULONG       tlvLen = 0;
    ULONG       opModeMask = 0;
    ULONG       ndisPort   = 0;
    UCHAR       mac[6];
    BOOLEAN     macPresent = FALSE;
    NDIS_STATUS status;

    status = VwifiGetTlvPayload(Req, &tlv, &tlvLen);
    if (status != NDIS_STATUS_SUCCESS) {
        VWIFI_ERR("CREATE_PORT: no TLV payload (0x%08x %s)",
                  status, VwifiNdisStatusName(status));
        return status;
    }

    status = VwifiTlvParseCreatePort(Adapter->WdiPeerVersion, tlv, tlvLen,
                                     &opModeMask, &ndisPort,
                                     mac, &macPresent);
    if (status != NDIS_STATUS_SUCCESS) {
        VWIFI_ERR("CREATE_PORT: parse failed 0x%08x %s",
                  status, VwifiNdisStatusName(status));
        return status;
    }

    VWIFI_INFO("OID: create port: wdi port 0x%04x, ndis port %u "
               "(req ndis %u), opmode mask 0x%x",
               VwifiGetWdiPortId(Req), ndisPort,
               Req->PortNumber, opModeMask);

    /* An explicit address means the host wants this port on a MAC other
     * than the adapter's own. The device has exactly one station
     * address, so honour it rather than transmit under an address the
     * host did not choose. */
    if (macPresent) {
        ULONG out_len = 0;

        VWIFI_INFO("OID:   port MAC %02x:%02x:%02x:%02x:%02x:%02x",
                   mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
        RtlCopyMemory(Adapter->CurrentMac, mac, 6);
        status = VwifiCtrlSendSync(Adapter, VWIFI_OP_SET_STA_MAC,
                                   mac, 6, NULL, &out_len);
        if (status != NDIS_STATUS_SUCCESS) {
            VWIFI_ERR("CREATE_PORT: SET_STA_MAC failed 0x%08x %s",
                      status, VwifiNdisStatusName(status));
            /* Report the failure through the completion, not the return
             * value: the task was accepted, it is its outcome that is
             * bad, and the host matches that on the transaction id. */
            VwifiSendWdiIndication(
                Adapter, VwifiGetWdiPortId(Req), Req->PortNumber,
                NDIS_STATUS_WDI_INDICATION_CREATE_PORT_COMPLETE,
                status, VwifiGetWdiTransactionId(Req), NULL, 0);
            return NDIS_STATUS_INDICATION_REQUIRED;
        }
    }

    /* ndisPort, not Req->PortNumber.
     *
     * The two are not the same and the difference is the whole point of
     * this assignment. Req->PortNumber is the NDIS port the CREATE_PORT
     * request arrived on, and CREATE_PORT is adapter-scoped -- there is
     * no port yet -- so it is zero. ndisPort comes out of the request's
     * WDI_TLV_PORT_ATTRIBUTES and is the number the OS will address the
     * station port by from here on.
     *
     * Recording the wrong one sent every subsequent link-state
     * indication to the default port instead of the station's, so the
     * station port was never told what its media state was. */
    Adapter->NdisPortNumber = ndisPort;
    Adapter->PortCreated    = TRUE;

    /* State the link, now that there is a port for it to belong to.
     * Disconnected is the truth here and it is a different thing from
     * never having said -- NDIS starts an adapter with no media state
     * asserted, and this driver only ever spoke up on association or
     * disconnect, so until something connected the OS had heard
     * nothing at all. */
    VwifiIndicateLinkState(Adapter, FALSE);

    /* The completion carries a mandatory PortAttributes container --
     * the created port's MAC and number. It is NOT header-only: that
     * describes WDI_TASK_CREATE_PORT's own results message, which is a
     * different thing from WDI_INDICATION_CREATE_PORT_COMPLETE. See the
     * comment on VwifiTlvGenerateCreatePortComplete. */
    {
        PVOID blob    = NULL;
        ULONG blobLen = 0;

        status = VwifiTlvGenerateCreatePortComplete(
                     Adapter->WdiPeerVersion, Adapter->CurrentMac,
                     ndisPort, &blob, &blobLen);
        if (status != NDIS_STATUS_SUCCESS) {
            VWIFI_ERR("CREATE_PORT: completion generate failed 0x%08x %s",
                      status, VwifiNdisStatusName(status));
            return status;
        }

        VwifiSendWdiIndication(Adapter, VwifiGetWdiPortId(Req),
                               Req->PortNumber,
                               NDIS_STATUS_WDI_INDICATION_CREATE_PORT_COMPLETE,
                               NDIS_STATUS_SUCCESS,
                               VwifiGetWdiTransactionId(Req),
                               blob, blobLen);
        VwifiTlvFreeGenerated(blob);
    }
    return NDIS_STATUS_INDICATION_REQUIRED;
}

static NDIS_STATUS
VwifiHandleTaskDeletePort(_Inout_ PVWIFI_ADAPTER Adapter,
                          _In_ PNDIS_OID_REQUEST Req)
{
    PVOID       tlv    = NULL;
    ULONG       tlvLen = 0;
    ULONG       portNumber = 0;
    NDIS_STATUS status;

    /* Parsed for the log only. The port to delete is identified by the
     * header's port id like every other port-scoped message; the body's
     * PortNumber is the NDIS one. A parse failure is not worth failing
     * a teardown over. */
    status = VwifiGetTlvPayload(Req, &tlv, &tlvLen);
    if (status == NDIS_STATUS_SUCCESS) {
        status = VwifiTlvParseDeletePort(Adapter->WdiPeerVersion,
                                         tlv, tlvLen, &portNumber);
        if (status != NDIS_STATUS_SUCCESS) {
            VWIFI_WARN("DELETE_PORT: parse failed 0x%08x %s; deleting anyway",
                       status, VwifiNdisStatusName(status));
        }
    }

    VWIFI_INFO("OID: delete port %u (ndis port %u)",
               Req->PortNumber, portNumber);

    Adapter->PortCreated = FALSE;

    VwifiSendWdiIndication(Adapter, VwifiGetWdiPortId(Req), Req->PortNumber,
                           NDIS_STATUS_WDI_INDICATION_DELETE_PORT_COMPLETE,
                           NDIS_STATUS_SUCCESS,
                           VwifiGetWdiTransactionId(Req),
                           NULL, 0);
    return NDIS_STATUS_INDICATION_REQUIRED;
}

/* ============================================================
 * OID_WDI_GET_STATISTICS
 *
 * The last message in the bring-up trace still answered NOT_SUPPORTED.
 * Both of its reply containers are mandatory, so it could not be
 * answered with a bare header the way the sets can -- which is why it
 * stayed unhandled while everything around it got fixed.
 *
 * Answered the same way as the capabilities: generated into the OID's
 * own output buffer, not indicated. Whether the WLAN component actually
 * minds a failed statistics query is unknown; it is the only thing left
 * being refused, and leaving one unexplained refusal in a trace that is
 * being read for exactly this is not worth the ambiguity.
 * ============================================================ */
static NDIS_STATUS
VwifiHandleGetStatistics(_Inout_ PVWIFI_ADAPTER Adapter,
                         _In_ PNDIS_OID_REQUEST Req)
{
    PVOID       blob    = NULL;
    ULONG       blobLen = 0;
    PVOID       out     = NULL;
    ULONG       outLen  = 0;
    NDIS_STATUS status;
    WDI_MESSAGE_HEADER *hdr;

    status = VwifiTlvGenerateStatistics(Adapter->WdiPeerVersion,
                                        &blob, &blobLen);
    if (status != NDIS_STATUS_SUCCESS) {
        VWIFI_ERR("statistics generate failed 0x%08x %s",
                  status, VwifiNdisStatusName(status));
        return status;
    }

    if (blobLen < sizeof(WDI_MESSAGE_HEADER)) {
        VwifiTlvFreeGenerated(blob);
        return NDIS_STATUS_FAILURE;
    }

    hdr = (WDI_MESSAGE_HEADER *)blob;
    RtlZeroMemory(hdr, sizeof(*hdr));
    hdr->PortId        = VwifiGetWdiPortId(Req);
    hdr->Status        = NDIS_STATUS_SUCCESS;
    hdr->TransactionId = VwifiGetWdiTransactionId(Req);

    VwifiOidOutBuffer(Req, &out, &outLen);
    if (out == NULL || outLen < blobLen) {
        VwifiOidSetNeeded(Req, blobLen);
        VwifiOidSetWritten(Req, 0);
        VwifiTlvFreeGenerated(blob);
        return NDIS_STATUS_BUFFER_TOO_SHORT;
    }

    RtlCopyMemory(out, blob, blobLen);
    VwifiOidSetWritten(Req, blobLen);
    VwifiTlvFreeGenerated(blob);

    VWIFI_INFO("OID: reported statistics (%u bytes)", blobLen);
    return NDIS_STATUS_SUCCESS;
}

/* ============================================================
 * OID_WDI_TASK_DOT11_RESET
 *
 * Sent immediately after the port is created, and again a moment later.
 * Returning NOT_SUPPORTED leaves the port in a state the component will
 * not connect from -- the trace showed it reach TASK_SCAN and stop
 * there, never issuing TASK_CONNECT.
 *
 * "Reset" here is the 802.11 MLME-RESET, not a hardware reset: put the
 * port back to a known state and, if SetDefaultMIB is set, restore its
 * MIB defaults. For this device that means dropping any association and
 * returning to idle, which the disconnect opcode already does. The
 * rings, the interrupt and the device itself stay up -- resetting those
 * would be a much bigger hammer than asked for, and would take the
 * adapter down with it.
 * ============================================================ */
static NDIS_STATUS
VwifiHandleTaskDot11Reset(_Inout_ PVWIFI_ADAPTER Adapter,
                          _In_ PNDIS_OID_REQUEST Req)
{
    PVOID       tlv    = NULL;
    ULONG       tlvLen = 0;
    BOOLEAN     setDefaultMib = FALSE;
    UCHAR       mac[6];
    BOOLEAN     macPresent = FALSE;
    NDIS_STATUS status;

    status = VwifiGetTlvPayload(Req, &tlv, &tlvLen);
    if (status == NDIS_STATUS_SUCCESS) {
        status = VwifiTlvParseDot11Reset(Adapter->WdiPeerVersion,
                                         tlv, tlvLen, &setDefaultMib,
                                         mac, &macPresent);
        if (status != NDIS_STATUS_SUCCESS) {
            VWIFI_WARN("DOT11_RESET: parse failed 0x%08x %s; resetting anyway",
                       status, VwifiNdisStatusName(status));
        }
    }

    VWIFI_INFO("OID: dot11 reset (defaultMIB=%u, mac=%u)",
               setDefaultMib, macPresent);

    /* An explicit address is the port's new MAC across the reset. */
    if (macPresent) {
        ULONG out_len = 0;

        RtlCopyMemory(Adapter->CurrentMac, mac, 6);
        (VOID)VwifiCtrlSendSync(Adapter, VWIFI_OP_SET_STA_MAC,
                                mac, 6, NULL, &out_len);
    }

    /* Back to idle. Best-effort: the device rejects a disconnect when
     * nothing is associated, which is the common case here and not a
     * reason to fail the reset. */
    {
        ULONG out_len = 0;
        struct vwifi_disconnect_req dreq;

        RtlZeroMemory(&dreq, sizeof(dreq));
        (VOID)VwifiCtrlSendSync(Adapter, VWIFI_OP_DISCONNECT,
                                &dreq, sizeof(dreq), NULL, &out_len);
    }

    /* And our own idea of the association, which the device call above
     * does not touch.
     *
     * OID_WDI_TASK_DOT11_RESET is documented as "Reset the port's MAC
     * entity to its initial state" and "Set the port state to INIT
     * before completing the dot11 reset operation". A driver that tells
     * the device to disconnect and goes on believing it is associated
     * has reset the hardware and not itself, and the next thing to read
     * Adapter->Associated gets an answer that is a reset old.
     *
     * The link state goes with it: INIT is a disconnected port, and
     * saying so is free. */
    Adapter->Associated = FALSE;
    RtlZeroMemory(Adapter->Bssid, 6);
    VwifiIndicateLinkState(Adapter, FALSE);

    /* The BSS cache deliberately survives this.
     *
     * It is tempting to read "reset the MAC entity to its initial
     * state" as covering the scan results too, and dropping them here
     * would look tidy. It would also be the end of connecting: the
     * host resets the port and then issues its connect within about ten
     * milliseconds, far too soon for a fresh scan to have found
     * anything, so the BSS the connect is built from has to be one we
     * were already holding.
     *
     * We advertise WDI_STATION_CAPABILITIES.BSSListCachemanagement =
     * TRUE precisely so the host will come and ask us for it via
     * OID_WDI_GET_BSS_ENTRY_LIST at that moment. Flushing here would
     * make that answer empty. Only OID_WDI_SET_FLUSH_BSS_ENTRY, which
     * is the host explicitly asking for a fresh view, may clear it. */

    VwifiSendWdiIndication(Adapter, VwifiGetWdiPortId(Req), Req->PortNumber,
                           NDIS_STATUS_WDI_INDICATION_DOT11_RESET_COMPLETE,
                           NDIS_STATUS_SUCCESS,
                           VwifiGetWdiTransactionId(Req),
                           NULL, 0);
    return NDIS_STATUS_INDICATION_REQUIRED;
}

/* ============================================================
 * OID_WDI_SET_RECEIVE_PACKET_FILTER / OID_WDI_SET_MULTICAST_LIST
 *
 * Both are plain sets answered with a bare header, and both were
 * returning NOT_SUPPORTED. The packet filter is the one that matters:
 * it says which received frames the component wants indicated, and a
 * port that never gets one configured has no reason to expect data.
 *
 * The device's own filter is the raw-capture mask used by monitor mode,
 * which is a different thing -- these bits are NDIS packet types, not
 * 802.11 frame classes. In STA mode the device already delivers exactly
 * the frames addressed to us, so there is nothing to push down and
 * accepting the filter is honest rather than lazy. Monitor mode still
 * goes through OID_GEN_CURRENT_PACKET_FILTER above, which does drive
 * the device.
 * ============================================================ */
static NDIS_STATUS
VwifiHandleSetReceivePacketFilter(_Inout_ PVWIFI_ADAPTER Adapter,
                                  _In_ PNDIS_OID_REQUEST Req)
{
    PVOID       tlv    = NULL;
    ULONG       tlvLen = 0;
    ULONG       filter = 0;
    NDIS_STATUS status;

    status = VwifiGetTlvPayload(Req, &tlv, &tlvLen);
    if (status == NDIS_STATUS_SUCCESS) {
        status = VwifiTlvParseReceivePacketFilter(Adapter->WdiPeerVersion,
                                                  tlv, tlvLen, &filter);
        if (status != NDIS_STATUS_SUCCESS) {
            VWIFI_WARN("RECEIVE_PACKET_FILTER: parse failed 0x%08x %s",
                       status, VwifiNdisStatusName(status));
        }
    }

    Adapter->WdiPacketFilter = filter;
    VWIFI_INFO("OID: receive packet filter 0x%08x", filter);

    return VwifiWdiAckHeaderOnly(Req, NDIS_STATUS_SUCCESS);
}

static NDIS_STATUS
VwifiHandleSetMulticastList(_Inout_ PVWIFI_ADAPTER Adapter,
                            _In_ PNDIS_OID_REQUEST Req)
{
    UNREFERENCED_PARAMETER(Adapter);

    /* The list itself is optional in the model and this device has no
     * multicast filter to program -- the medium delivers what it
     * delivers and the stack drops what it does not want. Accepted
     * rather than parsed, because storing a list nothing consults would
     * only suggest it does something. */
    VWIFI_INFO("OID: multicast list accepted (device has no filter)");
    return VwifiWdiAckHeaderOnly(Req, NDIS_STATUS_SUCCESS);
}

/* ============================================================
 * The rest of the station surface
 *
 * An audit of dot11wdi.h against this dispatcher found 15 of 66 WDI
 * OIDs handled. Most of the remainder describe features this device
 * genuinely lacks -- P2P, SoftAP, WoL, protocol and TCP offloads, FTM,
 * IHV extensions, device services -- and NOT_SUPPORTED is the correct
 * answer for those.
 *
 * These are not those. Every one of them is on the path a station
 * takes to associate, and every one was being refused. They are
 * gathered here because the model gives them all the same shape: the
 * reply is a bare WDI_MESSAGE_HEADER, so accepting one costs a line.
 * Refusing them cost a test cycle each to discover.
 *
 * What each one means for a device with no firmware to configure:
 *
 *   SET_PRIVACY_EXEMPTION_LIST  which ethertypes bypass encryption --
 *       EAPOL, so the 4-way handshake can run before keys exist. The
 *       device never encrypts on the host's behalf, so every frame is
 *       already exempt.
 *   SET_DEFAULT_KEY_ID          which group key index transmits. The
 *       device tracks key indices itself, from SET_KEY.
 *   SET_ASSOCIATION_PARAMETERS  per-BSSID association hints.
 *   SET_CONNECTION_QUALITY      roaming thresholds the host suggests.
 *   SET_ADVERTISEMENT_INFORMATION  what to advertise in probes.
 *   SET_POWER_STATE             its one reply container is optional, so
 *       a header is a complete answer. Nothing here sleeps.
 *
 * Accepting a setting this device does not implement is not the same
 * as pretending it works: none of these change what the radio does,
 * and refusing them stops the connect before it starts.
 * ============================================================ */
static NDIS_STATUS
VwifiHandleAcceptedSet(_Inout_ PVWIFI_ADAPTER Adapter,
                       _In_ PNDIS_OID_REQUEST Req,
                       _In_ PCSTR What)
{
    UNREFERENCED_PARAMETER(Adapter);
    VWIFI_INFO("OID: %s accepted (no device state to change)", What);
    return VwifiWdiAckHeaderOnly(Req, NDIS_STATUS_SUCCESS);
}

/* ============================================================
 * OID_WDI_ABORT_TASK
 *
 * Cancel whatever task is running on the port. For this driver that
 * means the scan -- it is the only task that outlives its OID.
 *
 * VwifiHandleTaskScanAbort has existed since the scan was written and
 * nothing ever called it, because this OID was never dispatched. A
 * component that wants to stop scanning so it can connect asks here,
 * gets NOT_SUPPORTED, and the scan it is waiting on never ends. Scans
 * repeating forever while a connect never starts is what that looks
 * like from the trace.
 * ============================================================ */
static NDIS_STATUS
VwifiHandleAbortTask(_Inout_ PVWIFI_ADAPTER Adapter,
                     _In_ PNDIS_OID_REQUEST Req)
{
    VWIFI_INFO("OID: abort task");
    (VOID)VwifiHandleTaskScanAbort(Adapter);
    return VwifiWdiAckHeaderOnly(Req, NDIS_STATUS_SUCCESS);
}

/* ============================================================
 * OID_WDI_TASK_SET_RADIO_STATE
 *
 * A task, so it completes by indication. The capabilities report the
 * radio as always enabled and this device has no way to turn it off,
 * so the state is accepted and reported complete rather than acted on.
 * ============================================================ */
static NDIS_STATUS
VwifiHandleTaskSetRadioState(_Inout_ PVWIFI_ADAPTER Adapter,
                             _In_ PNDIS_OID_REQUEST Req)
{
    VWIFI_INFO("OID: set radio state (device radio is always on)");

    VwifiSendWdiIndication(Adapter, VwifiGetWdiPortId(Req), Req->PortNumber,
                           NDIS_STATUS_WDI_INDICATION_SET_RADIO_STATE_COMPLETE,
                           NDIS_STATUS_SUCCESS,
                           VwifiGetWdiTransactionId(Req),
                           NULL, 0);
    return NDIS_STATUS_INDICATION_REQUIRED;
}

/* ============================================================
 * The real OID dispatcher — replaces the Phase-1 blanket stub.
 * ============================================================ */
_Use_decl_annotations_
NDIS_STATUS
VwifiOidRequest(
    NDIS_HANDLE MiniportAdapterContext,
    PNDIS_OID_REQUEST OidRequest)
{
    PVWIFI_ADAPTER adapter = (PVWIFI_ADAPTER)MiniportAdapterContext;
    NDIS_OID oid;

    /* Trace every request before dispatching.
     *
     * This is not incidental logging. The WLAN component's reaction to
     * an OID it does not like is to close the adapter, with nothing
     * logged on its side and nothing on ours if the OID falls through
     * to the default arm -- which is how a missing
     * GET_ADAPTER_CAPABILITIES handler presented as OpenAdapter
     * succeeding and CloseAdapter arriving 130 microseconds later with
     * an empty gap between them. The gap is the bug report; keep it
     * full. */
    switch (OidRequest->RequestType) {
    case NdisRequestSetInformation: oid = OidRequest->DATA.SET_INFORMATION.Oid;    break;
    case NdisRequestMethod:         oid = OidRequest->DATA.METHOD_INFORMATION.Oid; break;
    default:                        oid = OidRequest->DATA.QUERY_INFORMATION.Oid;  break;
    }
    VWIFI_INFO("OID: %s 0x%08x %s",
               VwifiOidRequestTypeName(OidRequest->RequestType),
               oid, VwifiOidName(oid));

    /* Answered the same way whichever arm it arrives in. WDI's own OIDs
     * are method requests, but this one is a pure get and the WLAN
     * component is documented loosely enough that it is not worth
     * guessing -- handling both costs one case and removes the
     * question. The trace above records which arm actually fired. */
    if (oid == OID_WDI_GET_ADAPTER_CAPABILITIES) {
        return VwifiHandleGetAdapterCapabilities(adapter, OidRequest);
    }
    if (oid == OID_WDI_SET_ADAPTER_CONFIGURATION) {
        return VwifiHandleSetAdapterConfiguration(adapter, OidRequest);
    }
    if (oid == OID_WDI_GET_STATISTICS) {
        return VwifiHandleGetStatistics(adapter, OidRequest);
    }
    if (oid == OID_WDI_ABORT_TASK) {
        return VwifiHandleAbortTask(adapter, OidRequest);
    }
    if (oid == OID_WDI_TASK_SET_RADIO_STATE) {
        return VwifiHandleTaskSetRadioState(adapter, OidRequest);
    }
    if (oid == OID_WDI_SET_PRIVACY_EXEMPTION_LIST) {
        return VwifiHandleAcceptedSet(adapter, OidRequest,
                                      "privacy exemption list");
    }
    if (oid == OID_WDI_SET_DEFAULT_KEY_ID) {
        return VwifiHandleAcceptedSet(adapter, OidRequest, "default key id");
    }
    if (oid == OID_WDI_SET_ASSOCIATION_PARAMETERS) {
        return VwifiHandleAcceptedSet(adapter, OidRequest,
                                      "association parameters");
    }
    if (oid == OID_WDI_SET_CONNECTION_QUALITY) {
        return VwifiHandleAcceptedSet(adapter, OidRequest,
                                      "connection quality");
    }
    if (oid == OID_WDI_SET_ADVERTISEMENT_INFORMATION) {
        return VwifiHandleAcceptedSet(adapter, OidRequest,
                                      "advertisement information");
    }
    if (oid == OID_WDI_SET_POWER_STATE) {
        return VwifiHandleAcceptedSet(adapter, OidRequest, "power state");
    }
    if (oid == OID_WDI_SET_FLUSH_BSS_ENTRY) {
        VWIFI_INFO("OID: flush BSS entries");
        VwifiScanFlushCache(adapter);
        return VwifiWdiAckHeaderOnly(OidRequest, NDIS_STATUS_SUCCESS);
    }
    if (oid == OID_WDI_GET_BSS_ENTRY_LIST) {
        /* The entries go back as a BSS_ENTRY_LIST indication, exactly as
         * a scan reports them; the OID itself only needs acknowledging
         * -- WABIModel's FromIhv message for this command is "No TLV
         * data needed, header is sufficient". Refusing this is what made
         * an already-discovered network disappear from `netsh wlan show
         * networks` and from the UI.
         *
         * The request's SSID is logged rather than used. It is the one
         * place the port driver states, in its own words, which network
         * it is asking about, and during a connect that is the network
         * being connected to. Printing it next to the SSID our entries
         * actually carry turns "the BSS looks right" into a comparison
         * of the two strings that have to match. Filtering on it would
         * only ever remove entries, and reporting a superset is within
         * contract, so nothing is filtered. */
        PVOID       ssidBuf = NULL;
        ULONG       ssidBufLen = 0;
        UCHAR       wantSsid[32];
        ULONG       wantLen = 0;
        NDIS_STATUS ssidStatus;

        ssidStatus = VwifiGetTlvPayload(OidRequest, &ssidBuf, &ssidBufLen);
        if (ssidStatus == NDIS_STATUS_SUCCESS) {
            ssidStatus = VwifiTlvParseBssListRequest(
                adapter->WdiPeerVersion, ssidBuf, ssidBufLen,
                wantSsid, &wantLen);
        }

        if (ssidStatus != NDIS_STATUS_SUCCESS) {
            /* Not fatal, but say so rather than printing "wildcard" and
             * inviting the reader to conclude the port driver asked for
             * one. The mandatory container failing to parse would itself
             * be a finding. */
            VWIFI_WARN("OID: cached BSS entry list requested, SSID "
                       "unparseable (0x%08x)", ssidStatus);
        } else if (wantLen == 0) {
            VWIFI_INFO("OID: cached BSS entry list requested (wildcard)");
        } else {
            CHAR  pretty[33];
            ULONG i;
            for (i = 0; i < wantLen; i++) {
                pretty[i] = (wantSsid[i] >= 0x20 && wantSsid[i] < 0x7f)
                                ? (CHAR)wantSsid[i] : '.';
            }
            pretty[wantLen] = '\0';
            VWIFI_INFO("OID: cached BSS entry list requested for "
                       "ssid='%s' (%u bytes)", pretty, wantLen);
        }
        VwifiScanIndicateCachedBss(adapter, VwifiGetWdiPortId(OidRequest),
                                   OidRequest->PortNumber);
        return VwifiWdiAckHeaderOnly(OidRequest, NDIS_STATUS_SUCCESS);
    }

    if (OidRequest->RequestType == NdisRequestSetInformation) {
        switch (oid) {
        /* Both op-mode routes are handled, because they carry different
         * modes. OID_WDI_TASK_CHANGE_OPERATION_MODE (NdisRequestMethod,
         * below) can only ever ask for STA — WDI_OPERATION_MODE has no
         * monitor mode. The Native 802.11 OID here is the only one that
         * can carry DOT11_OPERATION_MODE_NETWORK_MONITOR, so if monitor
         * mode works at all it works through this case. */
        case OID_DOT11_CURRENT_OPERATION_MODE:
            return VwifiHandleSetOpMode(adapter, OidRequest);
        case OID_DOT11_CURRENT_CHANNEL:
            return VwifiHandleSetChannel(adapter, OidRequest);
        case OID_DOT11_CURRENT_FREQUENCY:
            return VwifiHandleSetFrequency(adapter, OidRequest);
        case OID_GEN_CURRENT_PACKET_FILTER:
            return VwifiHandleSetPacketFilter(adapter, OidRequest);
        case OID_WDI_SET_ADD_CIPHER_KEYS:
            return VwifiHandleAddCipherKeys(adapter, OidRequest);
        case OID_WDI_SET_DELETE_CIPHER_KEYS:
            return VwifiHandleDeleteCipherKeys(adapter, OidRequest);
        default:
            break;
        }
    } else if (OidRequest->RequestType == NdisRequestQueryInformation) {
        switch (oid) {
        case OID_DOT11_CURRENT_OPERATION_MODE: {
            PDOT11_CURRENT_OPERATION_MODE mode;
            if (OidRequest->DATA.QUERY_INFORMATION.InformationBufferLength
                    < sizeof(*mode)) {
                OidRequest->DATA.QUERY_INFORMATION.BytesNeeded = sizeof(*mode);
                return NDIS_STATUS_BUFFER_TOO_SHORT;
            }
            mode = (PDOT11_CURRENT_OPERATION_MODE)
                OidRequest->DATA.QUERY_INFORMATION.InformationBuffer;
            mode->uReserved = 0;
            mode->uCurrentOpMode =
                (adapter->OpMode == VWIFI_MODE_MONITOR)
                    ? DOT11_OPERATION_MODE_NETWORK_MONITOR
                    : DOT11_OPERATION_MODE_EXTENSIBLE_STATION;
            OidRequest->DATA.QUERY_INFORMATION.BytesWritten = sizeof(*mode);
            return NDIS_STATUS_SUCCESS;
        }
        default:
            break;
        }
    } else if (OidRequest->RequestType == NdisRequestMethod) {
        /* WDI tasks arrive as method requests. */
        switch (oid) {
        case OID_WDI_TASK_SCAN:
            return VwifiHandleTaskScan(adapter, OidRequest);
        case OID_WDI_TASK_CONNECT:
            return VwifiHandleTaskConnect(adapter, OidRequest);
        case OID_WDI_TASK_DISCONNECT:
            return VwifiHandleTaskDisconnect(adapter, OidRequest);
        case OID_WDI_TASK_CHANGE_OPERATION_MODE:
            return VwifiHandleTaskChangeOpMode(adapter, OidRequest);
        case OID_WDI_TASK_CREATE_PORT:
            return VwifiHandleTaskCreatePort(adapter, OidRequest);
        case OID_WDI_TASK_DELETE_PORT:
            return VwifiHandleTaskDeletePort(adapter, OidRequest);
        case OID_WDI_TASK_DOT11_RESET:
            return VwifiHandleTaskDot11Reset(adapter, OidRequest);
        case OID_WDI_SET_RECEIVE_PACKET_FILTER:
            return VwifiHandleSetReceivePacketFilter(adapter, OidRequest);
        case OID_WDI_SET_MULTICAST_LIST:
            return VwifiHandleSetMulticastList(adapter, OidRequest);
        default:
            break;
        }
    }

    /* Everything else: let the Microsoft WLAN component handle it.
     *
     * Logged, not silent. NOT_SUPPORTED is a legitimate answer for most
     * OIDs, but it is also how a genuinely required one disappears, and
     * the two are indistinguishable without a line saying which OID
     * went unanswered. */
    VWIFI_INFO("OID: unhandled 0x%08x %s -> NOT_SUPPORTED",
               oid, VwifiOidName(oid));
    return NDIS_STATUS_NOT_SUPPORTED;
}

/* ============================================================
 * OID_WDI_TASK_CHANGE_OPERATION_MODE
 *
 * WABIModel.xml:
 *   <message commandId="WDI_TASK_CHANGE_OPERATION_MODE"
 *            type="WDI_TASK_CHANGE_OPERATION_MODE_PARAMETERS"
 *            direction="ToIhv">
 *     <containerRef id="WDI_TLV_OPERATION_MODE" name="OperationMode"
 *                   type="OperationModeContainer" />
 *   </message>
 *   <message commandId="WDI_TASK_CHANGE_OPERATION_MODE"
 *            type="WDI_TASK_CHANGE_OPERATION_MODE_RESULTS"
 *            description="No TLV data needed, header is sufficient"
 *            direction="FromIhv" />
 *
 * This is NOT the path Npcap's monitor-mode request takes, contrary to
 * what this file used to assume. WDI_OPERATION_MODE covers STA and the
 * three P2P roles; there is no network-monitor mode, and the string
 * "monitor" appears nowhere in dot11wdi.h, wditypes.hpp or
 * WABIModel.xml. So the only mode that can arrive here is STA, and the
 * shim rejects anything else.
 *
 * Monitor mode therefore has to come through the Native 802.11 OID
 * surface above (OID_DOT11_CURRENT_OPERATION_MODE with
 * DOT11_OPERATION_MODE_NETWORK_MONITOR, from windot11.h), which is a
 * plain NDIS set request rather than a WDI task. Whether the Microsoft
 * WLAN component actually forwards that OID to a WDI miniport is the
 * open question for Phase 1.5 — see the README.
 * ============================================================ */
NDIS_STATUS
VwifiHandleTaskChangeOpMode(_Inout_ PVWIFI_ADAPTER Adapter,
                            _In_ PNDIS_OID_REQUEST Req)
{
    ULONG devMode = 0;
    NDIS_STATUS status;
    PVOID tlvBuf;
    ULONG tlvLen;

    status = VwifiGetTlvPayload(Req, &tlvBuf, &tlvLen);
    if (status != NDIS_STATUS_SUCCESS) return status;

    status = VwifiTlvParseOperationMode(Adapter->WdiPeerVersion,
                                        tlvBuf, tlvLen, &devMode);
    if (status != NDIS_STATUS_SUCCESS) {
        VWIFI_ERR("operation mode parse failed 0x%x", status);
        return status;
    }

    VWIFI_INFO("op mode -> STA");

    status = VwifiSetOpMode(Adapter, devMode);
    if (status != NDIS_STATUS_SUCCESS) return status;

    /* The M0 needs no TLVs; completion arrives as
     * WDI_INDICATION_CHANGE_OPERATION_MODE_COMPLETE, which also needs
     * none. */
    VwifiSendWdiIndication(Adapter, VwifiGetWdiPortId(Req), Req->PortNumber,
                           NDIS_STATUS_WDI_INDICATION_CHANGE_OPERATION_MODE_COMPLETE,
                           NDIS_STATUS_SUCCESS,
                           VwifiGetWdiTransactionId(Req),
                           NULL, 0);
    /* INDICATION_REQUIRED, for the reason set out above the create-port
     * handler. This one was missed when the other four task handlers
     * were corrected: it sends its completion indication and then
     * returned SUCCESS as well, which is the same double completion. */
    return NDIS_STATUS_INDICATION_REQUIRED;
}
