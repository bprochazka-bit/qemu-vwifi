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
    VwifiSendWdiIndication(Adapter, Req->PortNumber,
                           NDIS_STATUS_WDI_INDICATION_CHANGE_OPERATION_MODE_COMPLETE,
                           NDIS_STATUS_SUCCESS,
                           VwifiGetWdiTransactionId(Req),
                           NULL, 0);
    return NDIS_STATUS_SUCCESS;
}
