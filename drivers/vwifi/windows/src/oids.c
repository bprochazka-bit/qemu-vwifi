/*
 * vwifi — oids.c
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Phase 1.5 OID handling. Npcap switches a Wi-Fi adapter into
 * monitor mode using the Native 802.11 OID surface, which the
 * Microsoft WLAN component surfaces to our miniport:
 *
 *   OID_DOT11_CURRENT_OPERATION_MODE  (set) -> op mode NETWORK_MONITOR
 *   OID_DOT11_CURRENT_CHANNEL         (set) -> channel number (2.4 GHz)
 *   OID_DOT11_CURRENT_FREQUENCY       (set) -> channel for 5 GHz
 *   OID_GEN_CURRENT_PACKET_FILTER     (set) -> raw data/mgmt bits
 *
 * We intercept exactly these and translate to control-ring commands.
 * Everything else returns NOT_SUPPORTED so the WLAN component's
 * default handling applies.
 */

#include "vwifi_drv.h"
#include <windot11.h>

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

    if (OidRequest->RequestType == NdisRequestSetInformation) {
        oid = OidRequest->DATA.SET_INFORMATION.Oid;

        switch (oid) {
        /* NOTE: OID_DOT11_CURRENT_OPERATION_MODE is NOT how operation
         * mode reaches a WDI miniport. Per WABIModel.xml the OS sends
         * OID_WDI_TASK_CHANGE_OPERATION_MODE (a method request carrying
         * WDI_TLV_OPERATION_MODE) — the Microsoft WLAN component
         * translates Npcap's Native 802.11 OID into that task before it
         * gets to us. Handled in the NdisRequestMethod arm below. */
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
        oid = OidRequest->DATA.QUERY_INFORMATION.Oid;

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
        oid = OidRequest->DATA.METHOD_INFORMATION.Oid;

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

    /* Everything else: let the Microsoft WLAN component handle it. */
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
 * This is the path Npcap's monitor-mode request actually takes: Npcap
 * asks Native 802.11 for dot11_operation_mode_network_monitor, and the
 * Microsoft WLAN component turns that into this task.
 * ============================================================ */
NDIS_STATUS
VwifiHandleTaskChangeOpMode(_Inout_ PVWIFI_ADAPTER Adapter,
                            _In_ PNDIS_OID_REQUEST Req)
{
    ULONG opMode = 0;
    ULONG devMode;
    NDIS_STATUS status;
    PVOID tlvBuf;
    ULONG tlvLen;

    status = VwifiGetTlvPayload(Req, &tlvBuf, &tlvLen);
    if (status != NDIS_STATUS_SUCCESS) return status;

    status = VwifiTlvParseOperationMode(Adapter->WdiPeerVersion,
                                        tlvBuf, tlvLen, &opMode);
    if (status != NDIS_STATUS_SUCCESS) {
        VWIFI_ERR("operation mode parse failed 0x%x", status);
        return status;
    }

    /* WDI_OPERATION_MODE is a bitmask of the same DOT11_OPERATION_MODE_*
     * values the Native interface uses. */
    if (opMode & DOT11_OPERATION_MODE_NETWORK_MONITOR) {
        devMode = VWIFI_MODE_MONITOR;
        VWIFI_INFO("op mode -> NETWORK_MONITOR");
    } else if (opMode & DOT11_OPERATION_MODE_EXTENSIBLE_STATION) {
        devMode = VWIFI_MODE_STA;
        VWIFI_INFO("op mode -> ExtSTA");
    } else {
        VWIFI_WARN("unsupported operation mode 0x%08x", opMode);
        return NDIS_STATUS_NOT_SUPPORTED;
    }

    status = VwifiSetOpMode(Adapter, devMode);
    if (status != NDIS_STATUS_SUCCESS) return status;

    /* The M0 needs no TLVs; completion arrives as
     * WDI_INDICATION_CHANGE_OPERATION_MODE_COMPLETE, which also needs
     * none. */
    VwifiSendWdiIndication(Adapter, Req->PortNumber,
                           NDIS_STATUS_WDI_INDICATION_CHANGE_OPERATION_MODE_COMPLETE,
                           NULL, 0);
    return NDIS_STATUS_SUCCESS;
}
