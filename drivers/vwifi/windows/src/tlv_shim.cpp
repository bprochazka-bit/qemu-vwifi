/*
 * vwifi — tlv_shim.cpp
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * The WDI TLV parse/generate layer.
 *
 * API shape (from "WDI TLV parser interface overview" and
 * "WDI TLV generator interface overview"):
 *
 *   Caller-allocation parse (we use this — stack local, no heap):
 *     WDI_SCAN_PARAMETERS parsed;
 *     st = ParseWdiTaskScan(len, buf, &ctx, &parsed);
 *     ... use parsed ...
 *     CleanupParsedWdiTaskScan(&parsed);
 *
 *   Generate:
 *     st = GenerateWdiIndicationBssEntryList(&params, headerSize,
 *                                           &ctx, &outLen, &pOut);
 *     ... use pOut ...
 *     FreeGenerated(pOut);
 *
 * Naming: for M1 messages (WDI -> IHV) the APIs are Parse<X>ToIhv;
 * for M0/M3/M4 (IHV -> WDI) they're Generate<X>FromIhv. Aliases are
 * provided so Parse<X> means ToIhv and Generate<X> means FromIhv,
 * which is precisely what an IHV miniport wants. We use the aliases.
 *
 * The parameter structures come from TlvGenerated_.hpp, which
 * TlvGeneratorParser.hpp includes; the leaf value types (WDI_MAC_ADDRESS,
 * WDI_BAND_ID, WDI_ASSOC_STATUS, the cipher enums) come from
 * dot11wdi.h and wditypes.hpp. Note that dot11wdi.h and wditypes.hpp sit
 * in the kit's normal include path but TlvGenerated_.hpp does not — see
 * WdiTlvIncludeDir in vwifi.vcxproj.
 * */

extern "C" {
#include <ndis.h>
#include <dot11wdi.h>
}

#include <wditypes.hpp>
#include "TlvGeneratorParser.hpp"

extern "C" {
#include "vwifi_abi.h"
#include "tlv_shim.h"
}

/* ============================================================
 * Context helper
 *
 * AllocationContext is passed through to our operator new untouched;
 * we don't need per-call-site allocation behaviour, so it stays 0.
 * PeerVersion is the whole point — see WDI TLV versioning.
 * ============================================================ */

static inline TLV_CONTEXT MakeCtx(ULONG PeerVersion)
{
    TLV_CONTEXT ctx;
    ctx.AllocationContext = 0;
    ctx.PeerVersion       = PeerVersion;
    return ctx;
}

/* WDI_BSS_ENTRY_CHANNEL_INFO wants a channel number and a band id, not
 * the centre frequency the device reports. */
static WDI_CHANNEL_NUMBER VwifiFreqToChannel(USHORT FreqMhz)
{
    if (FreqMhz == 2484)                        return 14;
    if (FreqMhz >= 2412 && FreqMhz <= 2472)     return (WDI_CHANNEL_NUMBER)((FreqMhz - 2407) / 5);
    if (FreqMhz >= 5160 && FreqMhz <= 5885)     return (WDI_CHANNEL_NUMBER)((FreqMhz - 5000) / 5);
    return 0;
}

/* ...and the inverse, for the channel WDI reports in a connect request. */
static USHORT VwifiChannelToFreq(WDI_CHANNEL_NUMBER Channel, WDI_BAND_ID Band)
{
    if (Channel == 0) return 0;
    if (Band == WDI_BAND_ID_5000) return (USHORT)(5000 + Channel * 5);
    if (Channel == 14)            return 2484;
    if (Channel <= 13)            return (USHORT)(2407 + Channel * 5);
    return 0;
}

static WDI_BAND_ID VwifiFreqToBandId(USHORT FreqMhz)
{
    return (FreqMhz >= 5000) ? WDI_BAND_ID_5000 : WDI_BAND_ID_2400;
}

/* Largest command we tell the OS it may send us. Kept in step with
 * VWIFI_CTRL_PAYLOAD_SIZE by a C_ASSERT on the C side. */
#define VWIFI_TLV_MAX_COMMAND_SIZE 2048

/* The generator reserves this much space at the front of the blob for
 * the message header. The caller (NdisMIndicateStatusEx path) does not
 * prepend anything itself, so we ask for the WDI message header. */
static const ULONG kHeaderReserve = sizeof(WDI_MESSAGE_HEADER);

/* ============================================================
 * Scan
 * ============================================================ */

extern "C"
NDIS_STATUS
VwifiTlvParseScanRequest(
    ULONG PeerVersion,
    const VOID *Buffer,
    ULONG BufferLen,
    struct vwifi_scan_req *ReqBuf,
    ULONG ReqCap,
    PULONG ReqLen)
{
    TLV_CONTEXT ctx = MakeCtx(PeerVersion);
    WDI_SCAN_PARAMETERS parsed = {};
    NDIS_STATUS st;

    if (ReqCap < sizeof(*ReqBuf)) return NDIS_STATUS_BUFFER_TOO_SHORT;

    st = ParseWdiTaskScan(BufferLen, static_cast<const UINT8 *>(Buffer), &ctx, &parsed);
    if (st != NDIS_STATUS_SUCCESS) {
        return st;
    }

    RtlZeroMemory(ReqBuf, sizeof(*ReqBuf));

    /* Channel hints. WDI may give a channel list; a scan is always
     * allowed to cover more than requested, so if anything is unclear
     * we leave both masks at zero and let VwifiHandleTaskScan fill them
     * from the device's advertised capabilities. That degrades to a
     * slower scan, never a broken one.
     *
     * Leaving them zero rather than hardcoding 2.4 GHz here is the
     * point: the device grew 5 GHz support, and a mask hardcoded in
     * this file meant a Windows guest never scanned above channel 14 no
     * matter what the device advertised. Capabilities belong to the one
     * place that reads them. */
    ReqBuf->channel_mask_24 = 0;
    ReqBuf->channel_mask_5  = 0;
    ReqBuf->dwell_ms        = 100;
    ReqBuf->flags           = 0;

    /* Directed SSID list. ArrayOfElements<WDI_SSID>, and WDI_SSID is
     * itself ArrayOfElements<UINT8>. */
    ULONG nssid = 0;
    if (parsed.SSIDList.ElementCount > 0 && parsed.SSIDList.pElements != nullptr) {
        UCHAR *trail = reinterpret_cast<UCHAR *>(ReqBuf) + sizeof(*ReqBuf);
        ULONG maxSsid = (ReqCap - sizeof(*ReqBuf)) / 34;

        for (ULONG i = 0; i < parsed.SSIDList.ElementCount && i < maxSsid; i++) {
            /* WDI_SSID is ArrayOfElements<UINT8> — a length-counted
             * byte array, not a struct with SSIDLength/SSID members. */
            const WDI_SSID *s = &parsed.SSIDList.pElements[i];
            UCHAR len = static_cast<UCHAR>(
                s->ElementCount > 32 ? 32 : s->ElementCount);
            trail[i * 34] = len;
            RtlCopyMemory(&trail[i * 34 + 1], s->pElements, len);
            RtlZeroMemory(&trail[i * 34 + 1 + len], 33 - len);
            nssid++;
        }
    }
    ReqBuf->num_ssids = static_cast<USHORT>(nssid);
    *ReqLen = sizeof(*ReqBuf) + nssid * 34;

    CleanupParsedWdiTaskScan(&parsed);
    return NDIS_STATUS_SUCCESS;
}

extern "C"
NDIS_STATUS
VwifiTlvGenerateBssEntryList(
    ULONG PeerVersion,
    const VWIFI_TLV_BSS_ITEM *Items,
    ULONG Count,
    VOID **Buffer,
    PULONG BufferLen)
{
    TLV_CONTEXT ctx = MakeCtx(PeerVersion);
    WDI_INDICATION_BSS_ENTRY_LIST_PARAMETERS params = {};
    NDIS_STATUS st;
    UINT8 *pOut = nullptr;
    ULONG outLen = 0;

    if (Count == 0) return NDIS_STATUS_INVALID_PARAMETER;

    /* Build the BSS entry array. The library takes the list as
     * pElements/ElementCount; we own this array, it owns the copy it
     * makes during generation. */
    WDI_BSS_ENTRY_CONTAINER *entries = static_cast<WDI_BSS_ENTRY_CONTAINER *>(
        ExAllocatePool2(POOL_FLAG_NON_PAGED,
                        sizeof(WDI_BSS_ENTRY_CONTAINER) * Count, 'eBvw'));
    if (entries == nullptr) return NDIS_STATUS_RESOURCES;

    for (ULONG i = 0; i < Count; i++) {
        const struct vwifi_bss_entry *e = Items[i].Entry;
        WDI_BSS_ENTRY_CONTAINER *w = &entries[i];

        RtlZeroMemory(w, sizeof(*w));

        /* WDI_BSS_ENTRY_CONTAINER carries far less than a beacon does,
         * because the OS parses the frame itself. There is no BSSType,
         * BeaconInterval, Capability or PhyType field to fill: those all
         * come out of the raw frame below. What the container adds is the
         * receive-side metadata the frame cannot carry — signal, channel,
         * and when we saw it. */
        RtlCopyMemory(w->BSSID.Address, e->bssid, 6);

        w->SignalInfo.RSSI        = e->rssi;
        w->SignalInfo.LinkQuality = 0;

        w->ChannelInfo.ChannelNumber = VwifiFreqToChannel(e->channel_freq);
        w->ChannelInfo.BandId        = VwifiFreqToBandId(e->channel_freq);

        w->EntryAgeInfo.HostTimeStamp     = e->tsf;
        w->EntryAgeInfo.CachedInformation = FALSE;
        w->Optional.EntryAgeInfo_IsPresent = TRUE;

        /* The WHOLE beacon / probe-response frame, verbatim.
         *
         * Per WABIModel.xml, BSSEntryContainer carries the raw frame as
         * a byte blob, in one of two TLVs:
         *   WDI_TLV_BEACON_FRAME          name="BeaconFrame"
         *   WDI_TLV_PROBE_RESPONSE_FRAME  name="ProbeResponseFrame"
         * both type="ByteBlob", both optional.
         *
         * The OS parses the IEs itself (SSID, RSN, rates, HT/VHT caps).
         * Handing over a reconstructed subset would silently lose
         * capabilities — which is exactly why the device keeps whole
         * frames rather than just the IE tail.
         *
         * WDI_BYTE_BLOB is ArrayOfElements<UINT8>: the bytes go directly
         * in ElementCount/pElements, with no Payload indirection. */
        if (e->capability_info & VWIFI_BSS_F_BEACON) {
            w->BeaconFrame.ElementCount = e->ie_len;
            w->BeaconFrame.pElements    = const_cast<UINT8 *>(Items[i].Frame);
            w->Optional.BeaconFrame_IsPresent = TRUE;
        } else {
            w->ProbeResponseFrame.ElementCount = e->ie_len;
            w->ProbeResponseFrame.pElements =
                const_cast<UINT8 *>(Items[i].Frame);
            w->Optional.ProbeResponseFrame_IsPresent = TRUE;
        }
    }

    /* The list member is called DeviceDescriptor, not BSSEntries, and it
     * is one of the optional TLVs. */
    params.DeviceDescriptor.ElementCount = Count;
    params.DeviceDescriptor.pElements    = entries;
    params.Optional.DeviceDescriptor_IsPresent = TRUE;

    st = GenerateWdiIndicationBssEntryList(&params, kHeaderReserve,
                                           &ctx, &outLen, &pOut);

    ExFreePoolWithTag(entries, 'eBvw');

    if (st != NDIS_STATUS_SUCCESS) {
        return st;
    }

    *Buffer    = pOut;
    *BufferLen = outLen;
    return NDIS_STATUS_SUCCESS;
}

/* No VwifiTlvGenerateScanComplete / VwifiTlvGenerateConnectComplete.
 * Both WDI_INDICATION_SCAN_COMPLETE_PARAMETERS and
 * WDI_INDICATION_CONNECT_COMPLETE_PARAMETERS are typedefs of
 * EmptyMessageStructureType: these messages carry no TLVs whatsoever,
 * which is what WABIModel means by "No TLV data needed, header is
 * sufficient". The completion status travels in WDI_MESSAGE_HEADER's
 * Status field, so wdi_common.c sends them header-only. Generating a
 * body for them was inventing a payload the OS does not read.
 */

/* ============================================================
 * Connect
 * ============================================================ */

/* Map a WDI cipher enum to our device ABI cipher. */
static USHORT WdiCipherToVwifi(ULONG algo)
{
    switch (algo) {
    case WDI_CIPHER_ALGO_CCMP:      return VWIFI_CIPHER_CCMP128;
    case WDI_CIPHER_ALGO_GCMP_256:  return VWIFI_CIPHER_GCMP256;
    case WDI_CIPHER_ALGO_TKIP:      return VWIFI_CIPHER_TKIP;
    case WDI_CIPHER_ALGO_WEP40:     return VWIFI_CIPHER_WEP40;
    case WDI_CIPHER_ALGO_WEP104:    return VWIFI_CIPHER_WEP104;
    case WDI_CIPHER_ALGO_NONE:      return VWIFI_CIPHER_NONE;
    default:                        return VWIFI_CIPHER_NONE;
    }
}

/* Map a WDI auth algorithm to our device ABI auth + AKM.
 *
 * WPA2-PSK is the case that matters and it's counter-intuitive:
 * the over-the-air authentication is Open System. The RSN handshake
 * happens *after* association, over EAPOL. So auth_algo=OPEN with
 * akm_suite=PSK is correct, not a bug.
 */
static VOID WdiAuthToVwifi(ULONG algo, USHORT *authOut, USHORT *akmOut)
{
    switch (algo) {
    case WDI_AUTH_ALGO_80211_OPEN:
        *authOut = VWIFI_AUTH_OPEN;
        *akmOut  = VWIFI_AKM_NONE;
        break;
    case WDI_AUTH_ALGO_80211_SHARED_KEY:
        *authOut = VWIFI_AUTH_SHARED;
        *akmOut  = VWIFI_AKM_NONE;
        break;
    case WDI_AUTH_ALGO_RSNA_PSK:
    case WDI_AUTH_ALGO_WPA_PSK:
        *authOut = VWIFI_AUTH_OPEN;      /* open auth, RSN after assoc */
        *akmOut  = VWIFI_AKM_PSK;
        break;
    case WDI_AUTH_ALGO_RSNA:
    case WDI_AUTH_ALGO_WPA:
        *authOut = VWIFI_AUTH_OPEN;
        *akmOut  = VWIFI_AKM_8021X;
        break;
    case WDI_AUTH_ALGO_WPA3_SAE:
        *authOut = VWIFI_AUTH_SAE;
        *akmOut  = VWIFI_AKM_SAE;
        break;
    default:
        *authOut = VWIFI_AUTH_OPEN;
        *akmOut  = VWIFI_AKM_NONE;
        break;
    }
}

extern "C"
NDIS_STATUS
VwifiTlvParseConnectRequest(
    ULONG PeerVersion,
    const VOID *Buffer,
    ULONG BufferLen,
    struct vwifi_connect_req *ReqBuf,
    ULONG ReqCap,
    PULONG ReqLen)
{
    TLV_CONTEXT ctx = MakeCtx(PeerVersion);
    WDI_TASK_CONNECT_PARAMETERS parsed = {};
    NDIS_STATUS st;

    if (ReqCap < sizeof(*ReqBuf)) return NDIS_STATUS_BUFFER_TOO_SHORT;

    st = ParseWdiTaskConnect(BufferLen, static_cast<const UINT8 *>(Buffer),
                             &ctx, &parsed);
    if (st != NDIS_STATUS_SUCCESS) return st;

    RtlZeroMemory(ReqBuf, sizeof(*ReqBuf));

    /* WDI_CONNECT_PARAMETERS_CONTAINER holds the security selections and
     * the SSID list; the BSS to join comes from the task's separate
     * PreferredBSSEntryList. Every security field is a *list*, because
     * the OS may accept more than one — we take the first, which is the
     * OS's preference order. */
    const WDI_CONNECT_PARAMETERS_CONTAINER *cp = &parsed.ConnectParameters;

    if (cp->SSIDList.ElementCount > 0 && cp->SSIDList.pElements != nullptr) {
        const WDI_SSID *ssid = &cp->SSIDList.pElements[0];
        ULONG ssidLen = ssid->ElementCount;
        if (ssidLen > 32) ssidLen = 32;
        ReqBuf->ssid_len = static_cast<USHORT>(ssidLen);
        if (ssidLen > 0 && ssid->pElements != nullptr) {
            RtlCopyMemory(ReqBuf->ssid, ssid->pElements, ssidLen);
        }
    }

    /* The BSSID, and — usefully — the channel. The earlier note in this
     * file that "WDI's connect parameters carry no channel" was right
     * about ConnectParameters and wrong about the request as a whole:
     * each preferred BSS entry carries a ChannelInfo. Use it when it is
     * there and leave zero otherwise, which still lets the device
     * resolve the channel from its own BSS table. */
    if (parsed.PreferredBSSEntryList.ElementCount > 0 &&
        parsed.PreferredBSSEntryList.pElements != nullptr) {
        const WDI_CONNECT_BSS_ENTRY_CONTAINER *bss =
            &parsed.PreferredBSSEntryList.pElements[0];

        RtlCopyMemory(ReqBuf->bssid, bss->BSSID.Address, 6);
        ReqBuf->channel_freq =
            VwifiChannelToFreq(bss->ChannelInfo.ChannelNumber,
                               bss->ChannelInfo.BandId);
    }

    if (cp->AuthenticationAlgorithms.ElementCount > 0 &&
        cp->AuthenticationAlgorithms.pElements != nullptr) {
        WdiAuthToVwifi(cp->AuthenticationAlgorithms.pElements[0],
                       &ReqBuf->auth_algo, &ReqBuf->akm_suite);
    }

    if (cp->UnicastCipherAlgorithms.ElementCount > 0 &&
        cp->UnicastCipherAlgorithms.pElements != nullptr) {
        ReqBuf->cipher_pairwise =
            WdiCipherToVwifi(cp->UnicastCipherAlgorithms.pElements[0]);
    }

    if (cp->MulticastCipherAlgorithms.ElementCount > 0 &&
        cp->MulticastCipherAlgorithms.pElements != nullptr) {
        ReqBuf->cipher_group =
            WdiCipherToVwifi(cp->MulticastCipherAlgorithms.pElements[0]);
    } else {
        ReqBuf->cipher_group = ReqBuf->cipher_pairwise;
    }

    /* Association-request IEs.
     *
     * WDI hands over only AssociationRequestVendorIE — vendor-specific
     * elements. There is no RSN element here and no field to put one in:
     * the OS describes the security it wants through the auth and cipher
     * lists above and expects the driver (or, here, the device) to build
     * the RSN element from them. So an empty assoc_ie_len is normal on
     * this path, not a parse failure, and WPA2 depends on the device
     * constructing RSN from auth_algo/akm_suite/cipher_*. */
    ULONG ieLen = 0;
    if (cp->Optional.AssociationRequestVendorIE_IsPresent &&
        cp->AssociationRequestVendorIE.pElements != nullptr) {
        ULONG ieRoom = ReqCap - sizeof(*ReqBuf);
        ieLen = cp->AssociationRequestVendorIE.ElementCount;
        if (ieLen > ieRoom) ieLen = ieRoom;
        RtlCopyMemory(reinterpret_cast<UCHAR *>(ReqBuf) + sizeof(*ReqBuf),
                      cp->AssociationRequestVendorIE.pElements, ieLen);
    }
    ReqBuf->assoc_ie_len = static_cast<USHORT>(ieLen);
    *ReqLen = sizeof(*ReqBuf) + ieLen;

    CleanupParsedWdiTaskConnect(&parsed);
    return NDIS_STATUS_SUCCESS;
}

/* No VwifiTlvGenerateAssociationStart: dot11wdi.h has no
 * ASSOCIATION_START message id and no matching NDIS_STATUS, so there is
 * nothing to indicate it as. wditypes.hpp's
 * WDI_INDICATION_ASSOCIATION_START_PARAMETERS is a container carried
 * inside the association result, not a standalone message. */

extern "C"
NDIS_STATUS
VwifiTlvGenerateAssociationResult(
    ULONG PeerVersion,
    const struct vwifi_assoc_result *Result,
    const UCHAR *Ies,
    VOID **Buffer,
    PULONG BufferLen)
{
    TLV_CONTEXT ctx = MakeCtx(PeerVersion);
    WDI_INDICATION_ASSOCIATION_RESULT_LIST params = {};
    WDI_ASSOCIATION_RESULT_CONTAINER entry = {};
    UINT8 *pOut = nullptr;
    ULONG outLen = 0;

    /* The indication is a *list* of association results, one per port,
     * even though a STA only ever reports one. */
    RtlCopyMemory(entry.BSSID.Address, Result->bssid, 6);

    entry.AssociationResultParameters.AssociationStatus =
        (Result->status_code == 0) ? WDI_ASSOC_STATUS_SUCCESS
                                   : WDI_ASSOC_STATUS_FAILURE;
    entry.AssociationResultParameters.StatusCode    = Result->status_code;
    entry.AssociationResultParameters.ReAssociation = FALSE;
    entry.AssociationResultParameters.PortAuthorized = FALSE;

    /* The AP's association-response frame, verbatim. The OS reads the
     * negotiated cipher suite out of its IEs, so it has to be real. */
    if (Ies != nullptr && Result->ie_len > 0) {
        entry.AssociationResponseFrame.ElementCount = Result->ie_len;
        entry.AssociationResponseFrame.pElements = const_cast<UINT8 *>(Ies);
        entry.Optional.AssociationResponseFrame_IsPresent = TRUE;
    }

    params.AssociationResults.ElementCount = 1;
    params.AssociationResults.pElements    = &entry;

    NDIS_STATUS st = GenerateWdiIndicationAssociationResult(
        &params, kHeaderReserve, &ctx, &outLen, &pOut);
    if (st != NDIS_STATUS_SUCCESS) return st;

    *Buffer    = pOut;
    *BufferLen = outLen;
    return NDIS_STATUS_SUCCESS;
}

extern "C"
NDIS_STATUS
VwifiTlvGenerateDisassociation(
    ULONG PeerVersion,
    const UCHAR *Bssid,
    BOOLEAN Local,
    VOID **Buffer,
    PULONG BufferLen)
{
    TLV_CONTEXT ctx = MakeCtx(PeerVersion);
    WDI_INDICATION_DISASSOCIATION_PARAMETERS params = {};
    UINT8 *pOut = nullptr;
    ULONG outLen = 0;

    /* Neither BSSID nor a reason code sits at the top level: both live
     * in the nested DisconnectIndicationParameters, and the reason is a
     * WDI_ASSOC_STATUS rather than an 802.11 reason code. */
    RtlCopyMemory(params.DisconnectIndicationParameters.MacAddress.Address,
                  Bssid, 6);
    /* WDI_ASSOC_STATUS is not the 802.11 reason-code space. Casting a
     * reason code straight in lands on unrelated enumerators — reason 2,
     * "previous authentication no longer valid", would arrive as
     * WDI_ASSOC_STATUS_UNREACHABLE. The distinction WDI actually draws
     * here is who initiated the disconnect, which the device tells us
     * directly in vwifi_disconnect_ev.local. */
    params.DisconnectIndicationParameters.DisassociationWABIReason =
        Local ? WDI_ASSOC_STATUS_DISASSOCIATED_BY_HOST
              : WDI_ASSOC_STATUS_PEER_DISASSOCIATED;

    NDIS_STATUS st = GenerateWdiIndicationDisassociation(
        &params, kHeaderReserve, &ctx, &outLen, &pOut);
    if (st != NDIS_STATUS_SUCCESS) return st;

    *Buffer    = pOut;
    *BufferLen = outLen;
    return NDIS_STATUS_SUCCESS;
}

/* ============================================================
 * Cipher keys
 * ============================================================ */

extern "C"
NDIS_STATUS
VwifiTlvParseAddCipherKeys(
    ULONG PeerVersion,
    const VOID *Buffer,
    ULONG BufferLen,
    VWIFI_TLV_KEY *Keys,
    ULONG KeysCap,
    PULONG Count)
{
    TLV_CONTEXT ctx = MakeCtx(PeerVersion);
    WDI_SET_ADD_CIPHER_KEYS_PARAMETERS parsed = {};
    NDIS_STATUS st;
    ULONG n = 0;

    st = ParseWdiSetAddCipherKeys(BufferLen, static_cast<const UINT8 *>(Buffer),
                                  &ctx, &parsed);
    if (st != NDIS_STATUS_SUCCESS) return st;

    for (ULONG i = 0;
         i < parsed.SetCipherKey.ElementCount && n < KeysCap;
         i++) {
        const WDI_SET_ADD_CIPHER_KEYS_CONTAINER *ck =
            &parsed.SetCipherKey.pElements[i];
        VWIFI_TLV_KEY *k = &Keys[n];

        RtlZeroMemory(k, sizeof(*k));

        /* PAIRWISE vs GROUP. Get this wrong and the first data frame
         * decrypts to garbage with no obvious cause: the device keeps
         * separate slots and separate PN spaces for each. */
        k->Pairwise = (ck->CipherKeyTypeInfo.KeyType ==
                       WDI_CIPHER_KEY_TYPE_PAIRWISE_KEY) ? TRUE : FALSE;
        k->KeyIndex = static_cast<UCHAR>(ck->CipherKeyID.CipherKeyID);
        k->Cipher   = WdiCipherToVwifi(ck->CipherKeyTypeInfo.CipherAlgorithm);

        RtlCopyMemory(k->PeerMac, ck->PeerMacAddress.Address, 6);

        /* The key bytes live in a per-cipher blob, not one KeyValue
         * field: CCMPKey, GCMPKey, GCMP_256Key, WEPKey and so on are
         * separate optional TLVs and only the negotiated one is set. */
        const WDI_PRIVATE_BYTE_BLOB *blob = nullptr;
        if (ck->Optional.CCMPKey_IsPresent)          blob = &ck->CCMPKey;
        else if (ck->Optional.GCMPKey_IsPresent)     blob = &ck->GCMPKey;
        else if (ck->Optional.GCMP_256Key_IsPresent) blob = &ck->GCMP_256Key;
        else if (ck->Optional.WEPKey_IsPresent)      blob = &ck->WEPKey;

        if (blob != nullptr && blob->pElements != nullptr) {
            ULONG klen = blob->ElementCount;
            if (klen > sizeof(k->KeyValue)) klen = sizeof(k->KeyValue);
            RtlCopyMemory(k->KeyValue, blob->pElements, klen);
            k->KeyLength = klen;
        }
        n++;
    }

    *Count = n;
    CleanupParsedWdiSetAddCipherKeys(&parsed);
    return NDIS_STATUS_SUCCESS;
}

extern "C"
NDIS_STATUS
VwifiTlvParseDeleteCipherKeys(
    ULONG PeerVersion,
    const VOID *Buffer,
    ULONG BufferLen,
    VWIFI_TLV_KEY *Keys,
    ULONG KeysCap,
    PULONG Count)
{
    TLV_CONTEXT ctx = MakeCtx(PeerVersion);
    WDI_SET_DELETE_CIPHER_KEYS_PARAMETERS parsed = {};
    NDIS_STATUS st;
    ULONG n = 0;

    st = ParseWdiSetDeleteCipherKeys(BufferLen, static_cast<const UINT8 *>(Buffer),
                                     &ctx, &parsed);
    if (st != NDIS_STATUS_SUCCESS) return st;

    for (ULONG i = 0;
         i < parsed.CipherKeyInfo.ElementCount && n < KeysCap;
         i++) {
        const WDI_SET_DELETE_CIPHER_KEYS_CONTAINER *ck =
            &parsed.CipherKeyInfo.pElements[i];
        VWIFI_TLV_KEY *k = &Keys[n];

        RtlZeroMemory(k, sizeof(*k));
        k->Pairwise = (ck->CipherKeyTypeInfo.KeyType ==
                       WDI_CIPHER_KEY_TYPE_PAIRWISE_KEY) ? TRUE : FALSE;
        k->KeyIndex = static_cast<UCHAR>(ck->CipherKeyID.CipherKeyID);
        RtlCopyMemory(k->PeerMac, ck->PeerMacAddress.Address, 6);
        n++;
    }

    *Count = n;
    CleanupParsedWdiSetDeleteCipherKeys(&parsed);
    return NDIS_STATUS_SUCCESS;
}

/* ============================================================
 * Capabilities
 * ============================================================ */

extern "C"
NDIS_STATUS
VwifiTlvGenerateAdapterCapabilities(
    ULONG PeerVersion,
    const struct vwifi_caps *Caps,
    const UCHAR *Mac,
    VOID **Buffer,
    PULONG BufferLen)
{
    TLV_CONTEXT ctx = MakeCtx(PeerVersion);
    WDI_GET_ADAPTER_CAPABILITIES_PARAMETERS params = {};
    UINT8 *pOut = nullptr;
    ULONG outLen = 0;

    /* WDI_GET_ADAPTER_CAPABILITIES_PARAMETERS is the largest structure
     * in the interface. Most of its members are optional TLVs with
     * presence bits; what is filled here is the mandatory core plus the
     * station limits the OS uses to size its own requests.
     *
     * There is no op-mode capability field to set. WDI has no
     * network-monitor mode at all — see VwifiTlvParseOperationMode. */
    /* Must not exceed the driver's ctrl payload scratch buffer
     * (VWIFI_CTRL_PAYLOAD_SIZE in vwifi_drv.h). That header is C-only
     * and cannot be included here, so the value is asserted against
     * instead of shared — see the C_ASSERT in wdi_common.c. */
    params.CommunicationAttributes.CommunicationCapabilities.MaxCommandSize =
        VWIFI_TLV_MAX_COMMAND_SIZE;
    params.Optional.CommunicationAttributes_IsPresent = TRUE;

    {
        WDI_INTERFACE_CAPABILITIES *ic =
            &params.InterfaceAttributes.InterfaceCapabilities;

        ic->MTUSize              = 1500;
        ic->MaxMultiCastListSize = 32;
        ic->BackFillSize         = 0;
        RtlCopyMemory(ic->Address.Address, Mac, 6);
        ic->MaxTxRate            = 54000;   /* kbps; ERP-OFDM ceiling */
        ic->MaxRxRate            = 54000;
        ic->HardwareRadioState   = TRUE;
        ic->SoftwareRadioState   = TRUE;
        ic->NumRxStreams         = 1;
        ic->NumTxStreams         = 1;
        ic->NumChannels          = 1;
        ic->ActionFramesSupported = FALSE;
    }

    {
        WDI_STATION_CAPABILITIES *sc =
            &params.StationAttributes.StationCapabilities;

        /* These are the sizes the OS sizes its own requests against, so
         * they have to match what the device actually accepts rather
         * than being aspirational. */
        sc->ScanSSIDListSize       = Caps->max_scan_ssids;
        sc->DesiredBSSIDListSize   = 1;
        sc->DesiredSSIDListSize    = 1;
        sc->KeyMappingTableSize    = 1;   /* one PTK */
        sc->DefaultKeyTableSize    = 4;   /* four GTK slots */
        sc->MaxNumPerSTA           = 1;
        sc->MFPCapable             = 0;
        sc->AutoPowerSaveMode      = FALSE;
        sc->BSSListCachemanagement = FALSE;
    }

    /* Not filled: BandInfo, PhyInfo, the cipher/auth algorithm pair
     * lists, and the country-region list. All are optional TLVs whose
     * presence bits stay clear, so the message is well-formed without
     * them — but the OS uses BandInfo to learn which channels exist, so
     * a scan may be limited until they are supplied. That is the next
     * piece of work on this function, and it needs the device's
     * supported_channels_24 / _5 masks translated into
     * WDI_BAND_INFO_CONTAINER entries. */

    NDIS_STATUS st = GenerateWdiGetAdapterCapabilities(
        &params, kHeaderReserve, &ctx, &outLen, &pOut);
    if (st != NDIS_STATUS_SUCCESS) return st;

    *Buffer    = pOut;
    *BufferLen = outLen;
    return NDIS_STATUS_SUCCESS;
}

/* ============================================================
 * Operation mode
 *
 * WDI_OPERATION_MODE (dot11wdi.h) enumerates STA, P2P_DEVICE,
 * P2P_CLIENT and P2P_GO. There is no network-monitor mode: the string
 * "monitor" appears nowhere in dot11wdi.h, wditypes.hpp or
 * WABIModel.xml. Anything other than STA is a mode this device does not
 * implement, so it is rejected rather than silently mapped.
 * ============================================================ */

extern "C"
NDIS_STATUS
VwifiTlvParseOperationMode(
    ULONG PeerVersion,
    const VOID *Buffer,
    ULONG BufferLen,
    PULONG DeviceMode)
{
    TLV_CONTEXT ctx = MakeCtx(PeerVersion);
    WDI_TASK_CHANGE_OPERATION_MODE_PARAMETERS parsed = {};
    NDIS_STATUS st;
    ULONG mode;

    *DeviceMode = VWIFI_MODE_IDLE;

    st = ParseWdiTaskChangeOperationMode(BufferLen,
                                         static_cast<const UINT8 *>(Buffer),
                                         &ctx, &parsed);
    if (st != NDIS_STATUS_SUCCESS) {
        return st;
    }

    /* WDI_OPERATION_MODE_CONTAINER is a plain typedef of
     * WDI_OPERATION_MODE — the member is the value, not a sub-struct. */
    mode = parsed.OperationMode;
    CleanupParsedWdiTaskChangeOperationMode(&parsed);

    if (mode & WDI_OPERATION_MODE_STA) {
        *DeviceMode = VWIFI_MODE_STA;
        return NDIS_STATUS_SUCCESS;
    }
    return NDIS_STATUS_NOT_SUPPORTED;
}

/* ============================================================
 * Cleanup
 * ============================================================ */

extern "C"
VOID
VwifiTlvFreeGenerated(VOID *Buffer)
{
    if (Buffer != nullptr) {
        FreeGenerated(static_cast<UINT8 *>(Buffer));
    }
}
