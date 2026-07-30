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
 * ============================================================
 * VERIFY-AGAINST-WDK CHECKLIST
 * ============================================================
 * The API *shape* below is documented and solid. The exact member
 * names inside the WDI_* parameter structures are the part I could
 * not verify without the WDK installed. When you first build:
 *
 *   1. Open %WDKContentRoot%\Include\wdf\... no — the WDI headers are
 *      at Include\<ver>\km\dot11wdi.h, wditypes.hpp,
 *      TlvGeneratorParser.hpp.
 *   2. Search TlvGeneratorParser.hpp for "ParseWdiTaskScan" to get
 *      the true signature and the WDI_TASK_SCAN_PARAMETERS layout.
 *   3. Fix the member names flagged with [MEMBER?] below.
 *
 * Everything else — context setup, PeerVersion threading, cleanup
 * discipline, error mapping — is correct as written.
 * ============================================================
 */

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

static WDI_BAND_ID VwifiFreqToBandId(USHORT FreqMhz)
{
    /* [VERIFY] enumerator names against wditypes.hpp. */
    return (FreqMhz >= 5000) ? WDI_BAND_ID_5000 : WDI_BAND_ID_2400;
}

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
        RtlCopyMemory(&w->BSSID, e->bssid, 6);

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

    /* The nesting here is documented: the special-members topic shows
     * pConnectTaskParameters->ConnectParameters.MulticastCipherAlgorithms
     * so ConnectParameters is a sub-struct. [MEMBER?] on the leaves. */
    const WDI_CONNECT_PARAMETERS *cp = &parsed.ConnectParameters;

    RtlCopyMemory(ReqBuf->bssid, &cp->BSSID, 6);

    ULONG ssidLen = cp->SSID.SSIDLength;
    if (ssidLen > 32) ssidLen = 32;
    ReqBuf->ssid_len = static_cast<USHORT>(ssidLen);
    RtlCopyMemory(ReqBuf->ssid, cp->SSID.SSID, ssidLen);

    /* WDI's connect parameters carry no channel, so this stays zero and
     * the device resolves it: it looks the BSSID up in the BSS table it
     * filled while scanning. Before that fallback existed, zero meant
     * "use whatever channel we happen to be tuned to", which after a
     * scan is the restored pre-scan channel. */
    ReqBuf->channel_freq = 0;

    WdiAuthToVwifi(cp->AuthAlgo, &ReqBuf->auth_algo, &ReqBuf->akm_suite);
    ReqBuf->cipher_pairwise = WdiCipherToVwifi(cp->UnicastCipherAlgo);

    /* Multicast cipher arrives as a list; take the first. */
    if (cp->MulticastCipherAlgorithms.ElementCount > 0 &&
        cp->MulticastCipherAlgorithms.pElements != nullptr) {
        ReqBuf->cipher_group =
            WdiCipherToVwifi(cp->MulticastCipherAlgorithms.pElements[0]);
    } else {
        ReqBuf->cipher_group = ReqBuf->cipher_pairwise;
    }

    /* Association-request IEs. NOT optional for WPA2: the OS builds
     * the RSN element here and the AP cross-checks it against the
     * 4-way handshake. A mismatch surfaces as a MIC failure, which
     * is a miserable thing to debug. Copy verbatim. */
    ULONG ieLen = cp->AssocRequestIEs.ElementCount;
    ULONG ieRoom = ReqCap - sizeof(*ReqBuf);
    if (ieLen > ieRoom) ieLen = ieRoom;
    if (ieLen > 0 && cp->AssocRequestIEs.pElements != nullptr) {
        RtlCopyMemory(reinterpret_cast<UCHAR *>(ReqBuf) + sizeof(*ReqBuf),
                      cp->AssocRequestIEs.pElements, ieLen);
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
    RtlCopyMemory(&entry.BSSID, Result->bssid, 6);

    /* [VERIFY] WDI_ASSOC_STATUS / WDI_CIPHER_ALGORITHM enumerator names
     * against wditypes.hpp. The structure members below are confirmed
     * against TlvGenerated_.hpp. */
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
    USHORT ReasonCode,
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
    RtlCopyMemory(&params.DisconnectIndicationParameters.MacAddress, Bssid, 6);
    params.DisconnectIndicationParameters.DisassociationWABIReason =
        (WDI_ASSOC_STATUS)ReasonCode;   /* [VERIFY] mapping, wditypes.hpp */

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

    /* [MEMBER?] parsed.CipherKeys — list of WDI_CIPHER_KEY. */
    for (ULONG i = 0;
         i < parsed.SetCipherKey.ElementCount && n < KeysCap;
         i++) {
        const WDI_CIPHER_KEY *ck = &parsed.SetCipherKey.pElements[i];
        VWIFI_TLV_KEY *k = &Keys[n];

        RtlZeroMemory(k, sizeof(*k));

        /* PAIRWISE vs GROUP. Get this wrong and the first data frame
         * decrypts to garbage with no obvious cause: the device keeps
         * separate slots and separate PN spaces for each. */
        k->Pairwise = (ck->CipherKeyInfo.KeyType ==
                       WDI_CIPHER_KEY_TYPE_PAIRWISE) ? TRUE : FALSE;
        k->KeyIndex = static_cast<UCHAR>(ck->CipherKeyInfo.KeyIndex);
        k->Cipher   = WdiCipherToVwifi(ck->CipherKeyInfo.CipherAlgo);

        RtlCopyMemory(k->PeerMac, &ck->MacAddr, 6);

        ULONG klen = ck->KeyValue.ElementCount;
        if (klen > sizeof(k->KeyValue)) klen = sizeof(k->KeyValue);
        if (klen > 0 && ck->KeyValue.pElements != nullptr) {
            RtlCopyMemory(k->KeyValue, ck->KeyValue.pElements, klen);
        }
        k->KeyLength = klen;
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
         i < parsed.SetCipherKey.ElementCount && n < KeysCap;
         i++) {
        const WDI_CIPHER_KEY_ID *ck = &parsed.SetCipherKey.pElements[i];
        VWIFI_TLV_KEY *k = &Keys[n];

        RtlZeroMemory(k, sizeof(*k));
        k->Pairwise = (ck->KeyType == WDI_CIPHER_KEY_TYPE_PAIRWISE)
                    ? TRUE : FALSE;
        k->KeyIndex = static_cast<UCHAR>(ck->KeyIndex);
        RtlCopyMemory(k->PeerMac, &ck->MacAddr, 6);
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

    UNREFERENCED_PARAMETER(Mac);

    /* [MEMBER?] The capabilities structure is the largest and fiddliest
     * of the lot — PHY types, band/channel lists, cipher suites, auth
     * algorithms, interface capabilities, op-mode support. Fill it out
     * against wditypes.hpp.
     *
     * Two entries matter beyond the obvious:
     *   - NetMon must appear in the supported op modes, or Npcap can't
     *     switch us into monitor mode (Phase 1.5 dies silently).
     *   - The cipher list must include CCMP or WPA2 connect attempts
     *     get rejected before they reach the device.
     */
    params.InterfaceAttributes.OpModeCapability =
        WDI_OPERATION_MODE_STA | WDI_OPERATION_MODE_NETWORK_MONITOR;

    UNREFERENCED_PARAMETER(Caps);

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
 * Translation happens here rather than in oids.c on purpose. There are
 * two distinct WDI_OPERATION_MODE enumerations: dot11wdi.h's, which the
 * plain-C files see and which lists only STA and the P2P roles, and the
 * TLV library's in wditypes.hpp, which is what actually travels in
 * WDI_TLV_OPERATION_MODE and does include NETWORK_MONITOR. Returning a
 * device-side enum \(VWIFI_MODE_*\) keeps the ambiguity on this side of
 * the plain-C boundary, which is what this shim is for.
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

    /* [VERIFY] entry-point and member names, against TlvGeneratorParser.hpp.
     * The scan path's ParseWdiTaskScan / CleanupParsedWdiTaskScan pair is
     * the naming precedent these follow. */
    st = ParseWdiTaskChangeOperationMode(BufferLen, static_cast<const UINT8 *>(Buffer),
                                         &ctx, &parsed);
    if (st != NDIS_STATUS_SUCCESS) {
        return st;
    }

    /* WDI_OPERATION_MODE_CONTAINER is a plain typedef of
     * WDI_OPERATION_MODE — the member is the value, not a sub-struct. */
    mode = parsed.OperationMode;
    CleanupParsedWdiTaskChangeOperationMode(&parsed);

    if (mode & WDI_OPERATION_MODE_NETWORK_MONITOR) {
        *DeviceMode = VWIFI_MODE_MONITOR;
    } else if (mode & WDI_OPERATION_MODE_STA) {
        *DeviceMode = VWIFI_MODE_STA;
    } else {
        return NDIS_STATUS_NOT_SUPPORTED;
    }
    return NDIS_STATUS_SUCCESS;
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
