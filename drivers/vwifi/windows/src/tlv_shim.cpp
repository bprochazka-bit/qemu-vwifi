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

/* Distinct from tlv_mem.cpp's tag: allocations made here are ours, not
 * the library's, and !poolused should say which is which. */
#define VWIFI_TLV_PARAMS_TAG  (ULONG)'pTvw'   /* "vTp" in WinDbg */

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

/* Per-frame TX slot size, reported as the datapath's allocation
 * granularity. Must stay a power of two, and must match
 * VWIFI_RX_BUFFER_SIZE in vwifi_drv.h, which is what rings.c carves the
 * TX buffer pool into. That header is C-only and cannot be included
 * here, hence the duplicate -- same arrangement, and same reason, as
 * VWIFI_TLV_MAX_COMMAND_SIZE above. */
#define VWIFI_TLV_TX_SLOT_SIZE 4096

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
VwifiTlvParseBssListRequest(
    ULONG PeerVersion,
    const VOID *Buffer,
    ULONG BufferLen,
    UCHAR *Ssid,
    PULONG SsidLen)
{
    TLV_CONTEXT ctx = MakeCtx(PeerVersion);
    WDI_GET_BSS_ENTRY_LIST_UPDATE_PARAMETERS parsed = {};
    NDIS_STATUS st;

    *SsidLen = 0;

    st = ParseWdiGetBssEntryListToIhv(BufferLen,
                                      static_cast<const UINT8 *>(Buffer),
                                      &ctx, &parsed);
    if (st != NDIS_STATUS_SUCCESS) {
        return st;
    }

    /* WDI_SSID is ArrayOfElements<UINT8>, same shape as the entries in
     * a scan request's SSIDList -- a counted byte array, not a struct
     * with an SSIDLength member. */
    ULONG len = parsed.SSID.ElementCount;
    if (len > 32) len = 32;
    if (len > 0 && parsed.SSID.pElements != nullptr) {
        RtlCopyMemory(Ssid, parsed.SSID.pElements, len);
        *SsidLen = len;
    }

    CleanupParsedWdiGetBssEntryListToIhv(&parsed);
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

        w->SignalInfo.RSSI = e->rssi;

        /* LinkQuality is documented as "0 through 100. A value of 100
         * specifies the highest link quality" -- so the zero this used
         * to report was not "unknown", it was the worst value in the
         * range, on every network, forever.
         *
         * Same mapping as VwifiRssiToLinkQuality in wdi_common.c, which
         * cannot be called from here: that header is C-only and this
         * translation unit is the C++ side of the shim. -50 dBm or
         * better is 100, -100 or worse is 0, linear between. */
        {
            LONG r = (LONG)e->rssi;
            w->SignalInfo.LinkQuality =
                (r >= -50)  ? 100 :
                (r <= -100) ? 0   : (UINT32)((r + 100) * 2);
        }

        w->ChannelInfo.ChannelNumber = VwifiFreqToChannel(e->channel_freq);
        w->ChannelInfo.BandId        = VwifiFreqToBandId(e->channel_freq);

        /* Age. The host clock, not the AP's.
         *
         * This used to pass `e->tsf` -- the AP's TSF, microseconds
         * since the AP powered on. WDI_TLV_BSS_ENTRY_AGE_INFO wants a
         * KeQuerySystemTime value, 100 ns units since 1601, and the OS
         * ages entries by subtracting this from its own clock. A TSF in
         * that field is roughly 1.3e17 units short, so every network
         * arrived already four hundred years old.
         *
         * That is a scan that appears to work in every log the driver
         * writes and produces a network list that empties itself: the
         * entry is indicated, shown, aged out, and gone. Nothing is
         * left for a connect to be built from, and the OS never gets as
         * far as issuing one -- which is why a connect attempt reached
         * this driver as no OID at all. */
        w->EntryAgeInfo.HostTimeStamp     = Items[i].HostTimeStamp;
        w->EntryAgeInfo.CachedInformation = Items[i].Cached;
        w->Optional.EntryAgeInfo_IsPresent = TRUE;

        /* The beacon and probe-response frame BODIES -- MAC header
         * removed from each.
         *
         * Per WABIModel.xml, BSSEntryContainer carries raw frames as
         * byte blobs, in two independent TLVs:
         *   WDI_TLV_BEACON_FRAME          name="BeaconFrame"
         *   WDI_TLV_PROBE_RESPONSE_FRAME  name="ProbeResponseFrame"
         * both type="ByteBlob", both optional.
         *
         * INDEPENDENT is the word that matters, and this code used to
         * get it wrong: it picked one of the two from a flag and emitted
         * exactly that one. Since a scan gets a probe response from
         * every BSS it probes, and a beacon only if one happens to land
         * inside the dwell, the flag said "probe response" essentially
         * always -- BEACON_FRAME has never been present in a BSS entry
         * this driver produced.
         *
         * "Optional" in this interface has repeatedly meant "the host
         * copes without it but does not do the thing you wanted": the
         * same was true of band and PHY info, of the datapath
         * attributes, and of the entry age. A BSS the host will list
         * but will not build a connect candidate from is the same
         * shape of symptom. Both are emitted now whenever both are
         * held.
         *
         * The OS parses the body itself -- SSID, RSN, rates, HT/VHT caps
         * -- so the whole thing is handed over rather than a
         * reconstructed subset, which would silently lose capabilities.
         * But it must start at the frame body, not the frame: WDI's
         * beacon/probe-response blobs are documented as not including
         * the 802.11 MAC header.
         *
         * Passing the whole frame does not fail, which is what makes it
         * worth a comment. The OS reads the first eight bytes of the MAC
         * header as the beacon timestamp, takes the next four as beacon
         * interval and capability, and starts walking information
         * elements from inside addr2. It finds no SSID element there and
         * reports a perfectly visible network as "Hidden Network".
         *
         * The device sends whole frames deliberately -- Linux's
         * cfg80211_inform_bss_frame_data() wants the MAC header -- so
         * the trimming belongs here, in the consumer that needs it.
         *
         * WDI_BYTE_BLOB is ArrayOfElements<UINT8>: the bytes go directly
         * in ElementCount/pElements, with no Payload indirection. */
        if (Items[i].Beacon != nullptr &&
            Items[i].BeaconLen > VWIFI_80211_MGMT_HDR_LEN) {
            w->BeaconFrame.pElements =
                const_cast<UINT8 *>(Items[i].Beacon) + VWIFI_80211_MGMT_HDR_LEN;
            w->BeaconFrame.ElementCount =
                Items[i].BeaconLen - VWIFI_80211_MGMT_HDR_LEN;
            w->Optional.BeaconFrame_IsPresent = TRUE;
        }

        if (Items[i].Probe != nullptr &&
            Items[i].ProbeLen > VWIFI_80211_MGMT_HDR_LEN) {
            w->ProbeResponseFrame.pElements =
                const_cast<UINT8 *>(Items[i].Probe) + VWIFI_80211_MGMT_HDR_LEN;
            w->ProbeResponseFrame.ElementCount =
                Items[i].ProbeLen - VWIFI_80211_MGMT_HDR_LEN;
            w->Optional.ProbeResponseFrame_IsPresent = TRUE;
        }
        /* Neither held, or both too short to hold a MAC header: no
         * blob, and both presence bits stay clear. The entry still
         * carries BSSID, signal and channel, so the BSS is reported --
         * as hidden, which is the honest answer for a BSS we have no
         * readable frame for. */
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

/* Our device-ABI cipher back to WDI's. The inverse of
 * WdiCipherToVwifi, for the association result, which has to state the
 * cipher that was negotiated rather than the one that was asked for. */
static WDI_CIPHER_ALGORITHM VwifiCipherToWdi(USHORT cipher)
{
    switch (cipher) {
    case VWIFI_CIPHER_CCMP128:  return WDI_CIPHER_ALGO_CCMP;
    case VWIFI_CIPHER_GCMP256:  return WDI_CIPHER_ALGO_GCMP_256;
    case VWIFI_CIPHER_TKIP:     return WDI_CIPHER_ALGO_TKIP;
    case VWIFI_CIPHER_WEP40:    return WDI_CIPHER_ALGO_WEP40;
    case VWIFI_CIPHER_WEP104:   return WDI_CIPHER_ALGO_WEP104;
    case VWIFI_CIPHER_NONE:     return WDI_CIPHER_ALGO_NONE;
    default:                    return WDI_CIPHER_ALGO_NONE;
    }
}

/* Our (auth, AKM) pair back to WDI's single auth algorithm.
 *
 * The AKM is what carries the information here, for the reason set out
 * above WdiAuthToVwifi: WPA2-PSK authenticates over the air as Open
 * System and does its real work in the EAPOL handshake afterwards, so
 * auth alone cannot tell WPA2-PSK from an open network. The forward
 * mapping folds both onto VWIFI_AUTH_OPEN; the AKM is what unfolds
 * them. */
static WDI_AUTH_ALGORITHM VwifiAuthToWdi(USHORT auth, USHORT akm)
{
    switch (akm) {
    case VWIFI_AKM_PSK:   return WDI_AUTH_ALGO_RSNA_PSK;
    case VWIFI_AKM_8021X: return WDI_AUTH_ALGO_RSNA;
    case VWIFI_AKM_SAE:   return WDI_AUTH_ALGO_WPA3_SAE;
    default:              break;
    }
    switch (auth) {
    case VWIFI_AUTH_SHARED: return WDI_AUTH_ALGO_80211_SHARED_KEY;
    case VWIFI_AUTH_SAE:    return WDI_AUTH_ALGO_WPA3_SAE;
    default:                return WDI_AUTH_ALGO_80211_OPEN;
    }
}

extern "C"
NDIS_STATUS
VwifiTlvGenerateAssociationResult(
    ULONG PeerVersion,
    const struct vwifi_assoc_result *Result,
    const VWIFI_ASSOC_PARAMS *Params,
    const UCHAR *Ies,
    VOID **Buffer,
    PULONG BufferLen)
{
    TLV_CONTEXT ctx = MakeCtx(PeerVersion);
    WDI_INDICATION_ASSOCIATION_RESULT_LIST params = {};
    WDI_ASSOCIATION_RESULT_CONTAINER entry = {};
    WDI_PHY_TYPE activePhys[2] = {};
    UINT32 nActivePhys = 0;
    WDI_BAND_ID band = VwifiFreqToBandId(Params->FreqMhz);
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

    /* The negotiated algorithms and the band. All four were left at
     * zero, and zero is not a valid value for three of them:
     * WDI_AUTH_ALGO_80211_OPEN is 1, WDI_BAND_ID_2400 is 1, and
     * WDI_DS_INFO has no zero enumerator at all (CHANGED 1, UNCHANGED
     * 2, UNKNOWN 3). Only WDI_CIPHER_ALGO_NONE is genuinely 0.
     *
     * DSInfo is UNKNOWN rather than UNCHANGED on purpose: "unchanged"
     * is a claim that this port is rejoining the same distribution
     * system it was on before, which is a roam's claim to make and not
     * a first connect's. */
    entry.AssociationResultParameters.AuthAlgorithm =
        VwifiAuthToWdi(Params->AuthAlgo, Params->AkmSuite);
    entry.AssociationResultParameters.UnicastCipherAlgorithm =
        VwifiCipherToWdi(Params->CipherPairwise);
    entry.AssociationResultParameters.MulticastDataCipherAlgorithm =
        VwifiCipherToWdi(Params->CipherGroup);
    /* No management-frame protection, consistently with MFPCapable 0
     * and with MulticastManagementAlgorithms being left out of the
     * advertised capabilities. */
    entry.AssociationResultParameters.MulticastMgmtCipherAlgorithm =
        WDI_CIPHER_ALGO_NONE;
    entry.AssociationResultParameters.BandID = band;
    entry.AssociationResultParameters.DSInfo = WDI_DS_UNKNOWN;

    /* ActivePhyTypeList is MANDATORY here, and leaving it empty is what
     * made every association fail.
     *
     * WABIModel.xml lists it twice, which is the whole point:
     *
     *   <containerRef id="WDI_TLV_PHY_TYPE_LIST" name="ActivePhyTypeList"
     *                 type="PhyTypeListContainer"
     *                 versionAdded="WDI_VERSION_1_1_4" />
     *   <containerRef id="WDI_TLV_PHY_TYPE_LIST" name="ActivePhyTypeList"
     *                 type="PhyTypeListContainer" optional="true"
     *                 versionRemoved="WDI_VERSION_1_1_4" />
     *
     * Optional up to 1.1.4, required from 1.1.4 on. This peer reports
     * 0x0001010a -- 1.1.10 -- so it is required, and a zeroed
     * ArrayOfElements is the same empty-mandatory-container that the
     * FirmwareVersion note above describes: the generator rejects the
     * whole message with NDIS_STATUS_INVALID_DATA, the driver logs
     * "ASSOCIATION_RESULT TLV generate failed", and the connect
     * completes 0xc0000001 having associated perfectly well.
     *
     * The contents are a judgement, the presence is not. dot11
     * ActivePhyList means the PHYs currently operating on the
     * interface, so this mirrors the ValidPhyTypes advertised for the
     * band in VwifiTlvGenerateAdapterCapabilities -- an 11b/g radio on
     * a 2.4 GHz ERP BSS is running both. Reporting something the
     * capabilities never claimed would be worse than reporting too
     * much.
     *
     * A FreqMhz of 0 lands on 2400, since VwifiFreqToBandId treats
     * anything under 5000 as 2.4 GHz. The connect request only carries
     * a frequency when it names a preferred BSS entry -- see
     * VwifiTlvParseConnectRequest -- and a connect with no preferred
     * BSS on a 5 GHz-only AP would be described wrongly here. Nothing
     * exercises that today: the device is 2.4 GHz and every connect so
     * far has arrived with its BSS named. */
    if (band == WDI_BAND_ID_2400) {
        activePhys[nActivePhys++] = WDI_PHY_TYPE_HRDSSS;   /* 11b */
        activePhys[nActivePhys++] = WDI_PHY_TYPE_ERP;      /* 11g */
    } else {
        activePhys[nActivePhys++] = WDI_PHY_TYPE_OFDM;     /* 11a */
    }
    entry.ActivePhyTypeList.ElementCount = nActivePhys;
    entry.ActivePhyTypeList.pElements    = activePhys;

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

/* 5 GHz channel numbers, indexed by bit position in
 * vwifi_caps.supported_channels_5. This table is a shared contract: it
 * must stay identical to vwifi_unii_channels[] in the Linux driver's
 * vwifi_cfg80211.c and to the table op_scan indexes in the QEMU device.
 * Reorder one and the three disagree about what a set bit means. */
static const UINT8 kUniiChannels[] = {
    36, 40, 44, 48, 52, 56, 60, 64,
    100, 104, 108, 112, 116, 120, 124, 128, 132, 136, 140, 144,
    149, 153, 157, 161, 165,
};

/* Supported rates in the 802.11 encoding: units of 500 kbps.
 *
 * Three lists, not two, because WDI_PHY_TYPE_ERP is neither of the
 * other two. 802.11b (HRDSSS) is CCK only, 802.11a (OFDM) is OFDM only,
 * and 802.11g (ERP) is *both* -- that is what the "extended rate" in
 * Extended Rate PHY means. Advertising ERP with an OFDM-only rate list
 * describes a PHY that cannot receive a CCK frame, which no real 11g
 * radio is and ours is not either.
 *
 * This matters beyond pedantry. wdiwifi resolves a 2.4 GHz BSS to one
 * of the PHYs we list here and then checks the BSS's basic rate set
 * against that PHY's rates. A BSS whose basic set it cannot satisfy is
 * not a candidate. With ERP carrying no CCK, the two 2.4 GHz PHYs
 * between them covered every rate but neither covered a normal mixed
 * 11g beacon on its own. */
static const UINT16 kRatesDsss[] = { 2, 4, 11, 22 };            /* 1 .. 11 */
static const UINT16 kRatesOfdm[] = { 12, 18, 24, 36, 48, 72, 96, 108 };
static const UINT16 kRatesErp[]  = { 2, 4, 11, 22,              /* CCK   */
                                     12, 18, 24, 36, 48, 72, 96, 108 };

#define VWIFI_CAPS_MAX_BANDS    2
#define VWIFI_CAPS_MAX_PHYS     3
#define VWIFI_CAPS_MAX_CHAN_24  14
#define VWIFI_CAPS_MAX_CHAN_5   RTL_NUMBER_OF(kUniiChannels)
#define VWIFI_CAPS_MAX_RATES    RTL_NUMBER_OF(kRatesErp)   /* the longest */
#define VWIFI_CAPS_MAX_ALGOS    16

/* VwifiFillPhy writes RateCount slots with no bound of its own, so the
 * scratch row has to be at least as long as the longest list any caller
 * passes. Checked here rather than trusted, because adding a rate to
 * one of these lists is exactly the edit that would overrun it. */
C_ASSERT(RTL_NUMBER_OF(kRatesDsss) <= VWIFI_CAPS_MAX_RATES);
C_ASSERT(RTL_NUMBER_OF(kRatesOfdm) <= VWIFI_CAPS_MAX_RATES);
C_ASSERT(RTL_NUMBER_OF(kRatesErp)  <= VWIFI_CAPS_MAX_RATES);

/* Backing store for every ArrayOfElements the capabilities message
 * points at. The library reads through those pointers during Generate,
 * so the memory has to outlive the call -- one allocation, freed with
 * the parameters block, rather than a dozen. */
struct VwifiCapsScratch {
    WDI_BAND_INFO_CONTAINER   Bands[VWIFI_CAPS_MAX_BANDS];
    WDI_PHY_INFO_CONTAINER    Phys[VWIFI_CAPS_MAX_PHYS];
    WDI_PHY_TYPE              PhyTypes[VWIFI_CAPS_MAX_BANDS][2];
    WDI_CHANNEL_MAPPING_ENTRY Chan24[VWIFI_CAPS_MAX_CHAN_24];
    WDI_CHANNEL_MAPPING_ENTRY Chan5[VWIFI_CAPS_MAX_CHAN_5];
    UINT32                    Widths[VWIFI_CAPS_MAX_BANDS];
    UINT32                    TxPower[VWIFI_CAPS_MAX_PHYS];
    WDI_DATA_RATE_LIST        Rates[VWIFI_CAPS_MAX_PHYS][VWIFI_CAPS_MAX_RATES];
    WDI_ALGO_PAIRS            Algos[VWIFI_CAPS_MAX_ALGOS];
};

static ULONG
VwifiFillChannels24(UINT32 Mask, WDI_CHANNEL_MAPPING_ENTRY *Out, ULONG Max)
{
    ULONG n = 0;

    /* Bit N is channel N, so bit 0 is unused -- matching the Linux
     * driver, which loops 1..14 over the same mask. */
    for (ULONG ch = 1; ch <= 14 && n < Max; ch++) {
        if (!(Mask & (1u << ch))) continue;
        Out[n].ChannelNumber          = (WDI_CHANNEL_NUMBER)ch;
        Out[n].ChannelCenterFrequency = (ch == 14) ? 2484u : (2407u + ch * 5u);
        n++;
    }
    return n;
}

static ULONG
VwifiFillChannels5(UINT64 Mask, WDI_CHANNEL_MAPPING_ENTRY *Out, ULONG Max)
{
    ULONG n = 0;

    /* Bit i indexes kUniiChannels[i]; the numbers are not contiguous. */
    for (ULONG i = 0; i < RTL_NUMBER_OF(kUniiChannels) && n < Max; i++) {
        if (!(Mask & (1ULL << i))) continue;
        Out[n].ChannelNumber          = (WDI_CHANNEL_NUMBER)kUniiChannels[i];
        Out[n].ChannelCenterFrequency = 5000u + 5u * kUniiChannels[i];
        n++;
    }
    return n;
}

static VOID
VwifiFillPhy(WDI_PHY_INFO_CONTAINER *Phy,
             WDI_PHY_TYPE Type,
             UINT32 *TxPowerSlot,
             WDI_DATA_RATE_LIST *RateSlots,
             const UINT16 *Rates,
             ULONG RateCount,
             ULONG BasicCount)
{
    Phy->PhyCapabilities.PhyType          = Type;
    Phy->PhyCapabilities.SupportsCFPoll   = 0;
    Phy->PhyCapabilities.MPDUMaxLength    = 2304;
    Phy->PhyCapabilities.TemperatureClass = 1;
    Phy->PhyCapabilities.DiversitySupport = 0;

    *TxPowerSlot = 20;                      /* dBm, matching the Linux
                                             * driver's max_power */
    Phy->TxPowerLevelList.ElementCount = 1;
    Phy->TxPowerLevelList.pElements    = TxPowerSlot;

    for (ULONG i = 0; i < RateCount; i++) {
        RateSlots[i].DataRateFlag  = (UINT8)(i < BasicCount ? 1 : 0);
        RateSlots[i].DataRateValue = Rates[i];
    }
    Phy->DataRateList.ElementCount = (UINT32)RateCount;
    Phy->DataRateList.pElements    = RateSlots;
}

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
    UINT8 *pOut = nullptr;
    ULONG outLen = 0;

    /* Heap, not the stack.
     *
     * WDI_GET_ADAPTER_CAPABILITIES_PARAMETERS is the largest structure
     * in the interface -- twelve nested attribute containers plus four
     * ArrayOfElements members -- and a kernel stack is 12 KB total,
     * shared with everything below us on a PnP start path. A local of
     * that size is a stack overflow waiting for a deep enough caller,
     * and a kernel stack overflow does not produce a bugcheck to read:
     * the guard page fault double-faults and the machine resets.
     *
     * This is insurance, not a diagnosis -- the struct has not been
     * measured on the target kit. It costs one pool allocation on a
     * path that runs once per adapter open.
     *
     * A raw zeroed allocation rather than `new`, and the constructors
     * are deliberately not run. Two reasons:
     *
     *   Zeroes are what the constructors produce. ArrayOfElements<T> is
     *   layout-identical to { UINT32 ElementCount; T* pElements;
     *   BOOLEAN MemoryInternallyAllocated; } -- the generated header
     *   asserts exactly that -- so all-bits-zero is a well-formed empty
     *   array. The Optional bitfields are all IsPresent flags, and zero
     *   is FALSE.
     *
     *   And `new` cannot be spelled here anyway. The library's operator
     *   new takes (size_t, ULONG_PTR AllocationContext), so a placement
     *   form needs a matching operator delete(void*, ULONG_PTR) -- but
     *   ULONG_PTR *is* size_t on x64, which makes that the usual sized
     *   deallocation function. The compiler then rejects the placement
     *   new outright (C2956) and rejects the definition as a duplicate
     *   of the sized delete (C2084). */
    auto *pParams = static_cast<WDI_GET_ADAPTER_CAPABILITIES_PARAMETERS *>(
        ExAllocatePool2(POOL_FLAG_NON_PAGED,
                        sizeof(WDI_GET_ADAPTER_CAPABILITIES_PARAMETERS),
                        VWIFI_TLV_PARAMS_TAG));
    if (pParams == nullptr) {
        return NDIS_STATUS_RESOURCES;
    }
    WDI_GET_ADAPTER_CAPABILITIES_PARAMETERS &params = *pParams;

    auto *scratch = static_cast<VwifiCapsScratch *>(
        ExAllocatePool2(POOL_FLAG_NON_PAGED, sizeof(VwifiCapsScratch),
                        VWIFI_TLV_PARAMS_TAG));
    if (scratch == nullptr) {
        ExFreePoolWithTag(pParams, VWIFI_TLV_PARAMS_TAG);
        return NDIS_STATUS_RESOURCES;
    }

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
     * instead of shared — see the C_ASSERT in wdi_common.c.
     *
     * BOTH presence bits, and for a long time only the outer one was
     * set. WABIModel nests two optional containers here:
     *
     *   <aggregateContainer name="CommunicationAttributesContainer">
     *     <containerRef id="WDI_TLV_COMMUNICATION_CAPABILITIES"
     *                   name="CommunicationCapabilities"
     *                   type="CommunicationCapabilitiesContainer"
     *                   optional="true" />
     *
     * so WDI_COMMUNICATION_ATTRIBUTES_CONTAINER carries an Optional
     * bitfield of its own. Setting only params.Optional emitted the
     * outer container with nothing inside it, and wdiwifi recorded
     * WfcAdapterPropertyMaxCommandSize as 0 rather than leaving it
     * unpopulated.
     *
     * Zero is not inert. CConnectJob::GenerateConnectTaskTlv reads that
     * property as the size limit for the connect message it has just
     * built --
     *
     *   GetPropertyULongOrDefault(adapterCache, 0x10, 0xFFFFFFFF)
     *   CMessageHelper::FitMessageToBufferSize(msg, 368, limit, ...)
     *
     * -- and 368 bytes into a limit of 0 comes back
     * NDIS_STATUS_INVALID_DATA. StartConnectRoamTask returns 0xc0010015
     * and OID_WDI_TASK_CONNECT is never sent, with nothing anywhere
     * naming the capability that caused it. Measured with a breakpoint
     * at the instruction after the property read: 0x10 = 0, msglen 368.
     *
     * The default of 0xFFFFFFFF is the tell in hindsight. An absent
     * property means "no limit"; it was PRESENT and zero that was
     * fatal, so leaving the whole container out would have worked
     * better than half-filling it.
     *
     * Every other nested container here sets both bits --
     * StationAttributes.Optional.UnicastAlgorithms_IsPresent,
     * DatapathAttributes.Optional.DataPathCapabilities_IsPresent. This
     * was the one that was missed. */
    params.CommunicationAttributes.CommunicationCapabilities.MaxCommandSize =
        VWIFI_TLV_MAX_COMMAND_SIZE;
    params.CommunicationAttributes.Optional.CommunicationCapabilities_IsPresent =
        TRUE;
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

    /* FirmwareVersion is NOT optional.
     *
     * WABIModel.xml, InterfaceAttributesContainer:
     *   <containerRef id="WDI_TLV_FIRMWARE_VERSION" name="FirmwareVersion"
     *                 type="ASCIIString"/>
     * with no optional="true", unlike the NonWdiOidsList beside it. Left
     * as the zeroed ArrayOfElements it starts out as -- count 0, null
     * pointer -- the generator rejects the whole message with
     * NDIS_STATUS_INVALID_DATA (0xc0010015) and the WLAN component
     * closes the adapter a moment later, having learned nothing about
     * it. There is no indication of which field was at fault; this one
     * was found by reading the model.
     *
     * The count includes the terminator: the container is described as
     * "Generic container for null-terminated ASCII strings", so the
     * consumer is entitled to expect one in the bytes it receives.
     *
     * Static storage, so it outlives the Generate call without an
     * allocation to free. Driver .rdata is non-paged -- only sections
     * explicitly named PAGE* are pageable -- so this is safe to hand to
     * a library that may touch it at raised IRQL. */
    {
        static const CHAR kFirmwareVersion[] = "vwifi-virt 1.0";

        params.InterfaceAttributes.FirmwareVersion.ElementCount =
            (UINT32)sizeof(kFirmwareVersion);
        params.InterfaceAttributes.FirmwareVersion.pElements =
            const_cast<CHAR *>(kFirmwareVersion);
        /* MemoryInternallyAllocated stays FALSE: the buffer is ours and
         * the library must not try to free it. */
    }

    {
        WDI_STATION_CAPABILITIES *sc =
            &params.StationAttributes.StationCapabilities;

        /* These are the sizes the OS sizes its own requests against, so
         * they have to match what the device actually accepts rather
         * than being aspirational.
         *
         * Every field of WDI_STATION_CAPABILITIES is set here, including
         * the ones whose honest answer is zero. The struct is
         * zero-constructed, so an unset field and a deliberate zero look
         * identical afterwards -- and the difference between "we decided
         * this is zero" and "we never thought about it" is exactly what
         * went wrong below. */
        sc->ScanSSIDListSize       = Caps->max_scan_ssids;
        sc->DesiredBSSIDListSize   = 1;
        sc->DesiredSSIDListSize    = 1;
        sc->KeyMappingTableSize    = 1;   /* one PTK */
        sc->DefaultKeyTableSize    = 4;   /* four GTK slots */
        sc->MaxNumPerSTA           = 1;
        sc->MFPCapable             = 0;

        /* The host sends OID_WDI_SET_PRIVACY_EXEMPTION_LIST as part of
         * every connect -- it is one of the two OIDs
         * OID_DOT11_CONNECT_REQUEST translates into -- and this is the
         * number of entries we are telling it we can hold. It was left
         * at zero while the handler accepted any list it was given,
         * which is the adapter saying "I take none of these" and then
         * taking them. */
        sc->PrivacyExemptionListSize = 32;

        /* Length in bytes of the longest WEP key we accept. Zero was a
         * straight contradiction of the algorithm pair list further
         * down, which advertises Open/WEP-40 and Open/WEP-104 -- and
         * which Windows believes: `netsh wlan show drivers` lists both.
         * 13 bytes is the 104-bit key. */
        sc->WEPKeyValueMaxLength   = 13;

        /* No WMM/QoS, no host FIPS, no power save, no network-offload
         * (NLO) list, no HESSID, no disconnected standby, no FTM, no
         * WPA3 host FIPS, no RSN override. All genuinely absent rather
         * than merely unimplemented in this block. */
        sc->SupportedQOSFlags            = 0;
        sc->HostFIPSModeImplemented      = 0;
        sc->AutoPowerSaveMode            = FALSE;
        sc->uMaxNetworkOffloadListSize   = 0;
        sc->HESSIDConnectionSupported    = FALSE;
        sc->DisconnectedStandbySupported = FALSE;
        sc->FTMAsInitiatorSupport        = FALSE;
        sc->FTMNumberOfSupportedTargets  = 0;
        sc->HostWPA3FIPSModeEnabled      = FALSE;
        sc->rsnOverrideSupported         = FALSE;

        /* We associate only to a BSSID the host names in the connect's
         * preferred list -- the device is told a BSSID and uses it --
         * so the override stays FALSE. */
        sc->ConnectBSSSelectionOverride  = FALSE;

        /* "If the adapter would maintain the Station BSS List cache."
         *
         * TRUE, because we do. wdi_scan.c keeps a 32-entry BSSID-keyed
         * cache that outlives the scan that filled it, and oids.c
         * answers OID_WDI_GET_BSS_ENTRY_LIST out of it.
         *
         * This said FALSE, and FALSE is what tells the host "the
         * adapter has no BSS list, so don't ask" -- which is why
         * OID_WDI_GET_BSS_ENTRY_LIST has never once arrived in any
         * trace this project has collected, despite the handler being
         * there the whole time.
         *
         * That is not merely a wasted handler. It decides who owns the
         * list a connect is built from, and it lines up with what the
         * traces show: the host does OID_WDI_TASK_DOT11_RESET, then
         * roughly ten milliseconds later OID_DOT11_CONNECT_REQUEST, and
         * fails it in 1.5 ms with STATUS_NETWORK_UNREACHABLE without
         * ever issuing OID_WDI_TASK_CONNECT to us. A dot11 reset with
         * bSetDefaultMIB clears the station MIB, the BSS list included;
         * with the host owning the only copy there is nothing left to
         * build WDI_TLV_CONNECT_BSS_ENTRY from, and that TLV is
         * mandatory in WDI_TASK_CONNECT. The host then scans -- which
         * the traces also show, immediately after the failure -- but by
         * then the connect has already been answered.
         *
         * Saying TRUE gives it somewhere to ask. Whether that is the
         * whole story is not settled: wdiwifi.sys makes this decision
         * internally and its WPP is not in any capture we have been
         * able to take. What is settled is that this field was
         * describing an adapter we are not. */
        sc->BSSListCachemanagement = TRUE;
    }

    /* Bands and PHYs.
     *
     * Optional TLVs, so leaving them out produced a message the
     * generator was happy with — and an adapter the WLAN component
     * silently declined to use. It read the capabilities, applied the
     * adapter configuration, then closed the adapter without asking
     * anything further. An adapter that reports no bands has no
     * channels, and there is nothing to scan or connect on.
     *
     * Every sub-container of each is itself mandatory:
     * BandCapabilities / ValidPhyTypes / ValidChannelTypes /
     * ChannelWidthList, and PhyCapabilities / TxPowerLevelList /
     * DataRateList. A band with any one of them missing is worse than
     * no band at all — it fails the whole generate. */
    {
        ULONG nBands = 0, nPhys = 0;

        if (Caps->supported_channels_24 != 0) {
            WDI_BAND_INFO_CONTAINER *b = &scratch->Bands[nBands];

            b->BandCapabilities.BandID    = WDI_BAND_ID_2400;
            b->BandCapabilities.BandState = TRUE;

            scratch->PhyTypes[nBands][0] = WDI_PHY_TYPE_HRDSSS;  /* 11b */
            scratch->PhyTypes[nBands][1] = WDI_PHY_TYPE_ERP;     /* 11g */
            b->ValidPhyTypes.ElementCount = 2;
            b->ValidPhyTypes.pElements    = scratch->PhyTypes[nBands];

            b->ValidChannelTypes.ElementCount = VwifiFillChannels24(
                Caps->supported_channels_24, scratch->Chan24,
                RTL_NUMBER_OF(scratch->Chan24));
            b->ValidChannelTypes.pElements = scratch->Chan24;

            scratch->Widths[nBands] = 20;      /* MHz; no HT40 here */
            b->ChannelWidthList.ElementCount = 1;
            b->ChannelWidthList.pElements    = &scratch->Widths[nBands];
            nBands++;

            VwifiFillPhy(&scratch->Phys[nPhys], WDI_PHY_TYPE_HRDSSS,
                         &scratch->TxPower[nPhys], scratch->Rates[nPhys],
                         kRatesDsss, RTL_NUMBER_OF(kRatesDsss),
                         RTL_NUMBER_OF(kRatesDsss));
            nPhys++;
            /* Basic count 4: 1, 2, 5.5 and 11 Mbps. An 11g AP's basic
             * set is normally exactly those, and a station whose basic
             * set is a superset of the BSS's can always join. */
            VwifiFillPhy(&scratch->Phys[nPhys], WDI_PHY_TYPE_ERP,
                         &scratch->TxPower[nPhys], scratch->Rates[nPhys],
                         kRatesErp, RTL_NUMBER_OF(kRatesErp), 4);
            nPhys++;
        }

        if (Caps->supported_channels_5 != 0) {
            WDI_BAND_INFO_CONTAINER *b = &scratch->Bands[nBands];

            b->BandCapabilities.BandID    = WDI_BAND_ID_5000;
            b->BandCapabilities.BandState = TRUE;

            scratch->PhyTypes[nBands][0] = WDI_PHY_TYPE_OFDM;    /* 11a */
            b->ValidPhyTypes.ElementCount = 1;
            b->ValidPhyTypes.pElements    = scratch->PhyTypes[nBands];

            b->ValidChannelTypes.ElementCount = VwifiFillChannels5(
                Caps->supported_channels_5, scratch->Chan5,
                RTL_NUMBER_OF(scratch->Chan5));
            b->ValidChannelTypes.pElements = scratch->Chan5;

            scratch->Widths[nBands] = 20;
            b->ChannelWidthList.ElementCount = 1;
            b->ChannelWidthList.pElements    = &scratch->Widths[nBands];
            nBands++;

            VwifiFillPhy(&scratch->Phys[nPhys], WDI_PHY_TYPE_OFDM,
                         &scratch->TxPower[nPhys], scratch->Rates[nPhys],
                         kRatesOfdm, RTL_NUMBER_OF(kRatesOfdm), 3);
            nPhys++;
        }

        if (nBands != 0) {
            params.BandInfo.ElementCount = nBands;
            params.BandInfo.pElements    = scratch->Bands;
            params.Optional.BandInfo_IsPresent = TRUE;
        }
        if (nPhys != 0) {
            params.PhyInfo.ElementCount = nPhys;
            params.PhyInfo.pElements    = scratch->Phys;
            params.Optional.PhyInfo_IsPresent = TRUE;
        }
    }

    /* Which security this adapter can do.
     *
     * This list is what Windows matches a network's advertised security
     * against before it will build a profile for it, so a pair missing
     * here is a network the OS will not offer to connect to. It is also
     * what `netsh wlan show drivers` prints as "Authentication and
     * cipher supported in infrastructure mode".
     *
     * It used to hold exactly two pairs -- open/none and RSNA-PSK/CCMP.
     * That is enough for the AP on the medium in principle, but it
     * describes an adapter unlike any real one, and the cost of being
     * complete here is a few lines.
     *
     * Bounded by what actually works, not by what would look
     * impressive. The device's cipher enum runs to WEP40, WEP104,
     * TKIP, CCMP128 and GCMP256, and its AKMs to PSK, SAE and 802.1X --
     * but wdi_keys.c installs CCMP and nothing else, so TKIP and SAE
     * are absent here. Advertising a cipher the key path cannot install
     * would move the failure from "will not try" to "tries and fails
     * during the handshake", which is worse.
     */
    {
        static const struct { WDI_AUTH_ALGORITHM Auth; WDI_CIPHER_ALGORITHM Cipher; }
        kAlgoPairs[] = {
            /* Open networks. */
            { WDI_AUTH_ALGO_80211_OPEN,       WDI_CIPHER_ALGO_NONE    },

            /* WEP, both authentications. Obsolete, and the device does
             * carry the ciphers; listed because an adapter that claims
             * no WEP at all is an unusual thing for the OS to see. */
            { WDI_AUTH_ALGO_80211_OPEN,       WDI_CIPHER_ALGO_WEP40   },
            { WDI_AUTH_ALGO_80211_OPEN,       WDI_CIPHER_ALGO_WEP104  },
            { WDI_AUTH_ALGO_80211_SHARED_KEY, WDI_CIPHER_ALGO_WEP40   },
            { WDI_AUTH_ALGO_80211_SHARED_KEY, WDI_CIPHER_ALGO_WEP104  },

            /* WPA with AES. */
            { WDI_AUTH_ALGO_WPA,              WDI_CIPHER_ALGO_CCMP    },
            { WDI_AUTH_ALGO_WPA_PSK,          WDI_CIPHER_ALGO_CCMP    },

            /* WPA2. RSNA_PSK/CCMP is WPA2-Personal with AES, which is
             * what the AP on the medium runs and the pair this whole
             * list exists to get accepted. */
            { WDI_AUTH_ALGO_RSNA,             WDI_CIPHER_ALGO_CCMP    },
            { WDI_AUTH_ALGO_RSNA_PSK,         WDI_CIPHER_ALGO_CCMP    },
        };

        for (ULONG i = 0; i < RTL_NUMBER_OF(kAlgoPairs); i++) {
            scratch->Algos[i].AuthAlgorithm   = kAlgoPairs[i].Auth;
            scratch->Algos[i].CipherAlgorithm = kAlgoPairs[i].Cipher;
        }

        params.StationAttributes.UnicastAlgorithms.ElementCount =
            RTL_NUMBER_OF(kAlgoPairs);
        params.StationAttributes.UnicastAlgorithms.pElements = scratch->Algos;
        params.StationAttributes.Optional.UnicastAlgorithms_IsPresent = TRUE;

        /* The group cipher list. Same pairs: a station accepts the same
         * ciphers for group traffic as for its own. */
        params.StationAttributes.MulticastDataAlgorithms.ElementCount =
            RTL_NUMBER_OF(kAlgoPairs);
        params.StationAttributes.MulticastDataAlgorithms.pElements =
            scratch->Algos;
        params.StationAttributes.Optional.MulticastDataAlgorithms_IsPresent =
            TRUE;

        /* MulticastManagementAlgorithms stays absent, consistently with
         * MFPCapable being 0: with no management-frame protection there
         * is no management cipher to name. WPA2 does not require it. */
    }

    /* Datapath attributes. Marked optional="true" in WABIModel.xml and
     * required in practice.
     *
     * This container is where the WLAN component gets the numbers it
     * needs to build the WDI_TXRX_TARGET_CONFIGURATION it passes to
     * MiniportWdiTalTxRxStart -- MaxNumPeers above all. Without it,
     * there is nothing to start the data path with, so it never calls
     * TalTxRxStart, never reaches OID_WDI_TASK_CREATE_PORT, and closes
     * the adapter instead. The documented order is
     *
     *   TalTxRxInitialize -> GET_ADAPTER_CAPABILITIES ->
     *   SET_ADAPTER_CONFIGURATION -> [TASK_SET_RADIO_STATE] ->
     *   TalTxRxStart -> TASK_CREATE_PORT
     *
     * and the trace stopped dead between the third and the fifth, which
     * is exactly the shape of this being missing. */
    {
        WDI_DATAPATH_CAPABILITIES *dp =
            &params.DatapathAttributes.DataPathCapabilities;

        /* MEMORY_MAPPED: the target can reach system memory directly.
         * That is what a PCI device with DMA rings is; MESSAGE_BASED is
         * for USB and SDIO parts that have to be fed. */
        dp->InterconnectType = WDI_INTERCONNECT_MEMORY_MAPPED;

        /* Sizes the component's peer table. A station has exactly one
         * peer, its AP; the headroom is for the SoftAP mode the device
         * advertises. */
        dp->MaxNumPeers = 8;

        /* 0: no priority queueing in the target, so the component keeps
         * its full scheduler rather than handing us port-level queues
         * we have nowhere to put. */
        dp->TxTargetPriorityQueueing = 0;

        /* One element per frame. The TX path copies each frame into a
         * single contiguous slot in TxBufferPool; there is no
         * scatter-gather to offer. */
        dp->TxMaxScatterGatherElementsPerFrame = 1;

        /* 0: completions for every frame, not only flagged ones. */
        dp->TxExplicitSendCompleteFlagRequired = 0;

        /* No minimum -- the device has no per-frame overhead that would
         * make a small frame cost more than its size. */
        dp->TxMinEffectiveFrameSize = 0;

        /* Must be a power of two. Matches the per-frame slot size in
         * TxBufferPool, which is the real allocation granularity. */
        dp->TxFrameSizeGranularity = VWIFI_TLV_TX_SLOT_SIZE;

        /* 0: the target does not forward received frames back out. */
        dp->RxTxForwarding = 0;

        /* Units of 0.5 Mbps, so 108 == 54 Mbps, agreeing with the
         * MaxRxRate reported in the interface capabilities above. */
        dp->RxMaxThroughput = 108;

        params.DatapathAttributes.Optional.DataPathCapabilities_IsPresent =
            TRUE;
        params.Optional.DatapathAttributes_IsPresent = TRUE;
    }

    /* Still not filled: the country-region list, PM capabilities, P2P,
     * AP and virtualization attributes. All optional, and all describe
     * things this device does not do. Note that "optional" in
     * WABIModel.xml has now twice meant "the message parses without it
     * and the adapter does not work" -- treat the model as describing
     * the wire format, not the requirements. */

    NDIS_STATUS st = GenerateWdiGetAdapterCapabilities(
        &params, kHeaderReserve, &ctx, &outLen, &pOut);

    /* Freed on both paths, and only after Generate has returned: every
     * ArrayOfElements in the message points into the scratch block, and
     * the library reads through those pointers while it serialises.
     *
     * No destructors to run. Nothing was constructed, and nothing here
     * ever took ownership: MemoryInternallyAllocated is FALSE
     * throughout, so the library knows none of it is its to release. */
    ExFreePoolWithTag(scratch, VWIFI_TLV_PARAMS_TAG);
    ExFreePoolWithTag(pParams, VWIFI_TLV_PARAMS_TAG);

    if (st != NDIS_STATUS_SUCCESS) return st;

    *Buffer    = pOut;
    *BufferLen = outLen;
    return NDIS_STATUS_SUCCESS;
}

/* ============================================================
 * Ports
 *
 * The host assigns the port id -- it is in the WDI_MESSAGE_HEADER of
 * the request, not in the TLV body. What the body carries is what the
 * port is *for*: which operation modes the host may later configure on
 * it, and the NDIS port number it will be registered under.
 * ============================================================ */

extern "C"
NDIS_STATUS
VwifiTlvParseCreatePort(
    ULONG PeerVersion,
    const VOID *Buffer,
    ULONG BufferLen,
    PULONG OpModeMask,
    PULONG NdisPortNumber,
    UCHAR *MacOut,
    BOOLEAN *MacPresent)
{
    TLV_CONTEXT ctx = MakeCtx(PeerVersion);
    WDI_TASK_CREATE_PORT_PARAMETERS parsed = {};
    NDIS_STATUS st;

    *OpModeMask     = 0;
    *NdisPortNumber = 0;
    *MacPresent     = FALSE;

    st = ParseWdiTaskCreatePort(BufferLen,
                                static_cast<const UINT8 *>(Buffer),
                                &ctx, &parsed);
    if (st != NDIS_STATUS_SUCCESS) {
        return st;
    }

    *OpModeMask     = parsed.CreatePortParameters.OpModeMask;
    *NdisPortNumber = parsed.CreatePortParameters.NdisPortNumber;

    if (parsed.Optional.MacAddress_IsPresent) {
        RtlCopyMemory(MacOut, parsed.MacAddress.Address, 6);
        *MacPresent = TRUE;
    }

    CleanupParsedWdiTaskCreatePort(&parsed);
    return NDIS_STATUS_SUCCESS;
}

/* The CREATE_PORT completion is NOT header-only, whatever the task's
 * own results message says.
 *
 * WABIModel.xml carries two separate messages here and it is easy to
 * read the wrong one:
 *
 *   WDI_TASK_CREATE_PORT / WDI_TASK_CREATE_PORT_RESULTS, FromIhv,
 *       "No TLV data needed, header is sufficient"      <- the M2
 *   WDI_INDICATION_CREATE_PORT_COMPLETE,
 *       containerRef WDI_TLV_PORT_ATTRIBUTES optional="false"  <- the M3
 *
 * The completion is the second one, and its PortAttributes container is
 * mandatory. Sent as a bare header, the component reads a create-port
 * completion that names no port, and tears the adapter down instead of
 * adding it. Every other completion this driver sends -- delete port,
 * scan, connect, change operation mode -- really is header-only; this
 * is the exception.
 */
extern "C"
NDIS_STATUS
VwifiTlvGenerateCreatePortComplete(
    ULONG PeerVersion,
    const UCHAR *Mac,
    ULONG PortNumber,
    VOID **Buffer,
    PULONG BufferLen)
{
    TLV_CONTEXT ctx = MakeCtx(PeerVersion);
    WDI_INDICATION_CREATE_PORT_COMPLETE_PARAMETERS params = {};
    UINT8 *pOut = nullptr;
    ULONG outLen = 0;

    RtlCopyMemory(params.PortAttributes.MacAddress.Address, Mac, 6);
    params.PortAttributes.PortNumber = (UINT16)PortNumber;

    NDIS_STATUS st = GenerateWdiIndicationCreatePortComplete(
        &params, kHeaderReserve, &ctx, &outLen, &pOut);
    if (st != NDIS_STATUS_SUCCESS) return st;

    *Buffer    = pOut;
    *BufferLen = outLen;
    return NDIS_STATUS_SUCCESS;
}

extern "C"
NDIS_STATUS
VwifiTlvParseDeletePort(
    ULONG PeerVersion,
    const VOID *Buffer,
    ULONG BufferLen,
    PULONG PortNumber)
{
    TLV_CONTEXT ctx = MakeCtx(PeerVersion);
    WDI_TASK_DELETE_PORT_PARAMETERS parsed = {};
    NDIS_STATUS st;

    *PortNumber = 0;

    st = ParseWdiTaskDeletePort(BufferLen,
                                static_cast<const UINT8 *>(Buffer),
                                &ctx, &parsed);
    if (st != NDIS_STATUS_SUCCESS) {
        return st;
    }

    *PortNumber = parsed.DeletePortParameters.PortNumber;

    CleanupParsedWdiTaskDeletePort(&parsed);
    return NDIS_STATUS_SUCCESS;
}

/* ============================================================
 * Statistics
 *
 * Both containers are optional="false" and multiContainer="true", so
 * the reply needs at least one of each -- an empty list is not a valid
 * answer here, which is why returning NOT_SUPPORTED was the only option
 * until now.
 *
 * The counters are honest zeroes. This device keeps none: the medium
 * has no retries, no ACK failures and no FCS errors to count, and the
 * per-frame outcomes those fields describe do not exist in it. Zero is
 * what "this never happened" looks like, and it is a truer answer than
 * an invented number.
 *
 * One MAC entry, for the broadcast address, which the field comment
 * defines as the multicast/broadcast bucket -- there is no per-peer
 * accounting to report, and claiming a peer that may not exist would be
 * worse than reporting the aggregate. One PHY entry, matching the ERP
 * PHY the capabilities advertise for 2.4 GHz.
 * ============================================================ */

extern "C"
NDIS_STATUS
VwifiTlvGenerateStatistics(
    ULONG PeerVersion,
    VOID **Buffer,
    PULONG BufferLen)
{
    TLV_CONTEXT ctx = MakeCtx(PeerVersion);
    WDI_GET_STATISTICS_PARAMETERS params = {};
    WDI_MAC_STATISTICS_CONTAINER mac = {};
    WDI_PHY_STATISTICS_CONTAINER phy = {};
    UINT8 *pOut = nullptr;
    ULONG outLen = 0;

    RtlFillMemory(mac.MACAddress.Address, 6, 0xFF);

    phy.PhyType = WDI_PHY_TYPE_ERP;

    params.PeerMACStatistics.ElementCount = 1;
    params.PeerMACStatistics.pElements    = &mac;
    params.PhyStatistics.ElementCount     = 1;
    params.PhyStatistics.pElements        = &phy;

    NDIS_STATUS st = GenerateWdiGetStatistics(&params, kHeaderReserve,
                                              &ctx, &outLen, &pOut);
    if (st != NDIS_STATUS_SUCCESS) return st;

    *Buffer    = pOut;
    *BufferLen = outLen;
    return NDIS_STATUS_SUCCESS;
}

/* ============================================================
 * Dot11 reset / receive packet filter
 * ============================================================ */

extern "C"
NDIS_STATUS
VwifiTlvParseDot11Reset(
    ULONG PeerVersion,
    const VOID *Buffer,
    ULONG BufferLen,
    BOOLEAN *SetDefaultMib,
    UCHAR *MacOut,
    BOOLEAN *MacPresent)
{
    TLV_CONTEXT ctx = MakeCtx(PeerVersion);
    WDI_TASK_DOT11_RESET_PARAMETERS parsed = {};
    NDIS_STATUS st;

    *SetDefaultMib = FALSE;
    *MacPresent    = FALSE;

    st = ParseWdiTaskDot11Reset(BufferLen,
                                static_cast<const UINT8 *>(Buffer),
                                &ctx, &parsed);
    if (st != NDIS_STATUS_SUCCESS) {
        return st;
    }

    *SetDefaultMib = parsed.Dot11ResetParameters.SetDefaultMIB ? TRUE : FALSE;

    if (parsed.Optional.ResetMACAddress_IsPresent) {
        RtlCopyMemory(MacOut, parsed.ResetMACAddress.Address, 6);
        *MacPresent = TRUE;
    }

    CleanupParsedWdiTaskDot11Reset(&parsed);
    return NDIS_STATUS_SUCCESS;
}

extern "C"
NDIS_STATUS
VwifiTlvParseReceivePacketFilter(
    ULONG PeerVersion,
    const VOID *Buffer,
    ULONG BufferLen,
    PULONG Filter)
{
    TLV_CONTEXT ctx = MakeCtx(PeerVersion);
    WDI_SET_RECEIVE_PACKET_FILTER_PARAMETERS parsed = {};
    NDIS_STATUS st;

    *Filter = 0;

    st = ParseWdiSetReceivePacketFilter(BufferLen,
                                        static_cast<const UINT8 *>(Buffer),
                                        &ctx, &parsed);
    if (st != NDIS_STATUS_SUCCESS) {
        return st;
    }

    /* UINT32_CONTAINER is a plain typedef of UINT32 -- the member is the
     * value, not a sub-struct. Same shape as WDI_OPERATION_MODE. */
    *Filter = parsed.PacketFilterType;

    CleanupParsedWdiSetReceivePacketFilter(&parsed);
    return NDIS_STATUS_SUCCESS;
}

/* ============================================================
 * Adapter configuration
 *
 * WDI_SET_ADAPTER_CONFIGURATION carries firmware-level settings, and
 * for this device exactly one of them means anything: the MAC address
 * the OS wants the adapter to use. Everything else in the message is
 * about P2P group-owner reset policy, unreachability detection, NLO
 * scan mode and PLDR — firmware concepts a virtual device has no
 * equivalent for.
 *
 * MacAddress is optional. Absent means "keep what you have", which is
 * the device's own default MAC, so a missing address is not an error.
 * ============================================================ */

extern "C"
NDIS_STATUS
VwifiTlvParseAdapterConfiguration(
    ULONG PeerVersion,
    const VOID *Buffer,
    ULONG BufferLen,
    UCHAR *MacOut,
    BOOLEAN *MacPresent)
{
    TLV_CONTEXT ctx = MakeCtx(PeerVersion);
    WDI_SET_FIRMWARE_CONFIGURATION_PARAMETERS parsed = {};
    NDIS_STATUS st;

    *MacPresent = FALSE;

    st = ParseWdiSetAdapterConfigurationToIhv(
             BufferLen, static_cast<const UINT8 *>(Buffer), &ctx, &parsed);
    if (st != NDIS_STATUS_SUCCESS) {
        return st;
    }

    if (parsed.Optional.MacAddress_IsPresent) {
        RtlCopyMemory(MacOut, parsed.MacAddress.Address, 6);
        *MacPresent = TRUE;
    }

    CleanupParsedWdiSetAdapterConfigurationToIhv(&parsed);
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
