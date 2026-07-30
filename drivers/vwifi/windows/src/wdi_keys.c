/*
 * vwifi — wdi_keys.c
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Phase 4: cipher key installation.
 *
 * The 4-way handshake runs entirely in the OS: wlansvc and the
 * supplicant exchange EAPOL-Key frames through our normal data
 * path (the device passes EAPOL through unencrypted — see the
 * eth_is_eapol() rule in vwifi_device.c). When the handshake
 * completes, the OS hands us the derived keys via:
 *
 *   OID_WDI_SET_ADD_CIPHER_KEYS     — install PTK, then GTK
 *   OID_WDI_SET_DELETE_CIPHER_KEYS  — remove them
 *
 * Our job is purely plumbing: unpack the keys and push them to the
 * device, which does the actual CCMP.
 *
 * PAIRWISE vs GROUP — the distinction that breaks things if you get
 * it wrong. The OS sends both, often back to back:
 *   - PTK: pairwise=TRUE, protects unicast to/from the AP
 *   - GTK: pairwise=FALSE with a key index 1-3, protects broadcast
 *          and multicast from the AP
 * Installing a GTK into the pairwise slot (or vice versa) means the
 * first data frame decrypts to garbage. The device keeps separate
 * slots and separate PN spaces for each; this file must classify
 * them correctly.
 *
 * TLV caveat: as with wdi_scan.c / wdi_connect.c, the parse below is
 * marked TLV BOUNDARY and needs adjusting against a pinned WDK.
 */

#include "vwifi_drv.h"
#include <dot11wdi.h>

/* ============================================================
 * TLV BOUNDARY — parse WDI_TLV_CIPHER_KEY entries
 *
 * Real implementation:
 *   WDI_TLV_ADD_CIPHER_KEYS_PARAMETERS *p = NULL;
 *   ParseWdiTlvAddCipherKeysParameters(buf, len, &ctx, &p);
 * then for each p->CipherKeys[i]:
 *   - key->KeyType     -> pairwise vs group
 *   - key->KeyIndex    -> 0 for PTK, 1..3 for GTK
 *   - key->CipherAlgo  -> map DOT11_CIPHER_ALGO_CCMP to
 *                         VWIFI_CIPHER_CCMP128
 *   - key->KeyValue    -> the 16 raw key bytes
 *   - key->MacAddr     -> peer for pairwise; zeros for group
 * ============================================================ */

static NDIS_STATUS
VwifiInstallOneKey(_Inout_ PVWIFI_ADAPTER Adapter,
                   _In_ BOOLEAN Pairwise,
                   _In_ UCHAR KeyIndex,
                   _In_reads_bytes_(6) const UCHAR *PeerMac,
                   _In_reads_bytes_(KeyLen) const UCHAR *KeyValue,
                   _In_ ULONG KeyLen)
{
    struct vwifi_key k;
    ULONG outLen = 0;
    NDIS_STATUS status;

    if (KeyLen != 16) {
        VWIFI_ERR("unsupported key length %u (CCMP-128 needs 16)", KeyLen);
        return NDIS_STATUS_NOT_SUPPORTED;
    }

    RtlZeroMemory(&k, sizeof(k));
    k.id.pairwise = Pairwise ? 1 : 0;
    k.id.key_idx  = KeyIndex;
    if (Pairwise && PeerMac) {
        RtlCopyMemory(k.id.mac, PeerMac, 6);
    }
    k.cipher  = VWIFI_CIPHER_CCMP128;
    k.key_len = 16;
    RtlCopyMemory(k.key, KeyValue, 16);

    status = VwifiCtrlSendSync(Adapter, VWIFI_OP_SET_KEY,
                               &k, sizeof(k), NULL, &outLen);

    /* Wipe our stack copy — key material shouldn't linger. */
    RtlSecureZeroMemory(&k, sizeof(k));

    if (status != NDIS_STATUS_SUCCESS) {
        VWIFI_ERR("device rejected SET_KEY (%s idx=%u): 0x%x",
                  Pairwise ? "pairwise" : "group", KeyIndex, status);
        return status;
    }

    VWIFI_INFO("installed %s key idx=%u",
               Pairwise ? "pairwise (PTK)" : "group (GTK)", KeyIndex);
    return NDIS_STATUS_SUCCESS;
}

/* Room for a PTK plus the three group-key slots. WDI never sends more
 * than that in one request for a STA. */
#define VWIFI_MAX_KEYS_PER_REQUEST 4

NDIS_STATUS
VwifiHandleAddCipherKeys(_Inout_ PVWIFI_ADAPTER Adapter,
                         _In_ PNDIS_OID_REQUEST Req)
{
    VWIFI_TLV_KEY keys[VWIFI_MAX_KEYS_PER_REQUEST];
    ULONG count = 0;
    ULONG i;
    NDIS_STATUS status;
    PVOID tlvBuf;
    ULONG tlvLen;

    status = VwifiGetTlvPayload(Req, &tlvBuf, &tlvLen);
    if (status != NDIS_STATUS_SUCCESS) return status;

    status = VwifiTlvParseAddCipherKeys(Adapter->WdiPeerVersion,
                                        tlvBuf, tlvLen,
                                        keys, RTL_NUMBER_OF(keys), &count);
    if (status != NDIS_STATUS_SUCCESS) {
        VWIFI_ERR("cipher key TLV parse failed 0x%x", status);
        return status;
    }

    for (i = 0; i < count; i++) {
        status = VwifiInstallOneKey(Adapter,
                                    keys[i].Pairwise,
                                    keys[i].KeyIndex,
                                    keys[i].PeerMac,
                                    keys[i].KeyValue,
                                    keys[i].KeyLength);
        if (status != NDIS_STATUS_SUCCESS) {
            RtlSecureZeroMemory(keys, sizeof(keys));
            return status;
        }
    }

    /* Key material has no business outliving this frame. */
    RtlSecureZeroMemory(keys, sizeof(keys));

    VWIFI_INFO("AddCipherKeys: installed %u key(s)", count);
    return NDIS_STATUS_SUCCESS;
}

NDIS_STATUS
VwifiHandleDeleteCipherKeys(_Inout_ PVWIFI_ADAPTER Adapter,
                            _In_ PNDIS_OID_REQUEST Req)
{
    struct vwifi_key_id id;
    ULONG outLen = 0;

    UNREFERENCED_PARAMETER(Req);

    /* Real implementation parses the TLV for which keys to remove.
     * Until then, drop the pairwise key — the common case is a
     * disconnect tearing down the PTK, and the device also clears
     * keys on DISCONNECT, so this is belt-and-braces. */
    RtlZeroMemory(&id, sizeof(id));
    id.pairwise = 1;
    id.key_idx  = 0;

    (VOID)VwifiCtrlSendSync(Adapter, VWIFI_OP_DEL_KEY,
                            &id, sizeof(id), NULL, &outLen);
    return NDIS_STATUS_SUCCESS;
}

/* ============================================================
 * KEY_INSTALLED device event
 * ============================================================ */

VOID
VwifiKeysOnInstalled(_Inout_ PVWIFI_ADAPTER Adapter,
                     _In_reads_bytes_(PayloadLen) const VOID *Payload,
                     _In_ ULONG PayloadLen)
{
    const struct vwifi_key_id *id = Payload;

    UNREFERENCED_PARAMETER(Adapter);

    if (PayloadLen < sizeof(*id)) return;
    VWIFI_INFO("device confirmed %s key idx=%u installed",
               id->pairwise ? "pairwise" : "group", id->key_idx);
}
