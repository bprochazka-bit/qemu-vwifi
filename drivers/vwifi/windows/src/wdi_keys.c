/*
 * vwifi — wdi_keys.c
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Phase 4: cipher key installation.
 *
 * The 4-way handshake runs entirely in the OS — and specifically in
 * nwifi.sys, in the kernel, not in a user-mode supplicant. ndisuio
 * carries 802.1X/EAP; the WPA2 key exchange never leaves ring 0. The
 * EAPOL-Key frames travel our normal data path (the device passes
 * EAPOL through unencrypted — see the eth_is_eapol() rule in
 * vwifi_device.c) and nwifi answers them itself. When the handshake
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

/* Key length the device's ciphers require, or 0 for one it cannot do.
 *
 * The cipher comes from the TLV rather than being assumed: this used to
 * hardcode CCMP-128 and reject anything that was not 16 bytes, which
 * happens to be right for WPA2-PSK and silently wrong for everything
 * else the capabilities advertise. */
static ULONG
VwifiCipherKeyLength(_In_ USHORT Cipher)
{
    switch (Cipher) {
    case VWIFI_CIPHER_CCMP128: return 16;
    case VWIFI_CIPHER_GCMP256: return 32;
    default:                   return 0;
    }
}

static NDIS_STATUS
VwifiInstallOneKey(_Inout_ PVWIFI_ADAPTER Adapter,
                   _In_ BOOLEAN Pairwise,
                   _In_ UCHAR KeyIndex,
                   _In_ USHORT Cipher,
                   _In_reads_bytes_(6) const UCHAR *PeerMac,
                   _In_reads_bytes_(KeyLen) const UCHAR *KeyValue,
                   _In_ ULONG KeyLen)
{
    struct vwifi_key k;
    ULONG outLen = 0;
    ULONG want = VwifiCipherKeyLength(Cipher);
    NDIS_STATUS status;

    if (want == 0) {
        VWIFI_ERR("unsupported cipher %u for the %s key idx=%u",
                  Cipher, Pairwise ? "pairwise" : "group", KeyIndex);
        return NDIS_STATUS_NOT_SUPPORTED;
    }
    if (KeyLen != want) {
        VWIFI_ERR("key length %u does not match cipher %u (wants %u)",
                  KeyLen, Cipher, want);
        return NDIS_STATUS_INVALID_LENGTH;
    }

    RtlZeroMemory(&k, sizeof(k));
    k.id.pairwise = Pairwise ? 1 : 0;
    k.id.key_idx  = KeyIndex;
    if (Pairwise && PeerMac) {
        RtlCopyMemory(k.id.mac, PeerMac, 6);
    }
    k.cipher  = Cipher;
    k.key_len = (UINT16)KeyLen;
    RtlCopyMemory(k.key, KeyValue, KeyLen);

    status = VwifiCtrlSendSync(Adapter, VWIFI_OP_SET_KEY,
                               &k, sizeof(k), NULL, &outLen);

    /* Wipe our stack copy — key material shouldn't linger. */
    RtlSecureZeroMemory(&k, sizeof(k));

    if (status != NDIS_STATUS_SUCCESS) {
        VWIFI_ERR("device rejected SET_KEY (%s idx=%u): 0x%x",
                  Pairwise ? "pairwise" : "group", KeyIndex, status);
        return status;
    }

    VWIFI_INFO("installed %s key idx=%u cipher %u len %u",
               Pairwise ? "pairwise (PTK)" : "group (GTK)",
               KeyIndex, Cipher, KeyLen);
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
                                    keys[i].Cipher,
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
    VWIFI_TLV_KEY keys[VWIFI_MAX_KEYS_PER_REQUEST];
    ULONG count = 0;
    ULONG i;
    NDIS_STATUS status;
    PVOID tlvBuf;
    ULONG tlvLen;

    status = VwifiGetTlvPayload(Req, &tlvBuf, &tlvLen);
    if (status != NDIS_STATUS_SUCCESS) return status;

    status = VwifiTlvParseDeleteCipherKeys(Adapter->WdiPeerVersion,
                                           tlvBuf, tlvLen,
                                           keys, RTL_NUMBER_OF(keys), &count);
    if (status != NDIS_STATUS_SUCCESS) {
        VWIFI_ERR("delete cipher key TLV parse failed 0x%x", status);
        return status;
    }

    /* Which keys, rather than "the pairwise one, probably".
     *
     * This used to ignore the request entirely and delete pairwise slot
     * 0, on the reasoning that a disconnect is the common case and the
     * device clears keys on DISCONNECT anyway. That is true of a
     * disconnect and false of a group rekey, where the OS deletes the
     * old GTK index and installs a new one -- and deleting the PTK
     * instead would take the unicast link down in the middle of an
     * association that was working. The parser exists; use it. */
    for (i = 0; i < count; i++) {
        struct vwifi_key_id id;
        ULONG outLen = 0;

        RtlZeroMemory(&id, sizeof(id));
        id.pairwise = keys[i].Pairwise ? 1 : 0;
        id.key_idx  = keys[i].KeyIndex;
        if (keys[i].Pairwise) {
            RtlCopyMemory(id.mac, keys[i].PeerMac, 6);
        }

        status = VwifiCtrlSendSync(Adapter, VWIFI_OP_DEL_KEY,
                                   &id, sizeof(id), NULL, &outLen);
        if (status != NDIS_STATUS_SUCCESS) {
            /* Not fatal, and not silent. A key the device has already
             * dropped -- on DISCONNECT, say -- is the ordinary reason,
             * and failing the OID over it would turn a tidy-up into a
             * connection error. */
            VWIFI_WARN("device refused DEL_KEY (%s idx=%u): 0x%x",
                       keys[i].Pairwise ? "pairwise" : "group",
                       keys[i].KeyIndex, status);
            continue;
        }
        VWIFI_INFO("deleted %s key idx=%u",
                   keys[i].Pairwise ? "pairwise" : "group",
                   keys[i].KeyIndex);
    }

    VWIFI_INFO("DeleteCipherKeys: %u key(s) requested", count);
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

    if (PayloadLen < sizeof(*id)) return;
    VWIFI_INFO("device confirmed %s key idx=%u installed",
               id->pairwise ? "pairwise" : "group", id->key_idx);

    /* The pairwise key is the end of the 4-way handshake, and so the
     * end of the window in which the device refuses to sweep.
     *
     * The release call is a backstop now rather than the main path: a
     * scan held behind a connect is drained at CONNECT_COMPLETE, and
     * one held for the handshake alone is bounded by its own short
     * watchdog, because an outstanding OID blocks the very request that
     * gets us here. It stays because clearing the flag without
     * offering the held task a drain would be a hold with no owner.
     *
     * The group key is not the end of anything -- a rekey installs one
     * mid-association -- so only the pairwise one clears the flag. */
    if (id->pairwise && Adapter->HandshakePending) {
        Adapter->HandshakePending = FALSE;
        VWIFI_INFO("4-way handshake complete -- the radio is free to "
                   "sweep again");
        VwifiScanReleaseDeferred(Adapter, TRUE);
    }
}
