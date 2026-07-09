/*
 * Virtual Atheros AR9285 – CCMP (AES-CCM) crypto engine emulation
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Real ath9k hardware offloads WPA2/CCMP encryption to a silicon crypto
 * engine driven by the on-chip key cache.  Because the driver advertises
 * that capability to mac80211, mac80211 stops doing software CCMP and
 * relies on the hardware for the data path.  This virtual device has no
 * such engine, so without emulation every encrypted data frame is dropped
 * (the negotiated PTK/GTK are never usable) and WPA2 associations carry no
 * traffic.  This header implements the AES-CCM transform the hardware would
 * perform, driven by the key material the driver programs into the key
 * cache.
 *
 * The implementation follows IEEE 802.11-2016 §12.5.3 (CCMP) and the CCM
 * mode of RFC 3610 (M = 8, L = 2).  The nonce/AAD construction mirrors the
 * mac80211 reference (net/mac80211/aes_ccm.c, ccmp_special_blocks()).
 *
 * The only primitive required is the AES-128 forward block cipher.  The
 * caller supplies it through the AES_KEY / AES_set_encrypt_key() / AES_encrypt()
 * API, which is provided identically by QEMU's "crypto/aes.h" (in the device
 * build) and by OpenSSL's <openssl/aes.h> (in the standalone unit test).
 * This header therefore does NOT include an AES header itself; the includer
 * must pull one in first.
 */

#ifndef VWIFI_ATH9K_CRYPTO_H
#define VWIFI_ATH9K_CRYPTO_H

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

/* CCMP constants */
#define VWIFI_CCMP_HDR_LEN   8   /* IV / PN header inserted by mac80211    */
#define VWIFI_CCMP_MIC_LEN   8   /* MIC appended by the (emulated) hardware */
#define VWIFI_CCM_L          2   /* length-field size (octets)             */
#define VWIFI_CCM_M          8   /* MIC size (octets)                      */
#define VWIFI_AES_BLK        16

/* ------------------------------------------------------------------ *
 *  802.11 header field extraction
 * ------------------------------------------------------------------ */

/* 802.11 header length (bytes) implied by the Frame Control field, matching
 * the logic used elsewhere in the device for TX/RX padding. */
static inline int vwifi_dot11_hdrlen(const uint8_t *frame, int frame_len)
{
    int hdrlen = 24;
    uint16_t fc;

    if (frame_len < 2) {
        return hdrlen;
    }
    fc = (uint16_t)(frame[0] | (frame[1] << 8));

    /* QoS data frame → +2 (QoS Control field) */
    if (((fc >> 2) & 3) == 2 && ((fc >> 4) & 0x8)) {
        hdrlen = 26;
    }
    /* 4-address frame (ToDS + FromDS) → +6 (Address 4) */
    if ((fc & 0x0300) == 0x0300) {
        hdrlen += 6;
    }
    return hdrlen;
}

/* True if the Protected-Frame bit is set in Frame Control. */
static inline bool vwifi_dot11_protected(const uint8_t *frame, int frame_len)
{
    if (frame_len < 2) {
        return false;
    }
    return (frame[1] & 0x40) != 0; /* IEEE80211_FCTL_PROTECTED = 0x4000 */
}

/* ------------------------------------------------------------------ *
 *  CCM nonce and AAD (Additional Authentication Data) construction
 * ------------------------------------------------------------------ */

/*
 * Build the 13-octet CCM nonce and the variable-length AAD for one MPDU.
 * `frame` points at the 802.11 header; the CCMP header (with the PN) sits at
 * offset `hdrlen`.  `aad` must have room for 30 octets.  Returns the AAD
 * length, or -1 if the frame is too short.
 */
static inline int vwifi_ccmp_build(const uint8_t *frame, int frame_len,
                                   int hdrlen, uint8_t nonce[13],
                                   uint8_t aad[30])
{
    uint16_t fc, mask_fc, sc;
    int type, stype, is_qos, is_mgmt, a4;
    const uint8_t *a1, *a2, *a3, *ccmp;
    uint8_t pn[6], qos_tid = 0;
    int p;

    if (frame_len < hdrlen + VWIFI_CCMP_HDR_LEN || hdrlen < 24) {
        return -1;
    }

    fc      = (uint16_t)(frame[0] | (frame[1] << 8));
    type    = (fc >> 2) & 3;
    stype   = (fc >> 4) & 0xf;
    is_qos  = (type == 2) && (stype & 0x8);
    is_mgmt = (type == 0);
    a4      = (fc & 0x0300) == 0x0300;

    a1   = frame + 4;      /* Address 1 (RA) */
    a2   = frame + 10;     /* Address 2 (TA) */
    a3   = frame + 16;     /* Address 3      */
    ccmp = frame + hdrlen; /* CCMP header    */

    if (is_qos) {
        qos_tid = frame[hdrlen - 2] & 0x0f; /* QoS Control precedes payload */
    }

    /* PN from the CCMP header, most-significant octet first (for the nonce).
     * CCMP header layout: PN0 PN1 rsvd keyid PN2 PN3 PN4 PN5 */
    pn[0] = ccmp[7]; /* PN5 (MSB) */
    pn[1] = ccmp[6]; /* PN4 */
    pn[2] = ccmp[5]; /* PN3 */
    pn[3] = ccmp[4]; /* PN2 */
    pn[4] = ccmp[1]; /* PN1 */
    pn[5] = ccmp[0]; /* PN0 (LSB) */

    /* Nonce: flags | A2 | PN */
    nonce[0] = (uint8_t)((is_mgmt ? 0x10 : 0x00) | (qos_tid & 0x0f));
    memcpy(nonce + 1, a2, 6);
    memcpy(nonce + 7, pn, 6);

    /* AAD: masked FC | A1 | A2 | A3 | masked SC | [A4] | [QoS] */
    mask_fc  = fc & ~0x3800;       /* clear Retry, PwrMgt, MoreData */
    if (!is_mgmt) {
        mask_fc &= ~0x0070;        /* clear subtype bits for data frames */
    }
    mask_fc |= 0x4000;             /* force Protected */

    p = 0;
    aad[p++] = (uint8_t)(mask_fc & 0xff);
    aad[p++] = (uint8_t)((mask_fc >> 8) & 0xff);
    memcpy(aad + p, a1, 6); p += 6;
    memcpy(aad + p, a2, 6); p += 6;
    memcpy(aad + p, a3, 6); p += 6;

    sc = (uint16_t)((frame[22] | (frame[23] << 8)) & 0x000f); /* keep frag num */
    aad[p++] = (uint8_t)(sc & 0xff);
    aad[p++] = (uint8_t)((sc >> 8) & 0xff);

    if (a4) {
        memcpy(aad + p, frame + 24, 6); /* Address 4 */
        p += 6;
    }
    if (is_qos) {
        aad[p++] = qos_tid;
        aad[p++] = 0;
    }
    return p;
}

/* ------------------------------------------------------------------ *
 *  CCM core: CBC-MAC (authentication) and CTR (encryption)
 * ------------------------------------------------------------------ */

static inline void vwifi_ccm_xor16(uint8_t *dst, const uint8_t *src)
{
    int i;
    for (i = 0; i < VWIFI_AES_BLK; i++) {
        dst[i] ^= src[i];
    }
}

/*
 * CBC-MAC over B0 | (len(a) | a padded) | (m padded), producing the
 * 8-octet unencrypted MIC (tag T).
 */
static inline void vwifi_ccm_cbc_mac(const AES_KEY *aes, const uint8_t nonce[13],
                                     const uint8_t *aad, int aad_len,
                                     const uint8_t *data, int data_len,
                                     uint8_t mic[8])
{
    uint8_t x[VWIFI_AES_BLK], b[VWIFI_AES_BLK];
    int i, pos;

    /* B0 = flags | nonce | l(m).  flags = 64*Adata + 8*M' + L'.
     * The Adata bit is set only when AAD is present (always true for real
     * 802.11 MPDUs, which carry a masked header as AAD). */
    b[0] = (uint8_t)((aad_len > 0 ? 0x40 : 0x00) |
                     (((VWIFI_CCM_M - 2) / 2) << 3) | (VWIFI_CCM_L - 1));
    memcpy(b + 1, nonce, 13);
    b[14] = (uint8_t)((data_len >> 8) & 0xff);
    b[15] = (uint8_t)(data_len & 0xff);
    AES_encrypt(b, x, aes);                 /* X1 = E(B0), since X0 = 0 */

    /* AAD blocks: 2-octet length prefix, then AAD, zero-padded.  Omitted
     * entirely when there is no AAD. */
    if (aad_len > 0) {
        memset(b, 0, sizeof(b));
        b[0] = (uint8_t)((aad_len >> 8) & 0xff);
        b[1] = (uint8_t)(aad_len & 0xff);
        pos = 2;
        for (i = 0; i < aad_len; i++) {
            b[pos++] = aad[i];
            if (pos == VWIFI_AES_BLK) {
                vwifi_ccm_xor16(x, b);
                AES_encrypt(x, x, aes);
                memset(b, 0, sizeof(b));
                pos = 0;
            }
        }
        if (pos > 0) {
            vwifi_ccm_xor16(x, b);
            AES_encrypt(x, x, aes);
        }
    }

    /* Payload blocks, zero-padded. */
    for (i = 0; i < data_len; i += VWIFI_AES_BLK) {
        int n = data_len - i;
        if (n > VWIFI_AES_BLK) {
            n = VWIFI_AES_BLK;
        }
        memset(b, 0, sizeof(b));
        memcpy(b, data + i, n);
        vwifi_ccm_xor16(x, b);
        AES_encrypt(x, x, aes);
    }

    memcpy(mic, x, VWIFI_CCM_M);
}

/*
 * CTR mode: encrypts (or decrypts – it is symmetric) `data` in place and
 * transforms the MIC with the S0 keystream block (T <-> U).  A0 encrypts the
 * MIC; A1, A2, … encrypt the payload.
 */
static inline void vwifi_ccm_ctr(const AES_KEY *aes, const uint8_t nonce[13],
                                 uint8_t *data, int data_len, uint8_t mic[8])
{
    uint8_t a[VWIFI_AES_BLK], s[VWIFI_AES_BLK];
    uint16_t ctr;
    int i;

    a[0] = (uint8_t)(VWIFI_CCM_L - 1);      /* CTR flags = L' */
    memcpy(a + 1, nonce, 13);

    /* A0 → MIC */
    a[14] = 0;
    a[15] = 0;
    AES_encrypt(a, s, aes);
    for (i = 0; i < VWIFI_CCM_M; i++) {
        mic[i] ^= s[i];
    }

    /* A1.. → payload */
    for (i = 0, ctr = 1; i < data_len; i += VWIFI_AES_BLK, ctr++) {
        int n = data_len - i;
        int j;
        if (n > VWIFI_AES_BLK) {
            n = VWIFI_AES_BLK;
        }
        a[14] = (uint8_t)((ctr >> 8) & 0xff);
        a[15] = (uint8_t)(ctr & 0xff);
        AES_encrypt(a, s, aes);
        for (j = 0; j < n; j++) {
            data[i + j] ^= s[j];
        }
    }
}

/* ------------------------------------------------------------------ *
 *  CCMP encapsulation / decapsulation on a whole MPDU
 * ------------------------------------------------------------------ */

/*
 * Encrypt an MPDU in place.  On entry `frame` holds
 *   [802.11 hdr][CCMP hdr][plaintext]
 * of length *frame_len; on success it holds
 *   [802.11 hdr][CCMP hdr][ciphertext][MIC]
 * and *frame_len is increased by VWIFI_CCMP_MIC_LEN.  `cap` is the size of
 * the buffer backing `frame`.  Returns 0 on success, -1 otherwise.
 */
static inline int vwifi_ccmp_encrypt(const uint8_t key[16], uint8_t *frame,
                                     int *frame_len, int cap)
{
    uint8_t nonce[13], aad[30], mic[8];
    AES_KEY aes;
    int hdrlen, aad_len, data_len;
    uint8_t *data;

    hdrlen = vwifi_dot11_hdrlen(frame, *frame_len);
    aad_len = vwifi_ccmp_build(frame, *frame_len, hdrlen, nonce, aad);
    if (aad_len < 0) {
        return -1;
    }
    data     = frame + hdrlen + VWIFI_CCMP_HDR_LEN;
    data_len = *frame_len - hdrlen - VWIFI_CCMP_HDR_LEN;
    if (data_len < 0 || *frame_len + VWIFI_CCMP_MIC_LEN > cap) {
        return -1;
    }

    AES_set_encrypt_key(key, 128, &aes);
    vwifi_ccm_cbc_mac(&aes, nonce, aad, aad_len, data, data_len, mic);
    vwifi_ccm_ctr(&aes, nonce, data, data_len, mic);
    memcpy(frame + *frame_len, mic, VWIFI_CCMP_MIC_LEN);
    *frame_len += VWIFI_CCMP_MIC_LEN;
    return 0;
}

/*
 * Decrypt an MPDU in place and verify the MIC.  On entry `frame` holds
 *   [802.11 hdr][CCMP hdr][ciphertext][MIC]
 * of length frame_len; on success the ciphertext is replaced by plaintext
 * (the CCMP header and trailing MIC are left in place for mac80211 to strip)
 * and the function returns 0.  Returns -1 on MIC failure or malformed frame.
 */
static inline int vwifi_ccmp_decrypt(const uint8_t key[16], uint8_t *frame,
                                     int frame_len)
{
    uint8_t nonce[13], aad[30], recv_mic[8], calc_mic[8];
    AES_KEY aes;
    int hdrlen, aad_len, data_len;
    uint8_t *data;

    hdrlen = vwifi_dot11_hdrlen(frame, frame_len);
    aad_len = vwifi_ccmp_build(frame, frame_len, hdrlen, nonce, aad);
    if (aad_len < 0) {
        return -1;
    }
    data     = frame + hdrlen + VWIFI_CCMP_HDR_LEN;
    data_len = frame_len - hdrlen - VWIFI_CCMP_HDR_LEN - VWIFI_CCMP_MIC_LEN;
    if (data_len < 0) {
        return -1;
    }
    memcpy(recv_mic, data + data_len, VWIFI_CCMP_MIC_LEN);

    AES_set_encrypt_key(key, 128, &aes);
    /* CTR is symmetric: recovers plaintext into `data` and T into recv_mic. */
    vwifi_ccm_ctr(&aes, nonce, data, data_len, recv_mic);
    vwifi_ccm_cbc_mac(&aes, nonce, aad, aad_len, data, data_len, calc_mic);

    if (memcmp(calc_mic, recv_mic, VWIFI_CCM_M) != 0) {
        return -1;
    }
    return 0;
}

#endif /* VWIFI_ATH9K_CRYPTO_H */
