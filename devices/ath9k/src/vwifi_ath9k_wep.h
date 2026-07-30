/*
 * Virtual Atheros AR9285 – WEP (RC4 + CRC-32 ICV) crypto engine emulation
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Like CCMP, ath9k offloads WEP to the on-chip crypto engine driven by the
 * key cache: the driver programs the WEP key into a cache slot (key type
 * 40 / 104 / 128), sets IEEE80211_KEY_FLAG_GENERATE_IV so mac80211 inserts
 * the 4-octet IV header, and reserves 4 octets of tailroom for the ICV that
 * the hardware appends.  mac80211 then relies on the hardware for the data
 * path.  With no emulated engine every WEP-protected frame is dropped, exactly
 * as happened for CCMP before its engine was added.  This header implements
 * the RC4 transform the silicon would perform.
 *
 * WEP (IEEE 802.11-2016 §12.3.2) over one MPDU:
 *   seed   = IV(3) || key(5 for WEP-40, 13 for WEP-104, 16 for WEP-128)
 *   ICV    = CRC-32 of the plaintext, appended little-endian
 *   cipher = RC4(seed) XOR (plaintext || ICV)
 * On the wire:  [802.11 hdr][IV(3) KeyID(1)][ciphertext][enc ICV(4)]
 *
 * This header is self-contained (RC4 + CRC-32 only); it can be reused by a
 * future TKIP engine, which shares the RC4 primitive.
 */

#ifndef VWIFI_ATH9K_WEP_H
#define VWIFI_ATH9K_WEP_H

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#define VWIFI_WEP_IV_LEN   4   /* IV(3) + KeyID(1), inserted by mac80211 */
#define VWIFI_WEP_ICV_LEN  4   /* CRC-32, appended by the (emulated) HW   */
#define VWIFI_WEP_SEED_MAX (3 + 16)

/* 802.11 header length implied by Frame Control (mirrors the CCMP engine). */
static inline int vwifi_wep_hdrlen(const uint8_t *frame, int frame_len)
{
    int hdrlen = 24;
    uint16_t fc;

    if (frame_len < 2) {
        return hdrlen;
    }
    fc = (uint16_t)(frame[0] | (frame[1] << 8));
    if (((fc >> 2) & 3) == 2 && ((fc >> 4) & 0x8)) { /* QoS data → +2 */
        hdrlen = 26;
    }
    if ((fc & 0x0300) == 0x0300) {                   /* 4-address → +6 */
        hdrlen += 6;
    }
    return hdrlen;
}

/* True if the frame's IV header uses the extended-IV format (CCMP/TKIP, an
 * 8-octet header) rather than plain WEP's 4-octet header.  The ExtIV bit is
 * bit 5 of the KeyID octet at IV offset 3. */
static inline bool vwifi_wep_is_extiv(const uint8_t *frame, int frame_len,
                                      int hdrlen)
{
    if (frame_len < hdrlen + 4) {
        return false;
    }
    return (frame[hdrlen + 3] & 0x20) != 0;
}

/* ------------------------------------------------------------------ *
 *  Primitives: RC4 stream cipher and CRC-32 (IEEE 802.3 / WEP ICV)
 * ------------------------------------------------------------------ */

/* RC4: KSA + PRGA, XORing the keystream over `data` in place. */
static inline void vwifi_rc4(const uint8_t *key, int keylen,
                             uint8_t *data, int len)
{
    uint8_t s[256];
    int i, j, k;

    for (i = 0; i < 256; i++) {
        s[i] = (uint8_t)i;
    }
    for (i = 0, j = 0; i < 256; i++) {
        uint8_t t;
        j = (j + s[i] + key[i % keylen]) & 0xff;
        t = s[i]; s[i] = s[j]; s[j] = t;
    }
    for (k = 0, i = 0, j = 0; k < len; k++) {
        uint8_t t;
        i = (i + 1) & 0xff;
        j = (j + s[i]) & 0xff;
        t = s[i]; s[i] = s[j]; s[j] = t;
        data[k] ^= s[(s[i] + s[j]) & 0xff];
    }
}

/* CRC-32 (reflected, poly 0xEDB88320) — identical to the IEEE 802.3 FCS and
 * to zlib's crc32(); this is the WEP ICV. */
static inline uint32_t vwifi_crc32(const uint8_t *data, int len)
{
    uint32_t crc = 0xffffffffu;
    int i, b;

    for (i = 0; i < len; i++) {
        crc ^= data[i];
        for (b = 0; b < 8; b++) {
            crc = (crc >> 1) ^ (0xEDB88320u & (0u - (crc & 1u)));
        }
    }
    return ~crc;
}

/* ------------------------------------------------------------------ *
 *  WEP encapsulation / decapsulation on a whole MPDU
 * ------------------------------------------------------------------ */

/*
 * Encrypt an MPDU in place.  On entry `frame` holds
 *   [802.11 hdr][IV/KeyID(4)][plaintext]
 * of length *frame_len; on success it holds
 *   [802.11 hdr][IV/KeyID(4)][ciphertext][enc ICV(4)]
 * and *frame_len grows by VWIFI_WEP_ICV_LEN.  `cap` is the backing-buffer
 * size.  `key`/`keylen` are the reconstructed WEP key (5/13/16 octets).
 * Returns 0 on success, -1 otherwise.
 */
static inline int vwifi_wep_encrypt(const uint8_t *key, int keylen,
                                    uint8_t *frame, int *frame_len, int cap)
{
    uint8_t seed[VWIFI_WEP_SEED_MAX];
    int hdrlen, data_len;
    uint8_t *iv, *data;
    uint32_t icv;

    if (keylen < 1 || keylen > 16) {
        return -1;
    }
    hdrlen = vwifi_wep_hdrlen(frame, *frame_len);
    if (*frame_len < hdrlen + VWIFI_WEP_IV_LEN ||
        *frame_len + VWIFI_WEP_ICV_LEN > cap) {
        return -1;
    }
    iv       = frame + hdrlen;
    data     = frame + hdrlen + VWIFI_WEP_IV_LEN;
    data_len = *frame_len - hdrlen - VWIFI_WEP_IV_LEN;
    if (data_len < 0) {
        return -1;
    }

    /* ICV = CRC-32(plaintext), appended little-endian. */
    icv = vwifi_crc32(data, data_len);
    data[data_len + 0] = (uint8_t)(icv & 0xff);
    data[data_len + 1] = (uint8_t)((icv >> 8) & 0xff);
    data[data_len + 2] = (uint8_t)((icv >> 16) & 0xff);
    data[data_len + 3] = (uint8_t)((icv >> 24) & 0xff);

    /* RC4 over plaintext+ICV with seed = IV(3) || key. */
    seed[0] = iv[0]; seed[1] = iv[1]; seed[2] = iv[2];
    memcpy(seed + 3, key, keylen);
    vwifi_rc4(seed, 3 + keylen, data, data_len + VWIFI_WEP_ICV_LEN);

    *frame_len += VWIFI_WEP_ICV_LEN;
    return 0;
}

/*
 * Decrypt an MPDU in place and verify the ICV.  On entry `frame` holds
 *   [802.11 hdr][IV/KeyID(4)][ciphertext][enc ICV(4)]
 * of length frame_len; on success the ciphertext is replaced by plaintext
 * (the IV header and trailing ICV are left in place for mac80211 to strip,
 * mirroring the CCMP path) and the function returns 0.  Returns -1 on ICV
 * failure (e.g. wrong key) or a malformed frame.
 */
static inline int vwifi_wep_decrypt(const uint8_t *key, int keylen,
                                    uint8_t *frame, int frame_len)
{
    uint8_t seed[VWIFI_WEP_SEED_MAX];
    int hdrlen, region, data_len;
    uint8_t *iv, *data;
    uint32_t icv, recv_icv;

    if (keylen < 1 || keylen > 16) {
        return -1;
    }
    hdrlen = vwifi_wep_hdrlen(frame, frame_len);
    if (frame_len < hdrlen + VWIFI_WEP_IV_LEN + VWIFI_WEP_ICV_LEN) {
        return -1;
    }
    iv       = frame + hdrlen;
    data     = frame + hdrlen + VWIFI_WEP_IV_LEN;
    region   = frame_len - hdrlen - VWIFI_WEP_IV_LEN; /* ciphertext + ICV */
    data_len = region - VWIFI_WEP_ICV_LEN;
    if (data_len < 0) {
        return -1;
    }

    seed[0] = iv[0]; seed[1] = iv[1]; seed[2] = iv[2];
    memcpy(seed + 3, key, keylen);
    vwifi_rc4(seed, 3 + keylen, data, region);

    icv = vwifi_crc32(data, data_len);
    recv_icv = (uint32_t)data[data_len] |
               ((uint32_t)data[data_len + 1] << 8) |
               ((uint32_t)data[data_len + 2] << 16) |
               ((uint32_t)data[data_len + 3] << 24);
    if (icv != recv_icv) {
        return -1;
    }
    return 0;
}

#endif /* VWIFI_ATH9K_WEP_H */
