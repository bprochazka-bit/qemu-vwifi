/*
 * Virtual Atheros AR9285 – TKIP (per-packet key mixing + RC4 + CRC-32 ICV)
 * crypto engine emulation
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * ath9k offloads only the *cipher* half of TKIP to the on-chip engine: the
 * hardware derives the per-packet RC4 key (TKIP phase 1 + phase 2 mixing of
 * the temporal key with the transmitter address and the 48-bit TSC), runs RC4,
 * and appends the CRC-32 ICV.  The Michael MIC is handled in software by
 * mac80211 — ath9k sets IEEE80211_KEY_FLAG_GENERATE_MMIC, so the MSDU handed
 * to the hardware already carries the 8-octet Michael MIC as ordinary payload,
 * and RX Michael verification is done by mac80211 after decryption.  This
 * engine therefore performs mixing + RC4 + ICV only and treats the Michael MIC
 * as opaque data; the "split MIC key" cache entry the driver programs is never
 * read here.
 *
 * The temporal key (TK) is reconstructed from the main key-cache slot using
 * the same key0..key4 split as CCMP/WEP.  Mixing follows IEEE 802.11-2016
 * §12.5.2.3 / the mac80211 reference (net/mac80211/tkip.c); the S-box, phase 1
 * and phase 2 below are the standard TKIP definitions and are verified against
 * the published 802.11i key-mixing test vectors in tests/test_tkip.c.
 *
 * On the wire:
 *   [802.11 hdr][TSC1 WEPSeed TSC0 KeyID|ExtIV TSC2 TSC3 TSC4 TSC5]
 *              [ciphertext(data ‖ MichaelMIC)][enc ICV(4)]
 *
 * RC4 and CRC-32 are shared with the WEP engine.
 */

#ifndef VWIFI_ATH9K_TKIP_H
#define VWIFI_ATH9K_TKIP_H

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "vwifi_ath9k_wep.h"   /* vwifi_rc4, vwifi_crc32, vwifi_wep_hdrlen */

#define VWIFI_TKIP_HDR_LEN  8   /* IV/KeyID + extended IV, inserted by mac80211 */
#define VWIFI_TKIP_ICV_LEN  4   /* CRC-32, appended by the (emulated) hardware  */
#define VWIFI_TKIP_P1K_LEN  5   /* phase-1 output words                         */

/* TKIP S-box (256 × 16-bit), from IEEE 802.11 / net/mac80211/tkip.c. */
static const uint16_t vwifi_tkip_sbox[256] = {
    0xC6A5, 0xF884, 0xEE99, 0xF68D, 0xFF0D, 0xD6BD, 0xDEB1, 0x9154,
    0x6050, 0x0203, 0xCEA9, 0x567D, 0xE719, 0xB562, 0x4DE6, 0xEC9A,
    0x8F45, 0x1F9D, 0x8940, 0xFA87, 0xEF15, 0xB2EB, 0x8EC9, 0xFB0B,
    0x41EC, 0xB367, 0x5FFD, 0x45EA, 0x23BF, 0x53F7, 0xE496, 0x9B5B,
    0x75C2, 0xE11C, 0x3DAE, 0x4C6A, 0x6C5A, 0x7E41, 0xF502, 0x834F,
    0x685C, 0x51F4, 0xD134, 0xF908, 0xE293, 0xAB73, 0x6253, 0x2A3F,
    0x080C, 0x9552, 0x4665, 0x9D5E, 0x3028, 0x37A1, 0x0A0F, 0x2FB5,
    0x0E09, 0x2436, 0x1B9B, 0xDF3D, 0xCD26, 0x4E69, 0x7FCD, 0xEA9F,
    0x121B, 0x1D9E, 0x5874, 0x342E, 0x362D, 0xDCB2, 0xB4EE, 0x5BFB,
    0xA4F6, 0x764D, 0xB761, 0x7DCE, 0x527B, 0xDD3E, 0x5E71, 0x1397,
    0xA6F5, 0xB968, 0x0000, 0xC12C, 0x4060, 0xE31F, 0x79C8, 0xB6ED,
    0xD4BE, 0x8D46, 0x67D9, 0x724B, 0x94DE, 0x98D4, 0xB0E8, 0x854A,
    0xBB6B, 0xC52A, 0x4FE5, 0xED16, 0x86C5, 0x9AD7, 0x6655, 0x1194,
    0x8ACF, 0xE910, 0x0406, 0xFE81, 0xA0F0, 0x7844, 0x25BA, 0x4BE3,
    0xA2F3, 0x5DFE, 0x80C0, 0x058A, 0x3FAD, 0x21BC, 0x7048, 0xF104,
    0x63DF, 0x77C1, 0xAF75, 0x4263, 0x2030, 0xE51A, 0xFD0E, 0xBF6D,
    0x814C, 0x1814, 0x2635, 0xC32F, 0xBEE1, 0x35A2, 0x88CC, 0x2E39,
    0x9357, 0x55F2, 0xFC82, 0x7A47, 0xC8AC, 0xBAE7, 0x322B, 0xE695,
    0xC0A0, 0x1998, 0x9ED1, 0xA37F, 0x4466, 0x547E, 0x3BAB, 0x0B83,
    0x8CCA, 0xC729, 0x6BD3, 0x283C, 0xA779, 0xBCE2, 0x161D, 0xAD76,
    0xDB3B, 0x6456, 0x744E, 0x141E, 0x92DB, 0x0C0A, 0x486C, 0xB8E4,
    0x9F5D, 0xBD6E, 0x43EF, 0xC4A6, 0x39A8, 0x31A4, 0xD337, 0xF28B,
    0xD532, 0x8B43, 0x6E59, 0xDAB7, 0x018C, 0xB164, 0x9CD2, 0x49E0,
    0xD8B4, 0xACFA, 0xF307, 0xCF25, 0xCAAF, 0xF48E, 0x47E9, 0x1018,
    0x6FD5, 0xF088, 0x4A6F, 0x5C72, 0x3824, 0x57F1, 0x73C7, 0x9751,
    0xCB23, 0xA17C, 0xE89C, 0x3E21, 0x96DD, 0x61DC, 0x0D86, 0x0F85,
    0xE090, 0x7C42, 0x71C4, 0xCCAA, 0x90D8, 0x0605, 0xF701, 0x1C12,
    0xC2A3, 0x6A5F, 0xAEF9, 0x69D0, 0x1791, 0x9958, 0x3A27, 0x27B9,
    0xD938, 0xEB13, 0x2BB3, 0x2233, 0xD2BB, 0xA970, 0x0789, 0x33A7,
    0x2DB6, 0x3C22, 0x1592, 0xC920, 0x8749, 0xAAFF, 0x5078, 0xA57A,
    0x038F, 0x59F8, 0x0980, 0x1A17, 0x65DA, 0xD731, 0x84C6, 0xD0B8,
    0x82C3, 0x29B0, 0x5A77, 0x1E11, 0x7BCB, 0xA8FC, 0x6DD6, 0x2C3A,
};

static inline uint16_t vwifi_tkip_le16(const uint8_t *p)
{
    return (uint16_t)(p[0] | (p[1] << 8));
}

/* S-box lookup: sbox[lo] XOR swap16(sbox[hi]). */
static inline uint16_t vwifi_tkip_s(uint16_t v)
{
    uint16_t hi = vwifi_tkip_sbox[(v >> 8) & 0xff];
    return (uint16_t)(vwifi_tkip_sbox[v & 0xff] ^ (uint16_t)((hi << 8) | (hi >> 8)));
}

static inline uint16_t vwifi_tkip_ror16(uint16_t v)
{
    return (uint16_t)((v >> 1) | (v << 15));
}

/* Phase 1: mix TK + TA + IV32 into the 80-bit intermediate key p1k[0..4]. */
static inline void vwifi_tkip_phase1(const uint8_t tk[16], const uint8_t *ta,
                                     uint32_t iv32, uint16_t p1k[5])
{
    int i, j;

    p1k[0] = (uint16_t)(iv32 & 0xffff);
    p1k[1] = (uint16_t)(iv32 >> 16);
    p1k[2] = vwifi_tkip_le16(ta + 0);
    p1k[3] = vwifi_tkip_le16(ta + 2);
    p1k[4] = vwifi_tkip_le16(ta + 4);

    for (i = 0; i < 8; i++) {
        j = 2 * (i & 1);
        p1k[0] += vwifi_tkip_s(p1k[4] ^ vwifi_tkip_le16(tk + 0 + j));
        p1k[1] += vwifi_tkip_s(p1k[0] ^ vwifi_tkip_le16(tk + 4 + j));
        p1k[2] += vwifi_tkip_s(p1k[1] ^ vwifi_tkip_le16(tk + 8 + j));
        p1k[3] += vwifi_tkip_s(p1k[2] ^ vwifi_tkip_le16(tk + 12 + j));
        p1k[4] += vwifi_tkip_s(p1k[3] ^ vwifi_tkip_le16(tk + 0 + j)) + (uint16_t)i;
    }
}

/* Phase 2: mix in IV16 and produce the 16-octet RC4 key (WEP seed). */
static inline void vwifi_tkip_phase2(const uint8_t tk[16], const uint16_t p1k[5],
                                     uint16_t iv16, uint8_t rc4key[16])
{
    uint16_t ppk[6];
    int i;

    ppk[0] = p1k[0];
    ppk[1] = p1k[1];
    ppk[2] = p1k[2];
    ppk[3] = p1k[3];
    ppk[4] = p1k[4];
    ppk[5] = (uint16_t)(p1k[4] + iv16);

    ppk[0] += vwifi_tkip_s(ppk[5] ^ vwifi_tkip_le16(tk + 0));
    ppk[1] += vwifi_tkip_s(ppk[0] ^ vwifi_tkip_le16(tk + 2));
    ppk[2] += vwifi_tkip_s(ppk[1] ^ vwifi_tkip_le16(tk + 4));
    ppk[3] += vwifi_tkip_s(ppk[2] ^ vwifi_tkip_le16(tk + 6));
    ppk[4] += vwifi_tkip_s(ppk[3] ^ vwifi_tkip_le16(tk + 8));
    ppk[5] += vwifi_tkip_s(ppk[4] ^ vwifi_tkip_le16(tk + 10));

    ppk[0] += vwifi_tkip_ror16(ppk[5] ^ vwifi_tkip_le16(tk + 12));
    ppk[1] += vwifi_tkip_ror16(ppk[0] ^ vwifi_tkip_le16(tk + 14));
    ppk[2] += vwifi_tkip_ror16(ppk[1]);
    ppk[3] += vwifi_tkip_ror16(ppk[2]);
    ppk[4] += vwifi_tkip_ror16(ppk[3]);
    ppk[5] += vwifi_tkip_ror16(ppk[4]);

    /* IV bytes mirror the on-wire TKIP header. */
    rc4key[0] = (uint8_t)(iv16 >> 8);
    rc4key[1] = (uint8_t)(((iv16 >> 8) | 0x20) & 0x7f);
    rc4key[2] = (uint8_t)(iv16 & 0xff);
    rc4key[3] = (uint8_t)(((ppk[5] ^ vwifi_tkip_le16(tk)) >> 1) & 0xff);
    for (i = 0; i < 6; i++) {
        rc4key[4 + 2 * i]     = (uint8_t)(ppk[i] & 0xff);
        rc4key[4 + 2 * i + 1] = (uint8_t)(ppk[i] >> 8);
    }
}

/*
 * Derive the 16-octet per-packet RC4 key for one MPDU from the TK, the frame's
 * transmitter address (A2), and the TSC carried in the TKIP header at `iv`.
 */
static inline void vwifi_tkip_rc4key(const uint8_t tk[16], const uint8_t *ta,
                                     const uint8_t *iv, uint8_t rc4key[16])
{
    uint16_t p1k[5];
    uint16_t iv16 = (uint16_t)((iv[0] << 8) | iv[2]);
    uint32_t iv32 = (uint32_t)iv[4] | ((uint32_t)iv[5] << 8) |
                    ((uint32_t)iv[6] << 16) | ((uint32_t)iv[7] << 24);

    vwifi_tkip_phase1(tk, ta, iv32, p1k);
    vwifi_tkip_phase2(tk, p1k, iv16, rc4key);
}

/* ------------------------------------------------------------------ *
 *  TKIP encapsulation / decapsulation on a whole MPDU
 * ------------------------------------------------------------------ */

/*
 * Encrypt an MPDU in place.  On entry `frame` holds
 *   [802.11 hdr][TKIP hdr(8)][data ‖ MichaelMIC]
 * of length *frame_len; on success it holds
 *   [802.11 hdr][TKIP hdr(8)][ciphertext][enc ICV(4)]
 * and *frame_len grows by VWIFI_TKIP_ICV_LEN.  Returns 0 on success, -1 else.
 * `tk` is the 128-bit temporal key from the key cache.
 */
static inline int vwifi_tkip_encrypt(const uint8_t tk[16], uint8_t *frame,
                                     int *frame_len, int cap)
{
    uint8_t rc4key[16];
    const uint8_t *ta;
    uint8_t *iv, *data;
    int hdrlen, data_len;
    uint32_t icv;

    hdrlen = vwifi_wep_hdrlen(frame, *frame_len);
    if (*frame_len < hdrlen + VWIFI_TKIP_HDR_LEN ||
        *frame_len + VWIFI_TKIP_ICV_LEN > cap) {
        return -1;
    }
    ta       = frame + 10;                        /* Address 2 = TA */
    iv       = frame + hdrlen;
    data     = frame + hdrlen + VWIFI_TKIP_HDR_LEN;
    data_len = *frame_len - hdrlen - VWIFI_TKIP_HDR_LEN;
    if (data_len < 0) {
        return -1;
    }

    icv = vwifi_crc32(data, data_len);
    data[data_len + 0] = (uint8_t)(icv & 0xff);
    data[data_len + 1] = (uint8_t)((icv >> 8) & 0xff);
    data[data_len + 2] = (uint8_t)((icv >> 16) & 0xff);
    data[data_len + 3] = (uint8_t)((icv >> 24) & 0xff);

    vwifi_tkip_rc4key(tk, ta, iv, rc4key);
    vwifi_rc4(rc4key, 16, data, data_len + VWIFI_TKIP_ICV_LEN);

    *frame_len += VWIFI_TKIP_ICV_LEN;
    return 0;
}

/*
 * Decrypt an MPDU in place and verify the ICV.  On entry `frame` holds
 *   [802.11 hdr][TKIP hdr(8)][ciphertext][enc ICV(4)]
 * of length frame_len; on success the ciphertext (data ‖ MichaelMIC) is
 * recovered in place (the TKIP header and trailing ICV are left for mac80211
 * to strip, and mac80211 then verifies/strips the Michael MIC).  Returns 0 on
 * success, -1 on ICV failure (e.g. wrong key) or a malformed frame.
 */
static inline int vwifi_tkip_decrypt(const uint8_t tk[16], uint8_t *frame,
                                     int frame_len)
{
    uint8_t rc4key[16];
    const uint8_t *ta;
    uint8_t *iv, *data;
    int hdrlen, region, data_len;
    uint32_t icv, recv_icv;

    hdrlen = vwifi_wep_hdrlen(frame, frame_len);
    if (frame_len < hdrlen + VWIFI_TKIP_HDR_LEN + VWIFI_TKIP_ICV_LEN) {
        return -1;
    }
    ta       = frame + 10;
    iv       = frame + hdrlen;
    data     = frame + hdrlen + VWIFI_TKIP_HDR_LEN;
    region   = frame_len - hdrlen - VWIFI_TKIP_HDR_LEN; /* ciphertext + ICV */
    data_len = region - VWIFI_TKIP_ICV_LEN;
    if (data_len < 0) {
        return -1;
    }

    vwifi_tkip_rc4key(tk, ta, iv, rc4key);
    vwifi_rc4(rc4key, 16, data, region);

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

#endif /* VWIFI_ATH9K_TKIP_H */
