/*
 * vwifi-virt — AES-128 + CCMP-128
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Self-contained crypto for the portable device core. No OpenSSL, no
 * QEMU crypto, no libvfio-user dependency — the same code must run
 * under every backend, and the device is the only place that knows
 * the keys.
 *
 * This has to be REAL CCMP, not a stub: the Windows guest associates
 * with hostapd running on another VM on the same virtual medium, and
 * hostapd will genuinely try to decrypt what we send. A fake would
 * fail on the first data frame.
 *
 * AES-128 is encrypt-only. CCM uses the forward direction for both
 * CTR mode and CBC-MAC, so decryption never needs the inverse cipher.
 *
 * References:
 *   FIPS-197  — AES
 *   RFC 3610  — Counter with CBC-MAC (CCM)
 *   IEEE 802.11-2016 §12.5.3 — CCMP encapsulation/decapsulation
 */

#include <string.h>
#include "vwifi_crypto.h"

/* ============================================================
 * AES-128
 * ============================================================ */

static const uint8_t aes_sbox[256] = {
    0x63,0x7c,0x77,0x7b,0xf2,0x6b,0x6f,0xc5,0x30,0x01,0x67,0x2b,0xfe,0xd7,0xab,0x76,
    0xca,0x82,0xc9,0x7d,0xfa,0x59,0x47,0xf0,0xad,0xd4,0xa2,0xaf,0x9c,0xa4,0x72,0xc0,
    0xb7,0xfd,0x93,0x26,0x36,0x3f,0xf7,0xcc,0x34,0xa5,0xe5,0xf1,0x71,0xd8,0x31,0x15,
    0x04,0xc7,0x23,0xc3,0x18,0x96,0x05,0x9a,0x07,0x12,0x80,0xe2,0xeb,0x27,0xb2,0x75,
    0x09,0x83,0x2c,0x1a,0x1b,0x6e,0x5a,0xa0,0x52,0x3b,0xd6,0xb3,0x29,0xe3,0x2f,0x84,
    0x53,0xd1,0x00,0xed,0x20,0xfc,0xb1,0x5b,0x6a,0xcb,0xbe,0x39,0x4a,0x4c,0x58,0xcf,
    0xd0,0xef,0xaa,0xfb,0x43,0x4d,0x33,0x85,0x45,0xf9,0x02,0x7f,0x50,0x3c,0x9f,0xa8,
    0x51,0xa3,0x40,0x8f,0x92,0x9d,0x38,0xf5,0xbc,0xb6,0xda,0x21,0x10,0xff,0xf3,0xd2,
    0xcd,0x0c,0x13,0xec,0x5f,0x97,0x44,0x17,0xc4,0xa7,0x7e,0x3d,0x64,0x5d,0x19,0x73,
    0x60,0x81,0x4f,0xdc,0x22,0x2a,0x90,0x88,0x46,0xee,0xb8,0x14,0xde,0x5e,0x0b,0xdb,
    0xe0,0x32,0x3a,0x0a,0x49,0x06,0x24,0x5c,0xc2,0xd3,0xac,0x62,0x91,0x95,0xe4,0x79,
    0xe7,0xc8,0x37,0x6d,0x8d,0xd5,0x4e,0xa9,0x6c,0x56,0xf4,0xea,0x65,0x7a,0xae,0x08,
    0xba,0x78,0x25,0x2e,0x1c,0xa6,0xb4,0xc6,0xe8,0xdd,0x74,0x1f,0x4b,0xbd,0x8b,0x8a,
    0x70,0x3e,0xb5,0x66,0x48,0x03,0xf6,0x0e,0x61,0x35,0x57,0xb9,0x86,0xc1,0x1d,0x9e,
    0xe1,0xf8,0x98,0x11,0x69,0xd9,0x8e,0x94,0x9b,0x1e,0x87,0xe9,0xce,0x55,0x28,0xdf,
    0x8c,0xa1,0x89,0x0d,0xbf,0xe6,0x42,0x68,0x41,0x99,0x2d,0x0f,0xb0,0x54,0xbb,0x16,
};

static const uint8_t aes_rcon[11] = {
    0x00, 0x01, 0x02, 0x04, 0x08, 0x10, 0x20, 0x40, 0x80, 0x1b, 0x36,
};

/* Multiply by 2 in GF(2^8) with the AES reduction polynomial. */
static inline uint8_t xtime(uint8_t x)
{
    return (uint8_t)((x << 1) ^ ((x & 0x80) ? 0x1b : 0x00));
}

void vwifi_aes128_key_expand(const uint8_t key[16], struct vwifi_aes128 *ctx)
{
    uint8_t *rk = ctx->rk;
    memcpy(rk, key, 16);

    for (unsigned i = 4; i < 44; i++) {
        uint8_t t[4];
        memcpy(t, rk + (i - 1) * 4, 4);

        if (i % 4 == 0) {
            /* RotWord + SubWord + Rcon */
            uint8_t tmp = t[0];
            t[0] = aes_sbox[t[1]];
            t[1] = aes_sbox[t[2]];
            t[2] = aes_sbox[t[3]];
            t[3] = aes_sbox[tmp];
            t[0] ^= aes_rcon[i / 4];
        }
        for (unsigned j = 0; j < 4; j++) {
            rk[i * 4 + j] = rk[(i - 4) * 4 + j] ^ t[j];
        }
    }
}

void vwifi_aes128_encrypt(const struct vwifi_aes128 *ctx,
                          const uint8_t in[16], uint8_t out[16])
{
    uint8_t s[16];
    const uint8_t *rk = ctx->rk;

    memcpy(s, in, 16);

    /* Initial AddRoundKey. */
    for (unsigned i = 0; i < 16; i++) s[i] ^= rk[i];

    for (unsigned round = 1; round <= 10; round++) {
        uint8_t t[16];

        /* SubBytes. */
        for (unsigned i = 0; i < 16; i++) s[i] = aes_sbox[s[i]];

        /* ShiftRows — state is column-major: s[r + 4c]. */
        t[0]  = s[0];  t[4]  = s[4];  t[8]  = s[8];  t[12] = s[12];
        t[1]  = s[5];  t[5]  = s[9];  t[9]  = s[13]; t[13] = s[1];
        t[2]  = s[10]; t[6]  = s[14]; t[10] = s[2];  t[14] = s[6];
        t[3]  = s[15]; t[7]  = s[3];  t[11] = s[7];  t[15] = s[11];
        memcpy(s, t, 16);

        /* MixColumns — skipped in the final round. */
        if (round != 10) {
            for (unsigned c = 0; c < 4; c++) {
                uint8_t *col = s + c * 4;
                uint8_t a0 = col[0], a1 = col[1], a2 = col[2], a3 = col[3];
                uint8_t x = a0 ^ a1 ^ a2 ^ a3;
                col[0] ^= x ^ xtime(a0 ^ a1);
                col[1] ^= x ^ xtime(a1 ^ a2);
                col[2] ^= x ^ xtime(a2 ^ a3);
                col[3] ^= x ^ xtime(a3 ^ a0);
            }
        }

        /* AddRoundKey. */
        for (unsigned i = 0; i < 16; i++) s[i] ^= rk[round * 16 + i];
    }

    memcpy(out, s, 16);
}

/* ============================================================
 * CCM (RFC 3610) with M=8, L=2 — the parameters CCMP uses.
 *
 * B_0    = flags || nonce(13) || l(m)(2)
 * flags  = 0x59 : Adata=1, M'=(8-2)/2=3, L'=L-1=1
 * A_i    = flags(0x01) || nonce(13) || counter(2)
 * ============================================================ */

#define CCM_MIC_LEN  8

static void ccm_cbc_mac(const struct vwifi_aes128 *aes,
                        const uint8_t b0[16],
                        const uint8_t *aad, size_t aad_len,
                        const uint8_t *msg, size_t msg_len,
                        uint8_t mic_out[CCM_MIC_LEN])
{
    uint8_t x[16], blk[16];
    size_t off;

    /* X_1 = E(K, B_0) */
    vwifi_aes128_encrypt(aes, b0, x);

    /* AAD blocks: first block carries a 2-byte big-endian length
     * prefix (valid for aad_len < 2^16 - 2^8, always true here). */
    if (aad_len) {
        size_t chunk;
        memset(blk, 0, 16);
        blk[0] = (uint8_t)(aad_len >> 8);
        blk[1] = (uint8_t)(aad_len & 0xFF);
        chunk = (aad_len < 14) ? aad_len : 14;
        memcpy(blk + 2, aad, chunk);
        for (unsigned i = 0; i < 16; i++) x[i] ^= blk[i];
        vwifi_aes128_encrypt(aes, x, x);

        off = chunk;
        while (off < aad_len) {
            memset(blk, 0, 16);
            chunk = (aad_len - off < 16) ? (aad_len - off) : 16;
            memcpy(blk, aad + off, chunk);
            for (unsigned i = 0; i < 16; i++) x[i] ^= blk[i];
            vwifi_aes128_encrypt(aes, x, x);
            off += chunk;
        }
    }

    /* Message blocks, zero-padded. */
    off = 0;
    while (off < msg_len) {
        size_t chunk = (msg_len - off < 16) ? (msg_len - off) : 16;
        memset(blk, 0, 16);
        memcpy(blk, msg + off, chunk);
        for (unsigned i = 0; i < 16; i++) x[i] ^= blk[i];
        vwifi_aes128_encrypt(aes, x, x);
        off += chunk;
    }

    memcpy(mic_out, x, CCM_MIC_LEN);
}

/* CTR-mode keystream application. Counter starts at `start_ctr`. */
static void ccm_ctr_xor(const struct vwifi_aes128 *aes,
                        const uint8_t nonce[13],
                        uint16_t start_ctr,
                        uint8_t *data, size_t len)
{
    uint8_t a[16], s[16];
    size_t off = 0;
    uint16_t ctr = start_ctr;

    a[0] = 0x01;                   /* flags: L' = L-1 = 1 */
    memcpy(a + 1, nonce, 13);

    while (off < len) {
        size_t chunk = (len - off < 16) ? (len - off) : 16;
        a[14] = (uint8_t)(ctr >> 8);
        a[15] = (uint8_t)(ctr & 0xFF);
        vwifi_aes128_encrypt(aes, a, s);
        for (size_t i = 0; i < chunk; i++) data[off + i] ^= s[i];
        off += chunk;
        ctr++;
    }
}

/* Encrypt the MIC with S_0 (counter 0). */
static void ccm_encrypt_mic(const struct vwifi_aes128 *aes,
                            const uint8_t nonce[13],
                            uint8_t mic[CCM_MIC_LEN])
{
    uint8_t a[16], s[16];
    a[0] = 0x01;
    memcpy(a + 1, nonce, 13);
    a[14] = 0;
    a[15] = 0;
    vwifi_aes128_encrypt(aes, a, s);
    for (unsigned i = 0; i < CCM_MIC_LEN; i++) mic[i] ^= s[i];
}

void vwifi_ccm_encrypt(const struct vwifi_aes128 *aes,
                       const uint8_t nonce[13],
                       const uint8_t *aad, size_t aad_len,
                       uint8_t *msg, size_t msg_len,
                       uint8_t mic_out[8])
{
    uint8_t b0[16];

    b0[0] = 0x59;                          /* Adata | M'=3 | L'=1 */
    memcpy(b0 + 1, nonce, 13);
    b0[14] = (uint8_t)(msg_len >> 8);
    b0[15] = (uint8_t)(msg_len & 0xFF);

    ccm_cbc_mac(aes, b0, aad, aad_len, msg, msg_len, mic_out);
    ccm_encrypt_mic(aes, nonce, mic_out);
    ccm_ctr_xor(aes, nonce, 1, msg, msg_len);
}

bool vwifi_ccm_decrypt(const struct vwifi_aes128 *aes,
                       const uint8_t nonce[13],
                       const uint8_t *aad, size_t aad_len,
                       uint8_t *msg, size_t msg_len,
                       const uint8_t mic_in[8])
{
    uint8_t b0[16], mic[CCM_MIC_LEN], recv_mic[CCM_MIC_LEN];
    uint8_t diff = 0;

    /* Decrypt the payload first — CTR is symmetric. */
    ccm_ctr_xor(aes, nonce, 1, msg, msg_len);

    /* Recompute the MIC over the now-plaintext message. */
    b0[0] = 0x59;
    memcpy(b0 + 1, nonce, 13);
    b0[14] = (uint8_t)(msg_len >> 8);
    b0[15] = (uint8_t)(msg_len & 0xFF);
    ccm_cbc_mac(aes, b0, aad, aad_len, msg, msg_len, mic);
    ccm_encrypt_mic(aes, nonce, mic);

    /* Constant-time compare — no early exit on first mismatch. */
    memcpy(recv_mic, mic_in, CCM_MIC_LEN);
    for (unsigned i = 0; i < CCM_MIC_LEN; i++) diff |= mic[i] ^ recv_mic[i];

    if (diff != 0) {
        /* Re-encrypt so the caller's buffer isn't left with plaintext
         * from a forged frame. */
        ccm_ctr_xor(aes, nonce, 1, msg, msg_len);
        return false;
    }
    return true;
}

/* ============================================================
 * CCMP (IEEE 802.11-2016 §12.5.3)
 * ============================================================ */

#define IEEE80211_FCTL_PROTECTED  0x4000
#define IEEE80211_FCTL_RETRY      0x0800
#define IEEE80211_FCTL_PM         0x1000
#define IEEE80211_FCTL_MOREDATA   0x2000
#define IEEE80211_FCTL_FROMDS     0x0200
#define IEEE80211_FCTL_TODS       0x0100
#define IEEE80211_STYPE_MASK      0x0070

static inline uint16_t rd_le16(const uint8_t *p)
{
    return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}

/* True if the frame carries a 4th address (both ToDS and FromDS). */
static bool hdr_has_a4(const uint8_t *hdr)
{
    uint16_t fc = rd_le16(hdr);
    return (fc & IEEE80211_FCTL_TODS) && (fc & IEEE80211_FCTL_FROMDS);
}

/* True if this is a QoS data frame (subtype bit 3 set on data). */
static bool hdr_is_qos(const uint8_t *hdr)
{
    uint16_t fc = rd_le16(hdr);
    if (((fc >> 2) & 0x3) != 2) return false;   /* not data */
    return (fc & 0x0080) != 0;                  /* subtype bit 3 */
}

size_t vwifi_ccmp_hdr_len(const uint8_t *hdr)
{
    size_t len = 24;
    if (hdr_has_a4(hdr)) len += 6;
    if (hdr_is_qos(hdr)) len += 2;
    return len;
}

/* Build the CCM nonce: flags(1) || A2(6) || PN(6). */
static void ccmp_nonce(const uint8_t *hdr, const uint8_t pn[6],
                       uint8_t nonce[13])
{
    uint16_t fc = rd_le16(hdr);
    uint8_t  tid = 0;
    bool     mgmt = (((fc >> 2) & 0x3) == 0);

    if (hdr_is_qos(hdr)) {
        size_t qos_off = hdr_has_a4(hdr) ? 30 : 24;
        tid = hdr[qos_off] & 0x0F;
    }

    nonce[0] = (uint8_t)(tid | (mgmt ? 0x10 : 0x00));
    memcpy(nonce + 1, hdr + 10, 6);   /* A2 */
    memcpy(nonce + 7, pn, 6);         /* PN, big-endian (PN5..PN0) */
}

/* Build the AAD from the MPDU header, per §12.5.3.3.3. */
static size_t ccmp_aad(const uint8_t *hdr, uint8_t *aad)
{
    uint16_t fc = rd_le16(hdr);
    uint16_t mask_fc;
    size_t   len = 0;
    bool     mgmt = (((fc >> 2) & 0x3) == 0);

    /* Mask Retry / PwrMgt / MoreData; set Protected; for non-mgmt
     * frames also mask the subtype bits b4 b5 b6. */
    mask_fc = fc;
    mask_fc &= (uint16_t)~(IEEE80211_FCTL_RETRY |
                           IEEE80211_FCTL_PM |
                           IEEE80211_FCTL_MOREDATA);
    if (!mgmt) mask_fc &= (uint16_t)~IEEE80211_STYPE_MASK;
    mask_fc |= IEEE80211_FCTL_PROTECTED;

    aad[len++] = (uint8_t)(mask_fc & 0xFF);
    aad[len++] = (uint8_t)(mask_fc >> 8);

    memcpy(aad + len, hdr + 4, 18);   /* A1 || A2 || A3 */
    len += 18;

    /* Sequence control: mask the sequence number, keep the fragment
     * number (low 4 bits). */
    aad[len++] = hdr[22] & 0x0F;
    aad[len++] = 0;

    if (hdr_has_a4(hdr)) {
        memcpy(aad + len, hdr + 24, 6);   /* A4 */
        len += 6;
    }

    if (hdr_is_qos(hdr)) {
        size_t qos_off = hdr_has_a4(hdr) ? 30 : 24;
        aad[len++] = hdr[qos_off] & 0x0F;   /* TID; other bits masked */
        aad[len++] = 0;
    }

    return len;
}

void vwifi_ccmp_build_header(uint8_t *out, const uint8_t pn[6], uint8_t key_id)
{
    /* PN is stored big-endian (pn[0] = PN5 = MSB). The on-air CCMP
     * header scatters it: PN0 PN1 rsvd keyid|extiv PN2 PN3 PN4 PN5. */
    out[0] = pn[5];                       /* PN0 (LSB) */
    out[1] = pn[4];                       /* PN1 */
    out[2] = 0;                           /* reserved */
    out[3] = (uint8_t)(0x20 | (key_id << 6));   /* ExtIV | KeyID */
    out[4] = pn[3];                       /* PN2 */
    out[5] = pn[2];                       /* PN3 */
    out[6] = pn[1];                       /* PN4 */
    out[7] = pn[0];                       /* PN5 (MSB) */
}

void vwifi_ccmp_parse_header(const uint8_t *in, uint8_t pn[6], uint8_t *key_id)
{
    pn[0] = in[7];
    pn[1] = in[6];
    pn[2] = in[5];
    pn[3] = in[4];
    pn[4] = in[1];
    pn[5] = in[0];
    if (key_id) *key_id = (in[3] >> 6) & 0x3;
}

int vwifi_ccmp_encrypt(const struct vwifi_aes128 *aes,
                       uint8_t *frame, size_t frame_len,
                       const uint8_t pn[6], uint8_t key_id,
                       size_t frame_cap)
{
    uint8_t aad[32];
    uint8_t nonce[13];
    size_t  hdr_len = vwifi_ccmp_hdr_len(frame);
    size_t  aad_len;
    size_t  data_len;
    uint8_t mic[8];

    if (frame_len < hdr_len) return -1;
    data_len = frame_len - hdr_len;

    /* Need room for the 8-byte CCMP header and the 8-byte MIC. */
    if (frame_len + VWIFI_CCMP_HDR_LEN + VWIFI_CCMP_MIC_LEN > frame_cap) {
        return -1;
    }

    /* Set the Protected bit before computing the AAD — the AAD is
     * built from the header as it appears on the air. */
    frame[1] |= 0x40;

    aad_len = ccmp_aad(frame, aad);
    ccmp_nonce(frame, pn, nonce);

    /* Make room for the CCMP header between the 802.11 header and
     * the payload. */
    memmove(frame + hdr_len + VWIFI_CCMP_HDR_LEN,
            frame + hdr_len, data_len);
    vwifi_ccmp_build_header(frame + hdr_len, pn, key_id);

    vwifi_ccm_encrypt(aes, nonce, aad, aad_len,
                      frame + hdr_len + VWIFI_CCMP_HDR_LEN, data_len,
                      mic);

    memcpy(frame + hdr_len + VWIFI_CCMP_HDR_LEN + data_len, mic,
           VWIFI_CCMP_MIC_LEN);

    return (int)(hdr_len + VWIFI_CCMP_HDR_LEN + data_len + VWIFI_CCMP_MIC_LEN);
}

int vwifi_ccmp_decrypt(const struct vwifi_aes128 *aes,
                       uint8_t *frame, size_t frame_len,
                       uint8_t pn_out[6], uint8_t *key_id_out)
{
    uint8_t aad[32];
    uint8_t nonce[13];
    uint8_t pn[6];
    size_t  hdr_len = vwifi_ccmp_hdr_len(frame);
    size_t  aad_len;
    size_t  data_len;

    if (frame_len < hdr_len + VWIFI_CCMP_HDR_LEN + VWIFI_CCMP_MIC_LEN) {
        return -1;
    }
    if (!(frame[1] & 0x40)) return -1;   /* Protected bit not set */

    vwifi_ccmp_parse_header(frame + hdr_len, pn, key_id_out);
    if (pn_out) memcpy(pn_out, pn, 6);

    /* AAD is computed over the header as received (Protected set). */
    aad_len = ccmp_aad(frame, aad);
    ccmp_nonce(frame, pn, nonce);

    data_len = frame_len - hdr_len - VWIFI_CCMP_HDR_LEN - VWIFI_CCMP_MIC_LEN;

    if (!vwifi_ccm_decrypt(aes, nonce, aad, aad_len,
                           frame + hdr_len + VWIFI_CCMP_HDR_LEN, data_len,
                           frame + hdr_len + VWIFI_CCMP_HDR_LEN + data_len)) {
        return -2;   /* MIC failure */
    }

    /* Strip the CCMP header, leaving hdr || plaintext. */
    memmove(frame + hdr_len,
            frame + hdr_len + VWIFI_CCMP_HDR_LEN, data_len);
    frame[1] &= (uint8_t)~0x40;   /* clear Protected */

    return (int)(hdr_len + data_len);
}

/* PN comparison: 48-bit big-endian counters. Returns <0, 0, >0. */
int vwifi_pn_cmp(const uint8_t a[6], const uint8_t b[6])
{
    return memcmp(a, b, 6);
}

void vwifi_pn_increment(uint8_t pn[6])
{
    for (int i = 5; i >= 0; i--) {
        if (++pn[i] != 0) break;
    }
}
