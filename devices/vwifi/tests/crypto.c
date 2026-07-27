/*
 * vwifi-virt — crypto tests
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * 1. AES-128 against the FIPS-197 Appendix C.1 known-answer vector.
 *    This anchors the primitive — if this fails nothing above it can
 *    be trusted.
 * 2. CCMP encrypt -> decrypt round trip preserves the payload.
 * 3. A tampered MIC is rejected.
 * 4. A tampered ciphertext is rejected.
 * 5. A tampered header (AAD) is rejected — this is the property that
 *    stops an attacker rewriting addresses on an encrypted frame.
 * 6. The wrong key is rejected.
 * 7. PN increment handles carry across all six bytes.
 *
 * Build:
 *   cc -Wall -Wextra -O2 -I../src -I. \
 *      ../src/vwifi_crypto.c crypto.c -o crypto
 */

#include <stdio.h>
#include <string.h>
#include <assert.h>

#include "vwifi_crypto.h"

static void hexdump(const char *label, const uint8_t *p, size_t n)
{
    printf("    %s: ", label);
    for (size_t i = 0; i < n; i++) printf("%02x", p[i]);
    printf("\n");
}

/* ---- 1. AES-128 known-answer test (FIPS-197 Appendix C.1) ---- */
static void test_aes_kat(void)
{
    /* FIPS-197 C.1:
     *   KEY        000102030405060708090a0b0c0d0e0f
     *   PLAINTEXT  00112233445566778899aabbccddeeff
     *   CIPHERTEXT 69c4e0d86a7b0430d8cdb78070b4c55a
     */
    const uint8_t key[16] = {
        0x00,0x01,0x02,0x03,0x04,0x05,0x06,0x07,
        0x08,0x09,0x0a,0x0b,0x0c,0x0d,0x0e,0x0f
    };
    const uint8_t pt[16] = {
        0x00,0x11,0x22,0x33,0x44,0x55,0x66,0x77,
        0x88,0x99,0xaa,0xbb,0xcc,0xdd,0xee,0xff
    };
    const uint8_t expect[16] = {
        0x69,0xc4,0xe0,0xd8,0x6a,0x7b,0x04,0x30,
        0xd8,0xcd,0xb7,0x80,0x70,0xb4,0xc5,0x5a
    };
    struct vwifi_aes128 ctx;
    uint8_t out[16];

    vwifi_aes128_key_expand(key, &ctx);
    vwifi_aes128_encrypt(&ctx, pt, out);

    if (memcmp(out, expect, 16) != 0) {
        printf("  AES-128 FIPS-197 C.1: FAIL\n");
        hexdump("expected", expect, 16);
        hexdump("got     ", out, 16);
        assert(0);
    }
    printf("  AES-128 FIPS-197 C.1 known-answer: PASS\n");
}

/* Build a QoS-less 802.11 data frame with an LLC/SNAP payload. */
static size_t build_data_frame(uint8_t *buf, size_t payload_len)
{
    static const uint8_t bssid[6] = { 0xAA,0xBB,0xCC,0x00,0x00,0x01 };
    static const uint8_t sta[6]   = { 0x00,0x03,0x7F,0xCC,0xDD,0x10 };
    static const uint8_t peer[6]  = { 0x11,0x22,0x33,0x44,0x55,0x66 };
    size_t len = 0;

    memset(buf, 0, 24);
    buf[0] = 0x08;          /* data, subtype 0 */
    buf[1] = 0x01;          /* ToDS */
    memcpy(buf + 4,  bssid, 6);
    memcpy(buf + 10, sta, 6);
    memcpy(buf + 16, peer, 6);
    buf[22] = 0x10;         /* seq ctrl low */
    buf[23] = 0x00;
    len = 24;

    for (size_t i = 0; i < payload_len; i++) {
        buf[len + i] = (uint8_t)(0x40 + (i & 0x3F));
    }
    return len + payload_len;
}

static void test_ccmp_roundtrip(void)
{
    const uint8_t key[16] = {
        0x0f,0x0e,0x0d,0x0c,0x0b,0x0a,0x09,0x08,
        0x07,0x06,0x05,0x04,0x03,0x02,0x01,0x00
    };
    const uint8_t pn[6] = { 0x00,0x00,0x00,0x00,0x00,0x01 };
    struct vwifi_aes128 aes;
    uint8_t frame[256], orig[256];
    size_t plain_len;
    int enc_len, dec_len;

    vwifi_aes128_key_expand(key, &aes);

    plain_len = build_data_frame(frame, 40);
    memcpy(orig, frame, plain_len);

    enc_len = vwifi_ccmp_encrypt(&aes, frame, plain_len, pn, 0, sizeof(frame));
    assert(enc_len > 0);
    assert((size_t)enc_len == plain_len + VWIFI_CCMP_HDR_LEN + VWIFI_CCMP_MIC_LEN);

    /* Protected bit must be set now. */
    assert(frame[1] & 0x40);
    /* Payload must actually differ from the plaintext. */
    assert(memcmp(frame + 24 + VWIFI_CCMP_HDR_LEN, orig + 24, 40) != 0);
    /* ExtIV bit set in the CCMP header. */
    assert(frame[24 + 3] & 0x20);

    uint8_t got_pn[6]; uint8_t got_kid = 0xFF;
    dec_len = vwifi_ccmp_decrypt(&aes, frame, (size_t)enc_len, got_pn, &got_kid);
    assert(dec_len > 0);
    assert((size_t)dec_len == plain_len);
    assert(memcmp(got_pn, pn, 6) == 0);
    assert(got_kid == 0);
    assert(memcmp(frame, orig, plain_len) == 0);   /* byte-identical */

    printf("  CCMP encrypt -> decrypt round trip: PASS\n");
}

static void test_ccmp_tamper_mic(void)
{
    const uint8_t key[16] = { 1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16 };
    const uint8_t pn[6] = { 0,0,0,0,0,5 };
    struct vwifi_aes128 aes;
    uint8_t frame[256];
    size_t plain_len;
    int enc_len;

    vwifi_aes128_key_expand(key, &aes);
    plain_len = build_data_frame(frame, 32);
    enc_len = vwifi_ccmp_encrypt(&aes, frame, plain_len, pn, 0, sizeof(frame));
    assert(enc_len > 0);

    frame[enc_len - 1] ^= 0x01;   /* flip a MIC bit */
    assert(vwifi_ccmp_decrypt(&aes, frame, (size_t)enc_len, NULL, NULL) == -2);
    printf("  tampered MIC rejected: PASS\n");
}

static void test_ccmp_tamper_payload(void)
{
    const uint8_t key[16] = { 1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16 };
    const uint8_t pn[6] = { 0,0,0,0,0,6 };
    struct vwifi_aes128 aes;
    uint8_t frame[256];
    size_t plain_len;
    int enc_len;

    vwifi_aes128_key_expand(key, &aes);
    plain_len = build_data_frame(frame, 32);
    enc_len = vwifi_ccmp_encrypt(&aes, frame, plain_len, pn, 0, sizeof(frame));
    assert(enc_len > 0);

    frame[24 + VWIFI_CCMP_HDR_LEN + 5] ^= 0x80;   /* flip a ciphertext bit */
    assert(vwifi_ccmp_decrypt(&aes, frame, (size_t)enc_len, NULL, NULL) == -2);
    printf("  tampered ciphertext rejected: PASS\n");
}

static void test_ccmp_tamper_header(void)
{
    const uint8_t key[16] = { 1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16 };
    const uint8_t pn[6] = { 0,0,0,0,0,7 };
    struct vwifi_aes128 aes;
    uint8_t frame[256];
    size_t plain_len;
    int enc_len;

    vwifi_aes128_key_expand(key, &aes);
    plain_len = build_data_frame(frame, 32);
    enc_len = vwifi_ccmp_encrypt(&aes, frame, plain_len, pn, 0, sizeof(frame));
    assert(enc_len > 0);

    /* Rewrite the destination address (A3). The header isn't encrypted
     * but it IS authenticated via the AAD, so this must be caught. */
    frame[16] ^= 0xFF;
    assert(vwifi_ccmp_decrypt(&aes, frame, (size_t)enc_len, NULL, NULL) == -2);
    printf("  tampered header (AAD) rejected: PASS\n");
}

static void test_ccmp_wrong_key(void)
{
    const uint8_t key1[16] = { 1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16 };
    const uint8_t key2[16] = { 1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,99 };
    const uint8_t pn[6] = { 0,0,0,0,0,8 };
    struct vwifi_aes128 a1, a2;
    uint8_t frame[256];
    size_t plain_len;
    int enc_len;

    vwifi_aes128_key_expand(key1, &a1);
    vwifi_aes128_key_expand(key2, &a2);

    plain_len = build_data_frame(frame, 32);
    enc_len = vwifi_ccmp_encrypt(&a1, frame, plain_len, pn, 0, sizeof(frame));
    assert(enc_len > 0);

    assert(vwifi_ccmp_decrypt(&a2, frame, (size_t)enc_len, NULL, NULL) == -2);
    printf("  wrong key rejected: PASS\n");
}

static void test_pn(void)
{
    uint8_t pn[6] = { 0, 0, 0, 0, 0, 0xFF };
    vwifi_pn_increment(pn);
    assert(pn[5] == 0x00 && pn[4] == 0x01);

    uint8_t pn2[6] = { 0, 0, 0, 0xFF, 0xFF, 0xFF };
    vwifi_pn_increment(pn2);
    assert(pn2[2] == 0x01 && pn2[3] == 0x00 && pn2[4] == 0x00 && pn2[5] == 0x00);

    uint8_t a[6] = { 0,0,0,0,0,1 };
    uint8_t b[6] = { 0,0,0,0,0,2 };
    assert(vwifi_pn_cmp(a, b) < 0);
    assert(vwifi_pn_cmp(b, a) > 0);
    assert(vwifi_pn_cmp(a, a) == 0);

    printf("  PN increment carry + comparison: PASS\n");
}

/* Different PNs must produce different ciphertext for identical
 * plaintext — this is what stops keystream reuse. */
static void test_ccmp_pn_varies_keystream(void)
{
    const uint8_t key[16] = { 1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16 };
    const uint8_t pn1[6] = { 0,0,0,0,0,1 };
    const uint8_t pn2[6] = { 0,0,0,0,0,2 };
    struct vwifi_aes128 aes;
    uint8_t f1[256], f2[256];
    size_t len1, len2;
    int e1, e2;

    vwifi_aes128_key_expand(key, &aes);
    len1 = build_data_frame(f1, 32);
    len2 = build_data_frame(f2, 32);
    assert(len1 == len2);

    e1 = vwifi_ccmp_encrypt(&aes, f1, len1, pn1, 0, sizeof(f1));
    e2 = vwifi_ccmp_encrypt(&aes, f2, len2, pn2, 0, sizeof(f2));
    assert(e1 > 0 && e2 == e1);

    /* Same plaintext, different PN -> different ciphertext. */
    assert(memcmp(f1 + 24 + VWIFI_CCMP_HDR_LEN,
                  f2 + 24 + VWIFI_CCMP_HDR_LEN, 32) != 0);
    printf("  distinct PN yields distinct keystream: PASS\n");
}

int main(void)
{
    test_aes_kat();
    test_ccmp_roundtrip();
    test_ccmp_tamper_mic();
    test_ccmp_tamper_payload();
    test_ccmp_tamper_header();
    test_ccmp_wrong_key();
    test_ccmp_pn_varies_keystream();
    test_pn();
    printf("crypto: PASS\n");
    return 0;
}
