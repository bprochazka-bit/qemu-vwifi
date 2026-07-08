/*
 * Standalone verification of vwifi_ath9k_crypto.h (the CCMP engine the
 * virtual AR9285 uses for WPA2 hardware-crypto offload).
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 *   [1] CCM core (CBC-MAC + CTR) vs OpenSSL EVP_aes_128_ccm over random inputs.
 *   [2] Full CCMP MPDU encrypt->decrypt round-trip.
 *   [3] MIC rejection (wrong key / tampered payload / tampered header).
 *   [4] Key-cache layout (inverts the ath9k key split) + the full device
 *       data path: TX encrypts with a cached slot, RX brute-forces the cache.
 *
 * Build & run (from the project root):
 *   make test-crypto
 * or directly:
 *   cc -O2 -Isrc -o tests/test_ccmp tests/test_ccmp.c -lcrypto && ./tests/test_ccmp
 */
#include <openssl/aes.h>
#include <openssl/evp.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>

#include "vwifi_ath9k_crypto.h"

static int fails = 0;

static void hexdump(const char *tag, const uint8_t *p, int n)
{
    int i;
    printf("  %s:", tag);
    for (i = 0; i < n; i++) printf(" %02x", p[i]);
    printf("\n");
}

/* Run the vwifi CCM core as a generic AEAD encrypt. */
static void my_ccm(const uint8_t key[16], const uint8_t nonce[13],
                   const uint8_t *aad, int aad_len,
                   const uint8_t *pt, int pt_len,
                   uint8_t *ct_out, uint8_t tag_out[8])
{
    AES_KEY aes;
    uint8_t mic[8];
    AES_set_encrypt_key(key, 128, &aes);
    memcpy(ct_out, pt, pt_len);
    vwifi_ccm_cbc_mac(&aes, nonce, aad, aad_len, ct_out, pt_len, mic);
    vwifi_ccm_ctr(&aes, nonce, ct_out, pt_len, mic);
    memcpy(tag_out, mic, 8);
}

/* OpenSSL reference AES-128-CCM (nonce=13, tag=8). */
static void ossl_ccm(const uint8_t key[16], const uint8_t nonce[13],
                     const uint8_t *aad, int aad_len,
                     const uint8_t *pt, int pt_len,
                     uint8_t *ct_out, uint8_t tag_out[8])
{
    EVP_CIPHER_CTX *c = EVP_CIPHER_CTX_new();
    int len;
    EVP_EncryptInit_ex(c, EVP_aes_128_ccm(), NULL, NULL, NULL);
    EVP_CIPHER_CTX_ctrl(c, EVP_CTRL_CCM_SET_IVLEN, 13, NULL);
    EVP_CIPHER_CTX_ctrl(c, EVP_CTRL_CCM_SET_TAG, 8, NULL);
    EVP_EncryptInit_ex(c, NULL, NULL, key, nonce);
    EVP_EncryptUpdate(c, NULL, &len, NULL, pt_len);          /* total msg len */
    EVP_EncryptUpdate(c, NULL, &len, aad, aad_len);          /* AAD */
    EVP_EncryptUpdate(c, ct_out, &len, pt, pt_len);
    EVP_EncryptFinal_ex(c, ct_out + len, &len);
    EVP_CIPHER_CTX_ctrl(c, EVP_CTRL_CCM_GET_TAG, 8, tag_out);
    EVP_CIPHER_CTX_free(c);
}

static void test_core_vs_openssl(void)
{
    int trial;
    printf("[1] CCM core vs OpenSSL EVP_aes_128_ccm (random)\n");
    for (trial = 0; trial < 2000; trial++) {
        uint8_t key[16], nonce[13];
        uint8_t aad[64], pt[512];
        uint8_t ct_a[512], tag_a[8];
        uint8_t ct_b[512], tag_b[8];
        int aad_len = rand() % 33;       /* 0..32 */
        int pt_len  = rand() % 300;      /* 0..299 */
        int i;

        for (i = 0; i < 16; i++) key[i] = rand();
        for (i = 0; i < 13; i++) nonce[i] = rand();
        for (i = 0; i < aad_len; i++) aad[i] = rand();
        for (i = 0; i < pt_len; i++) pt[i] = rand();

        my_ccm(key, nonce, aad, aad_len, pt, pt_len, ct_a, tag_a);
        ossl_ccm(key, nonce, aad, aad_len, pt, pt_len, ct_b, tag_b);

        if (memcmp(ct_a, ct_b, pt_len) || memcmp(tag_a, tag_b, 8)) {
            printf("  MISMATCH trial=%d aad_len=%d pt_len=%d\n",
                   trial, aad_len, pt_len);
            hexdump("mine ct", ct_a, pt_len);
            hexdump("ossl ct", ct_b, pt_len);
            hexdump("mine tag", tag_a, 8);
            hexdump("ossl tag", tag_b, 8);
            fails++;
            if (fails > 5) return;
        }
    }
    printf("  %s\n", fails ? "FAILED" : "OK (2000 trials match)");
}

/* Build a QoS-data 802.11 header + CCMP header for the MPDU round-trip. */
static int build_frame(uint8_t *f, int payload_len, int qos, int keyid)
{
    int hdrlen = qos ? 26 : 24;
    uint16_t fc = 0x0088;               /* data(2), subtype 8 = QoS data if qos */
    int i, off;
    if (!qos) fc = 0x0008;              /* data, subtype 0 */
    fc |= 0x4000;                       /* Protected */
    fc |= 0x0100;                       /* ToDS */
    f[0] = fc & 0xff; f[1] = (fc >> 8) & 0xff;
    f[2] = 0; f[3] = 0;                 /* duration */
    for (i = 0; i < 6; i++) f[4 + i]  = 0x10 + i;   /* A1 */
    for (i = 0; i < 6; i++) f[10 + i] = 0x20 + i;   /* A2 (TA) */
    for (i = 0; i < 6; i++) f[16 + i] = 0x30 + i;   /* A3 */
    f[22] = 0x40; f[23] = 0x05;         /* seq ctrl (frag 0, seq 0x54) */
    if (qos) { f[24] = 0x06; f[25] = 0x00; }        /* QoS ctrl, TID=6 */
    off = hdrlen;
    /* CCMP header: PN0 PN1 rsvd keyid PN2 PN3 PN4 PN5 */
    f[off+0] = 0x11; f[off+1] = 0x22; f[off+2] = 0x00;
    f[off+3] = (uint8_t)(0x20 | (keyid << 6));      /* ExtIV=1 */
    f[off+4] = 0x33; f[off+5] = 0x44; f[off+6] = 0x55; f[off+7] = 0x66;
    off += 8;
    for (i = 0; i < payload_len; i++) f[off + i] = (uint8_t)(0x80 + i);
    return off + payload_len;
}

static void test_roundtrip(void)
{
    int base = fails;
    uint8_t key[16];
    int qos, trial, i;
    printf("[2] CCMP MPDU encrypt->decrypt round-trip\n");
    for (i = 0; i < 16; i++) key[i] = i * 7 + 1;

    for (qos = 0; qos <= 1; qos++) {
        for (trial = 1; trial <= 200; trial++) {
            uint8_t f[4096], orig[4096];
            int payload_len = trial;
            int flen = build_frame(f, payload_len, qos, 1);
            int hdrlen = qos ? 26 : 24;
            memcpy(orig, f, flen);
            int flen_enc = flen;

            if (vwifi_ccmp_encrypt(key, f, &flen_enc, sizeof(f)) != 0) {
                printf("  encrypt failed qos=%d len=%d\n", qos, payload_len); fails++; return;
            }
            if (flen_enc != flen + 8) {
                printf("  bad enc len qos=%d\n", qos); fails++; return;
            }
            /* ciphertext must differ from plaintext */
            if (memcmp(f + hdrlen + 8, orig + hdrlen + 8, payload_len) == 0 && payload_len > 0) {
                printf("  ciphertext == plaintext qos=%d len=%d\n", qos, payload_len); fails++; return;
            }
            if (vwifi_ccmp_decrypt(key, f, flen_enc) != 0) {
                printf("  decrypt/MIC failed qos=%d len=%d\n", qos, payload_len); fails++; return;
            }
            if (memcmp(f + hdrlen + 8, orig + hdrlen + 8, payload_len) != 0) {
                printf("  plaintext mismatch after roundtrip qos=%d len=%d\n", qos, payload_len);
                fails++; return;
            }
            /* header + CCMP header preserved */
            if (memcmp(f, orig, hdrlen + 8) != 0) {
                printf("  header corrupted qos=%d\n", qos); fails++; return;
            }
        }
    }
    printf("  %s\n", (fails>base) ? "FAILED" : "OK (round-trip + plaintext recovered)");
}

static void test_mic_rejection(void)
{
    int base = fails;
    uint8_t key[16], wrongkey[16], f[512];
    int i, flen, flen_enc;
    printf("[3] MIC rejection (tampered payload / wrong key)\n");
    for (i = 0; i < 16; i++) { key[i] = i + 3; wrongkey[i] = i + 99; }

    flen = build_frame(f, 64, 1, 2);
    flen_enc = flen;
    vwifi_ccmp_encrypt(key, f, &flen_enc, sizeof(f));

    /* Wrong key must fail. */
    {
        uint8_t g[512]; memcpy(g, f, flen_enc);
        if (vwifi_ccmp_decrypt(wrongkey, g, flen_enc) == 0) {
            printf("  wrong key accepted!\n"); fails++;
        }
    }
    /* Tampered ciphertext must fail. */
    {
        uint8_t g[512]; memcpy(g, f, flen_enc);
        g[26 + 8 + 5] ^= 0x01;
        if (vwifi_ccmp_decrypt(key, g, flen_enc) == 0) {
            printf("  tampered payload accepted!\n"); fails++;
        }
    }
    /* Tampered header (AAD) must fail. */
    {
        uint8_t g[512]; memcpy(g, f, flen_enc);
        g[16] ^= 0x01; /* flip a bit in A3 */
        if (vwifi_ccmp_decrypt(key, g, flen_enc) == 0) {
            printf("  tampered header accepted!\n"); fails++;
        }
    }
    printf("  %s\n", (fails>base) ? "FAILED" : "OK (all forgeries rejected)");
}

/* ---- [4] Key-cache layout + full device data-path simulation ---- */

static uint32_t le32(const uint8_t *p){return p[0]|(p[1]<<8)|(p[2]<<16)|((uint32_t)p[3]<<24);}
static uint32_t le16(const uint8_t *p){return p[0]|(p[1]<<8);}

/* ath9k ath_hw_set_keycache_entry() key split (the "encoder"). */
static void kc_store(uint32_t *regs, unsigned slot, const uint8_t k[16])
{
    uint32_t b = 0x8800 + slot * 32;
    regs[(b + 0) / 4]  = le32(k + 0);
    regs[(b + 4) / 4]  = le16(k + 4);
    regs[(b + 8) / 4]  = le32(k + 6);
    regs[(b + 12) / 4] = le16(k + 10);
    regs[(b + 16) / 4] = le32(k + 12);
    regs[(b + 20) / 4] = 6; /* AR_KEYTABLE_TYPE_CCM */
}

/* Mirror of vwifi_ath9k_keycache_ccm()'s reconstruction. */
static bool kc_load(const uint32_t *regs, unsigned slot, uint8_t key[16])
{
    uint32_t b = 0x8800 + slot * 32, k0,k1,k2,k3,k4;
    if ((regs[(b + 20) / 4] & 7) != 6) return false;
    k0 = regs[(b+0)/4]; k1 = regs[(b+4)/4]&0xffff; k2 = regs[(b+8)/4];
    k3 = regs[(b+12)/4]&0xffff; k4 = regs[(b+16)/4];
    key[0]=k0;key[1]=k0>>8;key[2]=k0>>16;key[3]=k0>>24;
    key[4]=k1;key[5]=k1>>8;
    key[6]=k2;key[7]=k2>>8;key[8]=k2>>16;key[9]=k2>>24;
    key[10]=k3;key[11]=k3>>8;
    key[12]=k4;key[13]=k4>>8;key[14]=k4>>16;key[15]=k4>>24;
    return true;
}

static void test_keycache_and_datapath(void)
{
    int base = fails, i;
    static uint32_t regs[16384];
    uint8_t pairwise[16], group[16], got[16];
    printf("[4] Key-cache layout + device data-path (encrypt@TX -> decrypt@RX)\n");

    for (i = 0; i < 16; i++) { pairwise[i] = 0xA0 + i; group[i] = 0x5C ^ (i * 3); }

    /* Driver programs two CCM keys into the cache (like STA: pairwise + GTK). */
    kc_store(regs, 0, pairwise);   /* slot 0 = pairwise (PTK) */
    kc_store(regs, 2, group);      /* slot 2 = group (GTK)    */

    /* Layout reconstruction must invert the ath9k split exactly. */
    if (!kc_load(regs, 0, got) || memcmp(got, pairwise, 16)) { printf("  pairwise key reconstruct FAILED\n"); fails++; }
    if (!kc_load(regs, 2, got) || memcmp(got, group, 16))    { printf("  group key reconstruct FAILED\n"); fails++; }
    if (kc_load(regs, 5, got))                                { printf("  empty slot returned a key!\n"); fails++; }

    /* Full path: TX encrypts with a chosen slot; RX brute-forces the cache. */
    for (i = 0; i < 2; i++) {
        unsigned tx_slot = i ? 2 : 0;          /* group then pairwise */
        uint8_t txkey[16], f[512], orig[512];
        int flen = build_frame(f, 100, /*qos*/i, /*keyid*/i ? 1 : 0);
        int hdrlen = i ? 26 : 24, flen_enc = flen, slot, rx_slot = -1;
        memcpy(orig, f, flen);

        kc_load(regs, tx_slot, txkey);
        if (vwifi_ccmp_encrypt(txkey, f, &flen_enc, sizeof(f)) != 0) { printf("  tx enc failed\n"); fails++; break; }

        /* RX: try every populated CCM slot, accept the MIC that verifies. */
        for (slot = 0; slot < 16384/8; slot++) {
            uint8_t k[16], dec[512];
            if (!kc_load(regs, slot, k)) continue;
            memcpy(dec, f, flen_enc);
            if (vwifi_ccmp_decrypt(k, dec, flen_enc) == 0) {
                rx_slot = slot;
                if (memcmp(dec + hdrlen + 8, orig + hdrlen + 8, 100)) { printf("  rx plaintext mismatch\n"); fails++; }
                break;
            }
        }
        if (rx_slot != (int)tx_slot) { printf("  rx picked slot %d, expected %u\n", rx_slot, tx_slot); fails++; }
    }
    printf("  %s\n", (fails>base) ? "FAILED" : "OK (layout inverts ath9k split; TX->RX recovers plaintext & slot)");
}

int main(void)
{
    srand(12345);
    test_core_vs_openssl();
    test_roundtrip();
    test_mic_rejection();
    test_keycache_and_datapath();
    printf("\n%s\n", fails ? "*** TESTS FAILED ***" : "*** ALL TESTS PASSED ***");
    return fails ? 1 : 0;
}
