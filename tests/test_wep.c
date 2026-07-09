/*
 * Standalone verification of vwifi_ath9k_wep.h (the WEP RC4 + CRC-32 engine
 * the virtual AR9285 uses for hardware-crypto offload of WEP-40/104/128).
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 *   [1] RC4 keystream vs the well-known published test vectors.
 *   [2] CRC-32 (WEP ICV) vs zlib crc32() + the canonical check value.
 *   [3] WEP MPDU encrypt->decrypt round-trip (WEP-40/104/128, QoS/non-QoS).
 *   [4] ICV rejection (wrong key / tampered ciphertext).
 *   [5] Key-cache layout (inverts the ath9k key split for each WEP type).
 *   [6] ExtIV discrimination (plain WEP header vs CCMP/TKIP extended IV).
 *
 * Build & run (from the project root):
 *   make test-wep
 * or directly:
 *   cc -O2 -Isrc -o tests/test_wep tests/test_wep.c -lz && ./tests/test_wep
 */
#include <zlib.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>

#include "vwifi_ath9k_wep.h"

static int fails = 0;

static void hexdump(const char *tag, const uint8_t *p, int n)
{
    int i;
    printf("  %s:", tag);
    for (i = 0; i < n; i++) printf(" %02x", p[i]);
    printf("\n");
}

/* ---- [1] RC4 known-answer vectors (Wikipedia / classic RC4 KATs) ---- */
static void test_rc4_kat(void)
{
    int base = fails;
    struct { const char *key, *pt; uint8_t ct[32]; int n; } v[] = {
        { "Key", "Plaintext",
          { 0xBB,0xF3,0x16,0xE8,0xD9,0x40,0xAF,0x0A,0xD3 }, 9 },
        { "Wiki", "pedia",
          { 0x10,0x21,0xBF,0x04,0x20 }, 5 },
        { "Secret", "Attack at dawn",
          { 0x45,0xA0,0x1F,0x64,0x5F,0xC3,0x5B,0x38,0x35,0x52,
            0x54,0x4B,0x9B,0xF5 }, 14 },
    };
    int i;
    printf("[1] RC4 keystream vs published test vectors\n");
    for (i = 0; i < 3; i++) {
        uint8_t buf[32];
        memcpy(buf, v[i].pt, v[i].n);
        vwifi_rc4((const uint8_t *)v[i].key, (int)strlen(v[i].key), buf, v[i].n);
        if (memcmp(buf, v[i].ct, v[i].n) != 0) {
            printf("  RC4 vector %d MISMATCH\n", i);
            hexdump("got", buf, v[i].n);
            hexdump("exp", v[i].ct, v[i].n);
            fails++;
        }
    }
    printf("  %s\n", (fails > base) ? "FAILED" : "OK (3 vectors match)");
}

/* ---- [2] CRC-32 vs zlib + canonical check value ---- */
static void test_crc32(void)
{
    int base = fails, trial;
    printf("[2] CRC-32 (WEP ICV) vs zlib crc32()\n");

    /* Canonical CRC-32 check value: crc32(\"123456789\") == 0xCBF43926. */
    if (vwifi_crc32((const uint8_t *)"123456789", 9) != 0xCBF43926u) {
        printf("  canonical check value FAILED (got %08x)\n",
               vwifi_crc32((const uint8_t *)"123456789", 9));
        fails++;
    }
    for (trial = 0; trial < 5000; trial++) {
        uint8_t buf[300];
        int len = rand() % (int)sizeof(buf), i;
        uint32_t a, b;
        for (i = 0; i < len; i++) buf[i] = rand();
        a = vwifi_crc32(buf, len);
        b = (uint32_t)crc32(0, buf, len);
        if (a != b) {
            printf("  CRC mismatch len=%d mine=%08x zlib=%08x\n", len, a, b);
            fails++;
            if (fails - base > 5) break;
        }
    }
    printf("  %s\n", (fails > base) ? "FAILED" : "OK (canonical + 5000 vs zlib)");
}

/* Build an 802.11 data header + 4-byte WEP IV header for the round-trip. */
static int build_wep_frame(uint8_t *f, int payload_len, int qos, int keyid)
{
    int hdrlen = qos ? 26 : 24;
    uint16_t fc = qos ? 0x0088 : 0x0008;   /* data; subtype 8 = QoS if qos */
    int i, off;
    fc |= 0x4000;                          /* Protected */
    fc |= 0x0100;                          /* ToDS */
    f[0] = fc & 0xff; f[1] = (fc >> 8) & 0xff;
    f[2] = 0; f[3] = 0;
    for (i = 0; i < 6; i++) f[4 + i]  = 0x10 + i;   /* A1 */
    for (i = 0; i < 6; i++) f[10 + i] = 0x20 + i;   /* A2 */
    for (i = 0; i < 6; i++) f[16 + i] = 0x30 + i;   /* A3 */
    f[22] = 0x40; f[23] = 0x05;
    if (qos) { f[24] = 0x06; f[25] = 0x00; }
    off = hdrlen;
    /* Plain WEP IV header: IV(3) || KeyID(1); ExtIV bit (0x20) must be clear. */
    f[off + 0] = 0x9a; f[off + 1] = 0xbc; f[off + 2] = 0xde;
    f[off + 3] = (uint8_t)((keyid & 3) << 6);
    off += 4;
    for (i = 0; i < payload_len; i++) f[off + i] = (uint8_t)(0x80 + i);
    return off + payload_len;
}

/* ---- [3] WEP MPDU encrypt -> decrypt round-trip ---- */
static void test_roundtrip(void)
{
    int base = fails;
    int keylens[3] = { 5, 13, 16 };      /* WEP-40, WEP-104, WEP-128 */
    int ki, qos, len, i;
    printf("[3] WEP MPDU encrypt->decrypt round-trip\n");

    for (ki = 0; ki < 3; ki++) {
        uint8_t key[16];
        for (i = 0; i < keylens[ki]; i++) key[i] = (uint8_t)(i * 13 + 7);
        for (qos = 0; qos <= 1; qos++) {
            for (len = 0; len <= 200; len++) {
                uint8_t f[512], orig[512];
                int flen = build_wep_frame(f, len, qos, 2);
                int hdrlen = qos ? 26 : 24, flen_enc = flen;
                memcpy(orig, f, flen);

                if (vwifi_wep_encrypt(key, keylens[ki], f, &flen_enc,
                                      sizeof(f)) != 0 || flen_enc != flen + 4) {
                    printf("  encrypt failed kl=%d qos=%d len=%d\n",
                           keylens[ki], qos, len); fails++; return;
                }
                if (len > 0 && memcmp(f + hdrlen + 4, orig + hdrlen + 4, len) == 0) {
                    printf("  ciphertext == plaintext kl=%d len=%d\n",
                           keylens[ki], len); fails++; return;
                }
                if (vwifi_wep_decrypt(key, keylens[ki], f, flen_enc) != 0) {
                    printf("  decrypt/ICV failed kl=%d qos=%d len=%d\n",
                           keylens[ki], qos, len); fails++; return;
                }
                if (memcmp(f, orig, flen) != 0) {
                    printf("  frame mismatch after round-trip kl=%d qos=%d len=%d\n",
                           keylens[ki], qos, len); fails++; return;
                }
            }
        }
    }
    printf("  %s\n", (fails > base) ? "FAILED"
           : "OK (WEP-40/104/128, plaintext & header recovered)");
}

/* ---- [4] ICV rejection ---- */
static void test_icv_rejection(void)
{
    int base = fails, i;
    uint8_t key[13], wrong[13], f[256];
    int flen, flen_enc;
    printf("[4] ICV rejection (wrong key / tampered ciphertext)\n");
    for (i = 0; i < 13; i++) { key[i] = i + 3; wrong[i] = i + 77; }

    flen = build_wep_frame(f, 96, 1, 1);
    flen_enc = flen;
    vwifi_wep_encrypt(key, 13, f, &flen_enc, sizeof(f));

    { /* wrong key */
        uint8_t g[256]; memcpy(g, f, flen_enc);
        if (vwifi_wep_decrypt(wrong, 13, g, flen_enc) == 0) {
            printf("  wrong key accepted!\n"); fails++;
        }
    }
    { /* tampered ciphertext */
        uint8_t g[256]; memcpy(g, f, flen_enc);
        g[26 + 4 + 10] ^= 0x01;
        if (vwifi_wep_decrypt(key, 13, g, flen_enc) == 0) {
            printf("  tampered ciphertext accepted!\n"); fails++;
        }
    }
    printf("  %s\n", (fails > base) ? "FAILED" : "OK (forgeries rejected)");
}

/* ---- [5] Key-cache layout for WEP types ---- */
static uint32_t le32(const uint8_t *p){return p[0]|(p[1]<<8)|(p[2]<<16)|((uint32_t)p[3]<<24);}
static uint32_t le16(const uint8_t *p){return p[0]|(p[1]<<8);}

/* ath9k key split (same for all cipher types); type in the TYPE register. */
static void kc_store(uint32_t *regs, unsigned slot, const uint8_t k[16], uint32_t type)
{
    uint32_t b = 0x8800 + slot * 32;
    regs[(b + 0) / 4]  = le32(k + 0);
    regs[(b + 4) / 4]  = le16(k + 4);
    regs[(b + 8) / 4]  = le32(k + 6);
    regs[(b + 12) / 4] = le16(k + 10);
    regs[(b + 16) / 4] = le32(k + 12);
    regs[(b + 20) / 4] = type;
}

/* Mirror of the device's vwifi_ath9k_keycache_wep(): returns key length. */
static int kc_load_wep(const uint32_t *regs, unsigned slot, uint8_t key[16])
{
    uint32_t b = 0x8800 + slot * 32, k0,k1,k2,k3,k4;
    int keylen;
    switch (regs[(b + 20) / 4] & 7) {
        case 0: keylen = 5;  break;   /* AR_KEYTABLE_TYPE_40  */
        case 1: keylen = 13; break;   /* AR_KEYTABLE_TYPE_104 */
        case 3: keylen = 16; break;   /* AR_KEYTABLE_TYPE_128 */
        default: return 0;
    }
    k0 = regs[(b+0)/4]; k1 = regs[(b+4)/4]&0xffff; k2 = regs[(b+8)/4];
    k3 = regs[(b+12)/4]&0xffff; k4 = regs[(b+16)/4];
    key[0]=k0;key[1]=k0>>8;key[2]=k0>>16;key[3]=k0>>24;
    key[4]=k1;key[5]=k1>>8;
    key[6]=k2;key[7]=k2>>8;key[8]=k2>>16;key[9]=k2>>24;
    key[10]=k3;key[11]=k3>>8;
    key[12]=k4;key[13]=k4>>8;key[14]=k4>>16;key[15]=k4>>24;
    return keylen;
}

static void test_keycache(void)
{
    int base = fails, i;
    static uint32_t regs[16384];
    uint8_t k40[16] = {0}, k104[16] = {0}, k128[16], got[16];
    printf("[5] Key-cache layout (WEP-40/104/128 reconstruct) + data path\n");

    /* The ath9k driver resets every key-cache entry to CLR (7) during init,
     * so unused slots read as CLR, not zero.  Model that: WEP-40 is type 0 and
     * would otherwise be indistinguishable from a never-written slot. */
    for (i = 0; i < 128; i++) regs[(0x8800 + i * 32 + 20) / 4] = 7; /* CLR */

    for (i = 0; i < 5;  i++) k40[i]  = 0xA0 + i;
    for (i = 0; i < 13; i++) k104[i] = 0xB0 + i;
    for (i = 0; i < 16; i++) k128[i] = 0xC0 + i;

    kc_store(regs, 0, k40,  0); /* TYPE_40  */
    kc_store(regs, 1, k104, 1); /* TYPE_104 */
    kc_store(regs, 2, k128, 3); /* TYPE_128 */

    if (kc_load_wep(regs, 0, got) != 5  || memcmp(got, k40, 5))
        { printf("  WEP-40 reconstruct FAILED\n"); fails++; }
    if (kc_load_wep(regs, 1, got) != 13 || memcmp(got, k104, 13))
        { printf("  WEP-104 reconstruct FAILED\n"); fails++; }
    if (kc_load_wep(regs, 2, got) != 16 || memcmp(got, k128, 16))
        { printf("  WEP-128 reconstruct FAILED\n"); fails++; }
    if (kc_load_wep(regs, 7, got) != 0)
        { printf("  empty slot returned a key!\n"); fails++; }

    /* Full path: TX encrypts from a slot; RX brute-forces WEP slots + ICV. */
    {
        uint8_t txkey[16], f[256], orig[256];
        int flen = build_wep_frame(f, 120, 0, 1), flen_enc = flen;
        int kl, slot, rx_slot = -1;
        memcpy(orig, f, flen);
        kl = kc_load_wep(regs, 1, txkey);          /* use the WEP-104 slot */
        if (vwifi_wep_encrypt(txkey, kl, f, &flen_enc, sizeof(f)) != 0)
            { printf("  tx enc failed\n"); fails++; }
        for (slot = 0; slot < 8; slot++) {
            uint8_t k[16], dec[256]; int l = kc_load_wep(regs, slot, k);
            if (l == 0) continue;
            memcpy(dec, f, flen_enc);
            if (vwifi_wep_decrypt(k, l, dec, flen_enc) == 0) {
                rx_slot = slot;
                if (memcmp(dec, orig, flen)) { printf("  rx mismatch\n"); fails++; }
                break;
            }
        }
        if (rx_slot != 1) { printf("  rx picked slot %d, expected 1\n", rx_slot); fails++; }
    }
    printf("  %s\n", (fails > base) ? "FAILED"
           : "OK (per-type length + inverts split; TX->RX recovers frame & slot)");
}

/* ---- [6] ExtIV discrimination (WEP vs CCMP/TKIP header) ---- */
static void test_extiv(void)
{
    int base = fails;
    uint8_t f[64];
    int flen = build_wep_frame(f, 16, 0, 1);  /* plain WEP: ExtIV clear */
    printf("[6] ExtIV discrimination (WEP 4-byte vs extended 8-byte IV)\n");

    if (vwifi_wep_is_extiv(f, flen, 24)) {
        printf("  plain WEP header misdetected as ExtIV!\n"); fails++;
    }
    f[24 + 3] |= 0x20;                         /* set ExtIV (CCMP/TKIP) */
    if (!vwifi_wep_is_extiv(f, flen, 24)) {
        printf("  ExtIV header not detected!\n"); fails++;
    }
    printf("  %s\n", (fails > base) ? "FAILED" : "OK (ExtIV bit routes WEP vs CCMP)");
}

int main(void)
{
    srand(24680);
    test_rc4_kat();
    test_crc32();
    test_roundtrip();
    test_icv_rejection();
    test_keycache();
    test_extiv();
    printf("\n%s\n", fails ? "*** TESTS FAILED ***" : "*** ALL TESTS PASSED ***");
    return fails ? 1 : 0;
}
