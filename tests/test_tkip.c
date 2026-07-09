/*
 * Standalone verification of vwifi_ath9k_tkip.h (the TKIP per-packet key
 * mixing + RC4 + CRC-32 ICV engine the virtual AR9285 uses for hardware-crypto
 * offload of TKIP).
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 *   [1] Phase 1 + Phase 2 key mixing vs the published IEEE 802.11i test
 *       vectors (TK, TA, IV32, IV16 -> RC4 key).
 *   [2] TKIP MPDU encrypt->decrypt round-trip (QoS / non-QoS, many lengths).
 *   [3] ICV rejection (wrong TK / tampered ciphertext).
 *   [4] Key-cache layout (TKIP TK reconstruction) + full TX->RX data path.
 *
 * RC4 and CRC-32 are shared with the WEP engine and are known-answer tested in
 * tests/test_wep.c; this suite focuses on the TKIP-specific mixing and framing.
 *
 * Build & run (from the project root):
 *   make test-tkip
 * or directly:
 *   cc -O2 -Isrc -o tests/test_tkip tests/test_tkip.c && ./tests/test_tkip
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>

#include "vwifi_ath9k_tkip.h"

static int fails = 0;

static void hexdump(const char *tag, const uint8_t *p, int n)
{
    int i;
    printf("  %s:", tag);
    for (i = 0; i < n; i++) printf(" %02x", p[i]);
    printf("\n");
}

/* ---- [1] Phase 1 + Phase 2 vs published IEEE 802.11i key-mixing vectors ---- */
static void test_mixing_vectors(void)
{
    int base = fails, i;
    struct {
        uint8_t tk[16], ta[6];
        uint32_t iv32;
        uint16_t iv16;
        uint8_t rc4key[16];
    } v[] = {
        /* 802.11i TKIP test vector #3 */
        { {0x63,0x89,0x3B,0x25,0x08,0x40,0xB8,0xAE,
           0x0B,0xD0,0xFA,0x7E,0x61,0xD2,0x78,0x3E},
          {0x64,0xF2,0xEA,0xED,0xDC,0x25}, 0x20DCFD43u, 0xFFFF,
          {0xFF,0x7F,0xFF,0x93,0x81,0x0F,0xC6,0xE5,
           0x8F,0x5D,0xD3,0x26,0x25,0x15,0x44,0xCE} },
        /* 802.11i TKIP test vector #5 */
        { {0x98,0x3A,0x16,0xEF,0x4F,0xAC,0xB3,0x51,
           0xAA,0x9E,0xCC,0x27,0x1D,0x73,0x09,0xE2},
          {0x50,0x9C,0x4B,0x17,0x27,0xD9}, 0xF0A410FCu, 0x058C,
          {0x05,0x25,0x8C,0xF4,0xD8,0x51,0x52,0xF4,
           0xD9,0xAF,0x1A,0x64,0xF1,0xD0,0x70,0x21} },
    };
    printf("[1] Phase1+Phase2 vs published 802.11i key-mixing vectors\n");
    for (i = 0; i < 2; i++) {
        uint16_t p1k[5];
        uint8_t rc4key[16];
        vwifi_tkip_phase1(v[i].tk, v[i].ta, v[i].iv32, p1k);
        vwifi_tkip_phase2(v[i].tk, p1k, v[i].iv16, rc4key);
        if (memcmp(rc4key, v[i].rc4key, 16) != 0) {
            printf("  vector #%d MISMATCH\n", i);
            hexdump("got", rc4key, 16);
            hexdump("exp", v[i].rc4key, 16);
            fails++;
        }
    }
    printf("  %s\n", (fails > base) ? "FAILED" : "OK (2 IEEE 802.11i vectors match)");
}

/* Build an 802.11 data header + 8-byte TKIP IV header for the round-trip. */
static int build_tkip_frame(uint8_t *f, int payload_len, int qos, int keyid)
{
    int hdrlen = qos ? 26 : 24;
    uint16_t fc = qos ? 0x0088 : 0x0008;
    int i, off;
    fc |= 0x4000;                          /* Protected */
    fc |= 0x0100;                          /* ToDS */
    f[0] = fc & 0xff; f[1] = (fc >> 8) & 0xff;
    f[2] = 0; f[3] = 0;
    for (i = 0; i < 6; i++) f[4 + i]  = 0x10 + i;   /* A1 */
    for (i = 0; i < 6; i++) f[10 + i] = 0x20 + i;   /* A2 = TA */
    for (i = 0; i < 6; i++) f[16 + i] = 0x30 + i;   /* A3 */
    f[22] = 0x40; f[23] = 0x05;
    if (qos) { f[24] = 0x06; f[25] = 0x00; }
    off = hdrlen;
    /* TKIP header: TSC1 WEPSeed TSC0 KeyID|ExtIV TSC2 TSC3 TSC4 TSC5.
     * iv16 = (TSC1<<8)|TSC0 = 0x2BC3; iv32 = 0x2211EEDD. */
    f[off + 0] = 0x2B;                              /* TSC1 */
    f[off + 1] = (uint8_t)((0x2B | 0x20) & 0x7f);   /* WEPSeed dummy */
    f[off + 2] = 0xC3;                              /* TSC0 */
    f[off + 3] = (uint8_t)(0x20 | (keyid << 6));    /* ExtIV=1 + KeyID */
    f[off + 4] = 0xdd; f[off + 5] = 0xee;           /* TSC2, TSC3 */
    f[off + 6] = 0x11; f[off + 7] = 0x22;           /* TSC4, TSC5 */
    off += 8;
    for (i = 0; i < payload_len; i++) f[off + i] = (uint8_t)(0x80 + i);
    return off + payload_len;
}

/* ---- [2] TKIP MPDU encrypt -> decrypt round-trip ---- */
static void test_roundtrip(void)
{
    int base = fails, qos, len, i;
    uint8_t tk[16];
    printf("[2] TKIP MPDU encrypt->decrypt round-trip\n");
    for (i = 0; i < 16; i++) tk[i] = (uint8_t)(i * 17 + 5);

    for (qos = 0; qos <= 1; qos++) {
        for (len = 0; len <= 200; len++) {
            uint8_t f[512], orig[512];
            int flen = build_tkip_frame(f, len, qos, 1);
            int hdrlen = qos ? 26 : 24, flen_enc = flen;
            memcpy(orig, f, flen);

            if (vwifi_tkip_encrypt(tk, f, &flen_enc, sizeof(f)) != 0 ||
                flen_enc != flen + 4) {
                printf("  encrypt failed qos=%d len=%d\n", qos, len);
                fails++; return;
            }
            if (len > 0 &&
                memcmp(f + hdrlen + 8, orig + hdrlen + 8, len) == 0) {
                printf("  ciphertext == plaintext qos=%d len=%d\n", qos, len);
                fails++; return;
            }
            /* the on-wire header's first 3 bytes must equal the RC4 IV bytes */
            {
                uint8_t rc4key[16];
                vwifi_tkip_rc4key(tk, f + 10, f + hdrlen, rc4key);
                if (memcmp(rc4key, f + hdrlen, 3) != 0) {
                    printf("  TKIP hdr/RC4 IV mismatch qos=%d\n", qos);
                    fails++; return;
                }
            }
            if (vwifi_tkip_decrypt(tk, f, flen_enc) != 0) {
                printf("  decrypt/ICV failed qos=%d len=%d\n", qos, len);
                fails++; return;
            }
            if (memcmp(f, orig, flen) != 0) {
                printf("  frame mismatch after round-trip qos=%d len=%d\n",
                       qos, len);
                fails++; return;
            }
        }
    }
    printf("  %s\n", (fails > base) ? "FAILED"
           : "OK (round-trip; plaintext & TKIP header recovered)");
}

/* ---- [3] ICV rejection ---- */
static void test_icv_rejection(void)
{
    int base = fails, i;
    uint8_t tk[16], wrong[16], f[256];
    int flen, flen_enc;
    printf("[3] ICV rejection (wrong TK / tampered ciphertext)\n");
    for (i = 0; i < 16; i++) { tk[i] = i + 1; wrong[i] = i + 100; }

    flen = build_tkip_frame(f, 96, 1, 1);
    flen_enc = flen;
    vwifi_tkip_encrypt(tk, f, &flen_enc, sizeof(f));

    { uint8_t g[256]; memcpy(g, f, flen_enc);
      if (vwifi_tkip_decrypt(wrong, g, flen_enc) == 0) {
          printf("  wrong TK accepted!\n"); fails++; } }
    { uint8_t g[256]; memcpy(g, f, flen_enc);
      g[26 + 8 + 12] ^= 0x01;
      if (vwifi_tkip_decrypt(tk, g, flen_enc) == 0) {
          printf("  tampered ciphertext accepted!\n"); fails++; } }
    printf("  %s\n", (fails > base) ? "FAILED" : "OK (forgeries rejected)");
}

/* ---- [4] Key-cache layout + data path ---- */
static uint32_t le32(const uint8_t *p){return p[0]|(p[1]<<8)|(p[2]<<16)|((uint32_t)p[3]<<24);}
static uint32_t le16(const uint8_t *p){return p[0]|(p[1]<<8);}

/* ath9k key split; TKIP TK sits in the main slot like any other cipher. */
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

/* Mirror of vwifi_ath9k_keycache_tkip(): TK reconstruction gated on TYPE_TKIP. */
static bool kc_load_tkip(const uint32_t *regs, unsigned slot, uint8_t tk[16])
{
    uint32_t b = 0x8800 + slot * 32, k0,k1,k2,k3,k4;
    if ((regs[(b + 20) / 4] & 7) != 4) return false;   /* AR_KEYTABLE_TYPE_TKIP */
    k0 = regs[(b+0)/4]; k1 = regs[(b+4)/4]&0xffff; k2 = regs[(b+8)/4];
    k3 = regs[(b+12)/4]&0xffff; k4 = regs[(b+16)/4];
    tk[0]=k0;tk[1]=k0>>8;tk[2]=k0>>16;tk[3]=k0>>24;
    tk[4]=k1;tk[5]=k1>>8;
    tk[6]=k2;tk[7]=k2>>8;tk[8]=k2>>16;tk[9]=k2>>24;
    tk[10]=k3;tk[11]=k3>>8;
    tk[12]=k4;tk[13]=k4>>8;tk[14]=k4>>16;tk[15]=k4>>24;
    return true;
}

static void test_keycache(void)
{
    int base = fails, i;
    static uint32_t regs[16384];
    uint8_t tk[16], got[16];
    printf("[4] Key-cache layout (TKIP TK reconstruct) + data path\n");

    for (i = 0; i < 128; i++) regs[(0x8800 + i * 32 + 20) / 4] = 7; /* CLR */
    for (i = 0; i < 16; i++) tk[i] = 0xD0 + i;
    kc_store(regs, 3, tk, 4);   /* TYPE_TKIP */

    if (!kc_load_tkip(regs, 3, got) || memcmp(got, tk, 16))
        { printf("  TK reconstruct FAILED\n"); fails++; }
    if (kc_load_tkip(regs, 9, got))
        { printf("  empty slot returned a key!\n"); fails++; }

    /* Full path: TX encrypts from slot 3; RX brute-forces TKIP slots + ICV. */
    {
        uint8_t txk[16], f[256], orig[256];
        int flen = build_tkip_frame(f, 120, 0, 1), flen_enc = flen;
        int slot, rx_slot = -1;
        memcpy(orig, f, flen);
        kc_load_tkip(regs, 3, txk);
        if (vwifi_tkip_encrypt(txk, f, &flen_enc, sizeof(f)) != 0)
            { printf("  tx enc failed\n"); fails++; }
        for (slot = 0; slot < 16; slot++) {
            uint8_t k[16], dec[256];
            if (!kc_load_tkip(regs, slot, k)) continue;
            memcpy(dec, f, flen_enc);
            if (vwifi_tkip_decrypt(k, dec, flen_enc) == 0) {
                rx_slot = slot;
                if (memcmp(dec, orig, flen)) { printf("  rx mismatch\n"); fails++; }
                break;
            }
        }
        if (rx_slot != 3) { printf("  rx picked slot %d, expected 3\n", rx_slot); fails++; }
    }
    printf("  %s\n", (fails > base) ? "FAILED"
           : "OK (TK inverts split; TX->RX recovers frame & slot)");
}

int main(void)
{
    test_mixing_vectors();
    test_roundtrip();
    test_icv_rejection();
    test_keycache();
    printf("\n%s\n", fails ? "*** TESTS FAILED ***" : "*** ALL TESTS PASSED ***");
    return fails ? 1 : 0;
}
