/*
 * Standalone verification of vwifi_ath9k_ampdu.h — the A-MPDU / Block-Ack
 * sequence-number and bitmap arithmetic the virtual AR9285 uses to report
 * an immediate BlockAck after an 802.11n aggregate transmission.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * This is the one piece of the aggregation path that is pure arithmetic and
 * so can be checked on the host, independent of QEMU and a guest.  The
 * device writes the results straight into the TX status descriptor that the
 * unmodified ath9k driver reads via ath_tx_complete_aggr(); a wrong bitmap
 * makes the driver retransmit a whole aggregate, so getting the window-start
 * and wrap semantics exactly right (matching the kernel's ATH_BA_INDEX)
 * matters.
 *
 *   [1] Consecutive subframes  -> bits 0..N-1 set, seq_st = first seqno.
 *   [2] Sequence-number wrap across 4095->0 stays consecutive in the bitmap.
 *   [3] A dropped/missing subframe leaves exactly its bit clear.
 *   [4] Subframes beyond the 64-frame BA window fall outside the bitmap.
 *   [5] AR_SeqNum field packing round-trips the window start.
 *   [6] Sequence-number extraction from a real QoS-Data header.
 *
 * Build & run (from the project root):
 *   make test-ampdu
 * or directly:
 *   cc -O2 -Isrc -o tests/test_ampdu tests/test_ampdu.c && ./tests/test_ampdu
 */
#include <stdio.h>
#include <stdint.h>
#include <string.h>

#include "vwifi_ath9k_ampdu.h"

/* Mirror the device's AR_SeqNum packing (dma.h) so the test is self-contained. */
#define AR_SeqNum      0x00001FFE
#define AR_SeqNum_S    1

static int fails = 0;

static void check(const char *name, int cond)
{
    printf("  [%s] %s\n", cond ? "PASS" : "FAIL", name);
    if (!cond) {
        fails++;
    }
}

/* Reference: the kernel's ATH_BA_ISSET on a split low/high bitmap. */
static int ba_isset(uint32_t lo, uint32_t hi, int n)
{
    if (n < 32) {
        return (lo >> n) & 1u;
    }
    if (n < 64) {
        return (hi >> (n - 32)) & 1u;
    }
    return 0;
}

static void test_consecutive(void)
{
    uint16_t seq[8];
    uint16_t seq_st;
    uint32_t lo, hi;
    int i, ok;

    for (i = 0; i < 8; i++) {
        seq[i] = (uint16_t)(100 + i);
    }
    vwifi_ath9k_ba_bitmap(seq, 8, &seq_st, &lo, &hi);

    ok = (seq_st == 100) && (hi == 0) && (lo == 0xFFu);
    check("consecutive: seq_st=100, ba_low=0xFF, ba_high=0", ok);
}

static void test_wrap(void)
{
    /* 4094, 4095, 0, 1 — wraps the 12-bit sequence space. */
    uint16_t seq[4] = { 4094, 4095, 0, 1 };
    uint16_t seq_st;
    uint32_t lo, hi;
    int ok;

    vwifi_ath9k_ba_bitmap(seq, 4, &seq_st, &lo, &hi);
    /* Relative to base 4094 the indices are 0,1,2,3. */
    ok = (seq_st == 4094) && (hi == 0) && (lo == 0x0Fu);
    check("wrap 4094->1: seq_st=4094, ba_low=0x0F", ok);
}

static void test_hole(void)
{
    /* Subframe seqno 102 is missing (lost on the medium). */
    uint16_t seq[7] = { 100, 101, 103, 104, 105, 106, 107 };
    uint16_t seq_st;
    uint32_t lo, hi;
    int ok;

    vwifi_ath9k_ba_bitmap(seq, 7, &seq_st, &lo, &hi);
    /* Bits 0,1,3,4,5,6,7 set; bit 2 (seqno 102) clear. */
    ok = (seq_st == 100) &&
         ba_isset(lo, hi, 0) && ba_isset(lo, hi, 1) &&
         !ba_isset(lo, hi, 2) &&
         ba_isset(lo, hi, 3) && ba_isset(lo, hi, 7) &&
         (lo == ((1u << 8) - 1 - (1u << 2)));
    check("hole at seqno 102 leaves exactly bit 2 clear", ok);
}

static void test_full_window(void)
{
    uint16_t seq[64];
    uint16_t seq_st;
    uint32_t lo, hi;
    int i, ok;

    for (i = 0; i < 64; i++) {
        seq[i] = (uint16_t)((2000 + i) & 0x0FFF);
    }
    vwifi_ath9k_ba_bitmap(seq, 64, &seq_st, &lo, &hi);
    ok = (seq_st == 2000) && (lo == 0xFFFFFFFFu) && (hi == 0xFFFFFFFFu);
    check("full 64-frame window: all bits set", ok);
}

static void test_beyond_window(void)
{
    /* Base 500, plus one subframe 70 past it: index 70 >= 64 -> dropped. */
    uint16_t seq[2] = { 500, 570 };
    uint16_t seq_st;
    uint32_t lo, hi;
    int ok;

    vwifi_ath9k_ba_bitmap(seq, 2, &seq_st, &lo, &hi);
    ok = (seq_st == 500) && (lo == 0x1u) && (hi == 0);
    check("subframe past 64-frame window is left un-ACKed", ok);
}

static void test_seqnum_pack(void)
{
    uint16_t starts[4] = { 0, 1, 2047, 4095 };
    int i, ok = 1;

    for (i = 0; i < 4; i++) {
        uint32_t packed = ((uint32_t)starts[i] << AR_SeqNum_S) & AR_SeqNum;
        uint16_t back = (uint16_t)((packed & AR_SeqNum) >> AR_SeqNum_S);
        if (back != starts[i]) {
            ok = 0;
        }
    }
    check("AR_SeqNum packing round-trips window start (0..4095)", ok);
}

static void test_frame_seqno(void)
{
    /* A minimal QoS-Data header: FC(2) Dur(2) A1(6) A2(6) A3(6) SC(2) QoS(2).
     * Sequence Control at offset 22; seqno 0x123 -> raw 0x1230 (frag=0). */
    uint8_t frame[28];
    uint16_t sn;

    memset(frame, 0, sizeof(frame));
    frame[0] = 0x88;           /* QoS Data */
    frame[22] = 0x30;          /* SC low  = (0x123 << 4) & 0xFF */
    frame[23] = 0x12;          /* SC high = (0x123 << 4) >> 8   */
    sn = vwifi_ath9k_frame_seqno(frame, sizeof(frame));
    check("frame seqno extraction: 0x123", sn == 0x123);

    check("runt frame -> seqno 0", vwifi_ath9k_frame_seqno(frame, 10) == 0);
}

int main(void)
{
    printf("== vwifi-ath9k A-MPDU / Block-Ack unit tests ==\n");
    test_consecutive();
    test_wrap();
    test_hole();
    test_full_window();
    test_beyond_window();
    test_seqnum_pack();
    test_frame_seqno();

    if (fails) {
        printf("\n%d test(s) FAILED\n", fails);
        return 1;
    }
    printf("\nAll A-MPDU tests passed\n");
    return 0;
}
