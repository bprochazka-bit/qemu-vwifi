/*
 * Virtual Atheros AR9285 – 802.11n A-MPDU / Block-Ack helpers
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Pure, dependency-free logic for emulating an immediate Block Ack after
 * an A-MPDU transmission.  Kept in its own header (no QEMU includes) so
 * the sequence-number / bitmap arithmetic can be unit-tested on the host
 * (see tests/test_ampdu.c) — the part most likely to be wrong is the one
 * we can least afford to guess at, since the device only ever runs inside
 * a full QEMU + guest lab.
 *
 * Background.  On real AR5416-family hardware an A-MPDU is a chain of TX
 * descriptors, each carrying one MPDU (subframe), linked by ds_link with
 * AR_IsAggr set on every subframe and AR_MoreAggr set on all but the
 * last.  The hardware transmits the aggregate, receives the peer's
 * immediate BlockAck, and writes the BA result onto the *last* subframe's
 * descriptor: AR_TxBaStatus in ds_txstatus0, the 64-bit ACK bitmap in
 * ds_txstatus3/ds_txstatus4, and the BA window start in AR_SeqNum
 * (ds_txstatus9).  The ath9k driver's ath_tx_complete_aggr() reads those
 * to decide, per subframe, ACKed vs retransmit.  Get the bitmap wrong and
 * the driver retransmits the whole aggregate — worse than legacy.
 *
 * The virtual medium is lossless by default (MODEL_NONE), so we report
 * every subframe ACKed.  The bitmap is still computed per-subframe from
 * real sequence numbers so that (a) a configured SNR/path-loss model that
 * drops a subframe naturally leaves its bit clear → the driver retransmits
 * exactly that MPDU, and (b) the window-start/wrap arithmetic matches the
 * driver's ATH_BA_INDEX() exactly.
 */

#ifndef VWIFI_ATH9K_AMPDU_H
#define VWIFI_ATH9K_AMPDU_H

#include <stdint.h>

/* Block-Ack bitmap is 64 bits (WME_BA_BMP_SIZE in the kernel), so an
 * aggregate carries at most 64 subframes within one BA window. */
#define VWIFI_ATH9K_BA_BMP_BITS   64
#define VWIFI_ATH9K_MAX_AGGR      VWIFI_ATH9K_BA_BMP_BITS

/* 802.11 sequence numbers are 12-bit and wrap at 4096. */
#define VWIFI_SEQ_MASK            0x0FFF
#define VWIFI_SEQ_MAX             0x1000

/*
 * ATH_BA_INDEX() from the kernel: the bitmap index of sequence number
 * _seq relative to window start _st, modulo the 12-bit sequence space.
 */
static inline uint16_t vwifi_ath9k_ba_index(uint16_t seq_st, uint16_t seq)
{
    return (uint16_t)((seq - seq_st) & VWIFI_SEQ_MASK);
}

/*
 * Build the immediate-BlockAck bitmap that ACKs every subframe whose
 * 12-bit sequence number appears in seqnos[0..n).  The BA window start
 * (seq_st) is the first subframe's sequence number — the aggregate's
 * base — exactly as the peer's BlockAck would report it.  A subframe
 * more than 63 sequence numbers past the base falls outside the 64-bit
 * window and is left un-ACKed (the driver will retransmit it), matching
 * real BA-window semantics.
 *
 *   *out_seq_st  ← BA window start (goes into AR_SeqNum on the last desc)
 *   *out_ba_low  ← ACK bitmap bits 0..31   (ds_txstatus3)
 *   *out_ba_high ← ACK bitmap bits 32..63  (ds_txstatus4)
 */
static inline void vwifi_ath9k_ba_bitmap(const uint16_t *seqnos, int n,
                                         uint16_t *out_seq_st,
                                         uint32_t *out_ba_low,
                                         uint32_t *out_ba_high)
{
    uint16_t seq_st = (n > 0) ? (uint16_t)(seqnos[0] & VWIFI_SEQ_MASK) : 0;
    uint64_t bmp = 0;
    int i;

    for (i = 0; i < n; i++) {
        uint16_t idx = vwifi_ath9k_ba_index(seq_st,
                                            (uint16_t)(seqnos[i] & VWIFI_SEQ_MASK));
        if (idx < VWIFI_ATH9K_BA_BMP_BITS) {
            bmp |= (uint64_t)1u << idx;
        }
    }

    *out_seq_st  = seq_st;
    *out_ba_low  = (uint32_t)(bmp & 0xFFFFFFFFu);
    *out_ba_high = (uint32_t)(bmp >> 32);
}

/*
 * Extract the 12-bit 802.11 sequence number from an MPDU.  The Sequence
 * Control field sits at byte offset 22 (after FC[2] + Dur[2] + A1..A3[18])
 * for every 3-/4-address frame; the low 4 bits are the fragment number.
 * The header is never encrypted, so this is valid before or after the
 * hardware-crypto transform.  Returns 0 for a runt.
 */
static inline uint16_t vwifi_ath9k_frame_seqno(const uint8_t *frame, int len)
{
    uint16_t sc;
    if (len < 24) {
        return 0;
    }
    sc = (uint16_t)(frame[22] | (frame[23] << 8));
    return (uint16_t)((sc >> 4) & VWIFI_SEQ_MASK);
}

#endif /* VWIFI_ATH9K_AMPDU_H */
