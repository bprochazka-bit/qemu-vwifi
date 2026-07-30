/*
 * Virtual Atheros AR9285 – DMA Descriptor Definitions
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Descriptor format from Linux kernel drivers/net/wireless/ath/ath9k/mac.h.
 * The ar5416_desc is the hardware descriptor format used by AR5416 and
 * all successors including AR9285.
 *
 * Phase 2: TX/RX descriptor ring handling + interrupt delivery.
 */

#ifndef ATH9K_DMA_H
#define ATH9K_DMA_H

#include <stdint.h>

/* ================================================================
 *  ar5416_desc – Hardware DMA descriptor
 *
 *  Both TX and RX use this same structure.  The union differentiates
 *  the control/status words:
 *    TX: 10 control words (ctl2..ctl11) + 10 status words
 *    RX: 9 status words
 *
 *  Total size: 4 + 4 + 4 + 4 + max(10+10, 9)*4 = 4*24 = 96 bytes (TX)
 *  RX descriptors only use first 4 + 9*4 = 52 bytes of the union.
 *
 *  Layout in guest physical memory:
 *    [0x00] ds_link    – physical address of next descriptor (0 = end)
 *    [0x04] ds_data    – physical address of data buffer
 *    [0x08] ds_ctl0    – first control word
 *    [0x0C] ds_ctl1    – second control word (includes buffer length)
 *    [0x10] union      – TX ctl2..ctl11 + status, or RX status
 * ================================================================ */

/*
 * Total descriptor size in bytes.  The driver allocates this much
 * for each descriptor in its DMA ring.
 */
#define ATH9K_DESC_TX_SIZE          96  /* 4*4 header + 20*4 tx union */
#define ATH9K_DESC_RX_SIZE          52  /* 4*4 header + 9*4 rx union */

/* Descriptor header offsets (bytes from start of descriptor) */
#define DESC_OFF_LINK               0x00
#define DESC_OFF_DATA               0x04
#define DESC_OFF_CTL0               0x08
#define DESC_OFF_CTL1               0x0C

/* TX control word offsets (bytes from start of descriptor) */
#define DESC_OFF_TX_CTL2            0x10
#define DESC_OFF_TX_CTL3            0x14
#define DESC_OFF_TX_CTL4            0x18
#define DESC_OFF_TX_CTL5            0x1C
#define DESC_OFF_TX_CTL6            0x20
#define DESC_OFF_TX_CTL7            0x24
#define DESC_OFF_TX_CTL8            0x28
#define DESC_OFF_TX_CTL9            0x2C
#define DESC_OFF_TX_CTL10           0x30
#define DESC_OFF_TX_CTL11           0x34
/* TX status words start at 0x38 */
#define DESC_OFF_TX_STATUS0         0x38
#define DESC_OFF_TX_STATUS1         0x3C
#define DESC_OFF_TX_STATUS3         0x44  /* AR_BaBitmapLow  (BA ACK bits 0..31)  */
#define DESC_OFF_TX_STATUS4         0x48  /* AR_BaBitmapHigh (BA ACK bits 32..63) */
#define DESC_OFF_TX_STATUS5         0x4C
#define DESC_OFF_TX_STATUS8         0x58
#define DESC_OFF_TX_STATUS9         0x5C  /* contains AR_TxDone + AR_SeqNum */

/* RX status word offsets (bytes from start of descriptor) */
#define DESC_OFF_RX_STATUS0         0x10
#define DESC_OFF_RX_STATUS1         0x14
#define DESC_OFF_RX_STATUS2         0x18  /* AR_RcvTimestamp */
#define DESC_OFF_RX_STATUS3         0x1C
#define DESC_OFF_RX_STATUS4         0x20  /* AR_RxEVM0 */
#define DESC_OFF_RX_STATUS5         0x24
#define DESC_OFF_RX_STATUS6         0x28
#define DESC_OFF_RX_STATUS7         0x2C
#define DESC_OFF_RX_STATUS8         0x30  /* contains AR_RxDone */

/* ================================================================
 *  TX control word bit definitions
 * ================================================================ */

/* ds_ctl0 */
#define AR_FrameLen                 0x00000FFF
#define AR_VirtMoreFrag             0x00001000
#define AR_TxCtlRsvd00              0x0001E000
#define AR_XmitPower                0x003F0000
#define AR_XmitPower_S              16
#define AR_RTSEnable                0x00400000
#define AR_VEOL                     0x00800000
#define AR_ClrDestMask              0x01000000
#define AR_TxCtlRsvd01              0x1E000000
#define AR_TxIntrReq                0x20000000
#define AR_DestIdxValid             0x40000000
#define AR_CTSEnable                0x80000000

/* ds_ctl1 */
#define AR_BufLen                   0x00000FFF
#define AR_TxMore                   0x00001000
#define AR_DestIdx                  0x000FE000
#define AR_DestIdx_S                13
#define AR_FrameType                0x00F00000
#define AR_FrameType_S              20
#define AR_NoAck                    0x01000000
#define AR_InsertTS                 0x02000000
#define AR_CorruptFCS               0x04000000
#define AR_ExtOnly                  0x08000000
#define AR_ExtAndCtl                0x10000000
#define AR_MoreAggr                 0x20000000
#define AR_IsAggr                   0x40000000

/* ds_ctl6 – encryption type for hardware crypto offload.
 * The driver writes SM(keytype, AR_EncrType); the value is the
 * ath9k_key_type enum below. */
#define AR_EncrType                 0x0c000000
#define AR_EncrType_S               26

/* ath9k_key_type values placed in the AR_EncrType field */
#define AR_ENCR_TYPE_CLEAR          0  /* no encryption */
#define AR_ENCR_TYPE_WEP            1
#define AR_ENCR_TYPE_AES            2  /* AES-CCM (WPA2/CCMP) */
#define AR_ENCR_TYPE_TKIP           3

/* ================================================================
 *  TX status word bit definitions (ds_txstatus9, written at 0x5C)
 *
 *  AR_SeqNum here doubles as the Block-Ack window start for an A-MPDU's
 *  final descriptor (see AR_TxBaStatus / DESC_OFF_TX_STATUS3-4).
 * ================================================================ */

#define AR_TxDone                   0x00000001
#define AR_SeqNum                   0x00001FFE
#define AR_SeqNum_S                 1
#define AR_TxOpExceeded             0x00020000
#define AR_FinalTxIdx               0x00600000
#define AR_FinalTxIdx_S             21
#define AR_PowerMgmt                0x02000000

/* TX status1 bit definitions (ds_txstatus1) */
#define AR_FrmXmitOK                0x00000001
#define AR_ExcessiveRetries         0x00000002
#define AR_FIFOUnderrun             0x00000004
#define AR_Filtered                 0x00000008
#define AR_RTSFailCnt               0x000000F0
#define AR_RTSFailCnt_S             4
#define AR_DataFailCnt              0x00000F00
#define AR_DataFailCnt_S            8
#define AR_TxDelimUnderrun          0x00010000
#define AR_TxDataUnderrun           0x00020000
#define AR_DescCfgErr               0x00040000
#define AR_TxTimerExpired           0x00080000

/* TX status word bits for TX rate / RSSI (status0, status5) */
#define AR_TxRSSIAnt00              0x000000FF
#define AR_TxRSSICombined           0xFF000000
#define AR_TxRSSICombined_S         24

/* ds_txstatus0: set when the frame was part of an A-MPDU and the peer's
 * immediate BlockAck was received.  The ath9k aggregation-completion path
 * (ath_tx_complete_aggr) only consults the BA bitmap in ds_txstatus3/4 and
 * the window start in AR_SeqNum when this bit is set on the aggregate's
 * last descriptor; otherwise it retransmits the whole aggregate. */
#define AR_TxBaStatus               0x40000000

/* ================================================================
 *  RX control / status bit definitions
 * ================================================================ */

/* RX status0 (per-antenna RSSI for primary chains) */
#define AR_RxRSSIAnt00              0x000000FF
#define AR_RxRSSIAnt00_S            0
#define AR_RxRSSIAnt01              0x0000FF00
#define AR_RxRSSIAnt01_S            8
#define AR_RxRSSIAnt02              0x00FF0000
#define AR_RxRSSIAnt02_S            16
#define AR_RxRate                   0xFF000000
#define AR_RxRate_S                 24

/* RX status1 (data length) */
#define AR_DataLen                  0x00000FFF
#define AR_RxMore                   0x00001000
#define AR_NumDelim                 0x003FC000
#define AR_NumDelim_S               14

/* RX status2 = AR_RcvTimestamp (full 32-bit TSF low word) */
#define AR_RcvTimestamp             0xFFFFFFFF

/* RX status3 (GI, channel width, STBC, antenna) */
#define AR_GI                       0x00000001
#define AR_2040                     0x00000002

/* RX status4 = AR_RxEVM0 (EVM data, chain 0) */
/* RX status5 = AR_RxEVM1 */

/* RX status5 (ext RSSI for diversity chains) */
#define AR_RxRSSIAnt10              0x000000FF
#define AR_RxRSSIAnt10_S            0
#define AR_RxRSSIAnt11              0x0000FF00
#define AR_RxRSSIAnt11_S            8
#define AR_RxRSSIAnt12              0x00FF0000
#define AR_RxRSSIAnt12_S            16
#define AR_RxRSSICombined           0xFF000000
#define AR_RxRSSICombined_S         24

/* RX status8 (final status – AR_RxDone must be set by HW) */
#define AR_RxDone                   0x00000001
#define AR_RxFrameOK                0x00000002
#define AR_CRCErr                   0x00000004
#define AR_DecryptCRCErr            0x00000008
#define AR_PhyErr                   0x00000010
#define AR_MichaelErr               0x00000020
#define AR_PreDelimCRCErr           0x00000040
#define AR_RxKeyIdxValid            0x00000100
#define AR_KeyIdx                   0x0000FE00
#define AR_KeyIdx_S                 9
#define AR_RxMoreAggr               0x00010000
#define AR_RxAggr                   0x00020000
#define AR_PostDelimCRCErr          0x00040000
#define AR_RxStatusRsvd71           0x01F80000
#define AR_DecryptBusyErr           0x40000000
#define AR_KeyMiss                  0x80000000

/* RX descriptor control word (ds_ctl1 for RX) */
#define AR_RxCtlRsvd00              0x00001000
#define AR_RxIntrReq                0x00002000

/* ================================================================
 *  Virtual device DMA engine constants
 * ================================================================ */

/* Maximum number of TX queues we actively support.
 * ath9k uses queues 0-3 for AC_BE, AC_BK, AC_VI, AC_VO,
 * queue 8 for beacons, queue 9 for CAB.  We support all 10. */
#define ATH9K_VIRT_NUM_TX_QUEUES    10

/* Maximum number of descriptors we'll walk per DMA operation.
 * Safety limit to prevent infinite loops on circular lists. */
#define ATH9K_VIRT_MAX_DESC_WALK    4096

/* Size of the RX buffer the driver typically allocates (minus the
 * descriptor).  We don't need to fill it entirely for beacons. */
#define ATH9K_VIRT_RX_BUF_SIZE      4096

/* OFDM rate codes (from ieee80211) used in RX status */
#define ATH9K_RATE_6M               0x0B
#define ATH9K_RATE_9M               0x0F
#define ATH9K_RATE_12M              0x0A
#define ATH9K_RATE_18M              0x0E
#define ATH9K_RATE_24M              0x09
#define ATH9K_RATE_36M              0x0D
#define ATH9K_RATE_48M              0x08
#define ATH9K_RATE_54M              0x0C

/* CCK rate codes */
#define ATH9K_RATE_1M               0x1B
#define ATH9K_RATE_2M               0x1A
#define ATH9K_RATE_5_5M             0x19
#define ATH9K_RATE_11M              0x18

#endif /* ATH9K_DMA_H */
