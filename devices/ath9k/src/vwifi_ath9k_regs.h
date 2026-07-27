/*
 * Virtual Atheros AR9285 – Register Definitions
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Values from Linux kernel drivers/net/wireless/ath/ath9k/reg.h,
 * hw.h, mac.h and open-ath9k-htc firmware ar5416reg.h.
 *
 * Phase 1 + Phase 2 (TX/RX DMA, descriptor rings, interrupts, PHY).
 */

#ifndef ATH9K_REGS_H
#define ATH9K_REGS_H

/* ================================================================
 *  PCI identification
 * ================================================================ */
#define ATHEROS_VENDOR_ID           0x168C
#define AR9285_DEVID_PCIE           0x002B

/* ================================================================
 *  AR_SREV (0x4020) – Silicon Revision
 *
 *  The "new format" (val & 0xFF == 0xFF) field layout per kernel reg.h:
 *    Bits 31..18  AR_SREV_VERSION2   (macVersion is extracted >> TYPE2_S)
 *    Bits 17..12  AR_SREV_TYPE2      (used as shift base for macVersion)
 *    Bits 11..8   AR_SREV_REVISION2  (macRev)
 *    Bits  7..0   must be 0xFF to select new format
 *
 *  The kernel computes:
 *    macVersion = (srev & AR_SREV_VERSION2) >> AR_SREV_TYPE2_S
 *    macRev     = (srev & AR_SREV_REVISION2) >> AR_SREV_REVISION2_S
 * ================================================================ */
#define AR_SREV                     0x4020
#define AR_SREV_ID                  0x000000FF
#define AR_SREV_VERSION2            0xFFFC0000
#define AR_SREV_VERSION2_S          18
#define AR_SREV_TYPE2               0x0003F000
#define AR_SREV_TYPE2_S             12
#define AR_SREV_TYPE2_HOST_MODE     0x00002000
#define AR_SREV_REVISION2           0x00000F00
#define AR_SREV_REVISION2_S         8
#define AR_SREV_VERSION_9285        0x0C0

/*
 * AR9285 v1.2 SREV value:
 *   macVersion = 0x0C0  placed at bits shifted by TYPE2_S (12)
 *   macRev     = 0x2    placed at bits shifted by REVISION2_S (8)
 *   low byte   = 0xFF   to select "new format" path
 *   Result: 0x000C02FF
 */
#define ATH9K_SREV_AR9285_V12   \
    ((AR_SREV_VERSION_9285 << AR_SREV_TYPE2_S) | \
     (0x2 << AR_SREV_REVISION2_S) | 0xFF)

/* ================================================================
 *  MAC / DMA core registers
 * ================================================================ */
#define AR_CR                       0x0008
#define AR_CR_RXE                   0x00000004
#define AR_CR_RXD                   0x00000020
#define AR_CR_SWI                   0x00000040

#define AR_RXDP                     0x000C

#define AR_CFG                      0x0014
#define AR_CFG_SWTD                 0x00000001
#define AR_CFG_SWTB                 0x00000002
#define AR_CFG_SWRD                 0x00000004
#define AR_CFG_SWRB                 0x00000008
#define AR_CFG_SWRG                 0x00000010
#define AR_CFG_LED                  0x00000060
#define AR_CFG_SCLK_32KHZ          0x00000080
#define AR_CFG_HALT_REQ             0x00000800
#define AR_CFG_HALT_ACK             0x00001000

#define AR_RXCFG                    0x0018
#define AR_RXCFG_DMASZ_MASK         0x00000007

#define AR_IER                      0x0024
#define AR_IER_ENABLE               0x00000001
#define AR_IER_DISABLE              0x00000000

#define AR_TXCFG                    0x0030
#define AR_TXCFG_DMASZ_MASK         0x00000003
#define AR_FTRIG_M                  0x000003F0
#define AR_FTRIG_S                  4

#define AR_MIBC                     0x0040
#define AR_MIBC_COW                 0x00000001
#define AR_MIBC_FMC                 0x00000002
#define AR_MIBC_CMC                 0x00000004
#define AR_MIBC_MCS                 0x00000008

/* ================================================================
 *  Interrupt registers
 * ================================================================ */
#define AR_ISR                      0x0080
#define AR_ISR_RXOK                 0x00000001
#define AR_ISR_RXDESC               0x00000002
#define AR_ISR_RXERR                0x00000004
#define AR_ISR_RXNOPKT              0x00000008
#define AR_ISR_RXEOL                0x00000010
#define AR_ISR_RXORN                0x00000020
#define AR_ISR_TXOK                 0x00000040
#define AR_ISR_TXDESC               0x00000080
#define AR_ISR_TXERR                0x00000100
#define AR_ISR_TXNOPKT              0x00000200
#define AR_ISR_TXEOL                0x00000400
#define AR_ISR_TXURN                0x00000800
#define AR_ISR_MIB                  0x00001000
#define AR_ISR_SWI                  0x00002000
#define AR_ISR_RXPHY                0x00004000
#define AR_ISR_RXKCM                0x00008000
#define AR_ISR_SWBA                 0x00010000
#define AR_ISR_BMISS                0x00040000
#define AR_ISR_BCNMISC              0x00800000
#define AR_ISR_GENTMR               0x10000000
#define AR_ISR_TXINTM               0x40000000
#define AR_ISR_RXINTM               0x80000000

#define AR_ISR_S0                   0x0084
#define AR_ISR_S0_QCU_TXOK          0x000003FF
#define AR_ISR_S0_QCU_TXDESC        0x03FF0000

#define AR_ISR_S1                   0x0088
#define AR_ISR_S1_QCU_TXERR         0x000003FF
#define AR_ISR_S1_QCU_TXEOL         0x03FF0000

#define AR_ISR_S2                   0x008C
#define AR_ISR_S3                   0x0090
#define AR_ISR_S4                   0x0094
#define AR_ISR_S5                   0x0098

#define AR_IMR                      0x00A0
#define AR_IMR_RXOK                 0x00000001
#define AR_IMR_RXDESC               0x00000002
#define AR_IMR_RXERR                0x00000004
#define AR_IMR_RXEOL                0x00000010
#define AR_IMR_RXORN                0x00000020
#define AR_IMR_TXOK                 0x00000040
#define AR_IMR_TXDESC               0x00000080
#define AR_IMR_TXERR                0x00000100
#define AR_IMR_TXEOL                0x00000400
#define AR_IMR_TXURN                0x00000800
#define AR_IMR_MIB                  0x00001000
#define AR_IMR_SWI                  0x00002000
#define AR_IMR_RXPHY                0x00004000
#define AR_IMR_SWBA                 0x00010000
#define AR_IMR_BMISS                0x00040000
#define AR_IMR_BCNMISC              0x00800000
#define AR_IMR_GENTMR               0x10000000
#define AR_IMR_TXINTM               0x40000000
#define AR_IMR_RXINTM               0x80000000

#define AR_IMR_S0                   0x00A4
#define AR_IMR_S1                   0x00A8
#define AR_IMR_S2                   0x00AC
#define AR_IMR_S3                   0x00B0
#define AR_IMR_S4                   0x00B4
#define AR_IMR_S5                   0x00B8

#define AR_RXBUF_READ               0x00E8

/* Sync interrupt registers */
#define AR_INTR_SYNC_CAUSE          0x4028
#define AR_INTR_SYNC_ENABLE         0x402C
#define AR_INTR_ASYNC_MASK          0x4030
#define AR_INTR_ASYNC_CAUSE         0x4038
#define AR_INTR_ASYNC_ENABLE        0x403C
#define AR_INTR_MAC_IRQ             0x00000002

/* ================================================================
 *  TX Queue registers (10 QCUs)
 * ================================================================ */
#define AR_NUM_QCU                  10

#define AR_Q0_TXDP                  0x0800
#define AR_QTXDP(_q)                (AR_Q0_TXDP + ((_q) << 2))

#define AR_Q_TXE                    0x0840
#define AR_Q_TXE_M                  0x000003FF
#define AR_Q_TXD                    0x0880
#define AR_Q_TXD_M                  0x000003FF

#define AR_Q0_CBRCFG                0x08C0
#define AR_QCBRCFG(_q)              (AR_Q0_CBRCFG + ((_q) << 2))

#define AR_Q0_RDYTIMECFG            0x0900
#define AR_QRDYTIMECFG(_q)          (AR_Q0_RDYTIMECFG + ((_q) << 2))
#define AR_Q_RDYTIMECFG_DURATION    0x00FFFFFF
#define AR_Q_RDYTIMECFG_EN          0x01000000

#define AR_Q0_ONESHOTARM_SC         0x0940
#define AR_Q0_ONESHOTARM_CC         0x0980

#define AR_Q0_MISC                  0x09C0
#define AR_QMISC(_q)                (AR_Q0_MISC + ((_q) << 2))
#define AR_Q_MISC_FSP               0x0000000F
#define AR_Q_MISC_BEACON_USE        0x00000080

#define AR_Q0_STS                   0x0A00
#define AR_QSTS(_q)                 (AR_Q0_STS + ((_q) << 2))
#define AR_Q_STS_PEND_FR_CNT        0x00000003

#define AR_Q_STATUS_RING_START      0x0830
#define AR_Q_STATUS_RING_END        0x0834

/* ================================================================
 *  DCU (DMA Control Unit) registers
 * ================================================================ */
#define AR_NUM_DCU                  10

#define AR_D0_QCUMASK               0x1000
#define AR_DQCUMASK(_q)             (AR_D0_QCUMASK + ((_q) << 2))

#define AR_D0_LCL_IFS               0x1040
#define AR_DLCL_IFS(_q)             (AR_D0_LCL_IFS + ((_q) << 2))

#define AR_D0_RETRY_LIMIT           0x1080
#define AR_DRETRY_LIMIT(_q)         (AR_D0_RETRY_LIMIT + ((_q) << 2))

#define AR_D0_CHNTIME               0x10C0
#define AR_DCHNTIME(_q)             (AR_D0_CHNTIME + ((_q) << 2))

#define AR_D0_MISC                  0x1100
#define AR_DMISC(_q)                (AR_D0_MISC + ((_q) << 2))

#define AR_D0_SEQNUM                0x1140
#define AR_DSEQNUM(_q)              (AR_D0_SEQNUM + ((_q) << 2))

#define AR_D_GBL_IFS_SIFS           0x1030
#define AR_D_GBL_IFS_SLOT           0x1070
#define AR_D_GBL_IFS_EIFS           0x10B0
#define AR_D_GBL_IFS_MISC           0x10F0

#define AR_D_FPCTL                  0x1230
#define AR_D_TXBLK_BASE             0x1038

/* ================================================================
 *  Power management / RTC
 * ================================================================ */
#define AR_PM_STATE                 0x4008
#define AR_HOST_TIMEOUT             0x4018

#define AR_EEPROM                   0x401C
#define AR_EEPROM_ABSENT            0x00000100

#define AR_RTC_RC                   0x7000
#define AR_RTC_RC_MAC_WARM          0x00000001
#define AR_RTC_RC_MAC_COLD          0x00000002

#define AR_RTC_RESET                0x7040
#define AR_RTC_RESET_EN             0x00000001

#define AR_RTC_STATUS               0x7044
#define AR_RTC_STATUS_M             0x0000000F
#define AR_RTC_STATUS_ON            0x00000002
#define AR_RTC_STATUS_SHUTDOWN      0x00000001

#define AR_RTC_FORCE_WAKE           0x704C
#define AR_RTC_FORCE_WAKE_EN        0x00000001
#define AR_RTC_FORCE_WAKE_ON_INT    0x00000002

#define AR_RTC_PLL_CONTROL          0x7014
#define AR_RTC_PLL_CONTROL2         0x703C

/* ================================================================
 *  GPIO / EEPROM status / PCIe
 * ================================================================ */
#define AR_GPIO_IN_OUT              0x4048
#define AR_GPIO_OE_OUT              0x404C
#define AR_GPIO_INPUT_EN_VAL        0x4054
#define AR_EEPROM_STATUS_DATA       0x407C
#define AR_OBS                      0x4080
#define AR_PCIE_PM_CTRL             0x4014
#define AR_WA                       0x4004
#define AR9285_WA_DEFAULT           0x004a050b

/* DMA debug registers */
#define AR_DMADBG_0                 0x4200
#define AR_DMADBG_1                 0x4204
#define AR_DMADBG_2                 0x4208
#define AR_DMADBG_3                 0x420C
#define AR_DMADBG_4                 0x4210
#define AR_DMADBG_5                 0x4214
#define AR_DMADBG_6                 0x4218
#define AR_DMADBG_7                 0x421C

/* ================================================================
 *  Station / MAC / Beacon
 * ================================================================ */
#define AR_STA_ID0                  0x8000
#define AR_STA_ID1                  0x8004
#define AR_STA_ID1_SADH_MASK        0x0000FFFF

#define AR_BSS_ID0                  0x8008
#define AR_BSS_ID1                  0x800C
#define AR_BSS_ID1_AID              0xFFFF0000
#define AR_BSS_ID1_AID_S            16

#define AR_TIME_OUT                 0x8014
#define AR_RSSI_THR                 0x8018
#define AR_USEC                     0x801C
#define AR_BEACON_PERIOD            0x8020
#define AR_DBA_PERIOD               0x8024

#define AR_RX_FILTER                0x803C
#define AR_RX_FILTER_UCAST          0x00000001
#define AR_RX_FILTER_MCAST          0x00000002
#define AR_RX_FILTER_BCAST          0x00000004
#define AR_RX_FILTER_CONTROL        0x00000008
#define AR_RX_FILTER_BEACON         0x00000010
#define AR_RX_FILTER_PROM           0x00000020
#define AR_RX_FILTER_PROBEREQ       0x00000080

#define AR_DIAG_SW                  0x8048
#define AR_BCN_RSSI_AVE             0x8048  /* Note: shares addr in some docs */

#define AR_NEXT_TBTT_TIMER          0x8200
#define AR_NEXT_DMA_BEACON_ALERT    0x8204
#define AR_NEXT_SWBA                0x8208
#define AR_NEXT_HCF                 0x820C
#define AR_NEXT_TIM                 0x8210
#define AR_NEXT_DTIM                0x8214

#define AR_TIMER_MODE               0x8240
#define AR_TBTT_TIMER_EN            0x00000001
#define AR_DBA_TIMER_EN             0x00000002
#define AR_SWBA_TIMER_EN            0x00000004

#define AR_PCU_MISC_MODE2           0x8344
#define AR_PHY_ERR                  0x812C

/* ================================================================
 *  PHY registers (minimal for Phase 2)
 * ================================================================ */
#define AR_PHY_BASE                 0x9800
#define AR_PHY(_n)                  (AR_PHY_BASE + ((_n) << 2))
#define AR_PHY_ACTIVE               0x981C
#define AR_PHY_ACTIVE_EN            0x00000001
#define AR_PHY_ACTIVE_DIS           0x00000000

#define AR_PHY_MODE                 0x9808
#define AR_PHY_MODE_OFDM            0x00000000
#define AR_PHY_MODE_CCK             0x00000001
#define AR_PHY_MODE_DYNAMIC         0x00000004

#define AR_PHY_AGC_CONTROL          0x9860
#define AR_PHY_AGC_CONTROL_CAL      0x00000001
#define AR_PHY_AGC_CONTROL_NF       0x00000002

#define AR_PHY_CCA                  0x9864

#define AR_PHY_RFBUS_REQ            0x997C
#define AR_PHY_RFBUS_REQ_EN         0x00000001
#define AR_PHY_RFBUS_GRANT          0x9C20
#define AR_PHY_RFBUS_GRANT_EN       0x00000001

/* ================================================================
 *  Hardware key cache (WEP/TKIP/CCMP offload)
 *
 *  Values from Linux kernel drivers/net/wireless/ath/ath9k/reg.h.
 *  Each entry is 8 registers (32 bytes) wide.  The driver programs
 *  keys here via ath_hw_set_keycache_entry(); on real silicon the MAC
 *  encrypts/decrypts frames using the matching entry.  We emulate the
 *  cache and perform CCMP in software (see vwifi_ath9k_crypto.h).
 * ================================================================ */
#define AR_KEY_CACHE_SIZE           128
#define AR_KEYTABLE_0               0x8800
#define AR_KEYTABLE(_n)             (AR_KEYTABLE_0 + ((_n) * 32))
#define AR_KEYTABLE_KEY0(_n)        (AR_KEYTABLE(_n) + 0)
#define AR_KEYTABLE_KEY1(_n)        (AR_KEYTABLE(_n) + 4)
#define AR_KEYTABLE_KEY2(_n)        (AR_KEYTABLE(_n) + 8)
#define AR_KEYTABLE_KEY3(_n)        (AR_KEYTABLE(_n) + 12)
#define AR_KEYTABLE_KEY4(_n)        (AR_KEYTABLE(_n) + 16)
#define AR_KEYTABLE_TYPE(_n)        (AR_KEYTABLE(_n) + 20)
#define AR_KEYTABLE_MAC0(_n)        (AR_KEYTABLE(_n) + 24)
#define AR_KEYTABLE_MAC1(_n)        (AR_KEYTABLE(_n) + 28)

/* First and last byte offsets of the key cache in the MMIO window */
#define AR_KEYTABLE_END             (AR_KEYTABLE_0 + AR_KEY_CACHE_SIZE * 32)

/* AR_KEYTABLE_TYPE field (low 3 bits) */
#define AR_KEYTABLE_TYPE_MASK       0x00000007
#define AR_KEYTABLE_TYPE_40         0x00000000  /* WEP-40  */
#define AR_KEYTABLE_TYPE_104        0x00000001  /* WEP-104 */
#define AR_KEYTABLE_TYPE_128        0x00000003  /* WEP-128 */
#define AR_KEYTABLE_TYPE_TKIP       0x00000004
#define AR_KEYTABLE_TYPE_AES        0x00000005  /* AES-OCB (unused) */
#define AR_KEYTABLE_TYPE_CCM        0x00000006  /* AES-CCM (WPA2/CCMP) */
#define AR_KEYTABLE_TYPE_CLR        0x00000007  /* entry disabled */

/* AR_KEYTABLE_MAC1: valid (unicast) flag + high MAC bits */
#define AR_KEYTABLE_VALID           0x00008000

/* ================================================================
 *  Misc constants
 * ================================================================ */
#define AR5416_MAGIC                0x19641014
#define ATH_DEFAULT_NOISE_FLOOR     (-95)
#define AR9285_NUM_GPIO             12
#define AR9285_GPIO_MASK            0x00000FFF
#define AR9285_RDEXT_DEFAULT        0x1F

#endif /* ATH9K_REGS_H */
