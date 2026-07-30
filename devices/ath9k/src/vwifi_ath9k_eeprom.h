/*
 * Virtual Atheros AR9285 – EEPROM Image (4 KB format)
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * The AR9285 uses the "4k" EEPROM layout (struct ar5416_eeprom_4k in
 * the kernel).  This is 512 half-words (1024 bytes).
 *
 * The driver reads the EEPROM during ath9k_hw_init() via
 * ath9k_hw_4k_fill_eeprom().  Key fields it checks:
 *
 *   - magic number at offset 0 (0xa55a)
 *   - eeprom version at the baseEepHeader (must be >= 0x0E03)
 *   - regulatory domain
 *   - opCapFlags (must have 2 GHz support for AR9285)
 *   - MAC address (6 bytes)
 *   - TX power calibration data
 *
 * We generate a minimal EEPROM image that passes the driver's
 * check_eeprom() and fill_cap_info() without triggering errors.
 * Calibration data is set to conservative defaults.
 *
 * References:
 *   - Linux drivers/net/wireless/ath/ath9k/eeprom.h
 *   - Linux drivers/net/wireless/ath/ath9k/eeprom_4k.c
 *   - atheepmgr project (github.com/rsa9000/atheepmgr)
 */

#ifndef ATH9K_EEPROM_H
#define ATH9K_EEPROM_H

#include <stdint.h>
#include <string.h>

/*
 * Total EEPROM size for the 4K format.
 * The AR9285 has a 1 KB EEPROM: 512 x 16-bit words.
 */
#define ATH9K_EEPROM_4K_SIZE_WORDS   512

/*
 * EEPROM magic number – the driver rejects the EEPROM without it.
 * This is AR5416_EEPROM_MAGIC = 0xa55a.
 */
#define AR5416_EEPROM_MAGIC          0xa55a

/*
 * Key offsets in the 4K EEPROM (in 16-bit word units).
 *
 * The 4k EEPROM layout (struct ar5416_eeprom_4k) starts at offset 0
 * when read from the EEPROM window.  The structure is:
 *
 *   Offset (words)  Field
 *   0               magic (0xa55a)
 *   -- base eep header starts around word 1 --
 *
 * Because the kernel reads via REG_READ(ah, AR_EEPROM_OFFSET + off*2)
 * where off is the word offset, we index our array by word offset.
 *
 * Rather than trying to match the exact struct layout bit-for-bit here
 * (which varies by kernel version), we build the EEPROM as a raw byte
 * array matching the binary layout the kernel expects.
 */

/*
 * The structure offsets below are from the 4k EEPROM format.
 * All multi-byte fields are LITTLE-ENDIAN in the EEPROM.
 *
 * struct base_eep_header_4k {
 *     uint16_t length;          // word 0 of header (word 1 overall after magic)
 *     uint16_t checksum;        // word 1
 *     uint16_t version;         // word 2 (high byte = major, low = minor)
 *     uint8_t  opCapFlags;      // byte 6 of header
 *     uint8_t  eepMisc;
 *     uint16_t regDmn[2];       // regulatory domain pair
 *     uint8_t  macAddr[6];      // MAC address
 *     ...more fields...
 * };
 *
 * For simplicity and correctness, we build the EEPROM as a byte array
 * then copy it into the 16-bit word array for the device model.
 */

/* Byte offsets within the 4K EEPROM image */
#define EEP4K_OFF_MAGIC         0     /* 2 bytes: 0xa55a */

/*
 * The kernel reads the ar5416_eeprom_4k struct starting at word offset 64
 * (eep_start_loc = 64 in __ath9k_hw_4k_fill_eeprom), which means byte
 * offset 128 in our word array.  The struct's first field is
 * base_eep_header_4k which begins with: length, checksum, version, ...
 *
 * All EEP4K_OFF_HDR_* offsets below are absolute byte offsets in our
 * 512-word (1024-byte) raw array.
 */
#define EEP4K_STRUCT_START      128   /* byte offset = word 64 * 2 */
#define EEP4K_OFF_HDR_START     EEP4K_STRUCT_START
#define EEP4K_OFF_HDR_LENGTH    (EEP4K_STRUCT_START + 0)   /* uint16_t */
#define EEP4K_OFF_HDR_CHECKSUM  (EEP4K_STRUCT_START + 2)   /* uint16_t */
#define EEP4K_OFF_HDR_VERSION   (EEP4K_STRUCT_START + 4)   /* uint16_t */
#define EEP4K_OFF_HDR_OPCAP     (EEP4K_STRUCT_START + 6)   /* uint8_t  */
#define EEP4K_OFF_HDR_EEPMISC   (EEP4K_STRUCT_START + 7)   /* uint8_t  */
#define EEP4K_OFF_HDR_REGDMN0   (EEP4K_STRUCT_START + 8)   /* uint16_t */
#define EEP4K_OFF_HDR_REGDMN1   (EEP4K_STRUCT_START + 10)  /* uint16_t */
#define EEP4K_OFF_HDR_MACADDR   (EEP4K_STRUCT_START + 12)   /* 6 bytes */

/*
 * opCapFlags bits:
 *   0x01 = AR5416_OPFLAGS_11A  (5 GHz support)
 *   0x02 = AR5416_OPFLAGS_11G  (2.4 GHz support)
 *   0x04 = AR5416_OPFLAGS_N_5G_HT40
 *   0x08 = AR5416_OPFLAGS_N_2G_HT40
 *   0x10 = AR5416_OPFLAGS_N_5G_HT20
 *   0x20 = AR5416_OPFLAGS_N_2G_HT20
 *
 * AR9285 is 2.4 GHz only, so we set 11G + N_2G_HT40 + N_2G_HT20
 */
#define AR5416_OPFLAGS_11A          0x01
#define AR5416_OPFLAGS_11G          0x02
#define AR5416_OPFLAGS_N_5G_HT40   0x04
#define AR5416_OPFLAGS_N_2G_HT40   0x08
#define AR5416_OPFLAGS_N_5G_HT20   0x10
#define AR5416_OPFLAGS_N_2G_HT20   0x20

/*
 * eepMisc: bit 0 = big-endian EEPROM.  We use little-endian (bit 0 = 0).
 */

/*
 * Regulatory domain: 0x0000 means "use default / world".
 * Alternatively 0x003a = US, 0x0037 = ETSI, etc.
 * Using 0x0000 + 0x001f is safe and triggers the world regulatory domain.
 */

/*
 * Compute a simple 16-bit XOR checksum over the EEPROM struct data.
 *
 * The kernel reads SIZE_EEPROM_4K words from word offset 64 (eep_start_loc),
 * then does:  for (i = 0; i < el; i++) sum ^= eepdata[i];
 * and checks sum == 0xFFFF.
 *
 * So we checksum bytes [EEP4K_STRUCT_START .. end), skipping the checksum
 * field itself, and set checksum = 0xFFFF ^ (xor of all other words).
 */
static inline uint16_t ath9k_eeprom_calc_checksum(const uint8_t *data,
                                                   size_t total_len)
{
    uint16_t sum = 0;
    size_t i;

    for (i = EEP4K_STRUCT_START; i < total_len; i += 2) {
        uint16_t w = (uint16_t)(data[i]) | ((uint16_t)(data[i + 1]) << 8);
        /* Skip the checksum field itself */
        if (i == EEP4K_OFF_HDR_CHECKSUM) {
            continue;
        }
        sum ^= w;
    }
    return sum ^ 0xFFFF;
}

/*
 * Build a minimal valid AR9285 4K EEPROM image.
 *
 * @eeprom: output array of 16-bit words, must be at least
 *          ATH9K_EEPROM_4K_SIZE_WORDS entries
 * @count:  number of entries in @eeprom (should be ATH9K_EEPROM_4K_SIZE_WORDS)
 *
 * The image is built in a byte array first (for correct field alignment),
 * then copied into the 16-bit word array that the device model serves
 * through the EEPROM MMIO window.
 */
static inline void ath9k_eeprom_init_4k(uint16_t *eeprom, size_t count)
{
    uint8_t raw[ATH9K_EEPROM_4K_SIZE_WORDS * 2];
    size_t total_bytes = sizeof(raw);
    uint16_t csum;
    size_t i;

    /* Start with all zeros */
    memset(raw, 0x00, total_bytes);

    /* ---- Magic at word 0 (byte 0-1) ---- */
    raw[EEP4K_OFF_MAGIC]     = (AR5416_EEPROM_MAGIC) & 0xFF;
    raw[EEP4K_OFF_MAGIC + 1] = (AR5416_EEPROM_MAGIC >> 8) & 0xFF;

    /* ---- Header starts at byte 128 (word 64 = eep_start_loc) ---- */

    /* ---- Header length ---- */
    /* The kernel uses this to bound the checksum range.
     * Length = total struct size in bytes. */
    uint16_t hdr_length = (uint16_t)(total_bytes - EEP4K_STRUCT_START);
    raw[EEP4K_OFF_HDR_LENGTH]     = hdr_length & 0xFF;
    raw[EEP4K_OFF_HDR_LENGTH + 1] = (hdr_length >> 8) & 0xFF;

    /* ---- Checksum – placeholder, computed at the end ---- */
    raw[EEP4K_OFF_HDR_CHECKSUM]     = 0;
    raw[EEP4K_OFF_HDR_CHECKSUM + 1] = 0;

    /* ---- EEPROM version ----
     * The kernel extracts: ver = (version >> 12) & 0xF, rev = version & 0xFFF
     * It requires ver == 14 (0xE) and rev >= 3.
     * So we need uint16_t value = (14 << 12) | 14 = 0xE00E.
     * In little-endian bytes: low = 0x0E, high = 0xE0. */
    raw[EEP4K_OFF_HDR_VERSION]     = 0x0E;  /* low byte */
    raw[EEP4K_OFF_HDR_VERSION + 1] = 0xE0;  /* high byte */

    /* ---- opCapFlags: 2.4 GHz, HT20, HT40 ---- */
    raw[EEP4K_OFF_HDR_OPCAP] = AR5416_OPFLAGS_11G |
                                AR5416_OPFLAGS_N_2G_HT40 |
                                AR5416_OPFLAGS_N_2G_HT20;

    /* ---- eepMisc: little-endian EEPROM (bit 0 = 0) ---- */
    raw[EEP4K_OFF_HDR_EEPMISC] = 0x00;

    /* ---- Regulatory domain pair ---- */
    /* regDmn[0] = 0x0000 (world), regDmn[1] = 0x001F */
    raw[EEP4K_OFF_HDR_REGDMN0]     = 0x00;
    raw[EEP4K_OFF_HDR_REGDMN0 + 1] = 0x00;
    raw[EEP4K_OFF_HDR_REGDMN1]     = 0x1F;
    raw[EEP4K_OFF_HDR_REGDMN1 + 1] = 0x00;

    /*
     * ---- MAC address ----
     * Use the Atheros OUI (00:03:7F) with a distinguishable virtual suffix.
     * In Phase 3 each VM instance should get a unique address.
     */
    raw[EEP4K_OFF_HDR_MACADDR + 0] = 0x00;
    raw[EEP4K_OFF_HDR_MACADDR + 1] = 0x03;
    raw[EEP4K_OFF_HDR_MACADDR + 2] = 0x7F;
    raw[EEP4K_OFF_HDR_MACADDR + 3] = 0xAA;
    raw[EEP4K_OFF_HDR_MACADDR + 4] = 0xBB;
    raw[EEP4K_OFF_HDR_MACADDR + 5] = 0xCC;

    /*
     * ---- TX power / calibration stubs ----
     *
     * The 4K EEPROM has calibration data starting around byte offset 32.
     * The driver reads these during ath9k_hw_4k_set_board_values() which
     * is NOT called during initial probe – it's called during the first
     * channel set (Phase 2+).
     *
     * For Phase 1 (probe only), having zeros here is acceptable.  The
     * driver will read them, see benign values, and not crash.
     *
     * Key things the driver checks:
     *   - txMask and rxMask fields (1-chain for AR9285)
     *   - rfSilent settings
     *
     * We set txMask = 1, rxMask = 1 (single chain) at byte offsets
     * that correspond to the struct fields.  The exact offsets depend
     * on kernel version; for robustness we leave the rest zero.
     */

    /*
     * The txRxMask field is at a specific offset in base_eep_header_4k.
     * In recent kernels: byte offset ~20 from start of header.
     * txMask is upper nibble, rxMask is lower nibble of a single byte.
     * Value 0x11 = txMask=1, rxMask=1.
     */
    raw[EEP4K_OFF_HDR_START + 18] = 0x11;  /* txRxMask: tx=1, rx=1 */

    /* ---- Compute and store checksum ---- */
    csum = ath9k_eeprom_calc_checksum(raw, total_bytes);
    raw[EEP4K_OFF_HDR_CHECKSUM]     = csum & 0xFF;
    raw[EEP4K_OFF_HDR_CHECKSUM + 1] = (csum >> 8) & 0xFF;

    /* ---- Copy into the 16-bit word array ---- */
    for (i = 0; i < count && (i * 2 + 1) < total_bytes; i++) {
        eeprom[i] = (uint16_t)raw[i * 2] |
                    ((uint16_t)raw[i * 2 + 1] << 8);
    }
}

#endif /* ATH9K_EEPROM_H */
