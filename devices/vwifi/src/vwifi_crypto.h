/*
 * vwifi-virt — AES-128 + CCMP-128 API
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef VWIFI_CRYPTO_H
#define VWIFI_CRYPTO_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#define VWIFI_CCMP_HDR_LEN   8
#define VWIFI_CCMP_MIC_LEN   8
#define VWIFI_CCMP_KEY_LEN  16
#define VWIFI_PN_LEN         6

/* Expanded AES-128 key schedule: 11 round keys of 16 bytes. */
struct vwifi_aes128 {
    uint8_t rk[176];
};

void vwifi_aes128_key_expand(const uint8_t key[16], struct vwifi_aes128 *ctx);
void vwifi_aes128_encrypt(const struct vwifi_aes128 *ctx,
                          const uint8_t in[16], uint8_t out[16]);

/* CCM with M=8, L=2 (the CCMP parameters). `msg` is encrypted in
 * place; the 8-byte MIC is written to `mic_out`. */
void vwifi_ccm_encrypt(const struct vwifi_aes128 *aes,
                       const uint8_t nonce[13],
                       const uint8_t *aad, size_t aad_len,
                       uint8_t *msg, size_t msg_len,
                       uint8_t mic_out[8]);

/* Returns true if the MIC verified. On failure the message buffer is
 * restored to its ciphertext state. */
bool vwifi_ccm_decrypt(const struct vwifi_aes128 *aes,
                       const uint8_t nonce[13],
                       const uint8_t *aad, size_t aad_len,
                       uint8_t *msg, size_t msg_len,
                       const uint8_t mic_in[8]);

/* Length of the 802.11 header for this frame (24, +6 for A4,
 * +2 for QoS). */
size_t vwifi_ccmp_hdr_len(const uint8_t *hdr);

void vwifi_ccmp_build_header(uint8_t *out, const uint8_t pn[6], uint8_t key_id);
void vwifi_ccmp_parse_header(const uint8_t *in, uint8_t pn[6], uint8_t *key_id);

/* Encrypt an 802.11 frame in place: inserts the CCMP header, encrypts
 * the payload, appends the MIC, sets the Protected bit. `frame_cap`
 * is the buffer capacity (must allow +16 bytes). Returns the new
 * frame length, or negative on error. */
int vwifi_ccmp_encrypt(const struct vwifi_aes128 *aes,
                       uint8_t *frame, size_t frame_len,
                       const uint8_t pn[6], uint8_t key_id,
                       size_t frame_cap);

/* Decrypt an 802.11 frame in place: verifies the MIC, strips the
 * CCMP header, clears the Protected bit. Returns the new frame
 * length, -1 for malformed, -2 for MIC failure. */
int vwifi_ccmp_decrypt(const struct vwifi_aes128 *aes,
                       uint8_t *frame, size_t frame_len,
                       uint8_t pn_out[6], uint8_t *key_id_out);

int  vwifi_pn_cmp(const uint8_t a[6], const uint8_t b[6]);
void vwifi_pn_increment(uint8_t pn[6]);

#endif /* VWIFI_CRYPTO_H */
