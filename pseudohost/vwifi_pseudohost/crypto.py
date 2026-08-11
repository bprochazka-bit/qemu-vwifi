#
# vwifi-pseudohost — 802.11 cryptography
#
# SPDX-License-Identifier: GPL-2.0-or-later
#
# Everything a WPA2-PSK station needs to speak CCMP and run the four-way
# handshake, in one dependency-light module:
#
#   - AES-128 block cipher (pure-Python, FIPS-197)      aes128_encrypt/decrypt
#   - CCM* (M=8, L=2) — the CCMP AEAD mode              ccm_encrypt/decrypt
#   - CCMP frame encrypt/decrypt in place               ccmp_encrypt/decrypt
#   - RFC 3394 AES key unwrap (EAPOL-Key key data)      aes_key_unwrap
#   - PBKDF2-HMAC-SHA1 PMK derivation                   wpa_pmk
#   - IEEE 802.11 PRF-n (PTK expansion)                 prf
#   - EAPOL-Key MIC (HMAC-SHA1-128, key desc ver 2)     eapol_mic
#
# Why pure Python for AES?  The rest of qemu-vwifi builds with nothing
# but a C compiler and `make`; a lab tool that needs `pip install
# cryptography` before it can join a WPA2 network breaks that promise on
# exactly the machines where it is least convenient.  So the block
# cipher is bundled and correct on its own.  If PyCryptodome or the
# `cryptography` package happens to be installed we borrow its AES-ECB
# for speed (CCMP on a busy link is a lot of block operations), but
# nothing here *requires* it — see _load_fast_ecb().
#
# The CCMP nonce and AAD construction is deliberately byte-for-byte
# identical to devices/vwifi/src/vwifi_crypto.c (ccmp_nonce / ccmp_aad);
# a station whose AAD disagrees with the AP's by one bit fails the MIC
# with nothing to point at.  tests/test_crypto.py pins both against the
# same known-answer vectors that C file's tests use.
#

import hashlib
import hmac
import os
import struct

# ----------------------------------------------------------------------------
# AES-128 (FIPS-197), pure Python.
# ----------------------------------------------------------------------------
#
# A textbook implementation: it is small, obviously correct against the
# FIPS-197 Appendix C.1 vector, and never on a hot enough path in this
# tool to justify a table-driven rewrite.  When speed matters, the
# accelerated ECB backend below sidesteps it entirely.

_SBOX = bytes.fromhex(
    "637c777bf26b6fc53001672bfed7ab76ca82c97dfa5947f0add4a2af9ca472c0"
    "b7fd9326363ff7cc34a5e5f171d8311504c723c31896059a071280e2eb27b275"
    "09832c1a1b6e5aa0523bd6b329e32f8453d100ed20fcb15b6acbbe394a4c58cf"
    "d0efaafb434d338545f9027f503c9fa851a3408f929d38f5bcb6da2110fff3d2"
    "cd0c13ec5f974417c4a77e3d645d197360814fdc222a908846eeb814de5e0bdb"
    "e0323a0a4906245cc2d3ac629195e479e7c8376d8dd54ea96c56f4ea657aae08"
    "ba78252e1ca6b4c6e8dd741f4bbd8b8a703eb5664803f60e613557b986c11d9e"
    "e1f8981169d98e949b1e87e9ce5528df8ca1890dbfe6426841992d0fb054bb16"
)
_INV_SBOX = bytearray(256)
for _i, _v in enumerate(_SBOX):
    _INV_SBOX[_v] = _i
_INV_SBOX = bytes(_INV_SBOX)

_RCON = (0x01, 0x02, 0x04, 0x08, 0x10, 0x20, 0x40, 0x80, 0x1B, 0x36)


def _xtime(a):
    a <<= 1
    if a & 0x100:
        a ^= 0x11B
    return a & 0xFF


def _mul(a, b):
    """GF(2^8) multiply, for MixColumns."""
    p = 0
    for _ in range(8):
        if b & 1:
            p ^= a
        b >>= 1
        a = _xtime(a)
    return p


def aes128_key_expand(key):
    """Return 11 round keys (list of 16-byte bytearrays)."""
    assert len(key) == 16
    words = [list(key[i : i + 4]) for i in range(0, 16, 4)]
    for i in range(4, 44):
        temp = list(words[i - 1])
        if i % 4 == 0:
            temp = temp[1:] + temp[:1]                      # RotWord
            temp = [_SBOX[b] for b in temp]                 # SubWord
            temp[0] ^= _RCON[i // 4 - 1]
        words.append([words[i - 4][j] ^ temp[j] for j in range(4)])
    return [bytearray(b for w in words[r * 4 : r * 4 + 4] for b in w)
            for r in range(11)]


def _add_round_key(s, rk):
    for i in range(16):
        s[i] ^= rk[i]


def aes128_encrypt_rk(rks, block):
    """Encrypt one 16-byte block with pre-expanded round keys."""
    s = bytearray(block)
    _add_round_key(s, rks[0])
    for rnd in range(1, 10):
        s = bytearray(_SBOX[b] for b in s)                  # SubBytes
        # ShiftRows (state is column-major: byte index = col*4 + row)
        s = bytearray((
            s[0], s[5], s[10], s[15],
            s[4], s[9], s[14], s[3],
            s[8], s[13], s[2], s[7],
            s[12], s[1], s[6], s[11],
        ))
        for c in range(4):                                  # MixColumns
            col = s[c * 4 : c * 4 + 4]
            s[c * 4 + 0] = _mul(col[0], 2) ^ _mul(col[1], 3) ^ col[2] ^ col[3]
            s[c * 4 + 1] = col[0] ^ _mul(col[1], 2) ^ _mul(col[2], 3) ^ col[3]
            s[c * 4 + 2] = col[0] ^ col[1] ^ _mul(col[2], 2) ^ _mul(col[3], 3)
            s[c * 4 + 3] = _mul(col[0], 3) ^ col[1] ^ col[2] ^ _mul(col[3], 2)
        _add_round_key(s, rks[rnd])
    s = bytearray(_SBOX[b] for b in s)
    s = bytearray((
        s[0], s[5], s[10], s[15],
        s[4], s[9], s[14], s[3],
        s[8], s[13], s[2], s[7],
        s[12], s[1], s[6], s[11],
    ))
    _add_round_key(s, rks[10])
    return bytes(s)


def aes128_decrypt_rk(rks, block):
    """Decrypt one 16-byte block — only used by aes_key_unwrap()."""
    s = bytearray(block)
    _add_round_key(s, rks[10])
    for rnd in range(9, 0, -1):
        s = bytearray((                                     # InvShiftRows
            s[0], s[13], s[10], s[7],
            s[4], s[1], s[14], s[11],
            s[8], s[5], s[2], s[15],
            s[12], s[9], s[6], s[3],
        ))
        s = bytearray(_INV_SBOX[b] for b in s)              # InvSubBytes
        _add_round_key(s, rks[rnd])
        for c in range(4):                                  # InvMixColumns
            col = s[c * 4 : c * 4 + 4]
            s[c * 4 + 0] = (_mul(col[0], 14) ^ _mul(col[1], 11)
                            ^ _mul(col[2], 13) ^ _mul(col[3], 9))
            s[c * 4 + 1] = (_mul(col[0], 9) ^ _mul(col[1], 14)
                            ^ _mul(col[2], 11) ^ _mul(col[3], 13))
            s[c * 4 + 2] = (_mul(col[0], 13) ^ _mul(col[1], 9)
                            ^ _mul(col[2], 14) ^ _mul(col[3], 11))
            s[c * 4 + 3] = (_mul(col[0], 11) ^ _mul(col[1], 13)
                            ^ _mul(col[2], 9) ^ _mul(col[3], 14))
    s = bytearray((
        s[0], s[13], s[10], s[7],
        s[4], s[1], s[14], s[11],
        s[8], s[5], s[2], s[15],
        s[12], s[9], s[6], s[3],
    ))
    s = bytearray(_INV_SBOX[b] for b in s)
    _add_round_key(s, rks[0])
    return bytes(s)


# Optional accelerated ECB.  CCMP only ever needs single-block encrypt,
# so that is all we borrow; decrypt (key unwrap) stays on the pure path
# because it runs a handful of times per association at most.
def _silence_stderr():
    """Context manager that redirects the OS-level stderr fd to /dev/null.

    A mis-built `cryptography` wheel doesn't just raise on import — its
    Rust panic hook writes a backtrace straight to fd 2 first.  We catch
    the exception, but the backtrace still lands in the user's terminal.
    Muffle the fd for the duration of the probe so a broken optional
    backend is silent, not just non-fatal.
    """
    import contextlib

    @contextlib.contextmanager
    def _ctx():
        try:
            devnull = os.open(os.devnull, os.O_WRONLY)
        except OSError:
            yield
            return
        saved = os.dup(2)
        try:
            os.dup2(devnull, 2)
            yield
        finally:
            os.dup2(saved, 2)
            os.close(saved)
            os.close(devnull)
    return _ctx()


def _load_fast_ecb():
    # A broken optional backend (a mis-built `cryptography` wheel can raise
    # a Rust PanicException, which is a BaseException, not an Exception)
    # must never take the tool down — probe defensively and fall back to
    # the pure-Python cipher on anything at all.
    try:
        from Crypto.Cipher import AES as _AES  # PyCryptodome

        def enc(key, block):
            return _AES.new(key, _AES.MODE_ECB).encrypt(block)
        enc(b"\x00" * 16, b"\x00" * 16)         # smoke-test the backend
        return enc
    except BaseException:
        pass
    try:
        with _silence_stderr():
            from cryptography.hazmat.primitives.ciphers import (
                Cipher, algorithms, modes)

            def enc(key, block):
                c = Cipher(algorithms.AES(key), modes.ECB()).encryptor()
                return c.update(block) + c.finalize()
            enc(b"\x00" * 16, b"\x00" * 16)
        return enc
    except BaseException:
        pass
    return None


_FAST_ECB = _load_fast_ecb()


class AES128:
    """A keyed AES-128 block cipher, wrapping whichever backend is live."""

    __slots__ = ("_rks", "_key")

    def __init__(self, key):
        if len(key) != 16:
            raise ValueError("AES-128 needs a 16-byte key")
        self._key = bytes(key)
        self._rks = aes128_key_expand(key)

    def encrypt(self, block):
        if _FAST_ECB is not None:
            return _FAST_ECB(self._key, bytes(block))
        return aes128_encrypt_rk(self._rks, block)

    def decrypt(self, block):
        # No fast path: unwrap is rare and pure-Python decrypt is fine.
        return aes128_decrypt_rk(self._rks, block)


# ----------------------------------------------------------------------------
# CCM* (M=8, L=2) — the AEAD mode CCMP is built on.
# ----------------------------------------------------------------------------

def _xor16(a, b):
    return bytes(x ^ y for x, y in zip(a, b))


def _ccm_cbc_mac(aes, nonce, aad, msg):
    """CBC-MAC over B0 || AAD-blocks || msg-blocks; returns the 16-byte T."""
    # B0: flags || nonce(13) || l(msg) as 2 bytes.  Flags for M=8,L=2 with
    # AAD present: Adata(0x40) | ((M-2)/2 << 3)=0x18 | (L-1)=0x01 = 0x59.
    flags = 0x40 | (((8 - 2) // 2) << 3) | (2 - 1)
    b0 = bytes([flags]) + nonce + struct.pack(">H", len(msg))
    x = aes.encrypt(b0)

    # AAD is length-prefixed (2 bytes here — AAD < 2^16) then zero-padded
    # to a block boundary, and MAC'd block by block.
    a = struct.pack(">H", len(aad)) + aad
    if len(a) % 16:
        a += b"\x00" * (16 - len(a) % 16)
    for i in range(0, len(a), 16):
        x = aes.encrypt(_xor16(x, a[i : i + 16]))

    m = msg
    if len(m) % 16:
        m = m + b"\x00" * (16 - len(m) % 16)
    for i in range(0, len(m), 16):
        x = aes.encrypt(_xor16(x, m[i : i + 16]))
    return x


def _ccm_ctr_blocks(aes, nonce, count):
    """Yield keystream blocks S_0, S_1, ... for CTR mode."""
    # A_i: flags(L-1=0x01) || nonce(13) || counter(2 bytes).
    for i in range(count):
        a = bytes([2 - 1]) + nonce + struct.pack(">H", i)
        yield aes.encrypt(a)


def ccm_encrypt(aes, nonce, aad, msg):
    """CCM* encrypt.  Returns (ciphertext, 8-byte MIC)."""
    t = _ccm_cbc_mac(aes, nonce, aad, msg)
    ctr = _ccm_ctr_blocks(aes, nonce, (len(msg) + 15) // 16 + 1)
    s0 = next(ctr)
    mic = _xor16(t, s0)[:8]
    out = bytearray()
    for i in range(0, len(msg), 16):
        block = msg[i : i + 16]
        out += _xor16(block, next(ctr)[: len(block)])
    return bytes(out), bytes(mic)


def ccm_decrypt(aes, nonce, aad, ciphertext, mic):
    """CCM* decrypt.  Returns plaintext, or None if the MIC is wrong."""
    ctr = _ccm_ctr_blocks(aes, nonce, (len(ciphertext) + 15) // 16 + 1)
    s0 = next(ctr)
    out = bytearray()
    for i in range(0, len(ciphertext), 16):
        block = ciphertext[i : i + 16]
        out += _xor16(block, next(ctr)[: len(block)])
    t = _ccm_cbc_mac(aes, nonce, aad, bytes(out))
    if _xor16(t, s0)[:8] != bytes(mic):
        return None
    return bytes(out)


# ----------------------------------------------------------------------------
# CCMP frame encryption — mirrors devices/vwifi/src/vwifi_crypto.c.
# ----------------------------------------------------------------------------

CCMP_HDR_LEN = 8
CCMP_MIC_LEN = 8

_FCTL_PROTECTED = 0x40
_FCTL_RETRY = 0x08     # byte 1 bit
_FCTL_PM = 0x10        # byte 1 bit
_FCTL_MOREDATA = 0x20  # byte 1 bit
_STYPE_MASK = 0x00F0   # subtype bits in the 16-bit frame control


def _fc(hdr):
    return hdr[0] | (hdr[1] << 8)


def _is_mgmt(hdr):
    return ((_fc(hdr) >> 2) & 0x3) == 0


def _has_a4(hdr):
    b1 = hdr[1]
    return (b1 & 0x01) and (b1 & 0x02)          # ToDS and FromDS


def _is_qos(hdr):
    # QoS data subframes have subtype bit 0x08 within a data frame.
    return ((_fc(hdr) >> 2) & 0x3) == 2 and (hdr[0] & 0x80)


def ccmp_hdr_len(hdr):
    n = 24
    if _has_a4(hdr):
        n += 6
    if _is_qos(hdr):
        n += 2
    return n


def _ccmp_nonce(hdr, pn):
    tid = 0
    mgmt = _is_mgmt(hdr)
    if _is_qos(hdr):
        qos_off = 30 if _has_a4(hdr) else 24
        tid = hdr[qos_off] & 0x0F
    nonce = bytearray(13)
    nonce[0] = tid | (0x10 if mgmt else 0x00)
    nonce[1:7] = hdr[10:16]     # A2
    nonce[7:13] = pn            # PN, big-endian PN5..PN0
    return bytes(nonce)


def _ccmp_aad(hdr):
    fc = _fc(hdr)
    mgmt = _is_mgmt(hdr)
    mask_fc = fc
    # Mask Retry / PwrMgt / MoreData (byte-1 bits, i.e. <<8), set Protected.
    mask_fc &= ~((_FCTL_RETRY | _FCTL_PM | _FCTL_MOREDATA) << 8) & 0xFFFF
    if not mgmt:
        mask_fc &= ~_STYPE_MASK & 0xFFFF
    mask_fc |= _FCTL_PROTECTED << 8
    aad = bytearray()
    aad.append(mask_fc & 0xFF)
    aad.append((mask_fc >> 8) & 0xFF)
    aad += hdr[4:22]                    # A1 || A2 || A3
    aad.append(hdr[22] & 0x0F)          # seq ctrl: keep frag, mask seq
    aad.append(0)
    if _has_a4(hdr):
        aad += hdr[24:30]              # A4
    if _is_qos(hdr):
        qos_off = 30 if _has_a4(hdr) else 24
        aad.append(hdr[qos_off] & 0x0F)
        aad.append(0)
    return bytes(aad)


def ccmp_build_header(pn, key_id):
    """The 8-byte CCMP header: PN0 PN1 rsvd keyid|ExtIV PN2 PN3 PN4 PN5.

    The header stores the packet number low byte first (PN0 = least
    significant, at offset 0).  Our `pn` array is big-endian — pn[0] is
    the most significant byte, pn[5] the least — the same convention the
    nonce uses (memcpy of pn as PN5..PN0) and that pn_increment carries
    from.  So the header takes pn reversed.  This has to match the real
    802.11 device byte-for-byte or CCMP interop silently fails the MIC.
    """
    return bytes([pn[5], pn[4], 0x00, (key_id << 6) | 0x20,
                  pn[3], pn[2], pn[1], pn[0]])


def ccmp_parse_header(hdr8):
    key_id = (hdr8[3] >> 6) & 0x03
    # Rebuild the big-endian pn (PN5..PN0) from the header's low-first layout.
    pn = bytes([hdr8[7], hdr8[6], hdr8[5], hdr8[4], hdr8[1], hdr8[0]])
    return pn, key_id


def ccmp_encrypt(aes, frame, pn, key_id=0):
    """Encrypt an 802.11 frame in place-style; returns a new bytes object."""
    frame = bytearray(frame)
    hlen = ccmp_hdr_len(frame)
    if len(frame) < hlen:
        raise ValueError("frame shorter than its header")
    frame[1] |= _FCTL_PROTECTED
    aad = _ccmp_aad(frame)
    nonce = _ccmp_nonce(frame, pn)
    payload = bytes(frame[hlen:])
    ct, mic = ccm_encrypt(aes, nonce, aad, payload)
    return (bytes(frame[:hlen]) + ccmp_build_header(pn, key_id) + ct + mic)


def ccmp_decrypt(aes, frame):
    """Decrypt an 802.11 frame.  Returns (plaintext_frame, pn) or None."""
    frame = bytearray(frame)
    hlen = ccmp_hdr_len(frame)
    if len(frame) < hlen + CCMP_HDR_LEN + CCMP_MIC_LEN:
        return None
    if not (frame[1] & _FCTL_PROTECTED):
        return None
    pn, _kid = ccmp_parse_header(frame[hlen : hlen + CCMP_HDR_LEN])
    aad = _ccmp_aad(frame)              # AAD over header as received
    nonce = _ccmp_nonce(frame, pn)
    body = bytes(frame[hlen + CCMP_HDR_LEN : -CCMP_MIC_LEN])
    mic = bytes(frame[-CCMP_MIC_LEN:])
    pt = ccm_decrypt(aes, nonce, aad, body, mic)
    if pt is None:
        return None
    frame[1] &= ~_FCTL_PROTECTED
    return bytes(frame[:hlen]) + pt, pn


def pn_increment(pn):
    pn = bytearray(pn)
    for i in range(5, -1, -1):
        pn[i] = (pn[i] + 1) & 0xFF
        if pn[i]:
            break
    return bytes(pn)


# ----------------------------------------------------------------------------
# RFC 3394 AES key unwrap — decrypts the Key Data of EAPOL-Key msg 3.
# ----------------------------------------------------------------------------

def aes_key_wrap(kek, plain):
    """Wrap key material with the KEK (RFC 3394).  Length must be a
    multiple of 8 bytes.  Used to build test vectors and, later, group
    rekey frames from a pseudo-AP."""
    if len(plain) % 8 != 0 or len(plain) < 16:
        raise ValueError("key data must be a multiple of 8 bytes, >= 16")
    aes = AES128(kek)
    n = len(plain) // 8
    a = bytearray(b"\xa6\xa6\xa6\xa6\xa6\xa6\xa6\xa6")
    r = [bytearray(plain[8 * i : 8 * i + 8]) for i in range(n)]
    for j in range(6):
        for i in range(1, n + 1):
            b = aes.encrypt(bytes(a) + bytes(r[i - 1]))
            a = bytearray(b[:8])
            t = (n * j) + i
            a[7] ^= t & 0xFF
            a[6] ^= (t >> 8) & 0xFF
            r[i - 1] = bytearray(b[8:])
    return bytes(a) + b"".join(bytes(x) for x in r)


def aes_key_unwrap(kek, wrapped):
    """Unwrap a key with the KEK.  Returns the plaintext, or None on failure."""
    if len(wrapped) % 8 != 0 or len(wrapped) < 24:
        return None
    aes = AES128(kek)
    n = len(wrapped) // 8 - 1
    a = bytearray(wrapped[:8])
    r = [bytearray(wrapped[8 * (i + 1) : 8 * (i + 2)]) for i in range(n)]
    for j in range(5, -1, -1):
        for i in range(n, 0, -1):
            t = (n * j) + i
            # XOR t into A (big-endian); t is small so only low bytes set.
            av = bytearray(a)
            av[7] ^= t & 0xFF
            av[6] ^= (t >> 8) & 0xFF
            b = aes.decrypt(bytes(av) + bytes(r[i - 1]))
            a = bytearray(b[:8])
            r[i - 1] = bytearray(b[8:])
    if bytes(a) != b"\xa6\xa6\xa6\xa6\xa6\xa6\xa6\xa6":
        return None
    return b"".join(bytes(x) for x in r)


# ----------------------------------------------------------------------------
# WPA key derivation.
# ----------------------------------------------------------------------------

def wpa_pmk(passphrase, ssid):
    """PMK = PBKDF2-HMAC-SHA1(passphrase, ssid, 4096, 32)."""
    if isinstance(passphrase, str):
        passphrase = passphrase.encode("utf-8")
    if isinstance(ssid, str):
        ssid = ssid.encode("utf-8")
    return hashlib.pbkdf2_hmac("sha1", passphrase, ssid, 4096, 32)


def prf(key, label, data, nbits):
    """IEEE 802.11 PRF-n over HMAC-SHA1 (used for PTK expansion)."""
    if isinstance(label, str):
        label = label.encode("ascii")
    out = b""
    i = 0
    while len(out) * 8 < nbits:
        out += hmac.new(key, label + b"\x00" + data + bytes([i]),
                        hashlib.sha1).digest()
        i += 1
    return out[: (nbits + 7) // 8]


def ptk_from_pmk(pmk, aa, spa, anonce, snonce):
    """Derive the 384-bit PTK for CCMP.

    aa  = Authenticator (AP) MAC, spa = Supplicant (our) MAC.
    Returns (kck, kek, tk): KCK[16] signs EAPOL, KEK[16] wraps key data,
    TK[16] is the CCMP pairwise key.
    """
    data = (min(aa, spa) + max(aa, spa)
            + min(anonce, snonce) + max(anonce, snonce))
    ptk = prf(pmk, "Pairwise key expansion", data, 384)
    return ptk[0:16], ptk[16:32], ptk[32:48]


def eapol_mic(kck, eapol_frame):
    """HMAC-SHA1-128 MIC over an EAPOL-Key frame with the MIC field zeroed.

    Key descriptor version 2 (AES / CCMP) uses HMAC-SHA1 truncated to
    128 bits.  The caller must zero the 16-byte MIC field before calling.
    """
    return hmac.new(kck, eapol_frame, hashlib.sha1).digest()[:16]
