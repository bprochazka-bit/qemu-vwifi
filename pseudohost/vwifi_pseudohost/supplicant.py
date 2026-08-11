#
# vwifi-pseudohost — WPA2-PSK supplicant (the four-way handshake)
#
# SPDX-License-Identifier: GPL-2.0-or-later
#
# The station side of the 802.11i four-way handshake for WPA2-PSK with
# CCMP (key descriptor version 2 — HMAC-SHA1 MIC, AES key wrap).  The AP
# (hostapd, in the lab) is the Authenticator; this is the Supplicant.
#
#   AP  --1--> ANonce                                (unprotected)
#   STA --2--> SNonce + RSN IE, MIC
#   AP  --3--> GTK (wrapped) + RSN IE, MIC, Install
#   STA --4--> ack, MIC
#
# On success the pairwise TK and the group GTK are exposed for the
# station to install into its CCMP engine.  Everything cryptographic is
# in crypto.py; this file is just the state machine and the EAPOL-Key
# byte layout.
#
import os
import struct

from . import crypto

# EAPOL / 802.1X
EAPOL_VERSION = 2
EAPOL_TYPE_KEY = 3
KEY_DESC_RSN = 2

# Key Information bits
KI_TYPE_PAIRWISE = 1 << 3
KI_INSTALL = 1 << 6
KI_ACK = 1 << 7
KI_MIC = 1 << 8
KI_SECURE = 1 << 9
KI_ERROR = 1 << 10
KI_REQUEST = 1 << 11
KI_ENCRYPTED = 1 << 12
KI_VERSION_MASK = 0x0007

# EAPOL-Key field offsets from the start of the 802.1X frame.
_OFF_KEYINFO = 5
_OFF_KEYLEN = 7
_OFF_REPLAY = 9
_OFF_NONCE = 17
_OFF_MIC = 81
_OFF_KDLEN = 97
_OFF_KEYDATA = 99


class HandshakeError(Exception):
    pass


def verify_eapol_mic(kck, eapol):
    """Verify an EAPOL-Key MIC (HMAC-SHA1-128, key desc version 2).

    The MIC covers the EAPOL PDU as declared by the 802.1X Packet Body
    Length field (bytes 2-3), not any trailing padding a transmitter may
    have appended to reach a minimum frame size.  Computing it over the
    padded frame is a real interop failure mode, so honour the declared
    length when it is present and sane, and fall back to the whole buffer
    otherwise.
    """
    if len(eapol) < _OFF_KEYDATA:
        return False
    declared = 4 + struct.unpack_from(">H", eapol, 2)[0]
    n = declared if _OFF_KEYDATA <= declared <= len(eapol) else len(eapol)
    recv = bytes(eapol[_OFF_MIC : _OFF_MIC + 16])
    tmp = bytearray(eapol[:n])
    tmp[_OFF_MIC : _OFF_MIC + 16] = b"\x00" * 16
    return crypto.eapol_mic(kck, bytes(tmp)) == recv


class Supplicant:
    """Drives one WPA2-PSK four-way handshake to completion."""

    def __init__(self, pmk, sta_mac, ap_mac, rsn_ie, log=None):
        self.pmk = pmk
        self.spa = bytes(sta_mac)               # supplicant (us)
        self.aa = bytes(ap_mac)                 # authenticator (AP)
        self.rsn_ie = bytes(rsn_ie)             # our RSN element, echoed in m2
        self.snonce = os.urandom(32)
        self.anonce = None
        self.kck = self.kek = self.tk = None
        self.gtk = None
        self.gtk_key_id = 1
        self.completed = False
        self._log = log or (lambda *a: None)

    # -- helpers -----------------------------------------------------------
    @staticmethod
    def is_eapol_key(eapol):
        return (len(eapol) >= _OFF_KEYDATA
                and eapol[1] == EAPOL_TYPE_KEY
                and eapol[4] == KEY_DESC_RSN)

    def _derive_ptk(self):
        self.kck, self.kek, self.tk = crypto.ptk_from_pmk(
            self.pmk, self.aa, self.spa, self.anonce, self.snonce)

    def _build_key_frame(self, key_info, replay, nonce, key_data,
                         mic=True):
        # 802.1X body length: everything after the 4-byte EAPOL header,
        # i.e. the fixed 95-byte key descriptor plus the key data.
        body_len = 95 + len(key_data)
        f = bytearray(_OFF_KEYDATA + len(key_data))
        f[0] = EAPOL_VERSION
        f[1] = EAPOL_TYPE_KEY
        struct.pack_into(">H", f, 2, body_len)
        f[4] = KEY_DESC_RSN
        struct.pack_into(">H", f, _OFF_KEYINFO, key_info)
        struct.pack_into(">H", f, _OFF_KEYLEN, 16)          # CCMP TK length
        f[_OFF_REPLAY : _OFF_REPLAY + 8] = replay
        f[_OFF_NONCE : _OFF_NONCE + 32] = nonce
        struct.pack_into(">H", f, _OFF_KDLEN, len(key_data))
        f[_OFF_KEYDATA:] = key_data
        if mic:
            f[_OFF_MIC : _OFF_MIC + 16] = b"\x00" * 16
            mic_val = crypto.eapol_mic(self.kck, bytes(f))
            f[_OFF_MIC : _OFF_MIC + 16] = mic_val
        return bytes(f)

    def _verify_mic(self, eapol):
        return verify_eapol_mic(self.kck, eapol)

    # -- state machine -----------------------------------------------------
    def handle(self, eapol):
        """Consume one inbound EAPOL-Key frame.

        Returns the EAPOL-Key frame to send back (bytes) or None.  Sets
        .completed and .tk/.gtk when the handshake finishes.  Raises
        HandshakeError on a MIC failure or malformed message.
        """
        if not self.is_eapol_key(eapol):
            return None
        key_info = struct.unpack_from(">H", eapol, _OFF_KEYINFO)[0]
        replay = bytes(eapol[_OFF_REPLAY : _OFF_REPLAY + 8])
        pairwise = bool(key_info & KI_TYPE_PAIRWISE)
        has_mic = bool(key_info & KI_MIC)

        # Message 1: Ack set, MIC clear, pairwise.
        if pairwise and (key_info & KI_ACK) and not has_mic:
            self.anonce = bytes(eapol[_OFF_NONCE : _OFF_NONCE + 32])
            self._derive_ptk()
            self._log("handshake: msg1 rx (ANonce), sending msg2")
            m2_info = (2 | KI_TYPE_PAIRWISE | KI_MIC)     # version 2
            return self._build_key_frame(m2_info, replay, self.snonce,
                                         self.rsn_ie, mic=True)

        # Message 3: Install + Ack + MIC + (usually) Encrypted key data.
        if pairwise and (key_info & KI_ACK) and has_mic:
            if self.kck is None:
                raise HandshakeError("msg3 before msg1")
            if not self._verify_mic(eapol):
                raise HandshakeError("msg3 MIC verification failed")
            kd_len = struct.unpack_from(">H", eapol, _OFF_KDLEN)[0]
            key_data = bytes(eapol[_OFF_KEYDATA : _OFF_KEYDATA + kd_len])
            if key_info & KI_ENCRYPTED:
                key_data = crypto.aes_key_unwrap(self.kek, key_data)
                if key_data is None:
                    raise HandshakeError("msg3 key-data unwrap failed")
            self._extract_gtk(key_data)
            self._log("handshake: msg3 rx (GTK), sending msg4 — installed")
            m4_info = (2 | KI_TYPE_PAIRWISE | KI_MIC | KI_SECURE)
            frame = self._build_key_frame(m4_info, replay, b"\x00" * 32,
                                          b"", mic=True)
            self.completed = True
            return frame

        return None

    def _extract_gtk(self, key_data):
        """Pull the GTK out of the (unwrapped) key data of message 3.

        Key data is a sequence of KDEs: the GTK sits in a vendor-specific
        KDE with the 00-0F-AC data type 1.  If we can't find one we leave
        the group key unset — unicast still works, only broadcast/multicast
        RX would be undecryptable.
        """
        i = 0
        n = len(key_data)
        while i + 2 <= n:
            eid = key_data[i]
            ln = key_data[i + 1]
            if eid != 0xDD or i + 2 + ln > n:
                i += 2 + ln if ln else 2
                continue
            body = key_data[i + 2 : i + 2 + ln]
            # 00-0F-AC, type 1 == GTK KDE
            if (len(body) >= 6 and body[0] == 0x00 and body[1] == 0x0F
                    and body[2] == 0xAC and body[3] == 0x01):
                self.gtk_key_id = body[4] & 0x03
                self.gtk = bytes(body[6:])
                return
            i += 2 + ln
