#
# vwifi-pseudohost — WPA2-PSK authenticator (AP side of the handshake)
#
# SPDX-License-Identifier: GPL-2.0-or-later
#
# The access-point half of the 802.11i four-way handshake, the mirror of
# supplicant.py.  A pseudo-AP runs one of these per associating station:
#
#   AP  --1--> ANonce                                (this file builds)
#   STA --2--> SNonce + RSN IE, MIC                  (this file verifies)
#   AP  --3--> GTK (wrapped) + RSN IE, MIC, Install  (this file builds)
#   STA --4--> ack, MIC                              (this file verifies)
#
# On success the pairwise TK is exposed for the AP to install as that
# station's CCMP key; the group GTK is the AP's own (shared across all
# stations) and is handed in at construction.  The EAPOL-Key byte layout
# and the key derivation live in supplicant.py / crypto.py; this is only
# the authenticator state machine.
#
import os
import struct

from . import crypto
from . import supplicant as sup


class Authenticator:
    """Drives the AP side of one WPA2-PSK four-way handshake."""

    def __init__(self, pmk, ap_mac, sta_mac, gtk, gtk_key_id=1, rsn_ie=None,
                 log=None):
        self.pmk = pmk
        self.aa = bytes(ap_mac)                 # authenticator (us)
        self.spa = bytes(sta_mac)               # supplicant (the station)
        self.gtk = bytes(gtk)
        self.gtk_key_id = gtk_key_id
        self.rsn_ie = bytes(rsn_ie) if rsn_ie else b""
        self.anonce = os.urandom(32)
        self.snonce = None
        self.kck = self.kek = self.tk = None
        self.replay = 0
        self._msg3 = None                       # cached, for retransmits
        self.completed = False
        self.verbose = False
        self._log = log or (lambda *a: None)

    # -- message 1 ---------------------------------------------------------
    def start(self):
        """Build EAPOL-Key message 1 (ANonce) to send to the station."""
        self.replay = 1
        info = 2 | sup.KI_TYPE_PAIRWISE | sup.KI_ACK
        return self._frame(info, self.anonce, b"", mic=False)

    # -- message 2 -> message 3 -------------------------------------------
    def handle(self, eapol):
        """Consume an inbound EAPOL-Key (msg 2 or 4).

        Returns msg 3 to send after msg 2, or None after msg 4.  Sets
        .completed and .tk once the handshake finishes.  Raises
        HandshakeError on a MIC failure.
        """
        if not sup.Supplicant.is_eapol_key(eapol):
            return None
        key_info = struct.unpack_from(">H", eapol, sup._OFF_KEYINFO)[0]
        secure = bool(key_info & sup.KI_SECURE)

        # message 2: has MIC, not Secure, carries the SNonce.
        if (key_info & sup.KI_MIC) and not secure:
            snonce = bytes(eapol[sup._OFF_NONCE : sup._OFF_NONCE + 32])
            # A retransmitted msg2 with the same SNonce must get the *same*
            # msg3 back — same replay counter, same everything.  hostapd
            # does this; incrementing the replay counter (or re-keying)
            # on every retransmit makes stricter supplicants (Windows/WDI)
            # lose the thread and then fail msg4's MIC.  Only re-derive
            # when the SNonce actually changes (a genuinely new attempt).
            if snonce == self.snonce and self._msg3 is not None:
                self._log("authenticator: msg2 retransmit — resending msg3")
                return self._msg3
            self.snonce = snonce
            self.kck, self.kek, self.tk = crypto.ptk_from_pmk(
                self.pmk, self.aa, self.spa, self.anonce, self.snonce)
            if not self._verify_mic(eapol):
                raise sup.HandshakeError("msg2 MIC verification failed")
            if self.verbose:
                self._log("authenticator: msg2 rx snonce=%s kck=%s" % (
                    self.snonce[:4].hex(), self.kck[:4].hex()))
            else:
                self._log("authenticator: msg2 rx (SNonce) — sending msg3")
            self._msg3 = self._build_msg3()
            return self._msg3

        # message 4: has MIC and Secure, empty key data.
        if (key_info & sup.KI_MIC) and secure:
            if self.kck is None:
                raise sup.HandshakeError("msg4 before msg2")
            if not self._verify_mic(eapol):
                if self.verbose:
                    recv = bytes(eapol[sup._OFF_MIC : sup._OFF_MIC + 16])
                    tmp = bytearray(eapol)
                    tmp[sup._OFF_MIC : sup._OFF_MIC + 16] = b"\x00" * 16
                    got = crypto.eapol_mic(self.kck, bytes(tmp))
                    rep = struct.unpack_from(">Q", eapol, sup._OFF_REPLAY)[0]
                    self._log("authenticator: msg4 MIC FAIL len=%d replay=%d "
                              "kck=%s recv_mic=%s calc_mic=%s" % (
                                  len(eapol), rep, self.kck[:4].hex(),
                                  recv.hex(), got.hex()))
                    self._log("authenticator: msg4 hex=%s" % eapol.hex())
                raise sup.HandshakeError("msg4 MIC verification failed")
            self.completed = True
            self._log("authenticator: msg4 rx — keys installed")
            return None

        return None

    def _build_msg3(self):
        # msg3's replay counter is msg1's + 1 and stays fixed across
        # retransmits (see the msg2 handler).
        self.replay = 2
        # GTK KDE: dd len 00-0F-AC 01 keyid rsvd || GTK, padded to /8.
        kde_body = bytes([0x00, 0x0F, 0xAC, 0x01, self.gtk_key_id & 0x03,
                          0x00]) + self.gtk
        key_data = bytearray(self.rsn_ie)
        key_data += bytes([0xDD, len(kde_body)]) + kde_body
        if len(key_data) % 8:
            key_data += b"\x00" * (8 - len(key_data) % 8)
        wrapped = crypto.aes_key_wrap(self.kek, bytes(key_data))
        info = (2 | sup.KI_TYPE_PAIRWISE | sup.KI_ACK | sup.KI_MIC
                | sup.KI_INSTALL | sup.KI_SECURE | sup.KI_ENCRYPTED)
        return self._frame(info, self.anonce, wrapped, mic=True)

    # -- helpers -----------------------------------------------------------
    def _frame(self, key_info, nonce, key_data, mic):
        f = bytearray(sup._OFF_KEYDATA + len(key_data))
        f[0] = sup.EAPOL_VERSION
        f[1] = sup.EAPOL_TYPE_KEY
        struct.pack_into(">H", f, 2, 95 + len(key_data))
        f[4] = sup.KEY_DESC_RSN
        struct.pack_into(">H", f, sup._OFF_KEYINFO, key_info)
        struct.pack_into(">H", f, sup._OFF_KEYLEN, 16)
        struct.pack_into(">Q", f, sup._OFF_REPLAY, self.replay)
        f[sup._OFF_NONCE : sup._OFF_NONCE + 32] = nonce
        struct.pack_into(">H", f, sup._OFF_KDLEN, len(key_data))
        f[sup._OFF_KEYDATA:] = key_data
        if mic:
            f[sup._OFF_MIC : sup._OFF_MIC + 16] = crypto.eapol_mic(self.kck,
                                                                  bytes(f))
        return bytes(f)

    def _verify_mic(self, eapol):
        return sup.verify_eapol_mic(self.kck, eapol)
