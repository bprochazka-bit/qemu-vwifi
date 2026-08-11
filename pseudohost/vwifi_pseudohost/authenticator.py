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
        # One PTK candidate per SNonce we've seen.  A supplicant that
        # regenerates its SNonce on a msg2 retransmit (Windows/WDI does;
        # Linux does not) can end up committing to an *earlier* SNonce
        # than the one we last saw, so its msg4 is signed with an earlier
        # KCK.  We keep every candidate and accept msg4 against any of
        # them, then install that candidate's keys.
        self._candidates = []                   # [{snonce,kck,kek,tk,msg3}]
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
            # A retransmitted msg2 with the same SNonce gets the same msg3
            # back verbatim (same replay counter) — never a re-keyed one.
            existing = self._candidate(snonce)
            if existing is not None:
                self._log("authenticator: msg2 retransmit — resending msg3")
                return existing["msg3"]
            kck, kek, tk = crypto.ptk_from_pmk(
                self.pmk, self.aa, self.spa, self.anonce, snonce)
            if not sup.verify_eapol_mic(kck, eapol):
                raise sup.HandshakeError("msg2 MIC verification failed")
            self.snonce, self.kck, self.kek, self.tk = snonce, kck, kek, tk
            msg3 = self._build_msg3(kck, kek)
            self._candidates.append({"snonce": snonce, "kck": kck,
                                     "kek": kek, "tk": tk, "msg3": msg3})
            self._candidates = self._candidates[-6:]     # bound the memory
            if self.verbose:
                self._log("authenticator: msg2 rx snonce=%s kck=%s" % (
                    snonce[:4].hex(), kck[:4].hex()))
            else:
                self._log("authenticator: msg2 rx (SNonce) — sending msg3")
            return msg3

        # message 4: has MIC and Secure, empty key data.  Accept it against
        # any candidate SNonce, since the client may have committed to an
        # earlier one than we last saw.
        if (key_info & sup.KI_MIC) and secure:
            if not self._candidates:
                raise sup.HandshakeError("msg4 before msg2")
            for c in self._candidates:
                if sup.verify_eapol_mic(c["kck"], eapol):
                    self.kck, self.kek, self.tk = c["kck"], c["kek"], c["tk"]
                    self.snonce = c["snonce"]
                    self.completed = True
                    self._log("authenticator: msg4 rx — keys installed")
                    return None
            if self.verbose:
                recv = bytes(eapol[sup._OFF_MIC : sup._OFF_MIC + 16])
                rep = struct.unpack_from(">Q", eapol, sup._OFF_REPLAY)[0]
                cks = ",".join(c["kck"][:4].hex() for c in self._candidates)
                self._log("authenticator: msg4 MIC FAIL len=%d replay=%d "
                          "recv_mic=%s tried_kck=[%s]" % (
                              len(eapol), rep, recv.hex(), cks))
                self._log("authenticator: msg4 hex=%s" % eapol.hex())
            raise sup.HandshakeError("msg4 MIC verification failed")

        return None

    def _candidate(self, snonce):
        for c in self._candidates:
            if c["snonce"] == snonce:
                return c
        return None

    def _build_msg3(self, kck, kek):
        # msg3's replay counter is msg1's + 1 and stays fixed across
        # retransmits.
        self.replay = 2
        # GTK KDE: dd len 00-0F-AC 01 keyid rsvd || GTK, padded to /8.
        kde_body = bytes([0x00, 0x0F, 0xAC, 0x01, self.gtk_key_id & 0x03,
                          0x00]) + self.gtk
        key_data = bytearray(self.rsn_ie)
        key_data += bytes([0xDD, len(kde_body)]) + kde_body
        if len(key_data) % 8:
            key_data += b"\x00" * (8 - len(key_data) % 8)
        wrapped = crypto.aes_key_wrap(kek, bytes(key_data))
        info = (2 | sup.KI_TYPE_PAIRWISE | sup.KI_ACK | sup.KI_MIC
                | sup.KI_INSTALL | sup.KI_SECURE | sup.KI_ENCRYPTED)
        return self._frame(info, self.anonce, wrapped, mic=True, kck=kck)

    # -- helpers -----------------------------------------------------------
    def _frame(self, key_info, nonce, key_data, mic, kck=None):
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
            f[sup._OFF_MIC : sup._OFF_MIC + 16] = crypto.eapol_mic(
                kck if kck is not None else self.kck, bytes(f))
        return bytes(f)
