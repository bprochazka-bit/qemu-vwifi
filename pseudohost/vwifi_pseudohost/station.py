#
# vwifi-pseudohost — 802.11 station state machine
#
# SPDX-License-Identifier: GPL-2.0-or-later
#
# The radio half of a pseudo-host.  Given an ESSID it:
#
#   1. listens on the broadcast "channel 0" and probes, collecting
#      beacons until it has heard the target network and learned its
#      channel (find-channel);
#   2. pins that channel, does open-system Auth then Assoc;
#   3. for a WPA2 network, runs the four-way handshake (supplicant.py)
#      and installs the CCMP keys;
#   4. thereafter carries Ethernet frames both ways — encrypting on TX,
#      decrypting on RX — and hands decrypted L2 frames to a callback.
#
# The station knows nothing about IP.  It moves Ethernet frames; the
# netstack above it decides what they mean.  This split is what lets one
# station serve a Linux workstation, a printer, or a NAS unchanged.
#
import os
import time

from . import crypto
from . import ieee80211 as dot11
from . import medium as med
from .supplicant import Supplicant, HandshakeError

# station states
ST_IDLE = "idle"
ST_SCANNING = "scanning"
ST_AUTH = "authenticating"
ST_ASSOC = "associating"
ST_HANDSHAKE = "4way"
ST_RUN = "connected"
ST_FAILED = "failed"


class KeySet:
    """The CCMP keys and PN counters for an association."""

    def __init__(self):
        self.ptk_aes = None            # pairwise TK -> AES128
        self.gtk_aes = None            # group key -> AES128 (RX only for STA)
        self.tx_pn = bytes(6)

    def next_pn(self):
        self.tx_pn = crypto.pn_increment(self.tx_pn)
        return self.tx_pn


class Station:
    def __init__(self, client, ssid, passphrase=None, mac=None, log=None,
                 scan_time=3.0):
        """
        client      : connected MediumClient
        ssid        : network name (str or bytes)
        passphrase  : WPA2-PSK passphrase, or None for an open network
        mac         : station MAC (bytes); random locally-administered if None
        scan_time   : seconds to gather beacons before giving up on a channel
        """
        self.client = client
        self.ssid = ssid.encode() if isinstance(ssid, str) else bytes(ssid)
        self.passphrase = passphrase
        self.mac = bytes(mac) if mac else random_mac()
        self.log = log or (lambda *a: None)
        self.scan_time = scan_time

        self.state = ST_IDLE
        self.bssid = None
        self.channel_freq = 0
        self.seq = dot11.SeqCounter()
        self.keys = KeySet()
        self.sup = None
        self.on_eth_rx = None          # callback(src, dst, ethertype, sdu)
        self._bss_seen = {}
        self._pmk = None
        if passphrase:
            self._pmk = crypto.wpa_pmk(passphrase, self.ssid)

    # ------------------------------------------------------------------ TX
    def _tx_mgmt(self, frame):
        self.client.send_frame(frame, self.mac, tsf_us=_now_us())

    def send_eth(self, dst, ethertype, sdu):
        """Send an Ethernet payload to the DS as an 802.11 data frame."""
        if self.state != ST_RUN:
            return False
        protected = self.keys.ptk_aes is not None
        frame = dot11.build_data_frame(self.bssid, self.mac, dst,
                                       (ethertype, sdu),
                                       protected=protected,
                                       seq=self.seq.next() >> 4)
        if protected:
            pn = self.keys.next_pn()
            frame = crypto.ccmp_encrypt(self.keys.ptk_aes, frame, pn, 0)
        self.client.send_frame(frame, self.mac, tsf_us=_now_us())
        return True

    # -------------------------------------------------------------- connect
    def connect(self, timeout=15.0):
        """Drive scan -> auth -> assoc -> handshake.  Blocking.

        Returns True once associated (and keyed, for WPA2), else False.
        """
        self.client.send_hello()
        self.client.set_channel(0)                 # broadcast: hear everything
        deadline = time.time() + timeout
        self._enter_scan()
        while time.time() < deadline:
            if self.state in (ST_RUN, ST_FAILED):
                break
            self._pump(deadline)
            if self.state == ST_SCANNING:
                self._scan_maybe_probe()
        return self.state == ST_RUN

    def _pump(self, deadline):
        import select

        remaining = max(0.0, min(0.5, deadline - time.time()))
        r, _, _ = select.select([self.client], [], [], remaining)
        if not r:
            self._on_tick()
            return
        try:
            for rx in self.client.recv_frames():
                self._on_rx(rx)
                if self.state in (ST_RUN, ST_FAILED):
                    return
        except ConnectionError as e:
            self.log("medium: %s" % e)
            self.state = ST_FAILED

    # ------------------------------------------------------------ scan
    def _enter_scan(self):
        self.state = ST_SCANNING
        self._scan_deadline = time.time() + self.scan_time
        self._last_probe = 0.0
        self.log("scan: looking for ESSID %r" % self.ssid.decode(errors="replace"))

    def _scan_maybe_probe(self):
        now = time.time()
        if now - self._last_probe > 0.5:
            self._last_probe = now
            probe = dot11.build_probe_req(self.mac, self.ssid,
                                          self.seq.next())
            self.client.send_frame(probe, self.mac, channel_freq=0,
                                   tsf_us=_now_us())
        if now > self._scan_deadline and self.bssid is None:
            self.log("scan: ESSID %r not found" % self.ssid.decode(
                errors="replace"))
            self.state = ST_FAILED

    def _on_tick(self):
        if self.state == ST_SCANNING:
            self._scan_maybe_probe()

    # ------------------------------------------------------------ RX
    def _on_rx(self, rx):
        frame = rx.frame
        if len(frame) < 2:
            return
        if dot11.is_beacon_or_probe_resp(frame):
            self._observe_beacon(rx)
            return
        t, s = dot11.frame_type_subtype(frame)
        if t == dot11.FTYPE_MGMT:
            self._on_mgmt(s, frame)
        elif t == dot11.FTYPE_DATA:
            self._on_data(frame)

    def _observe_beacon(self, rx):
        try:
            b = dot11.Beacon(rx.frame, rx.channel_freq)
        except Exception:
            return
        if not b.ssid:
            return
        key = b.bssid
        if key not in self._bss_seen:
            self._bss_seen[key] = b
            self.log("scan: saw '%s' bssid=%s ch=%d%s" % (
                b.ssid.decode(errors="replace"), dot11.mac_str(b.bssid),
                dot11.freq_to_chan(b.channel_freq),
                " [WPA2]" if b.privacy else " [open]"))
        if self.state == ST_SCANNING and b.ssid == self.ssid:
            self._select_bss(b)

    def _select_bss(self, b):
        self.bssid = b.bssid
        self.channel_freq = b.channel_freq or dot11.chan_to_freq(6)
        secured = b.privacy or b.rsn is not None
        if secured and not self.passphrase:
            self.log("connect: '%s' is encrypted but no passphrase given" %
                     self.ssid.decode(errors="replace"))
            self.state = ST_FAILED
            return
        if self.passphrase and not secured:
            self.log("connect: '%s' is open; ignoring passphrase" %
                     self.ssid.decode(errors="replace"))
            self.passphrase = None
        # Pin the channel: from here we hear (and are heard on) it only.
        self.client.set_channel(self.channel_freq)
        self.log("connect: joining '%s' on channel %d (%s)" % (
            self.ssid.decode(errors="replace"),
            dot11.freq_to_chan(self.channel_freq),
            "WPA2-PSK" if self.passphrase else "open"))
        self._send_auth()

    # ----------------------------------------------------- auth / assoc
    def _send_auth(self):
        self.state = ST_AUTH
        self._tx_mgmt(dot11.build_auth(self.mac, self.bssid,
                                       self.seq.next()))

    def _send_assoc(self):
        self.state = ST_ASSOC
        self._tx_mgmt(dot11.build_assoc_req(
            self.mac, self.bssid, self.ssid, self.channel_freq,
            self.seq.next(), protected=bool(self.passphrase)))

    def _on_mgmt(self, subtype, frame):
        # Only frames addressed to us matter.
        if bytes(frame[4:10]) != self.mac:
            return
        if subtype == dot11.STYPE_AUTH and self.state == ST_AUTH:
            status = frame[28] | (frame[29] << 8)
            if status == dot11.STATUS_SUCCESS:
                self.log("auth: open-system OK")
                self._send_assoc()
            else:
                self.log("auth: rejected (status %d)" % status)
                self.state = ST_FAILED
        elif subtype == dot11.STYPE_ASSOC_RESP and self.state == ST_ASSOC:
            status = frame[26] | (frame[27] << 8)
            if status != dot11.STATUS_SUCCESS:
                self.log("assoc: rejected (status %d)" % status)
                self.state = ST_FAILED
                return
            self.log("assoc: associated (aid assigned)")
            if self.passphrase:
                self._start_handshake()
            else:
                self._connected()
        elif subtype == dot11.STYPE_DEAUTH:
            self.log("link: deauthenticated by AP")
            self.state = ST_FAILED

    def _start_handshake(self):
        self.state = ST_HANDSHAKE
        self.sup = Supplicant(self._pmk, self.mac, self.bssid,
                              dot11.rsn_ie_ccmp_psk(), log=self.log)
        self.log("4way: waiting for EAPOL msg1")

    def _connected(self):
        self.state = ST_RUN
        self.log("link: connected")

    # ------------------------------------------------------------ data
    def _on_data(self, frame):
        # Decrypt if protected and we hold keys.
        if frame[1] & dot11.FC1_PROTECTED:
            aes = self._rx_key_for(frame)
            if aes is None:
                return
            out = crypto.ccmp_decrypt(aes, frame)
            if out is None:
                return
            frame = out[0]
        parsed = dot11.parse_data_frame(frame)
        if parsed is None:
            return
        src, dst, ethertype, sdu = parsed
        if ethertype == dot11.ETH_P_PAE:
            self._on_eapol(src, sdu)
            return
        if self.state == ST_RUN and self.on_eth_rx:
            self.on_eth_rx(src, dst, ethertype, sdu)

    def _rx_key_for(self, frame):
        # Group-addressed (multicast/broadcast) RA -> group key, else pairwise.
        ra = frame[4:10]
        if ra[0] & 0x01:
            return self.keys.gtk_aes
        return self.keys.ptk_aes

    def _on_eapol(self, src, eapol):
        if self.sup is None:
            return
        try:
            resp = self.sup.handle(eapol)
        except HandshakeError as e:
            self.log("4way: %s" % e)
            self.state = ST_FAILED
            return
        if resp is not None:
            # EAPOL is sent as an unprotected data frame to the AP.
            frame = dot11.build_data_frame(self.bssid, self.mac, self.bssid,
                                           (dot11.ETH_P_PAE, resp),
                                           protected=False,
                                           seq=self.seq.next() >> 4)
            self.client.send_frame(frame, self.mac, tsf_us=_now_us())
        if self.sup.completed and self.keys.ptk_aes is None:
            self.keys.ptk_aes = crypto.AES128(self.sup.tk)
            if self.sup.gtk:
                self.keys.gtk_aes = crypto.AES128(self.sup.gtk[:16])
            self._connected()

    # ------------------------------------------------------------ run loop
    def poll(self, timeout=0.2):
        """Service the medium once (for use inside a host's event loop)."""
        import select

        r, _, _ = select.select([self.client], [], [], timeout)
        if not r:
            return
        try:
            for rx in self.client.recv_frames():
                self._on_rx(rx)
        except ConnectionError as e:
            self.log("medium: %s" % e)
            self.state = ST_FAILED

    def deauth(self, reason=3):
        if self.bssid:
            self._tx_mgmt(dot11.build_deauth(self.mac, self.bssid, reason,
                                             self.seq.next()))


def random_mac():
    """A locally-administered, unicast random MAC (02:xx:...)."""
    b = bytearray(os.urandom(6))
    b[0] = (b[0] & 0xFE) | 0x02
    return bytes(b)


def _now_us():
    return int(time.monotonic() * 1_000_000) & 0xFFFFFFFFFFFFFFFF
