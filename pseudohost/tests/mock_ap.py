#
# vwifi-pseudohost — in-process mock hub + AP for integration tests
#
# SPDX-License-Identifier: GPL-2.0-or-later
#
# A single thread that plays both roles the real lab splits between
# vwifi-medium and hostapd: it owns a Unix socket the pseudo-host
# connects to, speaks the v2 medium framing, and answers 802.11 like an
# AP — beacons, probe/auth/assoc responses, the WPA2 four-way handshake,
# and a one-lease DHCP server.  It exists so the whole station + stack +
# service path can be exercised with no QEMU, no kernel, and no radio.
#
# It is a test double, not a second implementation of the AP: it does
# only what these tests need and takes shortcuts a real AP could not
# (DHCP replies go out unicast, for instance).  crypto/ieee80211 are the
# real modules, so the wire formats it produces are the ones the station
# must handle in the field.
#
import os
import socket
import struct
import tempfile
import threading
import time

import sys
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from vwifi_pseudohost import crypto                # noqa: E402
from vwifi_pseudohost import ieee80211 as dot11    # noqa: E402
from vwifi_pseudohost import medium as med         # noqa: E402
from vwifi_pseudohost import supplicant as sup      # noqa: E402


class MockAP(threading.Thread):
    def __init__(self, ssid="Lab-AP-1", passphrase=None,
                 bssid="02:11:22:33:44:00", channel=6,
                 lease_ip="192.168.9.100", server_ip="192.168.9.1"):
        super().__init__(daemon=True)
        self.ssid = ssid.encode() if isinstance(ssid, str) else ssid
        self.passphrase = passphrase
        self.bssid = dot11.mac_bytes(bssid)
        self.freq = dot11.chan_to_freq(channel)
        self.lease_ip = tuple(int(x) for x in lease_ip.split("."))
        self.server_ip = tuple(int(x) for x in server_ip.split("."))
        self.pmk = crypto.wpa_pmk(passphrase, self.ssid) if passphrase else None

        d = tempfile.mkdtemp(prefix="vwifi-mock-")
        self.sock_path = os.path.join(d, "hub.sock")
        self._srv = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        self._srv.bind(self.sock_path)
        self._srv.listen(1)
        self._stop = threading.Event()

        self.sta_mac = None
        self.seq = 0
        self.tx_pn = bytes(6)
        self.tk_aes = None
        self.gtk = os.urandom(16)
        self.anonce = os.urandom(32)
        self.kck = self.kek = None
        self.keyed = False
        self.saw_dhcp_ack = False

    # -- lifecycle ---------------------------------------------------------
    def stop(self):
        self._stop.set()

    def run(self):
        try:
            self._serve()
        finally:
            self._srv.close()

    def _serve(self):
        self._srv.settimeout(0.2)
        conn = None
        while not self._stop.is_set() and conn is None:
            try:
                conn, _ = self._srv.accept()
            except socket.timeout:
                continue
        if conn is None:
            return
        conn.settimeout(0.1)
        buf = bytearray()
        last_beacon = 0.0
        got_hello = False
        try:
            while not self._stop.is_set():
                now = time.monotonic()
                if got_hello and now - last_beacon > 0.1:
                    self._send_beacon(conn)
                    last_beacon = now
                try:
                    data = conn.recv(65536)
                    if not data:
                        break
                    buf += data
                except socket.timeout:
                    continue
                got_hello = self._drain(conn, buf, got_hello)
        finally:
            conn.close()

    # -- framing -----------------------------------------------------------
    def _drain(self, conn, buf, got_hello):
        while len(buf) >= 4:
            (mlen,) = struct.unpack_from("!I", buf, 0)
            if len(buf) < 4 + mlen:
                break
            msg = bytes(buf[4 : 4 + mlen])
            del buf[: 4 + mlen]
            if not got_hello:
                # First message is the hello (magic then node_id).
                if len(msg) >= 4 and struct.unpack_from("<I", msg, 0)[0] == \
                        med.HELLO_MAGIC:
                    got_hello = True
                    continue
            self._on_medium_msg(conn, msg)
        return got_hello

    def _send_medium(self, conn, frame, tx_mac):
        hdr = med._HDR.pack(med.VWIFI_MAGIC, 2, len(frame), bytes(tx_mac),
                            med.DEFAULT_RATE, med.DEFAULT_RSSI, 0, 0, 0,
                            self.freq, med.CHAN_FLAG_2GHZ | med.CHAN_FLAG_HT20,
                            0, 0, 0, b"\x00\x00")
        msg = hdr + frame
        conn.sendall(struct.pack("!I", len(msg)) + msg)

    def _next_seq(self):
        s = (self.seq & 0x0FFF) << 4
        self.seq += 1
        return s

    def _on_medium_msg(self, conn, msg):
        rx = med.MediumClient._parse(msg)
        if rx is None or len(rx.frame) < 2:
            return
        self.sta_mac = self.sta_mac or rx.tx_mac
        frame = rx.frame
        t, s = dot11.frame_type_subtype(frame)
        if t == dot11.FTYPE_MGMT:
            self._on_mgmt(conn, s, frame)
        elif t == dot11.FTYPE_DATA:
            self._on_data(conn, frame)

    # -- 802.11 management -------------------------------------------------
    def _beacon_frame(self, subtype):
        b = dot11.mgmt_header(subtype, self.sta_mac or dot11.BROADCAST,
                              self.bssid, self.bssid, self._next_seq())
        # ESS + short preamble + short slot; Privacy bit only when secured.
        cap = 0x0421 | (0x0010 if self.passphrase else 0)
        b += struct.pack("<QHH", 0, 100, cap)
        b += dot11.ie(dot11.EID_SSID, self.ssid)
        b += dot11.supp_rate_ies()
        b += dot11.ie(dot11.EID_DS_PARAMS, bytes([dot11.freq_to_chan(self.freq)]))
        if self.passphrase:
            b += dot11.rsn_ie_ccmp_psk()
        return bytes(b)

    def _send_beacon(self, conn):
        self._send_medium(conn, self._beacon_frame(dot11.STYPE_BEACON),
                          self.bssid)

    def _on_mgmt(self, conn, subtype, frame):
        da = bytes(frame[4:10])
        sa = bytes(frame[10:16])
        self.sta_mac = sa
        if subtype == dot11.STYPE_PROBE_REQ:
            self._send_medium(conn,
                              self._beacon_frame(dot11.STYPE_PROBE_RESP),
                              self.bssid)
        elif subtype == dot11.STYPE_AUTH:
            resp = dot11.mgmt_header(dot11.STYPE_AUTH, sa, self.bssid,
                                     self.bssid, self._next_seq())
            resp += struct.pack("<HHH", 0, 2, 0)          # open, seq2, success
            self._send_medium(conn, bytes(resp), self.bssid)
        elif subtype == dot11.STYPE_ASSOC_REQ:
            resp = dot11.mgmt_header(dot11.STYPE_ASSOC_RESP, sa, self.bssid,
                                     self.bssid, self._next_seq())
            resp += struct.pack("<HHH", 0x0431, 0, 1)      # cap, status, AID
            resp += dot11.supp_rate_ies()
            self._send_medium(conn, bytes(resp), self.bssid)
            if self.passphrase:
                self._send_eapol_m1(conn)

    # -- data / EAPOL / DHCP ----------------------------------------------
    def _tx_data(self, conn, da, ethertype, sdu, protected):
        b = bytearray(24)
        b[0] = dot11.FTYPE_DATA << 2
        b[1] = dot11.FC1_FROMDS | (dot11.FC1_PROTECTED if protected else 0)
        b[4:10] = da                                       # addr1 = DA (STA)
        b[10:16] = self.bssid                              # addr2 = TA
        b[16:22] = self.bssid                              # addr3 = SA (from DS)
        struct.pack_into("<H", b, 22, self._next_seq())
        b += dot11.LLC_SNAP + struct.pack(">H", ethertype) + sdu
        frame = bytes(b)
        if protected and self.tk_aes is not None:
            self.tx_pn = crypto.pn_increment(self.tx_pn)
            frame = crypto.ccmp_encrypt(self.tk_aes, frame, self.tx_pn, 0)
        self._send_medium(conn, frame, self.bssid)

    def _send_eapol_m1(self, conn):
        info = 2 | sup.KI_TYPE_PAIRWISE | sup.KI_ACK
        f = bytearray(sup._OFF_KEYDATA)
        f[0] = sup.EAPOL_VERSION
        f[1] = sup.EAPOL_TYPE_KEY
        struct.pack_into(">H", f, 2, 95)
        f[4] = sup.KEY_DESC_RSN
        struct.pack_into(">H", f, sup._OFF_KEYINFO, info)
        struct.pack_into(">H", f, 7, 16)
        f[9:17] = struct.pack(">Q", 1)
        f[sup._OFF_NONCE : sup._OFF_NONCE + 32] = self.anonce
        struct.pack_into(">H", f, sup._OFF_KDLEN, 0)
        self._tx_data(conn, self.sta_mac, dot11.ETH_P_PAE, bytes(f), False)

    def _send_eapol_m3(self, conn, snonce):
        self.kck, self.kek, tk = crypto.ptk_from_pmk(
            self.pmk, self.bssid, self.sta_mac, self.anonce, snonce)
        self._pending_tk = tk
        kde_body = bytes([0x00, 0x0F, 0xAC, 0x01, 0x01, 0x00]) + self.gtk
        kde = bytes([0xDD, len(kde_body)]) + kde_body
        if len(kde) % 8:
            kde += b"\x00" * (8 - len(kde) % 8)
        wrapped = crypto.aes_key_wrap(self.kek, kde)
        info = (2 | sup.KI_TYPE_PAIRWISE | sup.KI_ACK | sup.KI_MIC
                | sup.KI_INSTALL | sup.KI_SECURE | sup.KI_ENCRYPTED)
        f = bytearray(sup._OFF_KEYDATA + len(wrapped))
        f[0] = sup.EAPOL_VERSION
        f[1] = sup.EAPOL_TYPE_KEY
        struct.pack_into(">H", f, 2, 95 + len(wrapped))
        f[4] = sup.KEY_DESC_RSN
        struct.pack_into(">H", f, sup._OFF_KEYINFO, info)
        struct.pack_into(">H", f, 7, 16)
        f[9:17] = struct.pack(">Q", 2)
        f[sup._OFF_NONCE : sup._OFF_NONCE + 32] = self.anonce
        struct.pack_into(">H", f, sup._OFF_KDLEN, len(wrapped))
        f[sup._OFF_KEYDATA:] = wrapped
        f[sup._OFF_MIC : sup._OFF_MIC + 16] = crypto.eapol_mic(self.kck,
                                                              bytes(f))
        self._tx_data(conn, self.sta_mac, dot11.ETH_P_PAE, bytes(f), False)

    def _on_data(self, conn, frame):
        if frame[1] & dot11.FC1_PROTECTED:
            if self.tk_aes is None:
                return
            out = crypto.ccmp_decrypt(self.tk_aes, frame)
            if out is None:
                return
            frame = out[0]
        parsed = dot11.parse_data_frame(frame)
        if parsed is None:
            return
        src, dst, ethertype, sdu = parsed
        if ethertype == dot11.ETH_P_PAE:
            self._on_eapol(conn, sdu)
        elif ethertype == dot11.ETH_P_IP:
            self._on_ip(conn, sdu)

    def _on_eapol(self, conn, eapol):
        if not sup.Supplicant.is_eapol_key(eapol):
            return
        key_info = struct.unpack_from(">H", eapol, sup._OFF_KEYINFO)[0]
        has_mic = bool(key_info & sup.KI_MIC)
        secure = bool(key_info & sup.KI_SECURE)
        if has_mic and not secure:
            # message 2: derive keys, send message 3
            snonce = bytes(eapol[sup._OFF_NONCE : sup._OFF_NONCE + 32])
            self._send_eapol_m3(conn, snonce)
        elif has_mic and secure:
            # message 4: install keys
            self.tk_aes = crypto.AES128(self._pending_tk)
            self.keyed = True

    # -- DHCP server -------------------------------------------------------
    def _on_ip(self, conn, ip):
        if len(ip) < 20 or ip[9] != 17:                   # UDP only
            return
        ihl = (ip[0] & 0x0F) * 4
        udp = ip[ihl:]
        dport = (udp[2] << 8) | udp[3]
        if dport != 67:
            return
        req = udp[8:]
        xid = req[4:8]
        from vwifi_pseudohost.dhcp import DHCPClient, MAGIC_COOKIE, \
            OPT_MSGTYPE, DISCOVER, REQUEST, OFFER, ACK
        opts = DHCPClient._parse_opts(req[240:])
        mtype = opts.get(OPT_MSGTYPE, b"\x00")[0]
        if mtype == DISCOVER:
            self._dhcp_reply(conn, xid, OFFER)
        elif mtype == REQUEST:
            self._dhcp_reply(conn, xid, ACK)
            self.saw_dhcp_ack = True

    def _dhcp_reply(self, conn, xid, mtype):
        from vwifi_pseudohost.dhcp import (MAGIC_COOKIE, OPT_MSGTYPE,
                                           OPT_SERVER_ID, OPT_SUBNET,
                                           OPT_ROUTER, OPT_DNS, OPT_LEASE,
                                           OPT_END, DHCP_SERVER_PORT,
                                           DHCP_CLIENT_PORT)
        chaddr = bytes(self.sta_mac) + bytes(10)
        hdr = struct.pack(">BBBBIHH4s4s4s4s16s64s128s",
                          2, 1, 6, 0, struct.unpack(">I", xid)[0], 0, 0,
                          bytes(4), bytes(self.lease_ip),
                          bytes(self.server_ip), bytes(4),
                          chaddr, bytes(64), bytes(128))
        opts = bytearray(MAGIC_COOKIE)
        opts += bytes([OPT_MSGTYPE, 1, mtype])
        opts += bytes([OPT_SERVER_ID, 4]) + bytes(self.server_ip)
        opts += bytes([OPT_SUBNET, 4, 255, 255, 255, 0])
        opts += bytes([OPT_ROUTER, 4]) + bytes(self.server_ip)
        opts += bytes([OPT_DNS, 4]) + bytes(self.server_ip)
        opts += bytes([OPT_LEASE, 4, 0, 0, 0x0e, 0x10])
        opts += bytes([OPT_END])
        bootp = hdr + bytes(opts)
        udp = struct.pack(">HHHH", DHCP_SERVER_PORT, DHCP_CLIENT_PORT,
                          8 + len(bootp), 0) + bootp
        total = 20 + len(udp)
        iph = bytearray(struct.pack(">BBHHHBBH4s4s", 0x45, 0, total, 0, 0,
                                    64, 17, 0, bytes(self.server_ip),
                                    bytes(self.lease_ip)))
        # unicast reply to the STA, encrypted if we're keyed
        self._tx_data(conn, self.sta_mac, dot11.ETH_P_IP, bytes(iph) + udp,
                      protected=self.keyed)
