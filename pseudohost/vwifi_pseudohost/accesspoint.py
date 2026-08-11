#
# vwifi-pseudohost — a simulated access point
#
# SPDX-License-Identifier: GPL-2.0-or-later
#
# The AP side of the medium: a peer that beacons a network and lets
# pseudo-hosts (or real VMs) associate to it.  Give it an ESSID, a
# channel, an encryption type and optionally a BSSID, and it:
#
#   - beacons the network and answers probe requests;
#   - runs open-system Auth and Assoc;
#   - for WPA2-PSK, runs the Authenticator side of the four-way handshake
#     per station (authenticator.py) and installs per-station CCMP keys;
#   - is the DS: it terminates IP for its own gateway address (ARP, ICMP,
#     a DHCP *server*, and any TCP/UDP services), and bridges frames
#     between associated stations.
#
# It reuses the same crypto, 802.11 and medium code the station side
# uses, so a pseudo-host and a pseudo-AP interoperate for real — see
# tests/test_ap_integration.py, which associates one to the other over an
# in-process hub and pulls a DHCP lease across the link.
#
import os
import time

from . import ieee80211 as dot11
from . import crypto
from .authenticator import Authenticator
from .dhcp_server import DHCPServer
from .medium import MediumClient
from .netstack import NetStack
from .services import ServiceRegistry
from .supplicant import HandshakeError
from .tcp import TCPStack

BROADCAST = dot11.BROADCAST


class _STA:
    """Per-associated-station state."""

    __slots__ = ("mac", "aid", "auth", "ptk_aes", "tx_pn", "keyed")

    def __init__(self, mac, aid):
        self.mac = mac
        self.aid = aid
        self.auth = None
        self.ptk_aes = None
        self.tx_pn = bytes(6)
        self.keyed = False

    def next_pn(self):
        self.tx_pn = crypto.pn_increment(self.tx_pn)
        return self.tx_pn


class _APLink:
    """Adapter that lets a NetStack drive the AP's DS.

    Presents the small interface NetStack expects of a "station" — a MAC
    and send_eth(dst, ethertype, sdu), plus an assignable on_eth_rx — and
    turns egress into FromDS frames on the medium.
    """

    def __init__(self, ap):
        self.ap = ap
        self.mac = ap.bssid
        self.on_eth_rx = None

    def send_eth(self, dst, ethertype, sdu):
        self.ap._tx_frame(bytes(dst), self.ap.bssid, ethertype, sdu)


class PseudoAP:
    def __init__(self, sock_path, essid, channel=6, encryption="open",
                 passphrase=None, bssid=None, node_id=None,
                 gateway_ip="192.168.4.1", netmask="255.255.255.0",
                 pool=("192.168.4.100", "192.168.4.200"), dns=None,
                 services=(), log=None, verbose=False):
        self.sock_path = sock_path
        self.essid = essid.encode() if isinstance(essid, str) else bytes(essid)
        self.channel = channel
        self.freq = dot11.chan_to_freq(channel)
        self.encryption = (encryption or "open").lower()
        self.secured = self.encryption not in ("open", "none", "")
        self.passphrase = passphrase
        self.log = log or _stderr_log
        self.verbose = verbose

        if self.secured and not passphrase:
            raise ValueError("encryption %r needs a passphrase" % encryption)

        self.bssid = dot11.mac_bytes(bssid) if bssid else _random_bssid()
        self.node_id = node_id or ("ap-%s-%s" % (
            self.essid.decode(errors="replace")[:8], self.bssid.hex()[-4:]))

        self.pmk = (crypto.wpa_pmk(passphrase, self.essid)
                    if self.secured else None)
        self.gtk = os.urandom(16)
        self.gtk_key_id = 1
        self.gtk_aes = crypto.AES128(self.gtk) if self.secured else None
        self.group_pn = bytes(6)

        self.client = MediumClient(sock_path, node_id=self.node_id)
        self.link = _APLink(self)
        self.stack = NetStack(self.link, hostname=self.node_id, log=self._slog)
        self.stack.configure(gateway_ip, netmask, gateway_ip, dns)
        self.tcp = TCPStack(self.stack, log=self._slog)
        self.dhcp = DHCPServer(self.stack, gateway_ip, netmask=netmask,
                               gateway=gateway_ip, dns=dns,
                               pool_start=pool[0], pool_end=pool[1],
                               log=self._slog)
        self.registry = ServiceRegistry(self)
        self._service_specs = list(services)

        self.stations = {}
        self._next_aid = 1
        self.seq = dot11.SeqCounter()
        self._running = False
        self._last_beacon = 0.0

    # ---- logging --------------------------------------------------------
    def _slog(self, msg):
        self.log("[%s] %s" % (self.node_id, msg))

    @property
    def hostname(self):
        return self.node_id

    # ---- lifecycle ------------------------------------------------------
    def start(self):
        self.client.connect()
        self.client.send_hello()
        self.client.set_channel(self.freq)
        self._slog("beaconing '%s' on ch %d (%s) bssid=%s" % (
            self.essid.decode(errors="replace"), self.channel,
            self.encryption if self.secured else "open",
            dot11.mac_str(self.bssid)))
        for svc in self._service_specs:
            self.registry.add(svc)
        self._running = True
        self._send_beacon()

    def run(self, duration=None):
        deadline = None if duration is None else time.time() + duration
        while self._running:
            if deadline is not None and time.time() > deadline:
                break
            self._pump(0.1)
            now = time.time()
            if now - self._last_beacon > 0.1:
                self._send_beacon()
                self._last_beacon = now
            self.registry.tick()

    def stop(self):
        self._running = False

    def close(self):
        self.client.close()

    def _pump(self, timeout):
        import select
        r, _, _ = select.select([self.client], [], [], timeout)
        if not r:
            return
        try:
            for rx in self.client.recv_frames():
                self._on_rx(rx)
        except ConnectionError as e:
            self._slog("medium: %s" % e)
            self._running = False

    # ---- beacon ---------------------------------------------------------
    def _send_beacon(self):
        frame = dot11.build_beacon(self.bssid, self.essid, self.freq,
                                   self.secured, self.seq.next())
        self.client.send_frame(frame, self.bssid)
        self._last_beacon = time.time()

    # ---- RX -------------------------------------------------------------
    def _on_rx(self, rx):
        frame = rx.frame
        if len(frame) < 2:
            return
        t, s = dot11.frame_type_subtype(frame)
        if t == dot11.FTYPE_MGMT:
            self._on_mgmt(s, frame)
        elif t == dot11.FTYPE_DATA:
            self._on_data(frame)

    def _on_mgmt(self, subtype, frame):
        if len(frame) < 24:
            return
        da = bytes(frame[4:10])
        sa = bytes(frame[10:16])
        if subtype == dot11.STYPE_PROBE_REQ:
            if self._probe_matches(frame):
                self._tx_mgmt(dot11.build_beacon(
                    self.bssid, self.essid, self.freq, self.secured,
                    self.seq.next(), da=sa,
                    subtype=dot11.STYPE_PROBE_RESP))
            return
        # Auth/Assoc/Deauth must be addressed to us.
        if da != self.bssid:
            return
        if subtype == dot11.STYPE_AUTH:
            self._tx_mgmt(dot11.build_auth_resp(self.bssid, sa,
                                                self.seq.next()))
        elif subtype == dot11.STYPE_ASSOC_REQ:
            self._assoc(sa)
        elif subtype == dot11.STYPE_DEAUTH:
            self.stations.pop(sa, None)

    def _probe_matches(self, frame):
        if len(frame) <= 24:
            return True
        ies = dot11.parse_ies(frame[24:])
        ssid = ies.get(dot11.EID_SSID, b"")
        return ssid in (b"", self.essid)

    def _assoc(self, sta_mac):
        sta = self.stations.get(sta_mac)
        if sta is None:
            sta = _STA(sta_mac, self._next_aid)
            self._next_aid += 1
            self.stations[sta_mac] = sta
        self._tx_mgmt(dot11.build_assoc_resp(self.bssid, sta_mac, sta.aid,
                                             self.seq.next()))
        self._slog("assoc: %s (aid %d)" % (dot11.mac_str(sta_mac), sta.aid))
        if self.secured:
            sta.auth = Authenticator(self.pmk, self.bssid, sta_mac, self.gtk,
                                     gtk_key_id=self.gtk_key_id,
                                     rsn_ie=dot11.rsn_ie_ccmp_psk(),
                                     log=self._slog)
            self._tx_eapol(sta_mac, sta.auth.start())
        else:
            sta.keyed = True

    # ---- data / EAPOL ---------------------------------------------------
    def _on_data(self, frame):
        ta = bytes(frame[10:16])            # addr2 = transmitter (STA)
        sta = self.stations.get(ta)
        if frame[1] & dot11.FC1_PROTECTED:
            if sta is None or sta.ptk_aes is None:
                if self.verbose:
                    self._slog("rx: protected data from %s but no key yet"
                               % dot11.mac_str(ta))
                return
            out = crypto.ccmp_decrypt(sta.ptk_aes, frame)
            if out is None:
                if self.verbose:
                    _t, _s = dot11.frame_type_subtype(frame)
                    self._slog("rx: CCMP decrypt FAILED from %s (subtype %d, "
                               "%d bytes)" % (dot11.mac_str(ta), _s, len(frame)))
                return
            frame = out[0]
        parsed = dot11.parse_data_frame(frame)
        if parsed is None:
            if self.verbose:
                self._slog("rx: unparseable data from %s" % dot11.mac_str(ta))
            return
        sa, da, ethertype, sdu = parsed
        if self.verbose:
            self._slog("rx data %s -> %s ethertype 0x%04x (%d bytes)" % (
                dot11.mac_str(sa), dot11.mac_str(da), ethertype, len(sdu)))
        if ethertype == dot11.ETH_P_PAE:
            self._on_eapol(ta, sdu)
            return
        group = (da == BROADCAST) or bool(da[0] & 0x01)
        if group:
            self.link.on_eth_rx(sa, da, ethertype, sdu)      # our stack
            self._tx_frame(da, sa, ethertype, sdu)           # flood (GTK)
        elif da == self.bssid:
            self.link.on_eth_rx(sa, da, ethertype, sdu)
        elif da in self.stations:
            self._tx_frame(da, sa, ethertype, sdu)           # bridge STA->STA
        else:
            self.link.on_eth_rx(sa, da, ethertype, sdu)      # act as router

    def _on_eapol(self, sta_mac, eapol):
        sta = self.stations.get(sta_mac)
        if sta is None or sta.auth is None:
            return
        try:
            resp = sta.auth.handle(eapol)
        except HandshakeError as e:
            self._slog("4way(%s): %s" % (dot11.mac_str(sta_mac), e))
            self.stations.pop(sta_mac, None)
            return
        if resp is not None:
            self._tx_eapol(sta_mac, resp)
        if sta.auth.completed and sta.ptk_aes is None:
            sta.ptk_aes = crypto.AES128(sta.auth.tk)
            sta.keyed = True
            self._slog("4way(%s): keyed" % dot11.mac_str(sta_mac))

    # ---- TX -------------------------------------------------------------
    def _tx_mgmt(self, frame):
        self.client.send_frame(frame, self.bssid)

    def _tx_eapol(self, sta_mac, eapol):
        # EAPOL-Key frames are always sent unprotected at the 802.11 layer.
        self._tx_frame(sta_mac, self.bssid, dot11.ETH_P_PAE, eapol,
                       protected=False)

    def _tx_frame(self, da, sa, ethertype, sdu, protected=None):
        group = (da == BROADCAST) or bool(da[0] & 0x01)
        if protected is None:
            if not self.secured:
                protected = False
            elif group:
                protected = True                 # GTK always available
            else:
                sta = self.stations.get(da)
                protected = bool(sta and sta.keyed)
        frame = dot11.build_data_fromds(self.bssid, sa, da,
                                        (ethertype, sdu), protected=protected,
                                        seq=self.seq.next() >> 4)
        if protected:
            if group:
                self.group_pn = crypto.pn_increment(self.group_pn)
                frame = crypto.ccmp_encrypt(self.gtk_aes, frame,
                                            self.group_pn, self.gtk_key_id)
            else:
                sta = self.stations.get(da)
                if sta is None or sta.ptk_aes is None:
                    return
                frame = crypto.ccmp_encrypt(sta.ptk_aes, frame,
                                            sta.next_pn(), 0)
        self.client.send_frame(frame, self.bssid)


def _random_bssid():
    b = bytearray(os.urandom(6))
    b[0] = (b[0] & 0xFE) | 0x02              # locally administered, unicast
    return bytes(b)


def _stderr_log(msg):
    import sys
    print(msg, file=sys.stderr, flush=True)
