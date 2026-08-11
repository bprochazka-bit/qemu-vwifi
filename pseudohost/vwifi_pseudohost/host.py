#
# vwifi-pseudohost — the inheritable PseudoHost base class
#
# SPDX-License-Identifier: GPL-2.0-or-later
#
# This is the class the whole package exists to provide.  A PseudoHost
# is a complete virtual machine-less host on the vwifi medium: it owns a
# radio (Station), an IP stack (NetStack), a DHCP client, and a set of
# Services.  Subclass it to describe a *kind* of device — a Linux
# workstation, a Windows box, a printer, a NAS — by setting a few class
# attributes and listing the services that device would run.
#
# The contract for a subclass:
#
#   class MyDevice(PseudoHost):
#       persona    = "linux-workstation"   # label, and the default OS
#       os_ttl     = 64                    # IPv4 TTL fingerprint
#       hostname   = "ws-linux"            # default host name
#       mac_oui    = None                  # 3-byte OUI to seed the MAC
#       services   = [SomeService, ...]    # classes or instances
#       icmp       = True                  # answer ping
#
#       def on_connected(self): ...        # optional lifecycle hooks
#       def on_lease(self, lease): ...
#
# Everything below persona/services is machinery: the base class turns
# those declarations into a running host and pumps a single event loop.
#
import os
import time

from . import ieee80211 as dot11
from . import station as station_mod
from .dhcp import DHCPClient
from .medium import MediumClient
from .netstack import NetStack
from .services import ServiceRegistry
from .tcp import TCPStack


def _stderr_log(msg):
    import sys
    print(msg, file=sys.stderr, flush=True)


class PseudoHost:
    # ---- profile knobs a subclass overrides -----------------------------
    persona = "generic"
    os_ttl = 64
    hostname = "pseudohost"
    mac_oui = None                 # bytes(3) or None for a random 02: MAC
    services = ()                  # Service classes or instances
    icmp = True

    def __init__(self, sock_path, essid, passphrase=None, node_id=None,
                 hostname=None, mac=None, scan_time=3.0, log=None,
                 print_dir=None):
        self.sock_path = sock_path
        self.essid = essid
        self.passphrase = passphrase
        self.hostname = hostname or self.hostname
        self.log = log or _stderr_log

        self.mac = dot11.mac_bytes(mac) if mac else self._make_mac()
        self.node_id = node_id or ("ph-%s-%s" % (
            self.persona, self.mac.hex()[-4:]))

        self.client = MediumClient(sock_path, node_id=self.node_id)
        self.station = station_mod.Station(
            self.client, essid, passphrase=passphrase, mac=self.mac,
            log=self._slog, scan_time=scan_time)
        self.stack = NetStack(self.station, hostname=self.hostname,
                              log=self._slog)
        self.stack.ttl = self.os_ttl
        self.stack.set_icmp_enabled(self.icmp)
        self.tcp = TCPStack(self.stack, log=self._slog)
        self.dhcp = DHCPClient(self.stack, hostname=self.hostname,
                               log=self._slog)
        # Where accepted print jobs go (printer profiles); None -> discard.
        from .printing import PrintSink
        self.print_sink = PrintSink(print_dir, log=self._slog)
        self.registry = ServiceRegistry(self)

        self._running = False
        self._lease_announced = False

    # ---- MAC ------------------------------------------------------------
    def _make_mac(self):
        if self.mac_oui:
            return bytes(self.mac_oui)[:3] + os.urandom(3)
        return station_mod.random_mac()

    # ---- logging --------------------------------------------------------
    def _slog(self, msg):
        self.log("[%s] %s" % (self.node_id, msg))

    # ---- lifecycle ------------------------------------------------------
    def start(self, connect_timeout=20.0):
        """Connect to the medium, join the ESSID, and lease an address.

        Returns True once associated.  On a WPA2 network "associated"
        means keyed.  DHCP then runs asynchronously in run().
        """
        self.client.connect()
        self._slog("persona=%s hostname=%s mac=%s" % (
            self.persona, self.hostname, dot11.mac_str(self.mac)))
        if not self.station.connect(timeout=connect_timeout):
            self._slog("failed to join '%s'" % self.essid)
            return False
        self.on_connected()
        self.dhcp.start()
        for svc in self.services:
            self.registry.add(svc)
        self._running = True
        return True

    def run(self, duration=None):
        """Service the host until stop() or `duration` seconds elapse."""
        deadline = None if duration is None else time.time() + duration
        while self._running:
            if deadline is not None and time.time() > deadline:
                break
            self.station.poll(timeout=0.2)
            if self.station.state == station_mod.ST_FAILED:
                self._slog("link lost")
                break
            self.dhcp.tick()
            self.registry.tick()
            if self.dhcp.bound() and not self._lease_announced:
                self._lease_announced = True
                self.on_lease(self.dhcp.lease)

    def stop(self):
        self._running = False

    def close(self):
        try:
            self.station.deauth()
        except Exception:
            pass
        self.client.close()

    # ---- convenience ----------------------------------------------------
    @property
    def ip(self):
        return self.stack.ip

    def ping(self, dst_ip, **kw):
        self.stack.ping(dst_ip, **kw)

    # ---- hooks a subclass may override ----------------------------------
    def on_connected(self):
        pass

    def on_lease(self, lease):
        from .netstack import ip_str
        self._slog("lease: %s/%s gw=%s dns=%s (persona %s, TTL %d)" % (
            ip_str(lease["ip"]), ip_str(lease["netmask"]),
            ip_str(lease["gateway"]) if lease["gateway"] else "-",
            ip_str(lease["dns"]) if lease["dns"] else "-",
            self.persona, self.os_ttl))
