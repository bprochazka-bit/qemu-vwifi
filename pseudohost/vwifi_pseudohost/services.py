#
# vwifi-pseudohost — service framework
#
# SPDX-License-Identifier: GPL-2.0-or-later
#
# A pseudo-host is only convincing if it answers on the ports the real
# device would.  This is the seam where that behaviour plugs in: a
# device profile lists Service objects, and each Service binds itself to
# the host's netstack and reacts to traffic.
#
# Phase 1 wires the UDP path end-to-end, because the netstack speaks UDP
# today.  The Service base class already declares tcp_ports so a profile
# can advertise, say, IPP/631 or SMB/445 now; those ports become live
# once the TCP layer lands (a later phase), and nothing about the
# profiles has to change when it does.  Designing the registry around
# "a service owns some ports and some behaviour" — rather than around
# UDP specifically — is what makes that later phase additive.
#
#   class Service            base: name, udp_ports, tcp_ports, hooks
#   class UDPService         convenience base for request/reply UDP
#   ServiceRegistry          binds a profile's services to one host
#
# Concrete examples live at the bottom (a discovery-style UDP responder);
# richer ones (mDNS, IPP, SMB, HTTP) are the natural next contributions
# and each is just another Service subclass.
#


class Service:
    """One network-visible behaviour of a pseudo-host.

    Subclasses set a name and the ports they own, then react to traffic.
    A Service is bound to exactly one host; `self.host` and
    `self.stack` are available after bind().
    """

    name = "service"
    udp_ports = ()
    tcp_ports = ()          # advertised now, serviced when TCP lands

    def __init__(self):
        self.host = None
        self.stack = None

    def bind(self, host):
        self.host = host
        self.stack = host.stack
        for port in self.udp_ports:
            self.stack.register_udp(port, self._udp_trampoline(port))
        self.on_start()

    def _udp_trampoline(self, port):
        def cb(src_ip, src_port, dst_ip, dst_port, payload):
            self.on_udp(src_ip, src_port, dst_ip, dst_port, payload)
        return cb

    # -- hooks a subclass overrides ---------------------------------------
    def on_start(self):
        pass

    def on_udp(self, src_ip, src_port, dst_ip, dst_port, payload):
        pass

    def tick(self):
        """Called periodically from the host loop (announcements, etc.)."""
        pass

    # -- convenience -------------------------------------------------------
    def reply_udp(self, dst_ip, dst_port, payload, src_port):
        self.stack.send_udp(dst_ip, dst_port, payload, src_port=src_port)

    def log(self, msg):
        if self.host:
            self.host.log("svc[%s]: %s" % (self.name, msg))


class UDPService(Service):
    """Request/reply UDP: override handle() and return bytes to answer."""

    port = None

    def __init__(self):
        super().__init__()
        if self.port is not None and not self.udp_ports:
            self.udp_ports = (self.port,)

    def on_udp(self, src_ip, src_port, dst_ip, dst_port, payload):
        reply = self.handle(src_ip, src_port, payload)
        if reply is not None:
            self.reply_udp(src_ip, src_port, reply, src_port=dst_port)

    def handle(self, src_ip, src_port, payload):
        return None


class ServiceRegistry:
    """Owns a host's live services and fans lifecycle/tick out to them."""

    def __init__(self, host):
        self.host = host
        self.services = []

    def add(self, service):
        # Accept a Service instance, a Service subclass, or a zero-arg
        # factory (a lambda) that produces one — profiles use all three.
        if isinstance(service, type):
            service = service()
        elif not isinstance(service, Service) and callable(service):
            service = service()
        service.bind(self.host)
        self.services.append(service)
        return service

    def tick(self):
        for s in self.services:
            s.tick()

    def open_ports(self):
        """(proto, port) the host advertises — useful for a scan summary."""
        out = []
        for s in self.services:
            out += [("udp", p) for p in s.udp_ports]
            out += [("tcp", p) for p in s.tcp_ports]
        return out


# ---------------------------------------------------------------------------
# Example services.
# ---------------------------------------------------------------------------

class UDPEchoService(UDPService):
    """Echo any datagram back to its sender.

    Trivial, but it exercises the whole path — a peer on the network can
    `echo | nc -u <ip> 7` and get its bytes back, proving the pseudo-host
    is really reachable at L4, not just answering ping.
    """

    name = "udp-echo"
    port = 7

    def handle(self, src_ip, src_port, payload):
        self.log("echo %d bytes to %s:%d" % (len(payload),
                                             _ip(src_ip), src_port))
        return payload


class HostnameBeaconService(Service):
    """Announce the host's name on a UDP port at a slow interval.

    A stand-in for the announce/advertise half of a discovery protocol
    (mDNS, NBNS, SSDP): it shows how a Service uses tick() to originate
    traffic rather than only answering it.  Off by default — a profile
    opts in — because a chatty beacon is not what every device would do.
    """

    name = "hostname-beacon"
    udp_ports = ()

    def __init__(self, port=5355, period=10.0):
        super().__init__()
        self.port = port
        self.period = period
        self._next = 0.0

    def tick(self):
        import time
        if self.stack.ip is None:
            return
        now = time.monotonic()
        if now < self._next:
            return
        self._next = now + self.period
        msg = b"pseudohost:" + self.host.hostname.encode()
        self.stack.send_udp("255.255.255.255", self.port, msg,
                            src_port=self.port)


def _ip(b):
    return ".".join(str(x) for x in b)
