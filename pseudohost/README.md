# pseudohost — virtual hosts on the vwifi medium, without a VM

A pseudo-host is a peer on the vwifi medium that behaves like a whole
machine — it scans, associates (open or WPA2-PSK), pulls a DHCP lease,
and answers on the network — but it is a single Python process, not a
QEMU guest. Where a VM boots an OS, a kernel driver, and a userspace to
put one station on the air, a pseudo-host puts one on the air directly:
it speaks the [`abi/vwifi.h`](../abi/vwifi.h) medium protocol on one
side and runs a small 802.11 station plus a userspace TCP/IP stack on
the other.

That trade — no kernel, no image, no driver — is what makes it cheap
enough to run *many* of them. The point is not one virtual laptop; it is
a populated network: workstations, printers, a NAS, and whatever else a
real LAN carries, all on one medium, none of them a VM.

```
        hostapd / another VM / the host radio          the AP side
                        │
                 vwifi-medium (the hub)                the shared air
                        │  unix socket, abi/vwifi.h
        ┌───────────────┼──────────────────────────┐
        │               │                          │
   ┌────▼────┐    ┌─────▼─────┐              ┌──────▼───────┐
   │ Linux VM│    │ Windows VM│   ...        │  pseudohost  │  ← this
   │ (ath9k) │    │ (vwifi.sys│              │  (a process) │
   └─────────┘    └───────────┘              └──────────────┘
```

The whole thing is stdlib-only Python. Nothing here needs QEMU, a guest
image, kernel headers, or a pip install — `python3` is the only
requirement, and the AES/CCMP/PBKDF2 the security depends on are bundled
and known-answer tested. (If `cryptography` or PyCryptodome happens to
be installed, the AES block op is borrowed from it for speed; if not,
the pure-Python cipher is used and everything still works.)

## Phase 1 — what it does today

Give it an ESSID and, for a protected network, a passphrase:

```bash
# Start a medium and an AP on it first — see the top-level README and
# examples/hostapd/. Then:

./pseudohost --sock /tmp/vwifi.sock --essid Lab-AP-1 \
             --passphrase correcthorse1
```

and it will:

1. **find the channel** — join the hub, listen on the broadcast channel
   and send probe requests, and read the target network's channel out of
   its beacons;
2. **associate** — pin that channel, do open-system Auth then Assoc,
   carrying the right RSN element for a WPA2 network;
3. **key up** — for WPA2-PSK, run the full four-way handshake
   (PBKDF2 PMK → PTK, EAPOL-Key msg 1–4, GTK unwrap) and install the
   CCMP pairwise and group keys;
4. **lease an address** — run a DHCP DISCOVER/REQUEST and configure the
   stack from the ACK;
5. **stay on the air** — answer ARP and ICMP echo, and run whatever
   services its device profile declares.

Both **open** and **WPA2-PSK/CCMP** networks are supported. The
in-tree lab APs (`examples/hostapd/lab-ap*.conf`) are exactly the
WPA2-PSK/CCMP shape this targets.

```
$ ./pseudohost --sock /tmp/vwifi.sock --essid Lab-AP-1 \
               --passphrase correcthorse1 --profile printer
[ph-network-printer-…] scan: looking for ESSID 'Lab-AP-1'
[ph-network-printer-…] scan: saw 'Lab-AP-1' bssid=02:11:22:33:44:00 ch=6 [WPA2]
[ph-network-printer-…] connect: joining 'Lab-AP-1' on channel 6 (WPA2-PSK)
[ph-network-printer-…] auth: open-system OK
[ph-network-printer-…] assoc: associated (aid assigned)
[ph-network-printer-…] handshake: msg1 rx (ANonce), sending msg2
[ph-network-printer-…] handshake: msg3 rx (GTK), sending msg4 — installed
[ph-network-printer-…] dhcp: bound 192.168.9.100 (lease 3600s)
[ph-network-printer-…] online as 192.168.9.100
```

## The inheritable class

`PseudoHost` is the class the package exists to provide. It owns a
radio, an IP stack, a DHCP client, and a set of services; a **device
profile** is a subclass that describes a *kind* of device by setting a
few attributes and listing its services.

```python
from vwifi_pseudohost.host import PseudoHost
from vwifi_pseudohost.services import UDPService

class SNMPPrinter(PseudoHost):
    persona  = "network-printer"
    os_ttl   = 255                     # embedded-stack TTL fingerprint
    hostname = "HPLJ-PH01"
    mac_oui  = bytes([0x02, 0x60, 0xB0])
    icmp     = True
    services = [PrinterService]        # ports + behaviour

    def on_lease(self, lease):
        super().on_lease(lease)
        # device-specific startup goes here
```

A profile is deliberately thin — a persona is mostly fingerprint knobs
(TTL, MAC OUI, hostname) plus the services the device runs. Adding a new
device type is a dozen lines, not a new stack. The shipped profiles
(`--profile`) are:

| profile      | persona             | TTL | stands in for |
|--------------|---------------------|-----|---------------|
| `generic`    | generic             | 64  | a bare host (ICMP only) |
| `linux`      | linux-workstation   | 64  | a Linux client |
| `windows`    | windows-workstation | 128 | a Windows client |
| `printer`    | network-printer     | 255 | an IPP/JetDirect print server |
| `nas`        | nas                 | 64  | SMB/NFS storage |
| `voip`       | voip-phone          | 64  | a SIP desk phone (answers OPTIONS on 5060) |
| `chromecast` | chromecast          | 64  | a Google Cast receiver (mDNS `_googlecast._tcp`) |
| `hp-mfp`     | hp-mfp              | 255 | an HP print/scan/fax MFP (mDNS `_ipp` + `_uscan`) |

The last three show the service framework doing real work over the live
UDP path: `voip` answers SIP `OPTIONS` with a phone-like `200 OK`, and
`chromecast` and `hp-mfp` run the shared mDNS responder (`mdns.py`) so
they are discoverable by `avahi-browse` / `dns-sd` — a Chromecast as a
Cast receiver, the MFP as both a printer (`_ipp._tcp`) and a scanner
(`_uscan._tcp`), which is what makes it a multifunction rather than a
plain print server. Their richer protocols (Cast/TLS on 8009, IPP,
eSCL, SNMP) are advertised as ports now and become listeners when the
TCP layer lands.

## Services — designed in from the start

The reason for the class hierarchy is that a pseudo-host is only
convincing if it answers on the ports the real device would. The service
framework is the seam where that behaviour plugs in:

```python
class Service:
    name      = "service"
    udp_ports = ()          # live now (the netstack speaks UDP)
    tcp_ports = ()          # advertised now, serviced when TCP lands
    def on_start(self): ...
    def on_udp(self, src_ip, src_port, dst_ip, dst_port, payload): ...
    def tick(self): ...     # for announcements (mDNS/SSDP-style)
```

Phase 1 wires the **UDP** path end to end, so a `UDPService` subclass is
fully live today (`UDPEchoService` is a working example — `nc -u <ip> 7`
and your bytes come back). The framework is built around "a service owns
some ports and some behaviour" rather than around UDP specifically, so
the `tcp_ports` a printer or NAS advertises become real listeners once
the TCP layer lands — a later phase that is purely additive: no profile
and no service signature changes.

That is the deliberate design consequence the request asked us to
consider up front: **the netstack, the station, and the service
framework are three separable layers**, and a device profile only ever
touches the top one. A printer, a NAS, and a Linux box are the same
station and the same IP stack with different services and different
fingerprints.

## Layout

```
pseudohost/
  pseudohost                     the CLI (phase-1 entry point)
  vwifi_pseudohost/
    crypto.py       AES-128, CCMP, PRF, PBKDF2, AES key wrap, EAPOL MIC
    ieee80211.py    802.11 frame + IE build/parse; Ethernet <-> 802.11
    medium.py       the hub transport (abi/vwifi.h wire protocol)
    supplicant.py   the WPA2-PSK four-way handshake
    station.py      scan -> auth -> assoc -> keys; the radio state machine
    netstack.py     ARP / IPv4 / ICMP / UDP; the userspace host stack
    dhcp.py         DHCPv4 client
    mdns.py         multicast-DNS / DNS-SD responder (Chromecast, MFP)
    services.py     Service base class + registry (the extension seam)
    host.py         PseudoHost — the inheritable base class
    profiles/       one subclass per device kind
  tests/            stdlib unittest; run.sh runs them all
```

## Tests

```bash
./tests/run.sh                 # or: make test-pseudohost   (from repo root)
```

The suite needs no medium and no network. It covers the crypto against
known-answer vectors (AES/FIPS-197, PBKDF2/PTK, CCMP round-trip mirroring
`devices/vwifi/tests/crypto.c`, RFC 3394 key wrap), the four-way
handshake against an in-process authenticator, the netstack and DHCP,
and a **full end-to-end connect** — scan, associate, handshake, lease —
against an in-process mock hub+AP over a real Unix socket, for both open
and WPA2 networks (and a wrong-passphrase case that must *fail*).

## Roadmap

Phase 1 is a keyed station with ARP/ICMP/DHCP/UDP and the service seam.
The natural next phases, in rough order:

- **TCP** — a minimal connection layer, which lights up the `tcp_ports`
  the printer/NAS profiles already advertise (IPP/631, JetDirect/9100,
  SMB/445, HTTP/80).
- **Real service protocols** — mDNS/SSDP/NBNS presence, an SNMP agent, a
  tiny HTTP admin page, an IPP responder — each a `Service` subclass.
- **More link security** — WPA3-SAE and WPA2-Enterprise (the device
  already models SAE auth on the AP side).
- **IPv6** — SLAAC and ICMPv6, for hosts that would have it.

Each is additive against the three-layer split above.
