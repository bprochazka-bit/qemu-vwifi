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
| `voip`         | voip-phone       | 64  | a SIP desk phone (answers OPTIONS on 5060) |
| `chromecast`   | chromecast       | 64  | a Google Cast receiver (mDNS `_googlecast._tcp`) |
| `hp-mfp`       | hp-mfp           | 255 | an HP print/scan/fax MFP (mDNS `_ipp` + `_uscan`) |
| `bambu`        | bambu-3dprinter  | 64  | a Bambu Lab 3D printer (SSDP `:1990`, MQTTS/FTPS/RTSP) |
| `smart-screen` | smart-screen     | 64  | a smart display (SSDP UPnP MediaRenderer + DIAL) |
| `pos`          | pos-terminal     | 64  | a Square-style POS terminal (mDNS pairing/status) |

These show the service framework doing real work over the live UDP path.
`voip` answers SIP `OPTIONS` with a phone-like `200 OK`. Two discovery
transports back the rest:

- **mDNS** (`mdns.py`) — `chromecast` (a Cast receiver), `hp-mfp` (both a
  printer `_ipp._tcp` and a scanner `_uscan._tcp` — what makes it a
  multifunction), the `smart-screen`'s HTTP UI, and the `pos` terminal's
  pairing/status records. Discoverable with `avahi-browse` / `dns-sd`.
- **SSDP / UPnP** (`ssdp.py`) — answers `M-SEARCH` and announces
  `ssdp:alive`. `bambu` advertises its vendor `urn:bambulab-com:...`
  device on port 1990 (how Bambu Studio finds a printer, with the
  DevModel/DevName headers); `smart-screen` advertises a UPnP
  rootdevice + MediaRenderer + DIAL on 1900.
- **WSD** (`wsd.py`) — WS-Discovery on 3702 plus the metadata HTTP
  endpoint on 5357. This is what stock **Windows** uses for "Network"
  and the "Add a printer" scan (it does not browse mDNS/Bonjour without
  extra software): the printer/`hp-mfp` profiles answer a WS-Discovery
  Probe with a ProbeMatch and serve the device metadata (manufacturer,
  model, friendly name, hosted print service) Windows fetches to show
  the device. Submitting jobs over WSD Print is a later phase.

Their richer protocols (Cast/TLS 8009, IPP, eSCL, SNMP, MQTTS 8883,
FTPS 990, the UPnP description HTTP) are advertised as ports now and
become listeners when the TCP/TLS layer lands.

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

Both transports are live. A `UDPService` answers datagrams
(`UDPEchoService` — `nc -u <ip> 7` and your bytes come back); a
`TCPService` accepts connections over the built-in minimal TCP
(`tcp.py`) and gets `on_connect` / `on_data` / `on_close` hooks with a
`conn.send()` / `conn.close()` back-channel. The shipped TCP services
(`netservices.py`) are real enough to use:

- **`LPDService`** — an RFC 1179 line-printer daemon on 515 that accepts
  an actual `lpr` job and accounts its bytes;
- **`HTTPService`** — a one-page HTTP/1.0 server (a device admin UI);
- **`SMBService` / `NASHTTPService`** — an SMB-shaped presence on 139/445
  plus a NAS admin page, which with an mDNS `_smb._tcp` advert reads as a
  file server to a scan.

The framework is built around "a service owns some ports and some
behaviour," so the same `Service` runs on a workstation, a printer, or an
**access point** unchanged.

That is the deliberate design consequence the request asked us to
consider up front: **the station/AP link, the netstack, and the service
framework are separable layers**, and a device profile (or an AP) only
ever touches the top one. A printer, a NAS, and a Linux box are the same
IP stack with different services and different fingerprints.

## Access points

The medium has an AP side too: `PseudoAP` (CLI `pseudoap`) beacons a
network that pseudo-hosts and real VMs can associate to. Give it an
ESSID, a channel, an encryption type and optionally a BSSID:

```bash
./pseudoap --sock /tmp/vwifi.sock --essid Lab-AP-1 --channel 6 \
           --encryption wpa2 --passphrase correcthorse1

./pseudoap --sock /tmp/vwifi.sock --essid Office --channel 11 \
           --encryption open --services lpd,nas

# DHCP scope as CIDR — sets gateway, netmask and pool in one flag:
./pseudoap --sock /tmp/vwifi.sock --essid Lab --channel 6 \
           --encryption open --subnet 10.10.10.0/24
```

`--subnet` accepts CIDR (`10.10.10.0/24`); the gateway defaults to the
first address (`.1`) and the pool to the `.100`–`.200` window, or the
whole usable range on a subnet too small for it. Write host bits to pin
the gateway (`10.10.10.254/24`), and `--gateway` / `--netmask` /
`--pool-start` / `--pool-end` still override any field individually. The
DHCP server allocates across the whole scope — any prefix, not just /24 —
and never hands out the gateway or its own address.

It beacons and answers probes, runs open-system Auth/Assoc, drives the
**Authenticator** side of the WPA2-PSK four-way handshake per station
(`authenticator.py`, the mirror of `supplicant.py`), and installs
per-station CCMP keys. It is the DS: it terminates IP for its own gateway
address (ARP, ICMP, a **DHCP server**, and any TCP/UDP services) and
bridges frames between associated stations. `--services lpd,nas,http`
runs those on the AP itself.

Because both sides share the same crypto, 802.11 and medium code, a
`PseudoAP` and a `PseudoHost` interoperate for real — the test suite
associates one to the other across an in-process hub and pulls a DHCP
lease across the encrypted link.

## Layout

```
pseudohost/
  pseudohost                     the station CLI (join a network)
  pseudoap                       the AP CLI (beacon a network)
  vwifi_pseudohost/
    crypto.py         AES-128, CCMP, PRF, PBKDF2, AES key wrap, EAPOL MIC
    ieee80211.py      802.11 frame + IE build/parse; Ethernet <-> 802.11
    medium.py         the hub transport (abi/vwifi.h wire protocol)
    supplicant.py     the STA side of the WPA2-PSK four-way handshake
    authenticator.py  the AP side of the four-way handshake
    station.py        scan -> auth -> assoc -> keys; the radio state machine
    accesspoint.py    PseudoAP: beacon/assoc/handshake, the DS, bridging
    netstack.py       ARP / IPv4 / ICMP / UDP / TCP demux; the host stack
    tcp.py            minimal server-side TCP (for TCP services)
    dhcp.py           DHCPv4 client
    dhcp_server.py    DHCPv4 server (the AP's lease pool)
    mdns.py           multicast-DNS / DNS-SD responder
    ssdp.py           SSDP / UPnP discovery responder
    wsd.py            WS-Discovery + metadata (Windows printer discovery)
    services.py       Service / UDPService / TCPService base + registry
    netservices.py    LPD, HTTP, NAS/SMB (TCP-backed services)
    host.py           PseudoHost — the inheritable base class
    profiles/         one subclass per device kind
  tests/              stdlib unittest; run.sh runs them all
```

## Tests

```bash
./tests/run.sh                 # or: make test-pseudohost   (from repo root)
```

The suite needs no medium and no network. It covers the crypto against
known-answer vectors (AES/FIPS-197, PBKDF2/PTK, CCMP round-trip mirroring
`devices/vwifi/tests/crypto.c`, RFC 3394 key wrap), the `Supplicant` and
`Authenticator` run against each other, the netstack, DHCP client and
server, the TCP layer and the LPD/HTTP services, the profiles' mDNS/SSDP
responders, and two **full end-to-end connects** over a real Unix
socket: a pseudo-host against a mock AP, and a real `PseudoHost` against
a real `PseudoAP` across an in-process hub — each for open and WPA2, with
a wrong-passphrase case that must *fail*.

## Roadmap

Now in place: keyed stations *and* access points, DHCP client and server,
ARP/ICMP/UDP and a minimal TCP with LPD/HTTP/NAS services, and mDNS/SSDP
discovery. The natural next steps, in rough order:

- **Richer service protocols** — an SNMP agent, an IPP/eSCL responder, a
  real (if minimal) SMB2 negotiate, NBNS/LLMNR — each a `Service`
  subclass over the existing UDP/TCP layers.
- **More link security** — WPA3-SAE and WPA2-Enterprise (the frame code
  already models SAE auth).
- **TCP completeness** — retransmission and out-of-order reassembly, for
  lossy or bridged-to-real-radio mediums where the lossless assumption
  no longer holds.
- **IPv6** — SLAAC and ICMPv6, for hosts that would have it.

Each is additive against the layer split above.
