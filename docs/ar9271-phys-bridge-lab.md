# Single-radio physical bridge: real STA → virtual OpenWRT APs (AR9271)

This runbook describes how to let an **unmodifiable real station** (e.g. an
Android phone) *associate* to **OpenWRT VM access points that live on the vwifi
medium**, using a **single** physical radio that serves **multiple BSSIDs on one
channel**. The VMs remain the literal APs; the real radio only provides the
PHY and the one timing-critical thing software cannot: the **MAC-layer ACK**.

> Scope note. The monitor `vwifi-phys-bridge` is great for *observation*
> (sniffing/injecting between real air and the sim). Letting a real STA
> *join* a virtual AP additionally requires hardware ACKs at SIFS, which is
> the subject of this document.

---

## 1. Why this is hard, in one paragraph

Every unicast frame a STA sends must be ACKed within **SIFS (~10–16 µs)**. That
ACK is produced by the receiver's MAC **hardware/firmware**, keyed only on the
frame's `addr1` (RA) matching the radio's address filter — it does **not**
depend on association state, keys, or sequence numbers. No host-side software
can hit SIFS across an air→host→hub→VM→back path. Therefore the radio the phone
talks to must auto-ACK locally. Everything *else* in association
(Auth/Assoc/EAPOL/data) has soft, millisecond-to-second timeouts and can be
relayed to the VM, which stays the real AP.

Only **ACK** is on the SIFS hot path. The phone ACKs the *downlink* itself, and
the vwifi driver already reports the VM's TX as ACKed, so the proxy's entire job
is: **auto-ACK uplink frames addressed to the AP BSSID set.**

---

## 2. Why AR9271

| Property | AR9271 | Consequence |
|---|---|---|
| MAC offload | **open, on-target firmware** (`qca/open-ath9k-htc-firmware`) | we can change ACK/address-filter behavior |
| ACK generation | on the device target | meets SIFS despite USB |
| Driver | `ath9k_htc` (shares `ath9k_hw` register layer) | standard BSSID-mask machinery available |
| Band / PHY | 2.4 GHz only, 11n 1×1, HT20/40 | fine for a single-channel lab; phone air link is 11n/legacy |
| Cost | TL-WN722N **v1**, Alfa AWUS036NHA | cheap, common |

The 11ac/ax USB parts (mt76, etc.) hide the MAC in **closed** firmware, which is
exactly what removes the ACK control. AR9271's open firmware is the unlock.

---

## 3. Architecture

```
  OpenWRT VM1  hostapd  bssid 02:11:22:33:44:00  ┐
  OpenWRT VM2  hostapd  bssid 02:11:22:33:44:01  ├─ all on channel 6, all virtual
  OpenWRT VM3  hostapd  bssid 02:11:22:33:44:02  ┘   (the *literal* APs)
        │  vwifi medium (hub)
        ▼
   vwifi-phys-bridge ──► AR9271 (ath9k_htc), ch6, monitor capture + inject
                          + on-target auto-ACK for the masked BSSID set  ◄─┐
                                                                           │ ACK (SIFS)
  Android phone ))) sees 3 SSIDs on ch6, joins ANY one ─────────────────────┘
```

What already works (no code changes):

- **Air → medium**: the bridge forwards every captured frame to the hub tagged
  with its configured channel. All three APs share that channel, so every frame
  reaches all three VM AP peers; each VM's hostapd accepts only frames addressed
  to its own BSSID and drops the rest. **The VMs self-demultiplex** — the bridge
  stays BSSID-agnostic.
- **Medium → air**: the bridge injects everything for that channel, so all three
  VMs' beacons/responses go out and the phone sees three SSIDs on one channel.
- Echo suppression and the per-channel accept filter are already in place.

The **only** missing capability is the hardware ACK (Section 6).

---

## 4. BSSID allocation (keep the mask tight)

The AR9271 auto-ACKs a frame when `(RA & mask) == (base & mask)`. The mask is a
bitmask, not a list, so choose BSSIDs that share a prefix and differ only in the
low bits:

| AP | BSSID |
|----|-------|
| AP1 | `02:11:22:33:44:00` |
| AP2 | `02:11:22:33:44:01` |
| AP3 | `02:11:22:33:44:02` |

Only the low two bits differ →

```
base = 02:11:22:33:44:00
mask = FF:FF:FF:FF:FF:FC      (care about everything except the low 2 bits)
```

The hardware then ACKs `02:11:22:33:44:00..03`: your three real BSSIDs plus one
unused phantom (`…:03`). Use locally-administered addresses (`02:…`) so you fully
own the space. Add more APs by widening the low-bit field (4 APs → mask
`…:FC` covers `00..03`; 8 APs → `…:F8`, etc.).

---

## 5. OpenWRT VM AP configuration

Run each AP on the **same channel** with its assigned BSSID, in **legacy
(non-HT) mode** so every data frame uses a simple ACK — this removes
**Block-Ack** and **RTS/CTS**, which are *also* SIFS responses the proxy does not
generate. See `examples/hostapd/lab-ap1.conf` … `lab-ap3.conf`. Key lines:

```ini
interface=wlan0          # the VM's vwifi virtual radio
driver=nl80211
channel=6
hw_mode=g
ieee80211n=0             # legacy → simple per-frame ACK, no A-MPDU/Block-Ack
wmm_enabled=0            # no QoS → no Block-Ack negotiation
ssid=Lab-AP-1
bssid=02:11:22:33:44:00
auth_algs=1
wpa=2
wpa_key_mgmt=WPA-PSK
rsn_pairwise=CCMP
wpa_passphrase=correcthorse1
```

Crypto stays in the VM. **Install no keys on the AR9271** (it relays the data
frames raw/encrypted; the ACK is pre-decryption).

> APs that only ever serve *virtual* clients can stay HT/VHT/HE at any width —
> the legacy restriction applies only to the APs a real phone will join.

---

## 6. The one component to build: AR9271 "ackmon"

Goal: make the AR9271 **auto-ACK uplink frames addressed to the masked BSSID
set while in monitor mode**, still delivering all frames raw to the host and
accepting raw injection.

**Source analysis says this is driver-only — probably no firmware rebuild.**
Reading mainline ath9k:

- The PCU ACKs a frame when `(RA & bssidmask) == (macaddr & bssidmask)`.
  `macaddr` → `AR_STA_ID` (carried to the AR9271 target as the monitor vif's
  `myaddr`); `bssidmask` → `AR_BSSMSK`, set by `ath_hw_setbssidmask()`.
- **`AR_DIAG_ACK_DIS` is defined but never used** — ACK is not disabled in
  monitor mode; it just never fires because nothing is addressed to the
  sniffer's MAC.
- `ath9k_htc_set_mac_bssid_mask()` already derives `macaddr`/`bssidmask` from
  the active vif addresses.

So ackmon overrides `macaddr` = base and `bssidmask` = the covering mask on a
monitor vif, and the chip ACKs the whole BSSID set while capturing/injecting
raw. The draft patch and full instructions are in
[`../ackmon/`](../ackmon/) (`ath9k_htc-ackmon.patch` + `README.md`).

**Validate the one residual risk first** (whether the AR9271 *firmware*
suppresses ACK in `HTC_M_MONITOR` regardless of `AR_STA_ID`): the zero-patch
injection test in `ackmon/README.md` — set the card's MAC to a BSSID in monitor,
inject a unicast to it from a second radio, and watch a third for the SIFS ACK.
Only if that test shows monitor ACK is firmware-gated do you fall back to
patching `qca/open-ath9k-htc-firmware`.

Fallback if even the firmware route stalls: run a stock local `hostapd` on the
AR9271 as a "dumb AP" bridged into the VM at L2 (local-MAC split). Reliable
today, but the phone then appears in the *host's* station table, not the
OpenWRT VM's — which is exactly the property "ackmon" buys back.

---

## 7. Host bring-up (once ackmon firmware/driver is in place)

```sh
# 1. Put the AR9271 into monitor on the lab channel
sudo ip link set wlan0 down
sudo iw dev wlan0 set type monitor
sudo ip link set wlan0 up
sudo iw dev wlan0 set channel 6

# 2. Enable ackmon for the BSSID set (interface defined by the ackmon hook)
#    e.g. via debugfs:
echo 02:11:22:33:44:00 | sudo tee /sys/kernel/debug/ieee80211/phyX/ath9k_htc/ackmon_base
echo FF:FF:FF:FF:FF:FC | sudo tee /sys/kernel/debug/ieee80211/phyX/ath9k_htc/ackmon_mask
echo 1                 | sudo tee /sys/kernel/debug/ieee80211/phyX/ath9k_htc/ackmon

# 3. Start the relay against the hub (single channel = ch6 = 2437 MHz)
sudo ./vwifi-phys-bridge /tmp/vwifi.sock wlan0 -c 6 -v
```

The bridge needs no per-BSSID configuration — it relays all of ch6 and the VMs
sort frames by their own BSSID.

---

## 8. Association walk-through (what happens on the wire)

1. Each VM beacons its BSSID → medium → bridge injects on ch6 → phone sees 3 SSIDs.
2. Phone → **Auth** (RA = BSSID). **AR9271 hardware ACKs** (BSSID in mask). Bridge
   relays Auth to the medium → the matching VM replies with Auth Resp → bridge
   injects → phone hardware-ACKs in the air.
3. Phone → **Assoc Req** → HW ACK → relay → VM → Assoc Resp → relay → phone.
4. **EAPOL 4-way** (data frames): each phone→AP frame HW-ACKed by the AR9271;
   relayed; the VM's hostapd runs the handshake; PTK derived in the VM and phone.
5. **Data**: phone→AP CCMP frames HW-ACKed and relayed raw (encrypted) to the VM,
   which holds the PTK; VM→phone frames injected, phone ACKs in the air.

Only the ACK ever needed SIFS, and the AR9271 supplies it.

---

## 9. Constraints and risks

- **Single channel** for all phone-joinable APs (the accepted trade for one radio).
- Phone's air link is **11n/legacy** (advertise HT-off on joinable APs to match).
- **Downlink losses are not retransmitted** (the vwifi driver fakes the VM's TX
  status) — rely on good signal and EAPOL/DHCP/TCP retries. A later refinement
  could feed the phone's real downlink ACKs back into the medium to make VM
  TX-status truthful.
- **Block-Ack/RTS/CTS must stay off** on joinable APs — `ieee80211n=0` +
  `wmm_enabled=0` guarantees this.
- The **ackmon firmware/driver hook is the real engineering** and must be
  validated on actual AR9271 silicon (USB HTC concurrency of monitor + inject +
  ACK is the make-or-break check).

---

## 10. Status of the pieces

| Piece | State |
|---|---|
| vwifi medium: channel match incl. mixed-width / wide channels | done |
| vwifi medium: physical-link physics exemption (`physical` hello flag) | done |
| vwifi-phys-bridge: single-channel multi-BSSID relay | works as-is |
| OpenWRT hostapd profiles (legacy, no-BA, prefix BSSIDs) | `examples/hostapd/` |
| ackmon source analysis (ACK = address-match, driver-only) | done — `ackmon/` |
| AR9271 ackmon `ath9k_htc` patch | **draft, untested** — `ackmon/` |
| Monitor-ACK feasibility test | **run first** — `ackmon/README.md` |
| Firmware fallback (`open-ath9k-htc-firmware`) | only if the test fails |
