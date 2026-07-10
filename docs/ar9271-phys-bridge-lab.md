# Real STA → virtual OpenWRT VM AP, over the vwifi medium

This runbook is the **proven, working** procedure for letting an **unmodifiable
real station** (an Android phone, a Linux laptop, any STA) *associate with WPA2*
to **OpenWRT VM access points that live on the vwifi medium** — using a physical
bridge on the host. The VMs remain the literal APs; the phone lands in the VM's
station table; the real radios only provide the PHY and the one thing software
can't: the SIFS-timed MAC ACK.

Status: **validated end to end** — VM client, Linux laptop, and an Android phone
all associate with WPA2-PSK, complete the 4-way handshake, get a DHCP lease, and
pass traffic.

---

## 0. TL;DR — the four things that had to be true

Each was a hard blocker discovered in turn; all four are now fixed/handled:

1. **The medium must ACK the STA's uplink at SIFS.** Only hardware can. The
   AR9271 (`ath9k_htc`) will **not** ACK in *monitor* mode, but **does** ACK in
   **AP opmode** for any address its `bssidmask` covers. So we run decoy hostapd
   APs on the AR9271 whose mask spans the VM BSSIDs. (§2, §3)
2. **Injected downlink must not self-retransmit.** Inject with radiotap
   `TX_FLAGS = NO_ACK` (the injecting radio can never hear "its" ACK, since the
   frame's source is the VM BSSID). Otherwise every frame hits the air ~15× and
   drowns the channel. (built into `vwifi-phys-bridge`)
3. **The handshake must survive a lossy, co-located channel.** Inject the
   critical frames redundantly (`-r 3`) and drop ambient junk from the medium.
   (§3)
4. **The VM must actually do the CCMP crypto.** The emulated `ath9k` in the VMs
   advertises hardware crypto it doesn't perform, so encrypted data was dropped
   (`rx drop misc`). Load the VM `ath9k` with **`nohwcrypt=1`** to force software
   crypto. This is the single most important fix — without it WPA2 associates
   but no data ever flows. (§4)

---

## 1. Why a SIFS ACK is the whole problem

Every unicast frame a STA sends must be ACKed within **SIFS (~10–16 µs)**. That
ACK is produced by the receiver's MAC hardware, keyed only on the frame's
`addr1` (RA) matching the radio's address filter under its BSSID mask — it does
**not** depend on association, keys, or sequence numbers. No host-side software
path (air → host → hub → VM → back) can hit SIFS. So the radio the STA talks to
must auto-ACK locally. Everything else in association (Auth/Assoc/EAPOL/data) has
millisecond-to-second timeouts and is relayed to the VM, which stays the real AP.

---

## 2. The AP-opmode ACK trick (the key insight)

Measured facts on the AR9271 (`ath9k_htc`, USB):

- In **monitor** mode with its MAC set to a BSSID, it captures fine but emits
  **zero** ACKs — the firmware gates the ACK responder off in `HTC_M_MONITOR`.
  (Verified with a co-channel witness: 0 ACKs to the STA, STA retransmits to its
  limit, handshake stalls.)
- The card's interface combinations **forbid `AP + monitor` on one radio**
  (`iw phy … info`: `#{AP,mesh,P2P-GO} ≤ 2`, monitor not combinable). So you
  cannot "add a monitor vif" next to an AP for capture on the same AR9271.
- But in **AP opmode** the PCU auto-ACKs any received frame matching
  `(RA & bssidmask) == (macaddr & bssidmask)` — **including BSSIDs no vif owns**,
  because the mask is a bitmask, not a list.

So: run **two decoy hostapd BSSes on the AR9271** that bound the BSSID range,
e.g. `02:11:22:33:44:00` and `02:11:22:33:44:03`. The driver derives
`bssidmask = ff:ff:ff:ff:ff:fc`, and the chip hardware-ACKs the whole
`02:11:22:33:44:00..03` range. The VM APs live at `…:01`/`…:02`; the AR9271 ACKs
their uplink even though hostapd only *owns* `…:00`/`…:03`. Those two decoys
double as the survey "wrong APs." (Need more VMs? Widen to `…:00`/`…:07`
→ mask `…:f8`, covering `…:00..07`, VMs at `…:01..06`.)

### Getting the second decoy's MAC to stick (ath9k_htc gotcha)

`ath9k_htc` advertises a **single hardware address**, so mac80211 cannot give a
second AP vif a *controlled* MAC on its own — hostapd's `bss=<iface>` interface
creation gets a **random** locally-administered MAC, whatever BSSID you
configure. A random second MAC yields a `bssidmask` that does **not** cover the
VM BSSIDs, and the AR9271 silently never ACKs the STA (the failure looks like
"associates, completes the 4-way on retransmits, then drops" — the STA
retransmits ~80 % of its uplink). Two facts make it work:

- a **direct `ip link set <iface> address …`** on a pre-created `__ap` vif *is*
  honored (mac80211 validates rather than randomizes), and
- **`hostapd interface=<iface>`** *adopts* an existing netdev without recreating
  it, whereas `bss=` recreates (and re-randomizes).

So pre-create the second vif with `iw`, pin its MAC, and run a **separate
single-BSS hostapd per vif** with `interface=` (never `bss=`). `scripts/lab-bringup.sh`
does this and **hard-fails** if either MAC doesn't land on its configured BSSID.

Capture + injection run on a **second** radio (the MT7921U, `mt76`) in monitor
mode, since the AR9271 can't do AP+monitor.

### BSSID mask math

The mask is `~(XOR of the AP-vif addresses)`; a bit is "don't-care" (ACK-matched)
only where the addresses differ. The two decoys bound the range:

```
02:11:22:33:44:00  XOR  02:11:22:33:44:03  = 00:00:00:00:00:03
  -> mask ff:ff:ff:ff:ff:fc  -> ACKs 02:11:22:33:44:00 .. :03  (4 BSSIDs)
```

In principle `:00`/`:FF` would span all 256, but **ath9k_htc will not accept a
spread that wide** (see the gotcha above — it randomizes the second vif). Keep
the span small and standard (`…:fc` for 4, `…:f8` for 8). Keep the fixed prefix
a locally-administered address (`02:…`) you own, and make the mask no wider than
your BSSID pool — every don't-care bit widens the set of addresses the card will
ACK, and if it ever overlaps a real neighbour's MAC the card will ACK traffic
that isn't yours.

---

## 3. Architecture

```
  OpenWRT VM1  hostapd  bssid 02:11:22:33:44:01  ┐  the REAL, joinable APs
  OpenWRT VM2  hostapd  bssid 02:11:22:33:44:02  ┘─ ch 11, legacy, WPA2-PSK
        │                                            (ath9k, nohwcrypt=1)
        │  vwifi medium (hub)
        ▼
   vwifi-phys-bridge on MT7921U (mt76), ch 11, monitor:
        capture STA uplink -> hub;  inject VM downlink (NO_ACK, -r)
        relevance filter -b 02:11:22:33:44:00 -m ff:ff:ff:ff:ff:fc
        │
   AR9271 (ath9k_htc), ch 11: two decoy hostapds (interface=) at :00 and :03
        -> AP opmode; hardware-ACKs the STA's uplink to :01/:02  ◄──┐ ACK (SIFS)
                                                                     │
  Real STA ))) sees the SSIDs on ch 11, joins one, associates ──────┘
```

Two radios, one channel. Put them on **USB extension cables a foot+ apart** — co-
located transmitters desense each other and that is the main source of the
periodic beacon-loss reconnects.

---

## 4. VM (OpenWRT AP) configuration — including the crypto fix

**The crypto fix (mandatory).** The emulated `ath9k` in the VM advertises hardware
crypto it does not perform, so mac80211 hands it plaintext to "encrypt" and marks
RX frames "decrypted" when they aren't → all CCMP data is dropped
(`rx drop misc` climbs, DHCP gets no lease). Force software crypto:

```sh
mkdir -p /etc/modprobe.d
echo 'options ath9k nohwcrypt=1' > /etc/modprobe.d/ath9k.conf   # MUST end in .conf
# apply now without a reboot (OpenWRT modprobe ignores CLI params; insmod honors them):
wifi down
rmmod ath9k
insmod "$(find /lib/modules/$(uname -r) -name 'ath9k.ko*' | head -1)" nohwcrypt=1
cat /sys/module/ath9k/parameters/nohwcrypt      # must read 1
wifi up
```

Set this on **every** VM that does crypto (each AP and any VM client). After a
reboot the `.conf` makes it stick. Note: OpenWRT's `/etc/modules.d/` is only a
list of modules to load — module *options* must go in `/etc/modprobe.d/*.conf`.

**AP profile.** Same channel, assigned BSSID inside the ACK mask but not owned
by a decoy (`…:01`/`…:02` for the `…:fc` mask), legacy (non-HT, no WMM) so every
data frame uses a simple per-frame ACK — no Block-Ack or RTS/CTS (those are
additional SIFS responses the bridge doesn't generate). Attach it to a network
that has a DHCP server (`network='lan'`).

```sh
uci set wireless.radio0.channel='11'
uci set wireless.radio0.htmode='NONE'
uci set wireless.default_radio0.macaddr='02:11:22:33:44:01'   # NOT :00 (host owns it)
uci set wireless.default_radio0.ssid='Lab-Real'
uci set wireless.default_radio0.network='lan'                 # DHCP lives here
uci set wireless.default_radio0.encryption='psk2'
uci set wireless.default_radio0.key='correcthorse1'
uci commit wireless && wifi reload
```

Crypto stays entirely in the VM; the AR9271 and the bridge are crypto-agnostic
(they relay the encrypted frames raw — the ACK is pre-decryption).

---

## 5. Host bring-up

Use `scripts/lab-bringup.sh` (set `AR9271_IF` / `MT_IF` to your interface names):

```sh
git pull && make
sudo AR9271_IF=wlxc01c300da281 MT_IF=wlx00c0cab57e6f ./scripts/lab-bringup.sh
sudo ./scripts/lab-bringup.sh stop     # tear down
```

It: pins the AR9271 primary vif to `:00` and pre-creates a second vif at `:03`,
then runs a separate single-BSS hostapd on each via `interface=` (ch 11, legacy,
slow beacons) and **hard-fails** if either MAC didn't stick; puts the MT7921U in
monitor on ch 11; starts `vwifi-phys-bridge` with
`-r 3 -b 02:11:22:33:44:00 -m ff:ff:ff:ff:ff:fc`; and runs a witness `tcpdump`.
Override injection redundancy with `INJECT_COPIES=N`. Equivalent manual bridge
invocation:

```sh
sudo ./vwifi-phys-bridge /run/vwifi/foo.sock wlx00c0cab57e6f \
     -c 11 -b 02:11:22:33:44:00 -m ff:ff:ff:ff:ff:fc -r 3 -v
```

### `vwifi-phys-bridge` flags that matter here

| Flag | Purpose |
|------|---------|
| `-b <bssid>` | relevance-filter match address (the range base) |
| `-m <mask>`  | relevance-filter mask — forward the whole `…:00..03` set, not one BSSID |
| `-r <n>`     | inject each critical frame N times (beacons/probe-resps excluded); use 2–3 on a lossy channel |
| `-R <mbps>`  | legacy over-air rate (`1,2,…,54`, or `0`=auto/echo the VM's rate). **mt76/ath9k_htc ignore this and always inject at ~1 Mbps** — use `-M`/`-L` on a Realtek. `INJECT_RATE=` in `lab-bringup.sh`. |
| `-M <mcs>`   | inject at **HT MCS** `0..31` (Realtek rtl88xxau honors it → real rates, MCS7=65 Mbps; pick a moderate MCS like 4 for robustness). Overrides `-R`. `INJECT_MCS=` in `lab-bringup.sh`. |
| `-L`         | inject **rate-less** (no rate field; driver picks). On a Realtek with `rtw_monitor_disable_1m=1` → HT-MCS7/VHT-MCS9. `INJECT_RATELESS=1` in `lab-bringup.sh`. |
| `-S <sec>`   | print a periodic throughput report: per-direction fps/Mbps and, crucially, inject drop counters (`enobufs`/`eagain` = frames the driver's monitor TX queue rejected — otherwise-silent downlink loss). `SIGUSR1` dumps on demand. |
| `-c <chan>`  | channel (must match the AP + AR9271 + MT7921U) |

The bridge already: injects `NO_ACK` (no 15× self-retransmit storm); drops
control frames and foreign beacons from the medium; suppresses echoes of its own
injections; auto-raises the capture MTU; and **self-heals** if the mt76 radio
flaps (reopens the socket instead of exiting).

---

## 6. Verifying each stage

```sh
# AR9271 is in AP opmode with the two decoy vifs (=> mask covers the VM range):
iw dev | awk '/Interface/{i=$2} /addr/{print i, $2}'

# The AR9271 is ACKing the STA's uplink (witness on the MT7921U, co-channel).
# Note real STAs randomize their MAC — grab it from the AP's assoc-resp target:
tshark -r witness.pcap -Y 'wlan.fc.type_subtype==0x1d && wlan.ra==<STA_MAC>' | wc -l   # > 0

# On the VM AP: STA associated, data flowing, NOT dropping:
iw dev phy0-ap0 station dump | grep -E 'Station|authorized|rx packets|rx drop misc'
#   rx packets climbing + rx drop misc flat  ==  crypto path healthy
```

Signs of each historical failure mode, for future debugging:

| Symptom | Cause | Fix |
|---|---|---|
| STA retransmits uplink forever, handshake stalls, 0 ACKs to STA | AR9271 in monitor (no ACK) | AP-opmode decoys (§2) |
| STA completes 4-way (on retransmits) then drops; decoy vif has a **random** MAC | 2nd decoy BSSID didn't stick → mask doesn't cover the VM | pre-create vif + `interface=` hostapd (§2); script hard-fails on this |
| Downlink frames appear ~15× with retry flag | injecting without NO_ACK | (built in) |
| 4-way sometimes "wrong key", sometimes completes | handshake frame loss | `-r 3`, separate radios |
| WPA2 associates, 4-way completes, **no data / no DHCP**, `rx drop misc` climbs | VM ath9k hw-crypto not real | `nohwcrypt=1` (§4) |
| ping shows `DUP!`, each downlink 2–3× | `-r` duplicating encrypted data | (built in: only mgmt/EAPOL duplicated) |
| `reason=3/4 locally_generated`, periodic reconnects | beacon/keepalive loss (co-location, congestion) | separate radios, clean channel |
| download ≪ upload (e.g. 0.4 vs 5 Mbps), witness shows downlink at ~1 Mbps | injector sent no radiotap rate → mt76 monitor default | set `-R`/`INJECT_RATE` (§5); auto now floors at 6 Mbps |
| Bridge exits: "raw socket read error: Network is down" | mt76 USB reset | self-heal (built in) |

---

## 7. Constraints and remaining rough edges

- **Single channel** for all phone-joinable APs (one radio pair). Pick the
  quietest channel you can — congestion directly causes the periodic reconnects.
- Joinable APs must be **legacy / no Block-Ack / no RTS-CTS** (`ieee80211n=0`,
  `wmm_enabled=0`, `htmode=NONE`). APs that only serve *virtual* clients can stay
  HT/VHT/HE at any width.
- **Downlink losses aren't retransmitted at L2** (NO_ACK injection); rely on
  `-r`, a clean channel, and upper-layer retries.
- **mt76 flaps** under sustained load; the bridge now self-heals, but fewer flaps
  = fewer blips (separate the radios, lower `-r`, a powered USB port/hub).
- The two radios are **co-located transmitters** — physical separation is the
  single best lever for the periodic beacon-loss reconnects.
- **Throughput asymmetry is expected**: uplink is hardware-ACKed by the AR9271
  (the STA does normal rate-control + L2 retransmit), so it runs near link speed;
  downlink is injected single-shot NO_ACK with no L2 retransmit, so a
  *download*-heavy test (speedtest, web browsing) rides entirely on the weak
  downlink and reads far lower than the upload. Measured example: phone upload
  5.3 Mbps vs download 0.39 Mbps, unloaded latency 14 ms (the relay itself is
  fast) but 241 ms loaded (downlink bufferbloat behind a slow inject queue).
- **The injected downlink now carries a radiotap RATE** (was absent → mt76 fell
  back to ~1 Mbps). Set it with `-R`/`INJECT_RATE`; NO_ACK injection has no rate
  fallback, so too high a rate for the link just trades throughput for loss.
  **But raising the rate alone did not lift the observed ceiling** — download
  stayed ~0.5 Mbps from 6 through 36 Mbps on-air. That rules the air PHY rate
  *out* as the current binding constraint and points at a frames-per-second cap
  in the relay (see §7.5).
- **Measured bottleneck: injected downlink throughput is byte-bound at ~1 Mbps,
  not fps-bound.** A `-s 200,700,1500` sweep shows per-frame time scaling
  *linearly* with size (~7 µs/byte + ~0.75 ms fixed) and throughput pinned at
  ~0.7–1.0 Mbps across all sizes — the signature of frames actually leaving at a
  ~1 Mbps PHY rate, i.e. the injected radiotap `RATE` being **ignored / falling
  back to a basic rate**. `PACKET_QDISC_BYPASS` (`-Q`) and a bigger `SO_SNDBUF`
  (`-B`) do **not** help — correctly, since it is not a queuing/pacing problem —
  and a co-channel `capture` shows **0 % air loss**. (This overturns the earlier
  "~100 Hz tick pacing" guess.) Note radios differ: the witness once showed the
  MT7921U honoring 6 Mbps, while the second radio here appears to inject at
  ~1 Mbps — measure the *actual* rate per radio with `capture` before concluding.
- **Confirmed: mt76 ignores the legacy injection rate on *both* radios.** The
  size sweep is identical whether injecting at `-R 6` or `-R 24` (~0.7/0.9/1.0
  Mbps at 200/700/1500 B), on the MT7921U (the bridge's radio) and the second
  mt76. So the legacy radiotap RATE is a no-op here and the on-air rate sits at
  the ~1 Mbps basic rate. `capture` prints the on-air rate parsed from the RX
  radiotap to confirm this directly on the radio the bridge injects on.
- **VERDICT (ground-truth confirmed): mt76 monitor injection sends everything at
  1 Mbps, ignoring both legacy RATE and HT MCS.** Injecting `-M 7` (MCS7) on the
  MT7921U and capturing on an independent witness radio shows
  `on-air rate : 1 Mbps` — the driver discards the MCS request and even downshifts
  to a *legacy* 1 Mbps frame. Legacy `-R 6`/`-R 24`/`-M 7` are all identical
  ~1 Mbps. `-Q`/`-B` do nothing (not a queuing problem) and air loss is 0 %. So
  the ~1 Mbps downlink ceiling (~0.5 Mbps TCP) is an **mt76 driver limitation on
  the inject path**, not our code — `vwifi-phys-bridge` builds the radiotap rate
  correctly; mt76 ignores it. (An injecting radio's *self-capture* echoes the
  requested rate, e.g. a witness on the same radio once showed 6 Mbps — always
  confirm the rate on a *separate* radio.)
- **ath9k_htc (AR9271) is *not* the fix — it is worse.** Injecting on the AR9271
  in monitor also comes out at **1 Mbps** on the witness, and it *ignores NO_ACK*
  so it retransmits each frame ~10–15× (11 fps, 89 ms/frame). Clearing NO_ACK on
  mt76 (`-A`) likewise stays 1 Mbps (with a ~15× duplication) — so NO_ACK is not
  the cause either. Both of the on-hand radios are hard-capped at 1 Mbps for
  monitor injection.
- **The fix is a Realtek rtl88xxau (RTL8812AU/8814AU) with the aircrack-ng
  driver.** Unlike mt76/ath9k_htc, it can inject above 1 Mbps. Set the module
  options (then reload the driver so they take — auto-load on insert misses them):
  ```
  # /etc/modprobe.d/88XXau.conf
  options 88XXau rtw_monitor_disable_1m=1 rtw_monitor_retransmit=0
  ```
  `rtw_monitor_disable_1m=1` makes a **rate-less** inject default to HT-MCS7 /
  VHT-MCS9 instead of the 1 Mbps floor; `rtw_monitor_retransmit=0` keeps
  single-shot (NO_ACK honored per-frame via the radiotap TX_FLAGS bit). Test with
  a **rate-less** inject (`-R 0` = no rate field) and read the decoded MCS on the
  witness:
  ```sh
  sudo ./scripts/mon-setup.sh <rtl-iface> 11
  sudo ./scripts/mon-setup.sh <witness-iface> 11
  sudo ./vwifi-linkbench capture <witness-iface> -t 8 &
  sudo ./vwifi-linkbench inject  <rtl-iface> -R 0 -s 1500 -t 6      # rate-less
  # also try -M 7 (explicit HT) and -R 24 (explicit legacy)
  ```
  If the witness shows `HT-MCS7` / `VHT-MCS9` and inject throughput jumps well
  above 1 Mbps, the fix is to make the bridge's **capture+inject radio the
  Realtek** (capture rate is irrelevant; only inject matters), keeping the AR9271
  as the AP-opmode SIFS ACK.
- **CONFIRMED + IMPLEMENTED.** On an RTL8812AU (aircrack-ng driver): rate-less
  inject → **VHT-MCS8, 37 Mbps** (3110 fps); `-M 7` → HT-MCS7, 37.7 Mbps;
  `-R 24` → 30 Mbps — all witnessed, all ~37× the mt76 ceiling. The bridge now
  supports `-M <mcs>` (HT MCS) and `-L` (rate-less); `lab-bringup.sh` takes
  `INJECT_MCS=` / `INJECT_RATELESS=1`. **Deployment:** point the bridge's radio
  (`MT_IF`) at the Realtek and pick a rate, e.g.:
  ```sh
  sudo AR9271_IF=<ar9271> MT_IF=<realtek> INJECT_MCS=4 ./scripts/lab-bringup.sh
  ```
  Start at a **moderate MCS (≈4, 39 Mbps)** and raise it while watching the phone:
  the rate-less VHT-MCS8 is aggressive (a witness saw ~57 % frame loss, though
  much of that is the mt76 *witness* not keeping up with 3110 fps, not true air
  loss). A lower MCS trades peak rate for delivery on the co-located channel;
  `-r`/separated radios/quiet channel still apply since NO_ACK has no L2 retransmit.
- **Probes** (`vwifi-linkbench inject`): `-R 0` rate-less (driver default);
  `-M <mcs>` HT; `-R <mbps>` legacy; `-s a,b,c` size sweep; `-A` drop NO_ACK;
  `-Q`/`-B` qdisc/sndbuf. `capture` decodes the ground-truth on-air rate
  (legacy Mbps, HT-MCS, or VHT-MCS). On mt76/ath9k_htc every variant is 1 Mbps.
- **Downlink loss still isn't retransmitted at L2** (NO_ACK), whether the loss is
  in the air or in the driver's monitor TX queue (`ENOBUFS`). Characterize which
  before tuning further (§7.5).
- **Remaining throughput levers**: the inject-queue ceiling (§7.5), a quieter
  channel, physically separated radios, and `INJECT_COPIES`. The thin ~600 ms
  over-air beacon cadence still drives the periodic beacon-loss reconnects that
  stall traffic (a stability, not peak-rate, limit).

## 7.5 Characterizing throughput (isolating the bottleneck)

When end-to-end download is low but the on-air rate is fine, the cap is
frames-per-second somewhere in the relay, not bits-per-second on air. Two
instruments isolate it without TCP/VM/phone in the path:

**Live bridge counters (`-S`).** Run the bridge with `-S 2` (or `INJECT_RATE=…`
plus edit the script, or send `SIGUSR1`) to get a per-2s readout:

```
bridge: stats 2.0s | hub->phys 431 fps 5.03 Mbps drop(enobufs=812 eagain=0 err=0) \
        | phys->hub 88 fps 0.9 Mbps | echoes 431
```

Read it like this:
- **`drop(enobufs=…)` climbing** → the mt76 monitor TX queue is the ceiling; the
  driver is silently discarding injects (this loss is invisible to any
  speedtest). Lever: fewer copies (`-r 1`), a powered USB port, a different mt76
  queue/driver setting, or pacing the injector.
- **`hub->phys fps` low with ~0 drops** → the bridge isn't being *offered* more
  frames; the bottleneck is upstream (VM rate control / medium), not the air.
- **high inject fps + drops ~0 but download still low** → the loss is in the air
  (single-shot NO_ACK on the co-located channel): separate the radios, quieter
  channel.

**`vwifi-linkbench` — an iperf for the PHY relay.** Measures each leg's raw
ceiling directly:

Set each radio up first — **monitor mode on a channel**, and both radios of a
pair on the *same* channel. `scripts/mon-setup.sh` does this and verifies it
(it hard-fails if the channel didn't take — the #1 mistake, which otherwise
leaves the radio "up" but off-frequency so injects vanish into a void and
linkbench prints an impossible multi-Gbps rate):

```sh
sudo ./scripts/mon-setup.sh wlx00c0cab57e6f 11      # sink radio
sudo ./scripts/mon-setup.sh wlx90de80152b9e 11      # source radio (same channel!)

# Inject ceiling of one radio (tear the lab down first so nothing else drives
# it). Sweep -R and frame size; watch accepted fps/Mbps and the ENOBUFS %:
sudo ./vwifi-linkbench inject wlx00c0cab57e6f -R 6 -s 1500 -t 10 -v

# Full air path + loss, two monitor radios on the same channel:
sudo ./vwifi-linkbench capture wlx00c0cab57e6f -t 12 &     # sink
sudo ./vwifi-linkbench inject  wlx90de80152b9e -R 6 -t 10   # source
# capture prints air-loss % from the injector's VWLB sequence stream.
```

A blocking inject that sustains an impossibly high fps (tens of thousands+) means
the frames are **not reaching the radio** — the interface isn't on a channel or
isn't really in monitor mode. Re-run `mon-setup.sh` before trusting any number.

If `linkbench inject` sustains, say, 400+ fps at 6 Mbps with near-zero ENOBUFS,
the inject path is not the cap and the problem is air loss or upstream offer; if
it plateaus at ~40–80 fps with heavy ENOBUFS, the monitor TX queue is the wall
and rate tuning can't help until that's addressed.

---

## 8. Component map

| Piece | Where | Role |
|---|---|---|
| `vwifi-medium` | host | the hub / medium |
| `vwifi_host.c` | host kernel module | a host-side virtual radio on the medium (also had a `SW_CRYPTO_CONTROL` crypto bug, now fixed) |
| emulated `ath9k` | in each VM | the VM's radio — needs `nohwcrypt=1` |
| `vwifi-phys-bridge` | host | capture/inject relay between real air and the medium (MT7921U) |
| `vwifi-linkbench` | host | iperf-style inject/capture microbenchmark for the PHY relay (§7.5) |
| AR9271 + hostapd decoys | host | the SIFS ACK for the VM BSSID range |
| `scripts/lab-bringup.sh` | host | one-command bring-up |
