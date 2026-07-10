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

Sections 0–6 are about making association/data *work at all*. **Throughput** is a
separate axis, covered in §7: the four blockers below get you connected, but the
downlink rate is capped by *which radio injects* — mt76/ath9k_htc are stuck at
1 Mbps, an RTL8812AU reaches tens of Mbps (measured 0.28 → ~7 Mbps end to end).

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
| `-D <n>`     | inject each **encrypted data** frame `n` times (1..4, default 1). On a fast radio with airtime to spare, 2–3 masks the single-shot NO_ACK downlink's air loss from TCP → goodput climbs toward the PHY rate. `INJECT_DATA_COPIES=` in `lab-bringup.sh`. |
| `-S <sec>`   | periodic per-direction fps/Mbps + inject-drop report (`STATS_INTERVAL=`). Read it during a phone test to see if downlink (`hub->phys`) or uplink (`phys->hub`) is the limit. |
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

## 7. Throughput and constraints

Association works on any of the radios; **throughput does not** — it is gated by
which radio injects the downlink. This section covers that (7.1), the standing
constraints (7.2), and how to measure where a given setup is limited (7.3).

## 7.1 Downlink throughput — the inject radio is everything

The uplink is fine by construction: the AR9271 hardware-ACKs it and the STA does
normal rate control + L2 retransmit, so it runs near link speed. **The downlink
is the whole story** — it is injected single-shot (NO_ACK, no L2 retransmit), so
a download-heavy test (speedtest, browsing) rides entirely on it, and its ceiling
is set by **what rate the inject radio actually transmits at.**

**The rate you request in radiotap is not necessarily what goes on air — and it
is driver-specific.** Measured, ground-truth (witnessed on a *separate* radio;
an injecting radio's self-capture only echoes the *requested* rate):

| Inject radio | Honors requested rate? | On-air result |
|---|---|---|
| **MT7921U** (`mt76`) | No — ignores legacy RATE *and* HT MCS | pinned to **1 Mbps** |
| **AR9271** (`ath9k_htc`) | No, *and* ignores NO_ACK (retransmits ~10–15×) | **1 Mbps**, ~11 fps |
| **RTL8812AU** (`rtl88xxau`, aircrack-ng) | **Yes** — legacy, HT MCS, and rate-less | up to **VHT-MCS8 / ~37 Mbps** |

So on mt76/ath9k_htc the downlink is hard-capped at ~1 Mbps (~0.5 Mbps TCP) no
matter what `-R`/`-M`/`-L` you pass — not a bug in the bridge (it builds the
radiotap correctly), a driver limitation. **Use a Realtek RTL8812AU/8814AU as the
bridge's capture+inject radio** (capture rate is irrelevant; only inject matters),
keeping the AR9271 as the AP-opmode SIFS-ACK radio.

### Realtek setup

```
# /etc/modprobe.d/88XXau.conf   — then reload the driver so options take
# (auto-load on insert misses modprobe.d): modprobe -r 88XXau && modprobe 88XXau
options 88XXau rtw_monitor_disable_1m=1 rtw_monitor_retransmit=0
```

`rtw_monitor_disable_1m=1` lifts the 1 Mbps floor (a rate-less inject then
defaults to HT-MCS7/VHT-MCS9); `rtw_monitor_retransmit=0` keeps single-shot so
per-frame NO_ACK is honored. Confirm the on-air rate before deploying (§7.3).

### Deploy + tune

```sh
sudo AR9271_IF=<ar9271> MT_IF=<realtek> INJECT_MCS=4 ./scripts/lab-bringup.sh
```

- **`INJECT_MCS=<n>`** (`-M`): fixed HT MCS — the recommended mode (deterministic,
  tunable). Start at a **moderate MCS 4** (39 Mbps PHY) and raise while watching
  the phone; higher MCS = more peak rate but needs better SNR on the co-located
  channel. `INJECT_RATELESS=1`/`-L` uses the driver default (VHT-MCS8 — fast but
  aggressive); legacy `-R <mbps>` also works. (Only relevant to synthetic
  `linkbench` at a *fixed* offered rate: offering more than the PHY can carry
  silently drops the excess — e.g. 1000 fps × 1500 B = 12 Mbps into MCS0's
  6.5 Mbps loses ~half. TCP self-limits, so this doesn't bite in deployment.)
- **`INJECT_DATA_COPIES=<n>`** (`-D`): inject each data frame `n`× to mask air
  loss from TCP. **Only helps on a genuinely lossy link** — at MCS4 the air loss
  is ~2 %, which is *not* the binding constraint, so `-D 2` there just doubles
  airtime for no goodput gain (measured). Leave at 1 unless a loss test (§7.3)
  shows real loss.
- **`INJECT_COPIES=<n>`** (`-r`): duplicate association-critical mgmt/EAPOL frames
  (2–3 on a lossy channel) — for reliable *association*, not steady-state rate.

### Measured result

0.28 Mbps → **~7 Mbps download (~25×)** at MCS4. The relay is not the cap (bridge
`-S` shows 0 drops; capture does ~10 Mbps on upload; air loss ~2 %). The residual
limit is **TCP over the single-shot downlink plus periodic full-connection
stalls** (beacon-loss reconnects — the `0.0×` intervals where *both* directions
collapse together in the `-S` log), and bufferbloat (loaded RTT ~240 ms). Those
are stability/environmental issues: the standing levers are **physically separate
the two radios**, a **quieter channel**, and (future work) redundant beacon
injection. Raising MCS or `-D` will not move them.

## 7.2 Other constraints

- **Single channel** for all phone-joinable APs (one radio pair) — congestion
  directly causes the periodic reconnects, so pick the quietest channel you can.
- Joinable APs must be **legacy / no Block-Ack / no RTS-CTS** (`ieee80211n=0`,
  `wmm_enabled=0`, `htmode=NONE`); those extras are additional SIFS responses the
  bridge doesn't generate. APs serving only *virtual* clients can stay HT/VHT/HE.
- **Downlink loss isn't retransmitted at L2** (NO_ACK); rely on a clean channel,
  separated radios, and upper-layer retries.
- **USB radios flap** under sustained load; the bridge self-heals (reopens the
  socket), but a powered hub / separate ports / lower `-r` mean fewer blips.
- The radios are **co-located transmitters** — physical separation is the single
  best lever for the periodic beacon-loss reconnects.

## 7.3 Characterizing throughput (isolating the bottleneck)

To find where a setup is limited, isolate each leg. Two instruments do it — one
on the live path, one synthetic without TCP/VM/phone.

**Live bridge counters (`STATS_INTERVAL=2` / `-S 2`, or send `SIGUSR1`).** During
a phone speedtest, `tail -f /run/vwifi-lab/bridge.log | grep stats`:

```
bridge: stats 2.0s | hub->phys 330 fps 7.52 Mbps (air 15.0) drop(enobufs=0 eagain=0 err=0) | phys->hub 231 fps 0.25 Mbps | echoes 0
```

`hub->phys` Mbps is downlink **goodput** (matches the client); `(air N)` appears
only when `-r`/`-D` copies make on-air bytes exceed it. `phys->hub` is the uplink
(TCP ACKs on download; data on upload). Read it:
- **`drop(enobufs=…)` climbing** → the inject radio's TX queue is the ceiling
  (silent loss, invisible to a speedtest). Lever: fewer copies, pacing, better USB.
- **`hub->phys` Mbps low, ~0 drops** → the bridge isn't being *offered* more; the
  limit is upstream (VM rate control / medium), not the air.
- **high `hub->phys`, ~0 drops, phone still slow** → downlink air loss (quantify
  with linkbench below) or TCP/RTT.
- **both directions periodically collapse to `~0` together** → a full-connection
  stall (beacon-loss reconnect); a stability problem, not rate. Separate radios,
  quieter channel.

**`vwifi-linkbench` — an iperf for the PHY relay.** Measures each leg's raw
ceiling directly, and — the crux of the whole throughput story — reports the
**ground-truth on-air rate** (`capture` decodes legacy Mbps / HT-MCS / VHT-MCS
from the RX radiotap) and **kernel capture drops** (`PACKET_STATISTICS`, so you
can tell real air loss from the witness overflowing):

Set each radio up first — **monitor mode on a channel**, and both radios of a
pair on the *same* channel. `scripts/mon-setup.sh` does this and verifies it
(it hard-fails if the channel didn't take — the #1 mistake, which otherwise
leaves the radio "up" but off-frequency so injects vanish into a void and
linkbench prints an impossible multi-Gbps rate):

```sh
sudo ./scripts/mon-setup.sh wlx00c0cab57e6f 11      # sink radio
sudo ./scripts/mon-setup.sh wlx90de80152b9e 11      # source radio (same channel!)

# Does this radio honor the injected rate? Inject on it, witness on the other.
# (mt76/ath9k_htc => 1 Mbps; RTL8812AU => the MCS you asked for.)
sudo ./vwifi-linkbench capture <witness> -t 10 &
sudo ./vwifi-linkbench inject  <radio>   -M 4 -s 1500 -t 8 -p 1000

# Air-loss vs MCS at a realistic paced rate (capture drop must read ~0 to trust
# the loss figure): sweep -M 0,4,7. Note a rate offered above the PHY rate shows
# as "loss" that is really overdrive (e.g. 1000fps*1500B=12Mbps into MCS0's 6.5).
```

`inject` probes: `-M <mcs>` HT · `-R <mbps>` legacy · `-R 0`/`-L` rate-less ·
`-s a,b,c` size sweep · `-p <pps>` pace · `-A` drop NO_ACK · `-Q`/`-B` qdisc/sndbuf.
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
| `vwifi-phys-bridge` | host | capture/inject relay between real air and the medium (best: RTL8812AU inject radio) |
| `vwifi-linkbench` | host | iperf-style inject/capture microbenchmark + on-air rate / air-loss measurement for the PHY relay (§7.3) |
| AR9271 + hostapd decoys | host | the SIFS ACK for the VM BSSID range |
| `scripts/lab-bringup.sh` | host | one-command bring-up (`INJECT_MCS=`, `INJECT_DATA_COPIES=`, `STATS_INTERVAL=`, …) |
| `scripts/mon-setup.sh` | host | put a radio in monitor mode on a channel and verify it took (§7.3) |
