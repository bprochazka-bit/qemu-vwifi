#!/usr/bin/env bash
#
# lab-bringup.sh — single-host bring-up for "real STA joins a virtual OpenWRT
# VM AP through the vwifi medium", using the AP-opmode ACK trick on an AR9271.
#
# Architecture (see docs/ar9271-phys-bridge-lab.md):
#
#   AR9271  (ath9k_htc)  2 hostapd BSS at ...:44:00 and ...:44:FF, channel 11
#                        -> chip is in AP opmode, so the PCU hardware-ACKs any
#                           frame matching the auto-computed bssidmask
#                           ff:ff:ff:ff:ff:00, i.e. 02:11:22:33:44:00..FF.
#                        -> hostapd only OWNS :00 and :FF (decoy "wrong" APs),
#                           so it never answers management for the VM BSSIDs;
#                           it just supplies the SIFS ACK for the whole range.
#   MT7921U (mt76)       monitor on channel 11: captures the STA uplink and
#                        injects the VM downlink (the vwifi-phys-bridge). Also
#                        doubles as the witness that can see the AR9271's ACKs.
#   OpenWRT VM(s)        the REAL APs at 02:11:22:33:44:01..FE (do all
#                        Auth/Assoc/EAPOL; the phone lands in the VM's table).
#
# Only the SIFS ACK is on the hardware hot path; everything else is relayed to
# the VM over the medium. Run this on the bridge HOST. Configure the VM APs
# separately (see the UCI block printed at the end, or docs/).
#
# Usage:
#   sudo ./scripts/lab-bringup.sh          # bring up + witness capture
#   sudo ./scripts/lab-bringup.sh stop     # tear everything down
#
set -euo pipefail

# ----------------------------------------------------------------------------
# Configuration — adjust the two interface names to your machine.
# ----------------------------------------------------------------------------
AR9271_IF="${AR9271_IF:-wlxc01c300da281}"   # AR9271: the AP / ACK radio
MT_IF="${MT_IF:-mon0}"                       # MT7921U: capture + inject radio

CHANNEL="${CHANNEL:-11}"                      # single lab channel
HUB_SOCK="${HUB_SOCK:-/run/vwifi/foo.sock}"   # vwifi hub socket

# BSSID plan. The two DECOY BSSIDs owned by the host sit at the extremes of the
# range so the driver's auto-mask spans the whole last byte. VM APs live in
# between (02:11:22:33:44:01 .. :FE) and are ACKed via that mask.
BSSID_LO="02:11:22:33:44:00"                  # host decoy A (mask low end)
BSSID_HI="02:11:22:33:44:FF"                  # host decoy B (mask high end)
FILTER_BASE="02:11:22:33:44:00"               # bridge relevance-filter base
FILTER_MASK="ff:ff:ff:ff:ff:00"               # -> forwards ...:44:00..FF

SSID_LO="Lab-Decoy-A"
SSID_HI="Lab-Decoy-B"

# Where this script lives -> find the bridge binary next to the repo root.
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
BRIDGE_BIN="${BRIDGE_BIN:-$REPO_DIR/vwifi-phys-bridge}"

RUN_DIR="/run/vwifi-lab"
HOSTAPD_CONF="$RUN_DIR/hostapd-ack.conf"
WITNESS_PCAP="${WITNESS_PCAP:-/tmp/lab-witness.pcap}"
PHONE_MAC="${PHONE_MAC:-}"                     # optional: for the ACK check

# ----------------------------------------------------------------------------
say()  { printf '\033[1;36m==>\033[0m %s\n' "$*"; }
warn() { printf '\033[1;33m[!]\033[0m %s\n' "$*" >&2; }
die()  { printf '\033[1;31m[x]\033[0m %s\n' "$*" >&2; exit 1; }

need_root() { [ "$(id -u)" = 0 ] || die "run with sudo"; }

teardown() {
    say "Tearing down..."
    pkill -f "vwifi-phys-bridge .*$MT_IF" 2>/dev/null || true
    pkill -f "hostapd .*$HOSTAPD_CONF"    2>/dev/null || true
    pkill -f "tcpdump -i $MT_IF"          2>/dev/null || true
    sleep 1
    ip link set "$AR9271_IF" down 2>/dev/null || true
    ip link set "$MT_IF" down 2>/dev/null || true
    say "Down. (radios left in monitor/down; re-run to bring up again)"
}

if [ "${1:-}" = "stop" ]; then need_root; teardown; exit 0; fi

# ----------------------------------------------------------------------------
# Preflight
# ----------------------------------------------------------------------------
need_root
say "Preflight checks"
for t in iw ip hostapd tcpdump; do
    command -v "$t" >/dev/null || die "missing tool: $t"
done
[ -x "$BRIDGE_BIN" ] || die "bridge binary not found/executable: $BRIDGE_BIN (run 'make' in $REPO_DIR)"
ip link show "$AR9271_IF" >/dev/null 2>&1 || die "AR9271 interface '$AR9271_IF' not found (set AR9271_IF=...)"
ip link show "$MT_IF"    >/dev/null 2>&1 || die "MT7921U interface '$MT_IF' not found (set MT_IF=...)"
[ -S "$HUB_SOCK" ] || warn "hub socket $HUB_SOCK not present yet — the bridge will retry; start the hub/VMs."

mkdir -p "$RUN_DIR"

# Stop anything that will fight hostapd for the AR9271.
say "Releasing $AR9271_IF from NetworkManager/wpa_supplicant (if present)"
nmcli device set "$AR9271_IF" managed no 2>/dev/null || true
pkill -f "wpa_supplicant.*$AR9271_IF" 2>/dev/null || true
teardown_quiet() { pkill -f "vwifi-phys-bridge .*$MT_IF" 2>/dev/null || true
                   pkill -f "hostapd .*$HOSTAPD_CONF" 2>/dev/null || true
                   pkill -f "tcpdump -i $MT_IF" 2>/dev/null || true; }
teardown_quiet
sleep 1

# ----------------------------------------------------------------------------
# 1. AR9271 -> two-BSS hostapd (AP opmode + bssidmask that covers the VM range)
# ----------------------------------------------------------------------------
say "Configuring AR9271 ($AR9271_IF): decoy APs $BSSID_LO / $BSSID_HI on ch $CHANNEL"
ip link set "$AR9271_IF" down
# First BSS uses the interface MAC; pin it to the low decoy BSSID.
iw dev "$AR9271_IF" set type managed 2>/dev/null || true
ip link set "$AR9271_IF" address "$BSSID_LO"

cat > "$HOSTAPD_CONF" <<EOF
# Decoy/ACK APs on the AR9271. hostapd OWNS only these two BSSIDs; their sole
# lab purpose is to put the chip in AP opmode and make the auto-computed
# bssidmask span 02:11:22:33:44:00..FF so the PCU hardware-ACKs the VM BSSIDs.
# Legacy (no HT / no WMM) => simple per-frame ACK, no Block-Ack / RTS-CTS.
interface=$AR9271_IF
driver=nl80211
hw_mode=g
channel=$CHANNEL
ieee80211n=0
wmm_enabled=0
ap_isolate=1

ssid=$SSID_LO
bssid=$BSSID_LO

bss=lab-decoy-b
ssid=$SSID_HI
bssid=$BSSID_HI
EOF

say "Starting hostapd (2 BSS)"
hostapd -B -P "$RUN_DIR/hostapd.pid" "$HOSTAPD_CONF" \
    || die "hostapd failed to start — check 'hostapd $HOSTAPD_CONF' output"
sleep 2

# Verify both AP vifs exist at the extremes. The driver derives the hardware
# bssidmask from these two addresses: :00 XOR :FF = ...:FF => mask ...:ff:00,
# which covers the entire 02:11:22:33:44:xx range.
say "AR9271 vif addresses (expect $BSSID_LO and $BSSID_HI):"
iw dev | awk '/Interface/{i=$2} /addr/{print "    "i"  "$2}' \
    | grep -Ei "${AR9271_IF}|lab-decoy-b" || true
say "=> hardware bssidmask spans ff:ff:ff:ff:ff:00 (ACKs 02:11:22:33:44:00..FF)"

# ----------------------------------------------------------------------------
# 2. MT7921U -> monitor on the same channel (capture + inject + witness)
# ----------------------------------------------------------------------------
say "Configuring MT7921U ($MT_IF): monitor on ch $CHANNEL"
ip link set "$MT_IF" down
iw dev "$MT_IF" set type monitor
ip link set "$MT_IF" up
iw dev "$MT_IF" set channel "$CHANNEL"
iw dev "$MT_IF" info | grep -E "type|channel|addr" | sed 's/^/    /'

# ----------------------------------------------------------------------------
# 3. Witness capture (mon0 sees the AR9271's ACKs to the phone) + bridge
# ----------------------------------------------------------------------------
say "Starting witness capture -> $WITNESS_PCAP"
tcpdump -i "$MT_IF" -w "$WITNESS_PCAP" -U >/dev/null 2>&1 &
sleep 1

say "Starting bridge on $MT_IF (forwarding $FILTER_BASE / $FILTER_MASK)"
"$BRIDGE_BIN" "$HUB_SOCK" "$MT_IF" -c "$CHANNEL" \
    -b "$FILTER_BASE" -m "$FILTER_MASK" -v \
    > "$RUN_DIR/bridge.log" 2>&1 &
sleep 1

if ! pgrep -f "vwifi-phys-bridge .*$MT_IF" >/dev/null; then
    warn "bridge exited — last log lines:"; tail -n 20 "$RUN_DIR/bridge.log" >&2
fi

# ----------------------------------------------------------------------------
# Ready
# ----------------------------------------------------------------------------
cat <<EOF

$(say "LAB IS UP on channel $CHANNEL")

  AR9271  : hostapd 2-BSS ($BSSID_LO / $BSSID_HI)  -> ACK for ...:44:00..FF
  MT7921U : monitor + bridge on $MT_IF             -> capture/inject to the hub
  witness : $WITNESS_PCAP    bridge log: $RUN_DIR/bridge.log

NEXT — on the OpenWRT VM, move the AP OFF the host-owned :00 to e.g. :01 and
pin it to channel $CHANNEL, legacy mode (run inside the VM):

  uci set wireless.radio0.channel='$CHANNEL'
  uci set wireless.radio0.htmode='NONE'
  uci set wireless.default_radio0.macaddr='02:11:22:33:44:01'
  uci set wireless.default_radio0.ssid='Lab-Real'
  uci set wireless.default_radio0.encryption='psk2'
  uci set wireless.default_radio0.key='correcthorse1'
  uci commit wireless && wifi reload

Then join SSID "Lab-Real" from the phone and verify:

  # 1) Is the AR9271 finally ACKing the phone's uplink?  (>0 = YES, the fix works)
  #    Set PHONE_MAC and re-run this check, or substitute the phone's MAC:
  tshark -r $WITNESS_PCAP -Y 'wlan.fc.type_subtype==0x1d && wlan.ra==<PHONE_MAC>' | wc -l

  # 2) Did the phone land in the VM's station table (not the host's)?  Inside the VM:
  iw dev wlan0 station dump | grep -i station

  # 3) Bridge forwarding sanity:
  grep -c 'phys->hub' $RUN_DIR/bridge.log

Stop everything:  sudo $0 stop
EOF

if [ -n "$PHONE_MAC" ]; then
    say "Witnessing for 60s (PHONE_MAC=$PHONE_MAC). Join the phone now..."
    sleep 60
    n=$(tshark -r "$WITNESS_PCAP" -Y "wlan.fc.type_subtype==0x1d && wlan.ra==$PHONE_MAC" 2>/dev/null | wc -l || echo 0)
    if [ "$n" -gt 0 ]; then
        say "RESULT: $n ACKs to the phone — AR9271 AP-opmode ACK WORKS."
    else
        warn "RESULT: 0 ACKs to the phone. Check vif addresses / that the phone actually associated."
    fi
fi
