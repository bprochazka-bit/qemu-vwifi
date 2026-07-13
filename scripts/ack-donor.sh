#!/usr/bin/env bash
#
# ack-donor.sh — stand up a radio as a *stateless SIFS-ACK donor* for one or
# more BSSIDs, reliably, and KEEP it correct.
#
# Why this exists
# ---------------
# The lab needs a real radio whose PCU hardware emits the SIFS ACK for the VM
# BSSID range (see docs/ar9271-phys-bridge-lab.md). The ACK is pure hardware,
# gated by the chip's bssidmask; the mask is the common bits of the BSSIDs
# configured on the radio's AP vifs. hostapd is only there to hold the vifs in
# AP opmode and beacon them.
#
# The pain: on ath9k_htc (AR9271) the chip has ONE hardware MAC, and mac80211
# reshuffles per-vif MACs to fit the mask -- sometimes reassigning a pinned MAC
# *asynchronously*, on the second-vif add or at AP start. A pin-then-sleep-
# then-check-once flow (as in lab-bringup.sh) can pass on a transient-good
# state that drifts a moment later, and has no runtime guard -> "it said the
# MACs were set, but they weren't." This tool fixes that with:
#   * configure + pin one vif at a time, primary first (the proven lab-bringup
#     order; churns the driver less than create-all-then-pin-all),
#   * a STABILITY GATE: declare ready only when every MAC is correct AND
#     unchanged for a full window of polls, not a single check,
#   * a WATCHDOG: keep verifying at runtime and RE-PIN on drift, so the mask
#     stays honest for the whole session,
#   * honest hard-fail with diagnostics if a MAC simply won't hold (some
#     drivers can't pin multi-BSS -- then you need a PCI ath9k, or one BSSID).
#
# It is a *stateless* donor by design: AP opmode + beacon + the SIFS ACK, and
# nothing else. It does NOT and CANNOT do BlockAcks (a BlockAck is a stateful
# per-sequence claim that only the real receiver can make -- see the project
# discussion). Its whole job is the one thing a non-receiver may honestly
# assert: the ACK.
#
# Usage:
#   sudo ACK_IF=wlxc01c300da281 CHANNEL=11 \
#        BSSIDS="02:11:22:33:44:00 02:11:22:33:44:03" \
#        ./scripts/ack-donor.sh
#
#   # Multiple/arbitrary BSSIDs: the covering mask is computed and printed,
#   # along with every address it will ACK (watch for over-coverage).
#   sudo ACK_IF=wlan0 CHANNEL=36 \
#        BSSIDS="02:11:22:33:44:00 02:11:22:33:44:03 02:11:22:33:44:0c" \
#        ./scripts/ack-donor.sh
#
#   sudo ... ./scripts/ack-donor.sh stop     # tear down vifs/hostapd
#
# Env knobs: SSID_PREFIX (default "ack-donor"), BEACON_INT (default 1000),
#   PIN_RETRIES (default 6), STABLE_CHECKS (default 6), STABLE_INTERVAL_MS
#   (default 300), WATCHDOG=1 (default; 0 = set up and exit).
#
set -uo pipefail

say()  { printf '\033[1;36m==>\033[0m %s\n' "$*"; }
ok()   { printf '  \033[1;32m[ok]\033[0m %s\n' "$*"; }
warn() { printf '  \033[1;33m[!]\033[0m %s\n' "$*" >&2; }
die()  { printf '\033[1;31m[x]\033[0m %s\n' "$*" >&2; exit 1; }

# ---- config -----------------------------------------------------------------
ACK_IF="${ACK_IF:-}"
CHANNEL="${CHANNEL:-11}"
BSSIDS="${BSSIDS:-}"
SSID_PREFIX="${SSID_PREFIX:-ack-donor}"
BEACON_INT="${BEACON_INT:-1000}"
PIN_RETRIES="${PIN_RETRIES:-6}"
STABLE_CHECKS="${STABLE_CHECKS:-6}"
STABLE_INTERVAL_MS="${STABLE_INTERVAL_MS:-300}"
WATCHDOG="${WATCHDOG:-1}"

RUN_DIR="${RUN_DIR:-/run/ack-donor}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

[ "$(id -u)" = 0 ] || die "run with sudo"
for t in iw ip hostapd; do command -v "$t" >/dev/null || die "missing tool: $t"; done
[ -n "$ACK_IF" ] || die "set ACK_IF=<radio interface>"
ip link show "$ACK_IF" >/dev/null 2>&1 || die "interface '$ACK_IF' not found"

# Managed vif names we create for the 2nd..Nth BSSID.
extra_vif() { printf 'ackd%d' "$1"; }

# ---- teardown ---------------------------------------------------------------
teardown() {
    say "Tearing down"
    pkill -f "hostapd .*$RUN_DIR/" 2>/dev/null || true
    sleep 1
    local i=1
    while [ "$i" -lt 32 ]; do
        iw dev "$(extra_vif "$i")" del 2>/dev/null || true
        i=$((i+1))
    done
    # Hand the radio back to NetworkManager so it is usable again.
    iw dev "$ACK_IF" set type managed 2>/dev/null || true
    nmcli device set "$ACK_IF" managed yes 2>/dev/null || true
}
if [ "${1:-}" = "stop" ]; then teardown; exit 0; fi

# ---- MAC helpers ------------------------------------------------------------
norm_mac() { printf '%s' "$1" | tr 'A-F' 'a-f'; }
read_mac() { tr 'A-F' 'a-f' < "/sys/class/net/$1/address" 2>/dev/null || echo ""; }

valid_mac() { [[ "$1" =~ ^([0-9a-fA-F]{2}:){5}[0-9a-fA-F]{2}$ ]]; }

# Pin $2 as the MAC of iface $1, verifying it actually stuck. Retries with the
# interface down (the only state in which ath9k reliably honors the change).
pin_mac() {
    local iface="$1" want; want="$(norm_mac "$2")"
    local n=0 got
    while [ "$n" -lt "$PIN_RETRIES" ]; do
        ip link set "$iface" down 2>/dev/null || true
        ip link set "$iface" address "$want" 2>/dev/null || true
        got="$(read_mac "$iface")"
        if [ "$got" = "$want" ]; then
            return 0
        fi
        n=$((n+1))
        sleep 0.3
    done
    got="$(read_mac "$iface")"
    warn "could not pin $iface to $want after $PIN_RETRIES tries (got '$got')"
    return 1
}

# ---- coverage math: mask = common bits of all BSSIDs; enumerate ACKed set ---
# Sets globals COVER_MASK and COVER_LIST.
compute_coverage() {
    local -a macs=("$@")
    local i b and or
    # AND and OR across all six octets.
    local -a A=(255 255 255 255 255 255) O=(0 0 0 0 0 0)
    for m in "${macs[@]}"; do
        IFS=: read -ra P <<<"$(norm_mac "$m")"
        for i in 0 1 2 3 4 5; do
            b=$((16#${P[i]}))
            A[i]=$(( A[i] & b )); O[i]=$(( O[i] | b ))
        done
    done
    # mask bit = 1 where AND==OR (all agree), else 0.
    local -a MK=()
    local diffbits=0
    for i in 0 1 2 3 4 5; do
        MK[i]=$(( 255 & ~(A[i] ^ O[i]) ))
        local x=$(( A[i] ^ O[i] ))
        while [ "$x" -gt 0 ]; do diffbits=$((diffbits + (x & 1))); x=$((x >> 1)); done
    done
    COVER_MASK="$(printf '%02x:%02x:%02x:%02x:%02x:%02x' "${MK[0]}" "${MK[1]}" "${MK[2]}" "${MK[3]}" "${MK[4]}" "${MK[5]}")"
    COVER_COUNT=$(( 1 << diffbits ))
}

# ---- build the vif/BSSID plan ----------------------------------------------
read -ra BSSID_ARR <<<"$(printf '%s' "$BSSIDS" | tr ',' ' ')"
[ "${#BSSID_ARR[@]}" -ge 1 ] || die "set BSSIDS=\"mac1 [mac2 ...]\""
[ "${#BSSID_ARR[@]}" -le 8 ] || die "at most 8 BSSIDs supported"
for m in "${BSSID_ARR[@]}"; do valid_mac "$m" || die "bad BSSID: $m"; done

case "$CHANNEL" in ''|*[!0-9]*) die "CHANNEL must be numeric";; esac

PHY="$(iw dev "$ACK_IF" info 2>/dev/null | awk '/wiphy/{print "phy"$2; exit}')"
[ -n "$PHY" ] || die "could not determine phy for $ACK_IF"

mkdir -p "$RUN_DIR"
say "Stateless ACK donor on $ACK_IF ($PHY), channel $CHANNEL, ${#BSSID_ARR[@]} BSSID(s)"

# vif list: primary uses ACK_IF, extras are ackd1..ackdN.
declare -a IFACES=() WANT=()
IFACES+=("$ACK_IF");         WANT+=("$(norm_mac "${BSSID_ARR[0]}")")
for i in $(seq 1 $(( ${#BSSID_ARR[@]} - 1 )) ); do
    IFACES+=("$(extra_vif "$i")"); WANT+=("$(norm_mac "${BSSID_ARR[i]}")")
done

# ---- 1. clean slate ---------------------------------------------------------
teardown >/dev/null 2>&1 || true
sleep 1

# Release the radio from anything that will fight hostapd for it. Without this,
# hostapd fails with "Match already configured / Could not configure driver
# mode" because NetworkManager/wpa_supplicant still holds the interface.
say "Releasing $ACK_IF from NetworkManager / wpa_supplicant"
nmcli device set "$ACK_IF" managed no 2>/dev/null || true
pkill -f "wpa_supplicant.*$ACK_IF" 2>/dev/null || true
rfkill unblock wifi 2>/dev/null || true

# Configure + pin each vif, PRIMARY FIRST, then add+pin each extra one at a
# time. This mirrors the proven lab-bringup order and churns the driver far
# less than create-all-then-pin-all (which left the primary unable to enter AP
# mode: "Could not configure driver mode"). The primary stays *managed* and
# hostapd does the managed->AP transition itself; extras are fresh __ap vifs.
say "Configuring + pinning vifs (primary first)"
ip link set "$ACK_IF" down 2>/dev/null || true
iw dev "$ACK_IF" set type managed 2>/dev/null || true
pin_mac "$ACK_IF" "${WANT[0]}" || die "could not pin primary $ACK_IF to ${WANT[0]}"
ok "$ACK_IF = ${WANT[0]}"
for i in $(seq 1 $(( ${#BSSID_ARR[@]} - 1 )) ); do
    v="$(extra_vif "$i")"
    iw dev "$v" del 2>/dev/null || true
    iw phy "$PHY" interface add "$v" type __ap \
        || die "could not create vif $v (driver may not support multi-BSS)"
    pin_mac "$v" "${WANT[i]}" \
        || die "could not pin $v to ${WANT[i]} -- driver may not support pinned multi-BSS"
    ok "$v = ${WANT[i]}"
done

# Start a minimal legacy hostapd on a vif, with retry + log capture. ath9k_htc
# intermittently rejects the managed->AP config ("Could not configure driver
# mode"); a kill+retry usually clears it, the same way rerunning lab-bringup
# does. On persistent failure the real hostapd log is printed.
start_hostapd() {   # $1=idx  $2=iface
    local idx="$1" v="$2" conf="$RUN_DIR/hostapd-$idx.conf" log="$RUN_DIR/hostapd-$idx.log" n=0
    cat > "$conf" <<EOF
interface=$v
driver=nl80211
hw_mode=g
channel=$CHANNEL
ieee80211n=0
wmm_enabled=0
ap_isolate=1
beacon_int=$BEACON_INT
ssid=${SSID_PREFIX}-$idx
bssid=${WANT[idx]}
EOF
    while [ "$n" -lt 5 ]; do
        pkill -f "$RUN_DIR/hostapd-$idx.conf" 2>/dev/null || true
        sleep 0.5
        if hostapd -B -P "$RUN_DIR/hostapd-$idx.pid" "$conf" >"$log" 2>&1; then
            return 0
        fi
        n=$((n+1))
        warn "hostapd on $v failed (try $n/5); retrying..."
        sleep 1
    done
    warn "hostapd on $v would not start. Last log:"
    sed 's/^/      /' "$log" >&2 2>/dev/null || true
    return 1
}

say "Starting beacons (legacy, no HT/WMM -> simple SIFS ACK only)"
for idx in "${!IFACES[@]}"; do
    start_hostapd "$idx" "${IFACES[idx]}" || die "hostapd would not start on \
${IFACES[idx]}. If this radio can't hold multiple AP vifs, try a single BSSID \
(BSSIDS=\"<one>\") or a PCI ath9k. Log: $RUN_DIR/hostapd-$idx.log"
done

# ---- 4. STABILITY GATE: correct AND unchanged for a full window ------------
say "Verifying stability (this is what the old single check missed)"
gate_ok=0
attempt=0
while [ "$attempt" -lt "$PIN_RETRIES" ]; do
    stable=1
    c=0
    while [ "$c" -lt "$STABLE_CHECKS" ]; do
        for idx in "${!IFACES[@]}"; do
            got="$(read_mac "${IFACES[idx]}")"
            if [ "$got" != "${WANT[idx]}" ]; then
                warn "${IFACES[idx]} drifted to '$got' (want ${WANT[idx]}) -- re-pinning"
                pkill -f "hostapd .*$RUN_DIR/hostapd-$idx" 2>/dev/null || true
                pin_mac "${IFACES[idx]}" "${WANT[idx]}" || true
                start_hostapd "$idx" "${IFACES[idx]}" || true
                stable=0
                break
            fi
        done
        [ "$stable" = 1 ] || break
        c=$((c+1))
        sleep "$(awk "BEGIN{print $STABLE_INTERVAL_MS/1000}")"
    done
    if [ "$stable" = 1 ]; then gate_ok=1; break; fi
    attempt=$((attempt+1))
done
[ "$gate_ok" = 1 ] || die "MACs will not hold stable (a MAC keeps drifting -- the \
driver is fighting the pin; this radio likely can't do stable pinned multi-BSS)."

# ---- 5. report coverage -----------------------------------------------------
compute_coverage "${WANT[@]}"
say "READY — SIFS ACK donor is up and stable"
ok "vifs:"
for idx in "${!IFACES[@]}"; do printf '      %-10s %s\n' "${IFACES[idx]}" "${WANT[idx]}"; done
ok "covering mask (implied): $COVER_MASK  -> ACKs $COVER_COUNT address(es)"
if [ "$COVER_COUNT" -gt "${#BSSID_ARR[@]}" ]; then
    warn "mask over-covers: it will ACK $COVER_COUNT addresses but you named ${#BSSID_ARR[@]}."
    warn "In a lab that's usually harmless (a few extra ACKed addresses); to tighten it,"
    warn "choose BSSIDs whose differing bits form a contiguous power-of-two block."
fi
say "This donates ONLY the SIFS ACK. It does not do BlockAcks (impossible for a"
say "non-receiver). Run your AP legacy/non-aggregated so the ACK is all that's needed."

# ---- 6. watchdog (keeps the mask honest for the whole session) -------------
if [ "$WATCHDOG" != 1 ]; then
    say "WATCHDOG=0 -> setup complete, exiting (vifs/hostapd left running)."
    exit 0
fi
trap 'teardown; exit 0' INT TERM
say "Watchdog running (re-pins on drift). Ctrl-C to tear down."
while :; do
    sleep 3
    for idx in "${!IFACES[@]}"; do
        got="$(read_mac "${IFACES[idx]}")"
        if [ "$got" != "${WANT[idx]}" ]; then
            warn "$(date '+%H:%M:%S') ${IFACES[idx]} drifted to '$got' -- re-pinning ${WANT[idx]}"
            pkill -f "hostapd .*$RUN_DIR/hostapd-$idx" 2>/dev/null || true
            pin_mac "${IFACES[idx]}" "${WANT[idx]}" || true
            start_hostapd "$idx" "${IFACES[idx]}" || true
        fi
    done
done
