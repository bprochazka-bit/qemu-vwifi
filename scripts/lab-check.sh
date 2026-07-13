#!/usr/bin/env bash
#
# lab-check.sh — preflight for the phys-bridge lab: enumerate the Wi-Fi radios,
# verify each is assigned to the right ROLE and correctly configured, and fail
# loudly on the mistakes that make "the phone won't connect / connects but no
# DHCP" — most often a MISSING ACK RADIO.
#
# Why this exists
# ---------------
# The lab needs (at least) two radios with distinct jobs (docs/ar9271-phys-
# bridge-lab.md, scripts/lab-bringup.sh):
#
#   INJECT/MONITOR radio  — runs vwifi-phys-bridge: monitor mode on the lab
#                           channel, captures the STA uplink, injects the VM
#                           downlink. Best on a Realtek (rtl88xxau) so injected
#                           HT-MCS rates are honored; mt76/ath9k_htc ignore the
#                           rate and pin to ~1 Mbps.
#   ACK radio             — a real radio in AP opmode whose hardware bssidmask
#                           covers the VM BSSID range, so its PCU emits the
#                           SIFS ACK for the STA's uplink. WITHOUT THIS the
#                           phone's Auth/Assoc/EAPOL/DHCP frames are never
#                           acked, it retransmits ~15x, and association or the
#                           4-way handshake or DHCP stalls. This is the single
#                           most common cause of "can't connect / no lease".
#
# This script is READ-ONLY by default: it inspects and reports, it does not
# reconfigure anything. Pass --apply-monitor to have it put the inject radio
# into monitor mode on the channel via mon-setup.sh. It does NOT set up the ACK
# radio — that is what lab-bringup.sh does (AP opmode + decoy-BSSID mask).
#
# Usage:
#   sudo ./scripts/lab-check.sh                       # roster only (no roles)
#   sudo INJECT_IF=wlx.. ACK_IF=wlx.. CHANNEL=11 \
#        BSSID_BASE=02:11:22:33:44:00 BSSID_MASK=ff:ff:ff:ff:ff:fc \
#        ./scripts/lab-check.sh
#   sudo INJECT_IF=wlx.. CHANNEL=11 ./scripts/lab-check.sh --apply-monitor
#
set -uo pipefail

say()  { printf '\033[1;36m==>\033[0m %s\n' "$*"; }
ok()   { printf '  \033[1;32m[PASS]\033[0m %s\n' "$*"; }
warn() { printf '  \033[1;33m[WARN]\033[0m %s\n' "$*"; WARN=$((WARN+1)); }
bad()  { printf '  \033[1;31m[FAIL]\033[0m %s\n' "$*"; FAIL=$((FAIL+1)); }
die()  { printf '\033[1;31m[x]\033[0m %s\n' "$*" >&2; exit 2; }

WARN=0
FAIL=0

# ---- inputs -----------------------------------------------------------------
INJECT_IF="${INJECT_IF:-}"
ACK_IF="${ACK_IF:-}"
CHANNEL="${CHANNEL:-11}"
BSSID_BASE="${BSSID_BASE:-02:11:22:33:44:00}"
BSSID_MASK="${BSSID_MASK:-ff:ff:ff:ff:ff:fc}"
APPLY_MONITOR=0
[ "${1:-}" = "--apply-monitor" ] && APPLY_MONITOR=1

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"

for t in iw ip; do command -v "$t" >/dev/null || die "missing tool: $t"; done

# ---- helpers ----------------------------------------------------------------
# Normalize a MAC to lowercase colon form.
norm_mac() { printf '%s' "$1" | tr 'A-F' 'a-f'; }

# driver name for a netdev, or "?"
drv_of() {
    local d; d="$(readlink -f "/sys/class/net/$1/device/driver" 2>/dev/null)"
    [ -n "$d" ] && basename "$d" || echo '?'
}

# phyN for a netdev
phy_of() { iw dev "$1" info 2>/dev/null | awk '/wiphy/{printf "phy%s",$2; exit}'; }

# does this iface's phy advertise a given interface mode? (e.g. monitor, AP)
# NB: it is `iw phy <phyname> info`, not `iw <phyname> info`; and Debian's
# default awk is mawk, which has no \s -- so match lines containing '*' while
# inside the "Supported interface modes" block instead. Advisory only: some
# out-of-tree drivers (esp. Realtek) under-report modes to nl80211, so callers
# treat the *current* interface type as authoritative and this as a hint.
phy_supports() {
    local ph="$1" mode="$2"
    iw phy "$ph" info 2>/dev/null | awk '
        /Supported interface modes/ {inlist=1; next}
        inlist && /\*/ {print; next}
        inlist {inlist=0}
    ' | grep -qiw -- "$mode"
}

# type/channel/addr for a netdev (from iw dev info)
type_of() { iw dev "$1" info 2>/dev/null | awk '/[[:space:]]type /{print $2; exit}'; }
chan_of() { iw dev "$1" info 2>/dev/null | awk '/[[:space:]]channel /{print $2; exit}'; }
addr_of() { cat "/sys/class/net/$1/address" 2>/dev/null | tr 'A-F' 'a-f'; }

# Is `mac` inside (base & mask)? All args colon-MACs. Returns 0 if in range.
mac_in_range() {
    local mac="$1" base="$2" mask="$3" i m b k
    IFS=: read -ra MA <<<"$(norm_mac "$mac")"
    IFS=: read -ra BA <<<"$(norm_mac "$base")"
    IFS=: read -ra KA <<<"$(norm_mac "$mask")"
    for i in 0 1 2 3 4 5; do
        m=$((16#${MA[i]})); b=$((16#${BA[i]})); k=$((16#${KA[i]}))
        [ $(( (m ^ b) & k )) -eq 0 ] || return 1
    done
    return 0
}

# Drivers that ignore the injected radiotap rate (pin to ~1 Mbps on inject).
rate_deaf_driver() {
    case "$1" in
        mt76*|ath9k_htc|ath9k_common|carl9170) return 0 ;;
        *) return 1 ;;
    esac
}

list_wifi_ifaces() {
    local n
    for n in /sys/class/net/*; do
        [ -e "$n/phy80211" ] || continue
        basename "$n"
    done
}

# ---- 1. roster --------------------------------------------------------------
say "Wireless interface roster"
found=0
for IF in $(list_wifi_ifaces); do
    found=1
    ph="$(phy_of "$IF")"
    modes=""
    for m in monitor AP managed mesh; do
        phy_supports "$ph" "$m" && modes="$modes $m"
    done
    printf '  %-16s drv=%-12s %-5s type=%-8s chan=%-4s mac=%s\n' \
        "$IF" "$(drv_of "$IF")" "$ph" \
        "$(type_of "$IF" 2>/dev/null || echo -)" \
        "$(chan_of "$IF" 2>/dev/null || echo -)" \
        "$(addr_of "$IF")"
    printf '  %-16s modes:%s\n' "" "${modes:- (iw phy reported none — driver may under-report; current type is authoritative)}"
done
[ "$found" = 1 ] || die "no wireless interfaces found (need phy80211 netdevs)"

if [ -z "$INJECT_IF" ] && [ -z "$ACK_IF" ]; then
    echo
    say "No roles given. Re-run with INJECT_IF=/ACK_IF=/CHANNEL=/BSSID_BASE=/BSSID_MASK="
    say "to verify the assignment. Example:"
    echo "  sudo INJECT_IF=<realtek> ACK_IF=<ap-radio> CHANNEL=$CHANNEL \\"
    echo "       BSSID_BASE=$BSSID_BASE BSSID_MASK=$BSSID_MASK $0"
    exit 0
fi

# ---- 2. inject/monitor radio ------------------------------------------------
echo
say "INJECT/MONITOR radio: ${INJECT_IF:-<unset>} (channel $CHANNEL)"
if [ -z "$INJECT_IF" ]; then
    bad "INJECT_IF not set — the bridge needs a monitor-capable radio to run on."
elif ! ip link show "$INJECT_IF" >/dev/null 2>&1; then
    bad "interface '$INJECT_IF' not found"
else
    iph="$(phy_of "$INJECT_IF")"; idrv="$(drv_of "$INJECT_IF")"

    if [ "$APPLY_MONITOR" = 1 ]; then
        say "--apply-monitor: putting $INJECT_IF into monitor on channel $CHANNEL"
        "$SCRIPT_DIR/mon-setup.sh" "$INJECT_IF" "$CHANNEL" || bad "mon-setup.sh failed"
    fi

    itype="$(type_of "$INJECT_IF")"; ichan="$(chan_of "$INJECT_IF")"
    # Current mode is authoritative: a radio that is already a monitor vif
    # obviously supports monitor, regardless of what `iw phy` chose to report.
    if [ "$itype" = monitor ]; then
        ok "type=monitor (radio is in monitor mode now)"
    elif phy_supports "$iph" monitor; then
        bad "type=$itype — supports monitor but not in it. Run: mon-setup.sh $INJECT_IF $CHANNEL (or --apply-monitor)"
    else
        bad "type=$itype and 'iw phy' didn't list monitor. If this radio DOES do monitor"
        bad "(out-of-tree Realtek drivers often under-report), run mon-setup.sh; otherwise it's the wrong radio."
    fi
    [ "$ichan" = "$CHANNEL" ] && ok "on channel $CHANNEL" \
        || bad "channel=$ichan (expected $CHANNEL — injection into the wrong/void channel)"

    if rate_deaf_driver "$idrv"; then
        warn "driver '$idrv' ignores the injected rate and pins to ~1 Mbps downlink."
        warn "Use a Realtek RTL8812AU/8814AU (rtl88xxau) with rtw_monitor_disable_1m=1"
        warn "as the inject radio for real throughput (docs §7.1)."
    else
        ok "driver '$idrv' should honor injected HT-MCS rates"
    fi
fi

# ---- 3. ACK radio (the usual culprit) --------------------------------------
echo
say "ACK radio: ${ACK_IF:-<unset>} — must SIFS-ACK the STA uplink for BSSIDs in"
say "  $BSSID_BASE / $BSSID_MASK"
if [ -z "$ACK_IF" ]; then
    bad "ACK_IF not set. Without an AP-opmode radio whose bssidmask covers the VM"
    bad "BSSID range, the STA's uplink (Auth/Assoc/EAPOL/DHCP) is NEVER acked ->"
    bad "it retransmits ~15x and association / 4-way / DHCP stalls."
    bad "This is the #1 cause of 'connects but no DHCP'. Set it up with"
    bad "scripts/lab-bringup.sh (AR9271 AP opmode + decoy-BSSID mask)."
else
    if ! ip link show "$ACK_IF" >/dev/null 2>&1; then
        bad "interface '$ACK_IF' not found"
    else
        aph="$(phy_of "$ACK_IF")"; atype="$(type_of "$ACK_IF")"; achan="$(chan_of "$ACK_IF")"
        amac="$(addr_of "$ACK_IF")"; adrv="$(drv_of "$ACK_IF")"

        if [ "$atype" = AP ]; then
            ok "type=AP (in AP opmode -> PCU hardware-ACKs matching frames)"
            if [ -n "$achan" ]; then
                [ "$achan" = "$CHANNEL" ] && ok "on channel $CHANNEL (matches inject radio)" \
                    || bad "channel=$achan (must equal the inject channel $CHANNEL)"
            else
                warn "channel not reported"
            fi
            # The AP MAC only matters once it IS an AP (bring-up pins it); judging
            # the managed MAC would be meaningless, so we only check it here.
            if mac_in_range "$amac" "$BSSID_BASE" "$BSSID_MASK"; then
                ok "AP MAC $amac is within the ACKed BSSID range"
            else
                warn "AP MAC $amac is OUTSIDE $BSSID_BASE/$BSSID_MASK — the auto bssidmask"
                warn "may not cover the VM BSSIDs. lab-bringup.sh pins :00 and :03 decoys so"
                warn "the mask spans the VM range :01..:02."
            fi
            warn "Userspace can't read the hardware bssidmask. Confirm the ACK is real:"
            warn "witness-capture on the inject radio for ACKs (subtype 0x1d) to the phone."
        else
            # Not in AP mode == not set up yet. This is the right card for the job
            # (ath9k_htc/AR9271 is the canonical ACK radio); it just needs bring-up.
            bad "type=$atype — ACK radio is NOT in AP opmode yet, so nothing SIFS-ACKs the"
            bad "STA uplink. This is a setup step, not a bad card ('$adrv' is fine). Bring it"
            bad "up (AP opmode + decoy-BSSID mask) with:"
            bad "    sudo AR9271_IF=$ACK_IF MT_IF=$INJECT_IF CHANNEL=$CHANNEL \\"
            bad "         $SCRIPT_DIR/lab-bringup.sh"
            bad "then re-run this check."
        fi
    fi
fi

# ---- 4. cross-checks + verdict ---------------------------------------------
echo
say "Summary"
if [ -n "$INJECT_IF" ] && [ -n "$ACK_IF" ] && [ "$INJECT_IF" = "$ACK_IF" ]; then
    bad "INJECT_IF and ACK_IF are the same radio. One radio cannot both inject as"
    bad "the VM BSSID and SIFS-ACK it — these must be SEPARATE radios."
fi

if [ "$FAIL" -eq 0 ]; then
    printf '  \033[1;32mREADY\033[0m — %d warning(s). Suggested bridge command:\n' "$WARN"
    echo
    echo "  sudo $REPO_DIR/vwifi-phys-bridge <hub.sock> $INJECT_IF -c $CHANNEL \\"
    echo "       -b $BSSID_BASE -m $BSSID_MASK -M 4 -D 2 -K -v -S 2"
    echo
    say "Note: -D 2 duplicates encrypted data frames to survive the single-shot"
    say "NO_ACK downlink; -K forwards BlockAcks for the A-MPDU experiment. Get a"
    say "clean association + DHCP + download working FIRST — only then does ba=<n>"
    say "in the -S line mean anything."
    exit 0
else
    printf '  \033[1;31mNOT READY\033[0m — %d failure(s), %d warning(s). Fix the FAILs above.\n' \
        "$FAIL" "$WARN"
    exit 1
fi
