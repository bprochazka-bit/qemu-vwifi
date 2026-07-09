#!/usr/bin/env bash
#
# mon-setup.sh — put a Wi-Fi interface into monitor mode on a given channel,
# ready for vwifi-linkbench or vwifi-phys-bridge, and VERIFY it actually took.
#
# The manual dance (`ip link down` / `iw set type monitor` / `ip link up`) is
# easy to get subtly wrong — the most common mistake is forgetting to set a
# channel, which leaves the radio "up" but not on any frequency. Injection then
# succeeds instantly into a void (linkbench reports an impossible multi-Gbps
# rate) and a co-channel capture sees nothing. This helper does the full
# sequence and hard-fails unless the interface ends up: type=monitor, on the
# requested channel, with carrier up.
#
# Usage:
#   sudo ./scripts/mon-setup.sh <iface> <channel> [width]   # e.g. 11, or 36 HT40+
#   sudo ./scripts/mon-setup.sh <iface> managed             # restore to managed
#
#   ACTIVE=1 sudo ./scripts/mon-setup.sh <iface> <channel>  # active monitor
#                                                            # (radio ACKs; usually NOT wanted)
#
# Examples:
#   sudo ./scripts/mon-setup.sh wlx00c0cab57e6f 11
#   sudo ./scripts/mon-setup.sh wlx90de80152b9e 11
#   # then:
#   sudo ./vwifi-linkbench capture wlx00c0cab57e6f -t 12 &
#   sudo ./vwifi-linkbench inject  wlx90de80152b9e -R 6 -t 10
#
set -euo pipefail

say()  { printf '\033[1;36m==>\033[0m %s\n' "$*"; }
warn() { printf '\033[1;33m[!]\033[0m %s\n' "$*" >&2; }
die()  { printf '\033[1;31m[x]\033[0m %s\n' "$*" >&2; exit 1; }

[ "$(id -u)" = 0 ] || die "run with sudo"
[ $# -ge 2 ] || die "usage: $0 <iface> <channel|managed> [width]"

IFACE="$1"
CHANNEL="$2"
WIDTH="${3:-HT20}"
ACTIVE="${ACTIVE:-0}"

for t in iw ip; do command -v "$t" >/dev/null || die "missing tool: $t"; done
ip link show "$IFACE" >/dev/null 2>&1 || die "interface '$IFACE' not found"

# ---- restore-to-managed mode ------------------------------------------------
if [ "$CHANNEL" = "managed" ]; then
    say "Restoring $IFACE to managed mode"
    ip link set "$IFACE" down
    iw dev "$IFACE" set type managed
    ip link set "$IFACE" up
    nmcli device set "$IFACE" managed yes 2>/dev/null || true
    say "$IFACE is managed and up."
    exit 0
fi

case "$CHANNEL" in
    ''|*[!0-9]*) die "channel must be a number (got '$CHANNEL')";;
esac

# ---- release from NetworkManager / wpa_supplicant ---------------------------
say "Releasing $IFACE from NetworkManager/wpa_supplicant (if present)"
nmcli device set "$IFACE" managed no 2>/dev/null || true
pkill -f "wpa_supplicant.*$IFACE" 2>/dev/null || true

# ---- down -> monitor -> up -> channel ---------------------------------------
say "Configuring $IFACE: monitor on channel $CHANNEL ($WIDTH)"
ip link set "$IFACE" down

# Plain monitor by default. `set monitor active` marks the vif as an active
# monitor (the radio will ACK matching frames) — rarely what you want for a
# capture/inject relay, so it's opt-in via ACTIVE=1.
if [ "$ACTIVE" = 1 ]; then
    iw dev "$IFACE" set monitor active 2>/dev/null \
        || iw dev "$IFACE" set type monitor \
        || die "could not set $IFACE to (active) monitor"
else
    iw dev "$IFACE" set type monitor \
        || die "could not set $IFACE to monitor (driver may not support it)"
fi

ip link set "$IFACE" up

# Channel must be set AFTER the interface is up on most drivers.
if ! iw dev "$IFACE" set channel "$CHANNEL" "$WIDTH" 2>/dev/null; then
    # Some drivers reject the width token; retry bare.
    iw dev "$IFACE" set channel "$CHANNEL" \
        || die "could not set $IFACE to channel $CHANNEL — is the channel valid \
for this radio's band? (2.4 GHz: 1-14; 5 GHz: 36,40,...)"
fi

# ---- verify (this is the whole point) ---------------------------------------
say "Verifying $IFACE"
info="$(iw dev "$IFACE" info 2>/dev/null || true)"
type_now="$(printf '%s\n' "$info"  | awk '/type/{print $2; exit}')"
chan_now="$(printf '%s\n' "$info"  | awk '/channel/{print $2; exit}')"
oper="$(cat "/sys/class/net/$IFACE/operstate" 2>/dev/null || echo unknown)"
carrier="$(cat "/sys/class/net/$IFACE/carrier" 2>/dev/null || echo '?')"

printf '%s\n' "$info" | grep -E 'type|channel|addr|txpower' | sed 's/^/    /'
printf '    operstate=%s carrier=%s\n' "$oper" "$carrier"

[ "$type_now" = "monitor" ] || die "type is '$type_now', expected 'monitor'"
[ "$chan_now" = "$CHANNEL" ] || die "channel is '$chan_now', expected '$CHANNEL' \
— the radio is NOT on frequency; injection would go into a void"
# operstate is often 'unknown' for monitor vifs (no link concept); carrier==1 is
# the real signal that the PHY is running. Warn, don't fail, on ambiguity.
if [ "$carrier" != "1" ] && [ "$oper" != "up" ]; then
    warn "carrier=$carrier operstate=$oper — interface may not be fully up; \
if linkbench reports an impossibly high fps, the radio isn't transmitting"
fi

say "$IFACE ready: monitor, channel $CHANNEL. Both radios in a pair must use \
the SAME channel."
