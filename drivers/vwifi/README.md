# vwifi-virt guest drivers

Drivers that bind to the [`vwifi-virt`](../../devices/vwifi/) PCI
device. One per guest OS; all of them implement the same contract,
[`abi/vwifi_abi.h`](../../abi/vwifi_abi.h).

| OS | Directory | Status |
|---|---|---|
| Linux | [`linux/`](linux/) | STA + monitor + injection; **compiles** clean against 6.8, not yet run in a guest |
| Windows 10 / 11 | [`windows/`](windows/) | Written (~5100 lines), **never compiled** — needs a WDK |
| macOS | [`macos/`](macos/) | Planned, with open feasibility questions |

## What they share

The register map, the four ring layouts, the control opcode set and the
event codes — all of it in one header, `vwifi_abi.h`, included verbatim
by every driver and by the device. A driver that disagrees with that
header does not get a compile error; it gets silent memory corruption
across the guest/host boundary. This has happened once already, which is
why the header lives in `abi/` and is never copied.

Every driver must read `VWIFI_REG_ABI_VERSION` at init and refuse to
bind on mismatch. That check is the backstop.

## What they don't share

Nothing else. Each OS's Wi-Fi stack has its own model of what a wireless
NIC is, and the driver's real work is translating between that model and
the opcode set:

- **Windows** wants WDI: a TLV-encoded task/property interface underneath
  NDIS, where "connect" is a multi-step task with its own state machine.
- **Linux** wants cfg80211/mac80211: either a full-MAC driver
  registering `cfg80211_ops`, or — since the device already does the
  802.11 work — something closer to a thin ops shim.
- **macOS** wants IO80211, which is not a public API. That port is the
  one with a real feasibility question attached, not just an effort
  estimate.

## Testing without a guest

Most of what a driver talks to can be exercised without booting
anything. The device's portable core runs on the host against a mock
backend — `make -C devices/vwifi test` drives scan, association,
WPA2/CCMP, monitor mode and SoftAP end to end in a few seconds.

Use it to answer "what should the device have done here?" before
reaching for a kernel debugger. The device tests are a much faster loop
than any guest, and they were written first for exactly that reason.
