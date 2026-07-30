# legacy — the ath9k_medium hubs

Superseded. Use `vwifi-medium` ([`../src/vwifi_medium.c`](../src/vwifi_medium.c)).

These are the medium hub's two earlier generations, from before the wire
protocol was renamed from `ath9k_medium` to `vwifi` and the two projects
were merged. They are kept because they are small, readable, and
occasionally useful for bisecting a behaviour change — not because
anything should be built against them.

| File | What it was |
|---|---|
| `ath9k_medium_hub.c` | First hub. `select()`, Unix socket only, no channel awareness |
| `ath9k_medium_hub_scalable.c` | Second. `poll()`, TCP inter-hub bridging, channel filtering, HT40 bonding |
| `ath9k_medium.h` | The v1/v2 protocol under its old name and `"A9KW"` magic |

## Why they are not the hub any more

`vwifi-medium` is the third generation and a superset: per-link SNR and
frame-error modelling from node positions and TX power, VHT/HE
center-frequency filtering, per-channel airtime survey, a runtime
control socket with config save/load, and the `physical` peer flag that
exempts a real bridged radio from the simulated propagation model.

## They are not wire-compatible

`ath9k_medium.h` uses magic `0x41394B57` (`"A9KW"`). The current
protocol uses `0x46495756` (`"VWIF"`). That change was deliberate: old
and new builds must not half-interoperate, because a peer that parses a
foreign header layout produces garbage rather than an error.

Nothing in the tree includes `ath9k_medium.h` — the two hub sources here
parse the wire format from byte offsets they define themselves, so this
header has no consumers at all. It is here as documentation of what the
old format was.

## Building them

```bash
make legacy     # from the repository root -> build/ath9k_medium_hub*
```
