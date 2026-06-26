# ackmon — single-radio hardware-ACK for the AR9271 physical bridge

`ackmon` lets one AR9271 (ath9k_htc) **hardware-ACK uplink frames addressed to a
set of BSSIDs** while running as a monitor vif, so an unmodifiable real station
(e.g. an Android phone) can *associate* to OpenWRT VM APs that live on the vwifi
medium — with the **VMs remaining the literal APs**. See
[`../docs/ar9271-phys-bridge-lab.md`](../docs/ar9271-phys-bridge-lab.md) for the
whole lab design.

## Why this is (almost certainly) driver-only — no firmware rebuild

Reading the mainline ath9k source settles it:

- The PCU ACKs a unicast frame when **`(RA & bssidmask) == (macaddr & bssidmask)`**.
  `macaddr` → `AR_STA_ID` (the monitor vif's `myaddr` on the AR9271 HTC target);
  `bssidmask` → `AR_BSSMSK`, programmed by `ath_hw_setbssidmask()`.
- **`AR_DIAG_ACK_DIS` is defined but never used** in the driver — ACK is *not*
  disabled in monitor mode. Monitor simply never has a frame addressed to its
  MAC, so the ACK never fires.
- `ath9k_htc_set_mac_bssid_mask()` already computes `macaddr`/`bssidmask` from
  the active vif addresses (the standard multi-vif mask algorithm).

So ackmon just overrides `macaddr` = your BSSID base and `bssidmask` = a mask
covering the set, on a monitor vif. The chip then ACKs every BSSID in the set
while the vif captures/injects raw. `ath9k_htc-ackmon.patch` is that override.

**The one residual risk** is whether the AR9271 *firmware* suppresses ACK in
`HTC_M_MONITOR` opmode independent of `AR_STA_ID`. The driver doesn't, but the
closed-to-us-until-built firmware might. Settle it with the test below **before**
investing in the patch — and only fall back to `open-ath9k-htc-firmware` if the
test says monitor ACK is firmware-gated.

## Decisive cheap test: does the AR9271 ACK in monitor when the address matches?

Zero patches. Three radios (or two hosts): the **AR9271 under test**, a
**sender**, and a **sniffer**.

1. On the AR9271, force its MAC to a target BSSID and go to monitor on ch6:
   ```sh
   sudo ip link set wlxc01c300da281 down
   sudo ip link set wlxc01c300da281 address 02:11:22:33:44:00
   sudo iw dev wlxc01c300da281 set type monitor
   sudo ip link set wlxc01c300da281 up
   sudo iw dev wlxc01c300da281 set channel 6
   ```
   (Single vif → `bssidmask` is all-ones, so the chip should match the exact
   MAC `02:11:22:33:44:00` and nothing else.)

2. From the **sender**, inject a unicast 802.11 data frame with
   `addr1 = 02:11:22:33:44:00` on ch6 (scapy/`packetforge-ng`/`aireplay-ng`).

3. On the **sniffer** (monitor, ch6), watch for an `ACK` control frame whose
   RA is the sender, ~SIFS after each injected frame:
   ```sh
   sudo tcpdump -i mon0 -y IEEE802_11_RADIO 'type ctl subtype ack' -e -vvv
   ```

- **ACKs appear** → the AR9271 ACKs in monitor on address match. ackmon is
  driver-only; apply the patch, widen the mask to cover all three BSSIDs, done.
- **No ACKs** → monitor ACK is firmware-gated; we patch
  `open-ath9k-htc-firmware` (keep the responder live in the monitor RX path).

## Building the patched driver (if the test is green)

You only need the ath9k modules, not a full kernel:

```sh
# match your running kernel headers
sudo apt install linux-headers-$(uname -r) build-essential   # or distro equiv.

# fetch the ath9k tree for your kernel version, apply, build the modules
# (out-of-tree backport build, or rebuild in a kernel source tree):
patch -p1 < ath9k_htc-ackmon.patch
make -C /lib/modules/$(uname -r)/build M=$PWD/drivers/net/wireless/ath/ath9k modules
sudo modprobe -r ath9k_htc
sudo insmod .../ath9k_htc.ko ackmon=1 \
     ackmon_base=02:11:22:33:44:00 ackmon_mask=ff:ff:ff:ff:ff:fc
```

Then bring up the monitor vif (Section "Load + bring-up" in the patch) and run
`vwifi-phys-bridge` on ch6 as usual. Validate end to end by associating the
phone to each of the three OpenWRT VM APs.

## Status

| Item | State |
|---|---|
| Source analysis (ACK = address-match, no `AR_DIAG_ACK_DIS`) | done |
| `ath9k_htc-ackmon.patch` (macaddr/bssidmask override) | **draft, untested** |
| Monitor-ACK feasibility test | **run this first** |
| Firmware fallback (`open-ath9k-htc-firmware`) | only if test fails |
