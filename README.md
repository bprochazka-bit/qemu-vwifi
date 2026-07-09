# QEMU Virtual Atheros AR9285 (ath9k) – Phase 1

A QEMU PCI-Express device emulation of the Atheros AR9285 wireless NIC,
targeting compatibility with the unmodified Linux `ath9k` kernel driver.

## Project Goals

**Phase 1 (this release):** Get the Linux `ath9k` driver to successfully
probe the virtual device.  The driver must bind to the PCI device, read the
silicon revision, parse the EEPROM, and progress through `ath9k_hw_init()`
without error.

Future phases (not yet implemented) will add TX/RX descriptor DMA, a
virtual-air medium for frame exchange between multiple VMs, and eventually
a physics-inspired channel model.

## Architecture

```
┌──────────────────────────────────────────────┐
│  Guest VM (Linux kernel + ath9k driver)      │
│                                              │
│  ath9k_pci_probe()                           │
│    └─ ath9k_hw_init()                        │
│         ├─ REG_READ(AR_SREV)     ──┐        │
│         ├─ ath9k_hw_set_reset()    │        │
│         ├─ ath9k_hw_4k_fill_eep() │        │
│         └─ ath9k_hw_fill_cap()    │        │
│                                    │        │
│  PCI BAR0 MMIO (64 KiB)          │        │
└────────────────────────────────────┼────────┘
                                     │ MMIO R/W
┌────────────────────────────────────▼────────┐
│  QEMU – vwifi_ath9k_pci.c device model     │
│                                              │
│  ┌─────────────────────────────────────────┐ │
│  │  Register File (uint32_t[16384])        │ │
│  │  • AR_SREV → returns AR9285 v1.2 ID    │ │
│  │  • AR_RTC_* → power/reset sequencing    │ │
│  │  • AR_EEPROM → EEPROM present + valid   │ │
│  │  • All others → shadow + log warning    │ │
│  └─────────────────────────────────────────┘ │
│  ┌─────────────────────────────────────────┐ │
│  │  EEPROM Image (512 × 16-bit words)     │ │
│  │  • 4K format with valid magic + CRC     │ │
│  │  • MAC: 00:03:7F:AA:BB:CC              │ │
│  │  • 2.4 GHz only, 1×1 MIMO              │ │
│  │  • World regulatory domain              │ │
│  └─────────────────────────────────────────┘ │
│                                              │
│  PCI ID: 168C:002B (Atheros AR9285 PCIe)    │
└──────────────────────────────────────────────┘
```

## Source Files

| File | Purpose |
|------|---------|
| `src/vwifi_ath9k_pci.c` | Main QEMU device model – MMIO handlers, PCI setup, lifecycle |
| `src/vwifi_ath9k_regs.h` | Register addresses and bit-field definitions from the kernel's `reg.h` |
| `src/vwifi_ath9k_eeprom.h` | EEPROM image generator – builds a valid 4K-format EEPROM at init |
| `src/vwifi_ath9k_dma.h` | TX/RX descriptor DMA ring definitions |
| `src/vwifi.h` | Virtual-medium wire protocol shared with the medium hub |
| `src/meson.build` | Meson build integration for QEMU's build system |
| `src/Kconfig` | Kconfig entry for enabling the device |
| `scripts/integrate.sh` | Shell script to copy files into a QEMU tree and patch build files |
| `tests/test_probe.sh` | Three-level test script (registration → instantiation → guest boot) |
| `Makefile` | Top-level convenience targets |

## Prerequisites

- **QEMU source tree**: Git clone of `https://github.com/qemu/qemu.git`.
  Tested with QEMU 8.x and 9.x.  Any version with Meson build should work.

- **Build dependencies** (for QEMU itself):
  ```
  # Debian/Ubuntu
  sudo apt-get install -y \
      git build-essential meson ninja-build pkg-config \
      libglib2.0-dev libpixman-1-dev libslirp-dev \
      python3 python3-venv flex bison

  # Fedora/RHEL
  sudo dnf install -y \
      git gcc meson ninja-build pkg-config \
      glib2-devel pixman-devel libslirp-devel \
      python3 flex bison
  ```

- **For Level 3 tests**: A Linux guest disk image (qcow2) with the `ath9k`
  module built into the kernel or available as a loadable module.

## Quick Start

```bash
# 1. Clone QEMU (if you haven't already)
git clone https://github.com/qemu/qemu.git /path/to/qemu
cd /path/to/qemu
git submodule update --init --recursive

# 2. Integrate the virtual ath9k device
cd /path/to/this-project
make integrate QEMU_SRC=/path/to/qemu

# 3. Configure QEMU (x86_64 only, with debug symbols)
make configure QEMU_SRC=/path/to/qemu

# 4. Build QEMU
make build QEMU_SRC=/path/to/qemu

# 5. Run the test suite
make test QEMU_SRC=/path/to/qemu

# 6. (Optional) Run with a Linux guest
make test QEMU_SRC=/path/to/qemu GUEST_IMAGE=/path/to/guest.qcow2

# 7. (Optional) Install the built binary system-wide
make install QEMU_SRC=/path/to/qemu
```

Or do it all in one command:

```bash
make all QEMU_SRC=/path/to/qemu
```

`make all` ends with `make install`, which installs the built
`qemu-system-x86_64` into `INSTALL_PREFIX` (default `/usr/local`). The
install step is skipped automatically if the binary is already present,
so re-running `make all` is safe. To install elsewhere (no root needed):

```bash
make install QEMU_SRC=/path/to/qemu INSTALL_PREFIX=$HOME/.local
```

During development the binary is usually already installed, so `make
install` becomes a no-op. Use `make upgrade` to force a reinstall and
overwrite the existing binary with a freshly built one:

```bash
make upgrade QEMU_SRC=/path/to/qemu
```

## Manual Build (Step by Step)

If you prefer not to use the Makefile:

```bash
# Copy files into QEMU tree
./scripts/integrate.sh /path/to/qemu

# Configure and build
cd /path/to/qemu
mkdir -p build && cd build
../configure --target-list=x86_64-softmmu --enable-debug
make -j$(nproc)

# Verify the device is registered
./qemu-system-x86_64 -device help 2>&1 | grep ath9k

# Run with the device attached
./qemu-system-x86_64 \
    -machine q35 \
    -m 512 \
    -device vwifi-ath9k \
    -d guest_errors,unimp \
    -D /tmp/ath9k.log \
    -drive file=guest.qcow2,format=qcow2,if=virtio \
    -nographic
```

## Device Arguments

The device is instantiated with `-device vwifi-ath9k[,key=value,...]`.
All arguments are optional; with no arguments the device runs in
standalone beacon-only mode with an auto-generated MAC address.

| Argument | Type | Default | Description |
|----------|------|---------|-------------|
| `medium` | string (path) | _(empty → standalone)_ | Filesystem path to the virtual-medium hub's `AF_UNIX` stream socket. When set, the device connects to the hub so frames can be exchanged with other VMs. When empty or omitted, the device runs standalone (beacon-only, no inter-VM traffic). |
| `node_id` | string | _(empty → auto)_ | Stable identity string for this node, sent in the hello handshake when connecting to the medium hub. Lets the hub recognise the same node across reconnects regardless of its (possibly random) MAC. Ignored when no `medium` is configured. |
| `macaddr` | string (`xx:xx:xx:xx:xx:xx`) | auto-random `00:03:7F:xx:xx:xx` | Station MAC address. The six colon-separated hex bytes are patched into the emulated EEPROM so the unmodified `ath9k` driver reads it through its normal EEPROM path. |

### `medium`

```bash
-device vwifi-ath9k,medium=/tmp/vwifi.sock
```

- Path to the Unix-domain stream socket exported by a medium hub
  (`src/ath9k_medium_hub.c` or `src/ath9k_medium_hub_scalable.c`).
- The hub does **not** need to be running first: if the socket is
  absent or the connection drops, the device automatically retries
  every 2 seconds (`MEDIUM_RECONNECT_MS`). On each successful
  connect it re-sends the `node_id` hello.
- Empty/omitted ⇒ **standalone mode**: the device still emulates the
  NIC and emits beacons internally, but no frames leave or enter the
  VM. Useful for driver-probe testing without a hub.

### `node_id`

```bash
-device vwifi-ath9k,medium=/tmp/vwifi.sock,node_id=ap1
```

- An arbitrary short string (e.g. `ap1`, `vm-a`, `station3`) sent to
  the hub in the connection hello so the hub can track this node by a
  stable name instead of its MAC.
- Recommended whenever `macaddr` is left to auto-random, so the node
  keeps a consistent identity across restarts/reconnects.
- Has no effect in standalone mode (no hub to receive the hello). When
  unset, the hello is sent without an id and logs show `node_id=(auto)`.

### `macaddr`

```bash
-device vwifi-ath9k,macaddr=00:03:7F:AA:BB:CC
```

- Must be six colon-separated hex octets (`xx:xx:xx:xx:xx:xx`). The
  value is parsed and written into EEPROM words at
  `EEP4K_OFF_HDR_MACADDR`, so the guest `ath9k` driver picks it up via
  its standard EEPROM MAC read.
- If the string is malformed it is rejected with a
  `LOG_GUEST_ERROR` warning and the device falls back to a random MAC.
- If omitted, a MAC is auto-generated using the Atheros OUI
  `00:03:7F` followed by three random bytes (unicast, globally
  administered), so the driver's OUI checks pass. Set an explicit
  `macaddr` when you need stable, non-colliding addresses across
  several VMs sharing a medium.

### Full multi-VM example

Two VMs sharing one medium hub, each with a stable id and MAC:

```bash
# VM A
qemu-system-x86_64 -machine q35 -m 2048 \
    -device vwifi-ath9k,medium=/tmp/vwifi.sock,node_id=vm-a,macaddr=00:03:7F:00:00:01 \
    -drive file=vm-a.qcow2,format=qcow2,if=virtio -nographic

# VM B
qemu-system-x86_64 -machine q35 -m 2048 \
    -device vwifi-ath9k,medium=/tmp/vwifi.sock,node_id=vm-b,macaddr=00:03:7F:00:00:02 \
    -drive file=vm-b.qcow2,format=qcow2,if=virtio -nographic
```

See `CHEATSHEET.md` for more ready-to-run command lines.

## Debugging

### Viewing Register Accesses

The device logs every register access through QEMU's logging system.
Enable it with:

```bash
-d guest_errors,unimp -D /tmp/ath9k.log
```

This produces output like:

```
vwifi-ath9k: READ  AR_SREV                  = 0x030020ff (AR9285 v1.2)
vwifi-ath9k: WRITE AR_RTC_FORCE_WAKE        = 0x00000001
vwifi-ath9k: READ  AR_RTC_STATUS            = 0x00000002
vwifi-ath9k: READ  EEPROM [0] = 0xa55a
vwifi-ath9k: WARNING: UNHANDLED read  REG(0x00a4)       = 0x00000000
```

**Every unhandled register** logs a warning with the exact address and
current value.  This is deliberate: it makes it trivial to identify which
registers the driver touches next, so you can add proper handling.

After 200 warnings of each type, further messages are suppressed to avoid
flooding the log.

### Guest Kernel Messages

Inside the guest VM, check `dmesg` for ath9k messages:

```bash
dmesg | grep -i 'ath\|wifi\|802\.11'
```

Expected output on success:

```
ath: EEPROM regdomain: 0x0
ath: Country alpha2 being used: 00
ath: Regpair used: 0x1f
ieee80211 phy0: Atheros AR9285 Rev:2
```

Expected output if EEPROM parsing fails:

```
ath: phy0: Unable to initialize hardware; initialization status: -22
ath9k 0000:XX:XX.0: Failed to initialize device
```

If you see the failure message, check the QEMU log for which EEPROM
offset or register read caused the issue, then adjust the EEPROM image
or register handler accordingly.

### Common Issues

| Symptom | Likely Cause | Fix |
|---------|-------------|-----|
| `vwifi-ath9k` not in `-device help` | Device not compiled in | Re-run `scripts/integrate.sh` and rebuild |
| `Mac Chip Rev 0x00.0 is not supported` | AR_SREV returning wrong value | Check that `ath9k_mmio_read()` handles `0x4020` |
| `Unable to initialize hardware; status: -22` | EEPROM validation failed | Check EEPROM magic, version, checksum, opCapFlags |
| `Unable to initialize hardware; status: -5` | RTC reset timeout | AR_RTC_STATUS must return `AR_RTC_STATUS_ON` |
| `no band has been marked as supported` | opCapFlags missing 11G bit | Verify `AR5416_OPFLAGS_11G` is set in EEPROM |
| `0xdeadbeef` in register reads | Read from unmapped offset | Ensure all accessed offsets return from the shadow array |

## Design Decisions

1. **Conservative defaults**: Unknown registers return zero from the shadow
   array, not `0xdeadbeef`.  The driver interprets `0xdeadbeef` as a hardware
   hang and forces a reset.  Zero is a safe no-op for most control registers.

2. **Logging over silence**: Every access is traced.  In Phase 1, visibility
   into the driver's behaviour is more valuable than performance.

3. **EEPROM in a header**: The EEPROM image is generated at device
   realization time by `ath9k_eeprom_init_4k()` in a header file.  This
   avoids external data file dependencies and keeps the build self-contained.

4. **Immediate completion**: Operations that the driver polls for (RTC
   wakeup, EEPROM reads) complete instantly.  The driver sees the done
   bit on its first poll.  This is acceptable for Phase 1 where we don't
   emulate timing.

5. **Single BAR**: The AR9285 uses a single 64 KiB memory-mapped BAR.
   We register it as BAR0 with `PCI_BASE_ADDRESS_SPACE_MEMORY`.

## Register Coverage

The following registers are explicitly handled in Phase 1:

| Register | Address | Read | Write | Notes |
|----------|---------|------|-------|-------|
| AR_SREV | 0x4020 | ✓ | — | Returns AR9285 v1.2 signature |
| AR_CFG | 0x0014 | ✓ | ✓ | Basic config, shadow register |
| AR_RTC_RC | 0x7000 | ✓ | ✓ | Reset control, sets status ON |
| AR_RTC_RESET | 0x7040 | ✓ | ✓ | Reset enable, sets status ON |
| AR_RTC_STATUS | 0x7044 | ✓ | — | Always returns ON |
| AR_RTC_FORCE_WAKE | 0x704C | ✓ | ✓ | Transitions to AWAKE |
| AR_RTC_PLL_CONTROL | 0x7014 | ✓ | ✓ | Shadow register |
| AR_EEPROM | 0x401C | ✓ | ✓ | Reports present + valid |
| AR_EEPROM_STATUS_DATA | 0x407C | ✓ | — | Reports not busy |
| EEPROM window | 0xC000+ | ✓ | — | 512 × 16-bit words, 4K format |
| AR_ISR/IMR/IER | various | ✓ | ✓ | Interrupt stubs (no IRQs raised) |
| AR_GPIO_* | 0x4048+ | ✓ | ✓ | Shadow registers |
| AR_WA | 0x4004 | ✓ | ✓ | Workaround register, AR9285 default |
| AR_STA_ID0/1 | 0x8000/4 | ✓ | ✓ | Station MAC, shadow |
| AR_PM_STATE | 0x4008 | ✓ | — | Reports awake |
| AR_KEYTABLE | 0x8800–0x97FF | ✓ | ✓ | Hardware key cache (CCMP / TKIP / WEP offload) |
| All others | * | ✓ | ✓ | Shadow register + UNHANDLED warning |

## Hardware Crypto (WPA2 / WPA3 / CCMP / TKIP / WEP)

Real ath9k hardware offloads CCMP (AES-CCM) to an on-chip crypto engine
driven by the key cache, and advertises that capability to mac80211.  Because
of that advertisement, mac80211 stops doing software CCMP and depends on the
hardware for the encrypted data path.  A virtual radio with no crypto engine
therefore associates and completes the WPA2 4-way handshake (EAPOL is computed
in userspace) but carries **no** data — every encrypted frame is dropped and
`rx drop misc` climbs.

This device emulates that engine:

- **Key cache** (`AR_KEYTABLE`, 0x8800): the driver's key-programming writes
  are shadowed in the register file; `vwifi_ath9k_keycache_ccm()` reconstructs
  the 128-bit key from the same `key0..key4` split that
  `ath_hw_set_keycache_entry()` uses.
- **TX**: when a TX descriptor requests encryption (valid `AR_DestIdx` +
  `AR_EncrType` = AES), the payload is encrypted with AES-CCM and the 8-byte
  MIC appended, exactly as the silicon would.
- **RX**: protected frames are decrypted (the matching key is found by MIC
  verification) and the RX status reports a valid key index so the driver sets
  `RX_FLAG_DECRYPTED` and mac80211 strips the CCMP header and MIC.

### Frame transformation

```
TX (guest → medium)
  mac80211 hands the driver:  [802.11 hdr][CCMP hdr][plaintext]   (Protected=1)
  driver DMAs (+align pad):   [802.11 hdr][pad][CCMP hdr][plaintext]
  device strips pad, encrypts:[802.11 hdr][CCMP hdr][ciphertext][MIC]  → medium

RX (medium → guest)
  from medium:                [802.11 hdr][CCMP hdr][ciphertext][MIC]
  device decrypts in place:   [802.11 hdr][CCMP hdr][plaintext][MIC]
  device DMAs (+pad,+FCS):     ...and sets RX status KeyIdxValid
  driver strips pad/FCS,
  mac80211 (RX_FLAG_DECRYPTED) strips CCMP hdr + MIC → [802.11 hdr][plaintext]
```

Both endpoints run this same emulated engine over the virtual medium, so the
transform round-trips; because it is real, spec-compliant AES-CCM it also
interoperates with a genuine software-CCMP peer.

### Key selection

On TX the key-cache slot comes straight from the descriptor (`AR_DestIdx`).
On RX, rather than replicate the AR9285's undocumented associative lookup, the
device tries each populated CCM slot and accepts the key whose MIC verifies —
a wrong key is rejected at ~2⁻⁶⁴, so this is both robust and faithful to the
"hardware found the key" behaviour the driver expects.  Pairwise and group
(GTK) keys are handled identically.

### WPA3-Personal (SAE) and Protected Management Frames

WPA3-Personal reuses the **same CCMP-128 data cipher** as WPA2, so its data
path needs no new engine.  The two things that make a connection "WPA3" sit
either off the device or in a path the engine already covers:

- **SAE** (the PSK replacement) is an authentication handshake run entirely in
  userspace (`hostapd` / `wpa_supplicant`) over unprotected Authentication
  management frames, exactly like the WPA2 4-way EAPOL handshake already is.
  The device just forwards those management frames over the medium; it never
  interprets them.
- **PMF / 802.11w** is mandatory for WPA3.  *Individually-addressed* robust
  management frames (protected Action, Deauth, Disassoc, SA-Query) are
  encrypted with the pairwise CCMP key and ride the **same** offload as data
  frames — the CCM nonce/AAD construction differs only in the Management bit
  and subtype masking (§12.5.3), which the engine handles.  *Group-addressed*
  robust management frames use BIP (AES-CMAC) with the IGTK; ath9k does not
  offload BIP, so mac80211 performs it in software and the device is not
  involved.

As a result WPA3-Personal associates and passes traffic on the unmodified
`ath9k` driver with no capability changes — the CCMP engine is the whole data
plane, and PMF is covered by the engine's management-frame path plus
mac80211's software BIP.

### WEP (RC4 + CRC-32)

WEP is offloaded to the same key cache: the driver programs the WEP key into a
slot (key type 40 / 104 / 128), mac80211 inserts the 4-octet IV header
(`IEEE80211_KEY_FLAG_GENERATE_IV`), and the hardware appends the 4-octet ICV —
so, like CCMP, an unemulated engine drops every WEP frame.  The engine lives in
`src/vwifi_ath9k_wep.h` and performs the §12.3.2 transform:
`RC4(IV ‖ key)` over `plaintext ‖ CRC-32(plaintext)`.

- **Cipher discrimination**: CCMP and TKIP set the *ExtIV* bit in the IV header
  (an 8-octet header); WEP does not (4-octet header).  On RX the device inspects
  that bit to route a protected frame to the WEP path or the extended-IV path.
- **Key length** comes from the slot's key type (5 / 13 / 16 octets), read back
  through the same `key0..key4` split the CCM path uses.  Unused slots read as
  `AR_KEYTABLE_TYPE_CLR`, so a genuinely empty slot is rejected even though
  WEP-40 is key type 0.
- **RX key selection**: WEP has no cryptographic MIC, only a 32-bit ICV, so the
  device tries each populated WEP slot and accepts the one whose ICV verifies.
  In practice ≤ 4 WEP keys are ever programmed, making the 2⁻³² per-slot
  false-accept rate negligible.

WEP is cryptographically broken and offered only for legacy interoperability
and driver testing; use WPA2/WPA3 for anything real.

### TKIP (per-packet key mixing + RC4 + CRC-32 ICV)

TKIP (WPA-Personal / WPA-TKIP) offloads only its *cipher* half to the engine.
ath9k sets `IEEE80211_KEY_FLAG_GENERATE_MMIC`, so **mac80211 computes and
verifies the 8-octet Michael MIC in software**; the hardware derives the
per-packet RC4 key, runs RC4, and appends the CRC-32 ICV.  The engine
(`src/vwifi_ath9k_tkip.h`) therefore does mixing + RC4 + ICV and treats the
Michael MIC as opaque payload — a much smaller surface than full TKIP.

- **Key mixing**: TKIP phase 1 (TK + transmitter address + `IV32`) and phase 2
  (+ `IV16`) produce the 16-octet RC4 key, per §12.5.2.3.  This S-box mixing is
  the one genuinely new primitive; it is verified against the published
  IEEE 802.11i key-mixing test vectors.
- **Temporal key**: only the 128-bit TK lives in the main key-cache slot (same
  `key0..key4` split as CCM); the companion "MIC key" slot the driver programs
  is never read, because Michael is done in software.
- **RX cipher selection**: CCMP and TKIP share the ExtIV header, so for an
  extended-IV frame the device tries CCM first (64-bit MIC) and then TKIP
  (32-bit ICV).  A network programs only one pairwise cipher, so at most one set
  of slots is populated and the MIC/ICV check picks the right key.
- **RC4 / CRC-32** are shared with the WEP engine.

TKIP is deprecated (and disallowed on 802.11n rates); it is provided for
legacy-AP interop and testing only.

### Scope and verification

The AES-CCM implementation lives in `src/vwifi_ath9k_crypto.h` and follows
IEEE 802.11-2016 §12.5.3 / RFC 3610 (M = 8, L = 2), reusing the `AES_KEY` API
shared by QEMU's `crypto/aes.h` and OpenSSL.  Run `make test-crypto` to build
and execute `tests/test_ccmp.c`, which verifies the CCM core against OpenSSL's
`EVP_aes_128_ccm` over 2000 random vectors, plus MPDU round-trip, forgery
rejection, the **protected-management-frame (PMF) path** — its nonce/AAD
checked against an independent 802.11 reference and its ciphertext/MIC against
OpenSSL — and the full key-cache TX→RX data path.

The WEP engine has its own suite (`make test-wep`, `tests/test_wep.c`): RC4
against the published keystream vectors, CRC-32 against `zlib` and the canonical
check value, WEP-40/104/128 MPDU round-trip, ICV forgery rejection, the
per-type key-cache reconstruction, and ExtIV discrimination.

The TKIP engine's suite (`make test-tkip`, `tests/test_tkip.c`) checks phase 1 +
phase 2 mixing against the published IEEE 802.11i key-mixing test vectors, plus
MPDU round-trip, ICV forgery rejection, and the TKIP key-cache TX→RX data path.

Supported: **WPA2-Personal and WPA3-Personal (CCMP-128, incl. PMF), WPA-Personal
(TKIP), and WEP-40 / 104 / 128.**  WPA3-Enterprise 192-bit (GCMP-256 /
BIP-GMAC-256) would need a separate AES-GCM engine and is out of scope.

## EEPROM Image

The generated EEPROM uses the AR9285 "4K" format:

- **Magic**: `0xa55a` (AR5416_EEPROM_MAGIC)
- **Version**: 14.14 (`0x0E0E`)
- **opCapFlags**: 2.4 GHz, HT20, HT40
- **Regulatory domain**: World (0x0000 / 0x001f)
- **MAC address**: `00:03:7F:AA:BB:CC` (Atheros OUI)
- **TX/RX chain mask**: 1×1 (single stream)
- **Checksum**: Valid XOR checksum

## Contributing

When adding support for new registers:

1. Add the register address to `vwifi_ath9k_regs.h`
2. Add a `case` in `ath9k_mmio_read()` and/or `ath9k_mmio_write()`
3. Add the register name to the `ath9k_reg_names[]` lookup table
4. Test with the guest and verify the UNHANDLED warning disappears

The coding style follows the QEMU project conventions:
- 4-space indentation (no tabs)
- `lower_case_with_underscores` for functions and variables
- `UPPER_CASE` for macros and constants
- Every function and data structure has a comment explaining its purpose

## License

This project is licensed under the GNU General Public License v2.0 or later
(GPL-2.0-or-later), matching the Linux ath9k driver and the QEMU project.

## References

- [Linux ath9k driver source](https://github.com/torvalds/linux/tree/master/drivers/net/wireless/ath/ath9k)
- [QEMU device model documentation](https://www.qemu.org/docs/master/devel/device-emulation.html)
- [open-ath9k-htc firmware](https://github.com/qca/open-ath9k-htc-firmware)
- [Linux Wireless ath9k documentation](https://wireless.docs.kernel.org/en/latest/en/users/drivers/ath9k.html)
- [mac80211_hwsim](https://github.com/torvalds/linux/blob/master/drivers/net/wireless/virtual/mac80211_hwsim.c) – reference virtual Wi-Fi driver
- [wmediumd](https://github.com/bcopeland/wmediumd) – virtual air medium daemon
