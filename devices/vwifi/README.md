# vwifi-virt — device model

The QEMU device model, refactored into a portable core + a QEMU
backend, so the same code can later be reused inside a vfio-user
server without touching the device logic itself.

## Layout

The shared headers are NOT in `src/` — `vwifi_abi.h` (the guest-driver
contract) and `vwifi.h` (the medium wire protocol) live once in the
repository's [`../../abi/`](../../abi/), and `scripts/integrate.sh`
copies them into the QEMU tree at integration time.

```
src/
  vwifi_backend.h    Vtable that abstracts host operations (DMA, IRQ,
                     medium send, logging). No QEMU/libvfio-user deps.
  vwifi_device.h     Public interface of the portable device module.
  vwifi_device.c     Portable device logic. Ring processing, control
                     opcode dispatch, medium protocol, MMIO semantics.
                     Talks to the host ONLY through vwifi_backend_ops.
                     Includes no QEMU or libvfio-user headers.
  vwifi_qemu.c       QEMU backend: PCIDevice boilerplate, property
                     parsing, chardev callbacks, MMIO/MSI-X init, and
                     one vwifi_backend_ops instance that adapts QEMU
                     APIs to the portable device.
  vwifi_crypto.h/c   AES-128 and CCMP-128.
  meson.build        Registers the source with QEMU's build system.
  Kconfig            Turns the device on in QEMU builds.
scripts/
  integrate.sh       Copies sources + abi/ headers into a QEMU tree and
                     patches its meson.build and Kconfig. Idempotent.
tests/
  mock_backend.h/c   In-memory backend for unit tests.
  run_tests.sh       Builds and runs every test below.
  medium_proto.c     Wire-protocol conformance against abi/vwifi.h.
  smoke_get_caps.c   GET_CAPS round-trip end-to-end.
  crypto.c           AES/CCMP known-answer and tamper tests.
  monitor_mode.c     Monitor RX, raw filtering, injection.
  scan.c             Channel hopping, beacon parsing, BSS table.
  connect.c          Auth/assoc, 802.3 <-> 802.11 framing.
  wpa2.c             WPA2-PSK, CCMP, replay and forgery rejection.
  softap.c           Beacons, probe responses, association.
```

## Building it into QEMU

Requires **QEMU 10.2 or newer**. The backend tracks QEMU's API, and
several things it touches changed incompatibly:

*In 10.0* — `qdev-properties*.h` moved under `hw/core/`, `class_init`
callbacks take a `const void *`, `Property` arrays became const and lost
their `DEFINE_PROP_END_OF_LIST` terminator, `DeviceClass::reset` was
replaced by `device_class_set_legacy_reset()`, and `sysemu/dma.h` became
`system/dma.h`.

*In 10.2* — `CharBackend` was renamed `CharFrontend`. This is the one
that makes 10.2 a hard floor rather than a preference: the type name is
in the device struct and every `qemu_chr_fe_*` call signature.

```bash
make integrate configure build QEMU_SRC=/path/to/qemu
```

Then attach a guest to a running hub:

```bash
qemu-system-x86_64 ... \
  -chardev socket,id=medium,path=/tmp/vwifi.sock,server=off \
  -device vwifi-virt,chardev=medium,node_id=win-guest
```

Properties: `chardev` (the hub socket), `node_id` (name in the hub's
peer list), `mac`, `verbose`.

## The abstraction seam

Every operation the device logic performs against its host environment
goes through one of five vtable ops:

```c
struct vwifi_backend_ops {
    int  (*dma_read) (void *be, uint64_t gpa, void *buf, size_t len);
    int  (*dma_write)(void *be, uint64_t gpa, const void *buf, size_t len);
    void (*raise_irq)(void *be, unsigned vec);
    int  (*medium_send)(void *be, const void *buf, size_t len);
    void (*log)(void *be, const struct vwifi_dev *dev,
                enum vwifi_log_level level, const char *fmt, va_list ap);
    uint64_t (*now_us)(void *be);
    void (*timer_mod)(void *be, uint32_t ms);
    void (*timer_del)(void *be);
};
```

- `dma_read` / `dma_write` — QEMU wraps `pci_dma_read/write`; vfio-user
  wraps `vfu_addr_to_sgl` + `vfu_sgl_read/write`; the mock does memcpy
  against a malloc'd region.
- `raise_irq` — QEMU calls `msix_notify`; vfio-user calls
  `vfu_irq_trigger`; mock records into a ring buffer.
- `medium_send` — QEMU calls `qemu_chr_fe_write`; vfio-user does a raw
  socket write; mock captures the bytes.
- `log` — routes to whatever the backend has (qemu_log, syslog, stderr).
- `now_us` — QEMU uses `QEMU_CLOCK_VIRTUAL`; vfio-user uses
  `CLOCK_MONOTONIC`; the mock uses a manually-advanced counter so
  scan tests are deterministic.
- `timer_mod` / `timer_del` — a single one-shot timer driving the scan
  dwell state machine. QEMU uses `QEMUTimer`; vfio-user uses its event
  loop; the mock lets tests fire it explicitly.

No other host interaction happens outside these ops. The device logic
is a pure state machine driven by four public entry points:

```c
uint64_t vwifi_reg_read (struct vwifi_dev *d, uint32_t off, unsigned size);
void     vwifi_reg_write(struct vwifi_dev *d, uint32_t off, uint64_t v, unsigned s);
void     vwifi_medium_rx_bytes(struct vwifi_dev *d, const void *buf, size_t len);
void     vwifi_medium_link_event(struct vwifi_dev *d, bool link_up);
void     vwifi_timer_expired(struct vwifi_dev *d);
```

The backend calls these when the guest touches MMIO, when bytes arrive
on the hub socket, when the hub link opens/closes, or when the one-shot
timer fires.

## Porting to vfio-user

The Phase-5 port is expected to be:

1. Copy `vwifi_qemu.c` to `vwifi_vfio_user.c`.
2. Replace QEMU includes with `<libvfio-user.h>`.
3. Replace the five backend op implementations with libvfio-user calls.
4. Replace the `PCIDevice` boilerplate with `vfu_create_ctx` +
   `vfu_setup_region` + `vfu_setup_device_nr_irqs`.
5. Replace chardev callbacks with a poll loop that reads the hub
   socket and calls `vwifi_medium_rx_bytes`.
6. Add a `main()` that parses argv.

`vwifi_device.c`, `vwifi_device.h`, `vwifi_backend.h`, and `vwifi_abi.h`
do not change.

## Tests

```
make test          # or: cd tests && ./run_tests.sh
```

Should end with `All tests passed.` No QEMU, no guest, no WDK.

| Test | Covers |
|---|---|
| `crypto` | AES-128 FIPS-197 known-answer, CCMP round trip, tamper and wrong-key rejection, PN handling |
| `medium_proto` | `vwifi_frame_hdr` size and every field offset, hello handshake bytes, transmitted frame header |
| `smoke_get_caps` | Ring setup, control dispatch, GET_CAPS round trip, MSI-X raise |
| `monitor_mode` | Monitor RX with RAW flag + metadata, raw-filter gating, injection sourcing tx_mac from frame addr2 |
| `scan` | Channel hopping, probe-request injection, beacon parsing, BSS table, per-scan dedup, IE capture, SCAN_COMPLETE, abort |
| `connect` | Auth and assoc exchange, 802.3 <-> 802.11 translation, foreign-BSSID filtering, deauth, connect timeout |
| `wpa2` | WPA2-PSK association, CCMP encrypt/decrypt against a peer, EAPOL passthrough, replay and forgery rejection, GTK |
| `softap` | Beacon timing, probe responses, auth/assoc handling, STOP_AP teardown, 5 GHz |

### Why `medium_proto` exists

This device was developed against a hand-written stub of
`ath9k_medium.h`, because the real header lived in another repository.
The stub got the struct layout right and the magic value wrong, and
byte-swapped the hello handshake. Nothing about that fails loudly: the
hub simply never registers the peer and drops every frame, silently.

The stub is gone — the device now includes the authoritative
`abi/vwifi.h` — and `medium_proto.c` asserts the wire format field by
field so the two cannot drift apart again.

The mock backend is a real proof point that the abstraction works —
we can drive the device logic through GET_CAPS end-to-end, verify the
response payload, and confirm an MSI-X vector was raised, all without
any QEMU or libvfio-user in the process.
