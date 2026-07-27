# macOS driver for vwifi-virt — planned, with caveats

Nothing here yet. Unlike the [Linux port](../linux/), this one has an
open feasibility question, and it should be answered before anyone
budgets time for it.

## The problem

macOS has no public API for writing a Wi-Fi driver.

Ethernet and most other classes have a supported path: DriverKit, since
macOS 10.15, with `IOEthernetDriverKit` for NICs, running in userspace,
installable without disabling System Integrity Protection. There is no
`IO80211` equivalent in DriverKit. The 802.11 family — `IO80211Family`,
`IO80211Controller`, `IO80211Interface` — is a private kernel framework
with no published headers and no stability guarantee across releases.

The kext route (`IOKit`, `IO80211Controller`) is what third-party
projects have historically used, and it costs:

- A kernel extension, which on Apple Silicon means Reduced Security mode
  and a per-machine approval dance.
- Reverse-engineered headers that Apple can change in any point release.
- Codesigning with a KEXT-enabled Developer ID, which Apple grants
  case by case and is increasingly unwilling to grant at all.
- No forward path — Apple has said kexts are deprecated, repeatedly.

## Options worth evaluating first

**1. Present as Ethernet instead of Wi-Fi.** A DriverKit
`IOEthernetDriverKit` driver over the same `vwifi_abi.h` rings, with the
device kept in a fixed associated state. Fully supported, no SIP
changes, no kext. The guest gets working networking over the virtual
medium — but no Wi-Fi UI, no `airport` scan results, no association
control from inside the guest. For a lab where the *medium* is the
interesting part and macOS is just a node on it, this may be entirely
sufficient. **This is the option to cost out first.**

**2. Userspace client, no driver.** Expose the medium to a macOS guest
through a userspace process speaking the medium protocol directly, with
no PCI device involved. Nothing binds to the network stack, so this only
serves guests running purpose-built test software — but it needs no
driver at all.

**3. A real `IO80211` kext.** The only option that produces genuine
Wi-Fi behaviour in the macOS UI, and the only one with an ongoing
maintenance burden proportional to Apple's release cadence.

## If option 3 is chosen anyway

The device side is done and does not change: the same
[`vwifi_abi.h`](../../../abi/vwifi_abi.h) rings and opcodes, the same
device-owned 802.11 state machine, the same host-side tests
(`make -C devices/vwifi test`) to check expected behaviour without a VM
in the loop. The port is entirely a question of the macOS-facing half.

Start by getting a stub kext to bind to PCI `1AF4:0E00` and read
`VWIFI_REG_SIGNATURE`. If that much cannot be made to load and stay
loaded on the target macOS version and hardware, nothing after it
matters, and the answer is option 1.
