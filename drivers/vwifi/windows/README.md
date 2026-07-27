# vwifi — Windows WDI miniport for qemu-vwifi

Phase 1 skeleton of the Windows kernel driver that binds to the
`vwifi-virt` QEMU PCI device and presents a WDI miniport to Windows.

Status: **loads cleanly under test-signing, probes the device, reads
capabilities, no user-facing Wi-Fi yet.** The scan/connect/key path
and monitor-mode RX indications land in Phases 2 and 1.5 respectively.

## Layout

The shared ABI header is NOT in this directory — it lives once at
the top of the repository, in `abi/vwifi_abi.h`, and is shared verbatim
with the QEMU device. Add `..\..\..\abi` to the project's *Additional
Include Directories* (the path is relative to the project file, which
sits in this directory). Keeping two copies is how they drift; see
[`../../../abi/README.md`](../../../abi/README.md).

```
src/
  vwifi_drv.h       driver-private header; adapter context
  driver.c          DriverEntry + NDIS miniport handlers + send path
  hardware.c        PCI resource parsing, MMIO map, MSI-X, reset
  rings.c           DMA-coherent rings + sync control command helper
  wdi_ops.c         WDI callback stubs (Allocate / Open / Start etc.)
  oids.c            OID dispatch — Native 802.11 monitor-mode OIDs
  monitor.c         Phase 1.5 monitor RX (NBL + DOT11_EXTSTA_RECV_CONTEXT),
                    injection TX, op-mode/channel/filter control helpers
inf/
  vwifi.inx         install file (preprocessed to vwifi.inf during build)
```

## Build environment

You have two options. Both work; pick based on preference.

### Option A — Enterprise WDK (EWDK), no Visual Studio install

The EWDK is a command-line, self-contained WDK. Good when you just
want to build the driver from a fresh Windows VM without installing
VS Community.

1. Download the EWDK ISO matching your target Windows version from
   Microsoft's WDK downloads page.
2. Mount the ISO. Run `LaunchBuildEnv.cmd` to get a shell with the
   build environment set up.
3. From that shell:
   ```
   cd path\to\vwifi-driver
   msbuild /p:Configuration=Debug /p:Platform=x64 vwifi.sln
   ```

You'll need a `vwifi.vcxproj` and `vwifi.sln`. Rather than check
those into the repo (they're opinionated and bulky), generate them
fresh in the next step:

### Option B — Visual Studio 2022 + WDK (recommended for first build)

1. Install **Visual Studio 2022 Community** with the "Desktop
   development with C++" and "Windows kernel mode driver
   development tools" workloads.
2. Install the **WDK** matching Visual Studio (10.0.22621 or later).
   The installer also registers the MSBuild driver templates.
3. In Visual Studio, create a new project:
   - Template: **Kernel Mode Driver, Empty (KMDF)** → no, pick
     **NDIS Miniport** if it's in your installed samples, otherwise
     **Empty WDM Driver** and we'll flip it to NDIS.
4. Remove the template's `.c` file. Right-click the project →
   Add → Existing Item, then add all of `src/*.c` and `src/*.h`.
5. Right-click project → Properties. Set:
   - **Configuration Properties → General → Target OS Version**: Windows 10
   - **Configuration Properties → General → Target Platform**: Desktop
   - **Configuration Properties → Driver Settings → General → Target OS Version**: Windows 10
   - **Configuration Properties → Driver Settings → General → Target Platform**: Desktop
   - **C/C++ → General → Additional Include Directories**: `$(WDKContentRoot)\Include\wdf\kmdf\$(Version_Number);` — you likely also want to add the Native Wi-Fi headers, but they're in the default include path once WDK is installed.
   - **Linker → Input → Additional Dependencies**: add `ndis.lib`.
6. Add `inf/vwifi.inx` to the project as a Driver Install File.
7. Build.

## Test-signing and installation

Inside the Windows VM where you want to run the driver:

```powershell
# ONE-TIME SETUP (requires reboot)
bcdedit /set testsigning on
# Disable memory integrity if enabled:
#   Windows Security → Device Security → Core Isolation → Memory Integrity → Off
# Reboot

# INSTALL (as admin, after copying vwifi.sys / vwifi.inf / vwifi.cat)
pnputil /add-driver vwifi.inf /install
```

For the driver to be signed at all, the build produces an
unsigned `.sys`. Sign it with a self-created test cert:

```powershell
# On your build machine, one-time cert creation:
$cert = New-SelfSignedCertificate -Type CodeSigningCert `
    -Subject "CN=vwifi-test-cert" `
    -KeyUsage DigitalSignature `
    -CertStoreLocation "Cert:\CurrentUser\My" `
    -HashAlgorithm sha256

# Export it so the VM can trust it:
Export-Certificate -Cert $cert -FilePath vwifi-test-cert.cer

# Sign the driver + inf catalog:
Inf2Cat /driver:. /os:10_x64,Server10_x64
signtool sign /v /fd sha256 /s my /n "vwifi-test-cert" /t http://timestamp.digicert.com vwifi.cat
signtool sign /v /fd sha256 /s my /n "vwifi-test-cert" /t http://timestamp.digicert.com vwifi.sys
```

On the Windows VM, import the cert into both the Trusted Root CAs
*and* the Trusted Publishers stores:

```powershell
certutil -addstore Root       vwifi-test-cert.cer
certutil -addstore TrustedPublisher vwifi-test-cert.cer
```

Then `pnputil /add-driver` will install cleanly without prompting.

## Running it

1. Build QEMU with the `vwifi-virt` device (see qemu-vwifi repo,
   `vwifi-virt` device model).
2. Start the hub:
   ```
   ./ath9k_medium_hub_scalable /tmp/vwifi.sock
   ```
3. Boot the Windows guest with the device attached:
   ```
   qemu-system-x86_64 \
     -machine q35 -m 4096 -smp 4 -enable-kvm \
     -drive file=windows.qcow2,if=virtio \
     -chardev socket,id=medium,path=/tmp/vwifi.sock,server=off \
     -device vwifi-virt,chardev=medium,node_id=win-1 \
     -netdev user,id=net0 -device virtio-net-pci,netdev=net0
   ```
4. Inside Windows, Device Manager should show a "vwifi virtual
   Wi-Fi adapter" under Network Adapters. If it's a yellow bang,
   right-click → Update driver → point at `vwifi.inf`.

## Debugging

Two approaches, both useful:

### DbgPrint viewing with DebugView
All `VWIFI_INFO/WARN/ERR` output goes to the kernel debug output
and can be captured live with Sysinternals DebugView running as
admin with `Capture → Capture Kernel` enabled.

### Kernel debugging with WinDbg over a QEMU named pipe

Add to the QEMU command line:
```
-serial pipe:\\.\pipe\windbg-vwifi
```
Inside the Windows VM, enable kernel debugging over COM1:
```
bcdedit /debug on
bcdedit /dbgsettings SERIAL DEBUGPORT:1 BAUDRATE:115200
```
Reboot. Then on the host:
```
windbg -k com:pipe,port=\\.\pipe\windbg-vwifi,resets=0,reconnect
```
Set breakpoints in `VwifiMiniportInitializeEx`, `VwifiCtrlSendSync`,
`VwifiMessageIsr` — the lifecycle is linear enough that stepping
through once shows you everything.

## Phase 1.5 — monitor mode with Wireshark + Npcap

Once the Phase-1 driver loads, monitor mode lets Wireshark capture raw
802.11 traffic from the virtual medium inside the Windows guest.

### Setup

1. Install **Npcap** (latest) with **"Support raw 802.11 traffic (and
   monitor mode) for wireless adapters"** checked. This is the #1 thing
   people forget — without it Wireshark sees nothing.
2. Launch **Wireshark as Administrator**.

### Capture

1. In Wireshark's capture interface list, find the vwifi adapter.
2. Enable its **Monitor mode** checkbox (Capture → Options → Monitor).
3. Set the channel with WlanHelper (ships with Npcap):
   ```
   WlanHelper "vwifi virtual Wi-Fi adapter" mode monitor
   WlanHelper "vwifi virtual Wi-Fi adapter" channel 6
   ```
4. Start the capture. With a Linux VM running hostapd on channel 6
   against the same hub, you should see beacons decoded with the
   802.11 dissector and a radiotap header (freq 2437 MHz, the RSSI
   the medium reported).

### How it works

- Npcap issues `OID_DOT11_CURRENT_OPERATION_MODE` = NETWORK_MONITOR,
  `OID_DOT11_CURRENT_CHANNEL`, and a raw packet filter. `oids.c`
  intercepts these and pushes `SET_OP_MODE` / `SET_CHANNEL` /
  `SET_RAW_FILTER` to the device.
- The device forwards every medium frame on the tuned channel with
  `VWIFI_RX_F_RAW` set.
- `monitor.c`'s `VwifiRxDrainMonitor` wraps each frame in an NBL and
  attaches a `DOT11_EXTSTA_RECV_CONTEXT` carrying RSSI, channel,
  data rate, and TSF. Npcap reads that OOB info and synthesizes the
  radiotap header for Wireshark.

### Injection

Wireshark/Npcap raw injection (`pcap_sendpacket` with a radiotap +
802.11 buffer) flows through `VwifiMiniportSendNetBufferLists` →
`VwifiInjectFrame`, which strips any radiotap prefix and pushes the
802.11 frame to the TX ring with `VWIFI_TX_F_INJECT`. The device
sources the medium `tx_mac` from the frame's own addr2, so the
medium sees it as coming from whoever the injector claims to be.

### What's NOT modeled

- **No FCS**: the medium doesn't carry an FCS; frames are indicated
  without one. Wireshark shows FCS as absent, not as failed.
- **No HT/VHT MCS in radiotap**: `DOT11_EXTSTA_RECV_CONTEXT` has no
  MCS field, so HT/VHT frames report a legacy-rate equivalent.

## What works in Phase 1

- Driver loads under test-signing, binds to `PCI\VEN_1AF4&DEV_0E00`
- MmioMap BAR0, read `REG_SIGNATURE` and `REG_ABI_VERSION`
- Allocate + program all four rings
- Connect MSI-X interrupts (4 vectors)
- Issue `VWIFI_OP_GET_CAPS` synchronously and parse the reply
- Set the station MAC via `VWIFI_OP_SET_STA_MAC`
- Medium link status reflected in `VWIFI_REG_STATUS`
- `VwifiMiniportCheckForHangEx` probes the signature register
- `VwifiMiniportReset` performs a ring re-init
- All WDI handlers exist and return success; the Microsoft WLAN
  component can drive the lifecycle without wedging

## What doesn't work yet

- No scan → no BSS list in `netsh wlan show networks`
- No connect → no data path
- No RX indication (`VwifiRxDrain` logs the frame descriptor fields
  and re-arms the slot; no NBLs are built)
- No monitor mode (Phase 1.5)
- No security / key install (Phase 4)

## Next steps per the plan

Phase 1.5 (monitor mode): implement `VwifiRxDrain` for real — build
NBLs with `DOT11_EXTSTA_RECV_CONTEXT` attached as media-specific
info, indicate via `NdisMIndicateReceiveNetBufferLists`. Switch TX
to accept raw 802.11 injection when op mode is monitor.

Phase 2 (scan): handle `OID_WDI_TASK_SCAN` in `VwifiOidRequest` by
dispatching a `VWIFI_OP_SCAN` control request; collect BSS events
from the ctrl-rsp ring; emit `NDIS_STATUS_WDI_INDICATION_BSS_ENTRY
_LIST` and `_SCAN_COMPLETE` indications back up.
