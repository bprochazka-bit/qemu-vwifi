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
vwifi.sln           solution — one project, Debug|x64 and Release|x64
vwifi.vcxproj       MSBuild driver project (EWDK or VS 2022 + WDK)
build.cmd           debug build wrapper; writes build-Debug.{log,err,wrn}
sign.cmd            assembles the package, catalogs, test-signs
sign.ps1            what sign.cmd actually runs
install.cmd         run in the guest: replaces any installed vwifi package
install.ps1         what install.cmd actually runs
find-wdk-file.ps1   locates the WDI TLV header and library for build.cmd
guest-debug-setup.ps1  run in the guest: test-signing, cert, print filter
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

The project files are checked in, so there is nothing to generate.

### Option A — Enterprise WDK (EWDK), no Visual Studio install

The EWDK is a command-line, self-contained WDK: one ISO, no installer.

1. Mount the EWDK ISO (right-click → Mount, or `Mount-DiskImage`).
   Use an EWDK for Windows 11 22H2 (10.0.22621) or later — `tlv_mem.cpp`
   calls `ExAllocatePool2`, which older kits don't have.
2. From a normal `cmd.exe`, launch the build environment. This sets
   `PATH`, `DDK_LIB_PATH` and the rest, and it **must** be a `cmd`
   shell, not PowerShell:
   ```
   D:\LaunchBuildEnv.cmd
   ```
3. In that shell:
   ```
   cd C:\src\qemu-vwifi\drivers\vwifi\windows
   build.cmd
   ```

`build.cmd` builds `Debug|x64` and writes three logs beside itself:
`build-Debug.log` (everything), `build-Debug.err` (errors only) and
`build-Debug.wrn` (warnings only). On failure it prints the error log
and exits non-zero — `build-Debug.err` is the file to paste when
reporting a broken build.

Output lands flat in `x64\Debug\`: `vwifi.sys`, `vwifi.inf`,
`vwifi.pdb`. Note there is no `vwifi\` subfolder at this point —
MSBuild only creates the driver-package folder as part of its own
signing step, which this project turns off so a missing certificate
cannot fail a build. `sign.cmd` assembles `x64\Debug\vwifi\` itself,
because `Inf2Cat` catalogs a directory rather than a file list. **That
subfolder is the thing you copy to the guest.**

### Option B — Visual Studio 2022 + WDK

1. Install **Visual Studio 2022** with the "Desktop development with
   C++" workload, then the **WDK** matching it (10.0.22621 or later);
   the WDK installer registers the MSBuild driver targets.
2. Open `vwifi.sln`, pick **Debug | x64**, Build.

Either way the same `vwifi.vcxproj` is used, so the two produce the
same binary.

### What the Debug configuration turns on

| Setting | Effect |
|---|---|
| `Optimization=Disabled` | locals aren't elided; stepping matches the source |
| `DBG=1` | `NT_ASSERT` and `ASSERT` are live |
| `DebugInformationFormat=OldStyle` | debug info in the `.obj`s; the linker still emits a full `vwifi.pdb`, but nothing depends on a shared `vc145.pdb` surviving to link time |
| `TreatWarningAsError=false` | one warning doesn't hide the next fifty on a first build |
| `TreatLinkerWarningAsErrors=false` | the driver targets pass `/WX` to the linker too, and it is governed separately from the compiler's |
| `SignMode=Off` | signing is `sign.cmd`'s job, so a missing cert isn't a build failure |
| `SpectreMitigation=false` | the Spectre-mitigated libs are an optional EWDK component |
| `RunCodeAnalysis=false` | run it deliberately: `msbuild /p:RunCodeAnalysis=true vwifi.sln` |

Turn `TreatWarningAsError` back on (`/p:TreatWarningAsError=true`) once
it builds clean — `/W4` on a kernel driver catches real bugs.

### Signing

```
sign.cmd
```

Assembles `x64\Debug\vwifi\` from the build output, creates
`CN=vwifi-test-cert` in `Cert:\CurrentUser\My` the first time (reused
after that), exports it to `vwifi-test-cert.cer`, runs `Inf2Cat` over
the package folder, and signs both `vwifi.cat` and `vwifi.sys`. Copy
the `.cer` to the guest along with the package.

### If the first build fails

This driver has never been compiled — it was written and reviewed
against the WDK headers, not built against them. Expect errors, and
expect most of them to be shallow. The ones worth recognising:

| Symptom | Cause and fix |
|---|---|
| `msbuild is not recognized` | `LaunchBuildEnv.cmd` wasn't run, or it was run from PowerShell. It only sets up a `cmd.exe` environment. |
| `Cannot open include file: 'vwifi_abi.h'` | the project was copied out of the repo; the include path is `..\..\..\abi`, relative to the `.vcxproj`. |
| `Cannot open include file: 'dot11wdi.h'` / `'wditypes.hpp'` | the mounted kit is too old or is an SDK rather than a WDK. These ship in the WDK's `Include\<ver>\shared`. |
| Errors *inside* `TlvGenerated_.hpp` — `MLO_LINK_INFO` undeclared, `unknown override specifier`, `__C_ASSERT__` redefinition | the wrong copy of the header was picked. A kit ships several: this driver needs `Include\<ver>\km\wlan\1.0` (kernel-mode, WDI 1.x, matching the `dot11wdi.h` it is written against). `um\wlan\2.0` is user-mode WDI 2.0 and cannot compile in a kernel translation unit. Check the `/I` path in the build log, and override with `set WdiTlvIncludeDir=<the km\wlan\1.0 folder>` if the search picked wrong. |
| `Cannot open include file: 'TlvGeneratorParser.hpp'` | that one is *not* on the kit's default include path even when the other two are. `build.cmd` searches for it and passes its folder as `WdiTlvIncludeDir`; if the search comes up empty, find it with `dir /s /b "%ProgramFiles(x86)%\Windows Kits\10\TlvGeneratorParser.hpp"` and re-run as `set WdiTlvIncludeDir=<folder> && build.cmd`. If it is nowhere on the machine, the kit doesn't ship it — it comes from Microsoft's WDI TLV generator/parser tooling and has to be obtained separately. |
| `ExAllocatePool2: identifier not found` | kit older than 10.0.20348. Either mount a newer EWDK, or follow the comment in `tlv_mem.cpp` and swap to `ExAllocatePoolWithTag(NonPagedPoolNx, ...)` + `RtlZeroMemory`. |
| `NDIS version not defined` / `NDIS_SUPPORT_NDIS650` errors | the `NDIS650_MINIPORT=1` define didn't take. Check the `PreprocessorDefinitions` in `vwifi.vcxproj` survived any local edit. |
| Redefinition storms from `ntddk.h` / `ntifs.h` | `vwifi_drv.h` includes `<ntifs.h>` before `<ndis.h>`, and `ndis.h` pulls in `ntddk.h`. The driver uses nothing that `ntddk.h` lacks, so dropping the `ntifs.h` include is the fix if your kit doesn't tolerate the pairing. |
| `unresolved external symbol Ndis*` | `ndis.lib` isn't being linked — check `$(DDK_LIB_PATH)` resolved (it's set by `LaunchBuildEnv.cmd`). |
| `LNK2019: unresolved external symbol ParseWdiTaskScanToIhv` (and the other `ParseWdi*` / `GenerateWdi*` / `FreeGenerated`) | the WDI TLV static library isn't linked. `TlvGenerated_.hpp` only *declares* these; the code ships in a `.lib` in the kit's Lib tree. `build.cmd` searches for it and passes it as `WdiTlvLib`; if the search comes up empty, find it with <code>dir /s /b "%ProgramFiles(x86)%\Windows Kits\10\Lib\*.lib" \| findstr /i wlan</code> and re-run as `set WdiTlvLib=<full path> && build.cmd`. |
| `unresolved external symbol "void * __cdecl operator new"` | the TLV library wants an overload `tlv_mem.cpp` doesn't provide yet; add it there, matching the existing ones. |
| `LNK4099: PDB 'vc145.pdb' was not found` then `LNK1218` | the shared compiler PDB didn't survive to link time — common when building from a mapped or network drive. The project uses `/Z7` (`OldStyle`) so there is no shared PDB to lose; if you see this, something has overridden `DebugInformationFormat`. |
| `sign.cmd` dies with `Unexpected token` and mojibake like `â€”` in the message | a `.ps1` file has non-ASCII in it. Windows PowerShell 5.1 reads scripts as ANSI unless they carry a UTF-8 BOM, so an em dash in a string literal becomes three bytes and breaks the parse. The scripts here are kept ASCII-only for that reason; keep them that way when editing. |
| `Inf2Cat`: `Could not find file <path>` for a file that is plainly there, then `Signability test failed.` | the package is on a mapped network drive. Inf2Cat cannot resolve driver directories on one, and fails the same way through a UNC path. `sign.ps1` stages to a local folder under `%TEMP%`, catalogs and signs there, then copies the signed files back — so this should not recur, but that is the cause if it ever does. |
| `drvcat.exe` exits `1003` after `Unable to determine if catalog is signed. Error = 0x800B0109` | the previous `sign.cmd` left a signed `vwifi.cat` in the output tree, and `drvcat` will not overwrite a signature it cannot validate — `0x800B0109` is `CERT_E_UNTRUSTEDROOT`, i.e. our own test certificate. So the first build after every successful sign fails. `EnableInf2Cat=false` does not suppress this: kits from 10.0.26100 on run `drvcat.exe`, and that property names the old tool. `build.cmd` now deletes `x64\<cfg>\*.cat` recursively before invoking MSBuild. |
| `Inf2Cat` reports a signability error | the INF, not the code — `setupapi.dev.log` isn't involved yet. Its message names the offending directive. Note Inf2Cat exits 0 even when it fails, so `sign.ps1` checks whether the catalog actually appeared rather than trusting the exit code. |

Warnings are not errors here (`TreatWarningAsError` is off), but read
`build-Debug.wrn` anyway — on a first kernel build `/W4` warnings about
truncation, uninitialised locals and signedness are usually real.

### What the first real build turned up

The driver was written against Microsoft's WDI documentation rather
than against `dot11wdi.h`. The first compile found where the two differ.
Recorded here because each one is a trap the next WDI driver hits too:

- **Role types are not the names in the characteristics struct.** The
  field is `StartOperationHandler` of type
  `MINIPORT_WDI_START_OPERATION_HANDLER`, but the *function* type to
  declare against is `MINIPORT_WDI_START_ADAPTER_OPERATION`. Same for
  stop, post-pause, post-restart and hang-diagnose. Declaring against
  the `_HANDLER` name silently declares a data variable, and the error
  surfaces later as "redefinition; previous definition was 'data
  variable'" at the function body — nowhere near the real mistake.
  `MINIPORT_DRIVER_UNLOAD` vs `MINIPORT_UNLOAD` in `ndis.h` is the same
  trap.
- **AllocateAdapter is the WDI model's `MiniportInitializeEx`.** Its
  real signature takes the NDIS miniport handle, the PnP init
  parameters and an `_Inout_` registration attributes block to fill in
  — not the "adapter attributes" pointer the docs imply. Adapter
  creation therefore lives in `wdi_ops.c`, and `driver.c` no longer
  registers `InitializeHandlerEx`/`HaltEx` at all.
- **OpenAdapter and CloseAdapter are asynchronous.** The completion
  routines arrive in `NDIS_WDI_INIT_PARAMETERS`. Returning success
  without calling them leaves the WLAN component waiting forever, with
  nothing logged.
- **`WDI_MESSAGE_HEADER` has no message id and no length field.** It
  carries `PortId`, `Status`, `TransactionId`, `IhvSpecificId`; the id
  and length come from the NDIS status code and buffer size. Task
  completions must echo the request's `TransactionId` or the OS never
  matches them to the task.
- **The TLV generator already reserves the message header.** Every
  `Generate*` call passes `kHeaderReserve`, so the returned blob starts
  with that many bytes for the driver to fill in and its length counts
  them. Prepending another header produces a message with two, the
  second one all zeroes.
- **There is no `ASSOCIATION_START` indication.** `dot11wdi.h` has
  `ASSOCIATION_RESULT` (76) and `CONNECT_COMPLETE` (64); the connect
  sequence is those two. `WDI_INDICATION_ASSOCIATION_START_PARAMETERS`
  in `wditypes.hpp` is a container inside the result, not a message.
- **WDI has no monitor mode.** `WDI_OPERATION_MODE` covers STA and the
  three P2P roles; the string "monitor" appears nowhere in
  `dot11wdi.h`, `wditypes.hpp` or `WABIModel.xml`. So
  `OID_WDI_TASK_CHANGE_OPERATION_MODE` can only ever ask for STA, and
  the Native 802.11 `OID_DOT11_CURRENT_OPERATION_MODE` — a plain NDIS
  set request carrying `DOT11_OPERATION_MODE_NETWORK_MONITOR` from
  `windot11.h` — is the only route left. Whether the Microsoft WLAN
  component forwards that OID down to a WDI miniport at all is the open
  question for Phase 1.5; both paths are wired up in `oids.c` so the
  debug log will say which one fires.
- **`AllocateAdapter` cannot allocate anything NDIS-managed.** It fills
  in registration attributes that the WLAN component applies *after* it
  returns, so during the call the adapter is not yet a registered
  miniport. Rings, NBL pools and interrupts belong in `VwifiHwStart`,
  called from `OpenAdapter`.
- **`NdisMAllocateSharedMemory` never works here, whenever you call
  it.** A 1536-byte ring allocation returns `NDIS_STATUS_RESOURCES` on
  a completely idle machine — from `AllocateAdapter`, and still from
  `OpenAdapter` after the split above, where the registration
  attributes are unambiguously live. The routine needs
  `NDIS_MINIPORT_ATTRIBUTES_BUS_MASTER` in effect; we do request that
  flag, so the inference is that the WLAN component does not carry our
  `AttributeFlags` through when it applies the attributes on our
  behalf. The log proves only that NDIS refuses, not why. Either way
  the rings now come from the PDO's own DMA adapter —
  `IoGetDmaAdapter` in `VwifiHwStart`, `AllocateCommonBuffer` in
  `VwifiDmaAlloc` — which depends on none of the miniport attribute
  state. Symptom if you get this wrong: Code 10, with
  `ring ctrl-req: ... alloc failed (1536 bytes)` in DebugView.
- **The generated TLV headers are split across two folders.**
  `dot11wdi.h` and `wditypes.hpp` are on the kit's default include
  path; `TlvGeneratorParser.hpp` and the `TlvGenerated_.hpp` it
  includes are not. `build.cmd` finds them and passes the folder as
  `WdiTlvIncludeDir`.

## Test-signing and installation

`sign.cmd` leaves everything the guest needs in `x64\Debug\vwifi\` —
the driver, the install scripts, the setup script and the certificate.
Copy that one folder across, then, in an **elevated PowerShell** there:

```powershell
.\guest-debug-setup.ps1 -CertPath .\vwifi-test-cert.cer -KernelDebug
# reboot
.\install.cmd
```

### Replacing an already-installed build

This is the loop you will run dozens of times, and it has one trap
worth knowing about.

`pnputil /add-driver vwifi.inf /install` on its own is **not** enough to
replace a driver. PnP keeps the package already bound to the device
unless the new one ranks better, and ranking is on `DriverVer`. stampinf
writes that as `<date>,<version>` — a date, not a timestamp — so two
builds on the same day with the same version are indistinguishable, the
old package wins, and you spend an afternoon testing a `.sys` you
thought you had replaced.

Two things address it:

- `build.cmd` now stamps a version derived from the build time
  (`1.0.MMdd.HHmm`), so every build strictly supersedes the last.
- `install.cmd` removes every previously installed vwifi package
  (`pnputil /delete-driver oemNN.inf /uninstall /force`) before adding
  the new one, rather than relying on ranking at all.

So the full cycle, after any code change:

```
build.cmd  &&  sign.cmd          (on the build machine)
```
copy `vwifi.sys` / `.inf` / `.cat` / `.pdb` to the guest, then there:
```
install.cmd                       (elevated)
```

`install.cmd` finishes by printing the device's status and problem
code, so you know immediately whether it started. Nothing needs a
reboot — only the one-time `guest-debug-setup.ps1` does.

Two things worth keeping straight while iterating:

- **Re-run `sign.cmd` after every build.** `vwifi.cat` hashes
  `vwifi.sys`; a rebuilt `.sys` with the old catalog is a signature
  mismatch, which PnP reports as Code 52 rather than anything about
  staleness.
- **Copy the `.pdb` too, and keep it beside the `.sys`.** Not needed to
  install, but WinDbg resolves symbols from it and a stale one is worse
  than none.

`guest-debug-setup.ps1` enables test-signing, imports the certificate
into both Root and TrustedPublisher, raises the `IHVNETWORK` debug
print filter so the driver's logging is actually emitted, and (with
`-KernelDebug`) turns on kernel debugging over COM1. Add `-Verifier`
to enable Driver Verifier's standard checks against `vwifi.sys` —
worth it for the first few boots, since it converts silent corruption
into an attributable bugcheck.

Two prerequisites the script cannot set for you, both of which will
otherwise block a test-signed driver:

- **Secure Boot** must be off in the VM firmware.
- **Memory integrity** (Windows Security → Device security → Core
  isolation) must be off.

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

All `VWIFI_INFO/WARN/ERR` output goes to the kernel debug output and
can be captured live with Sysinternals DebugView running as admin with
`Capture → Capture Kernel` (and `Capture → Enable Verbose Kernel
Output`) enabled.

**The output is filtered off by default.** `VWIFI_DBG` calls
`DbgPrintEx(DPFLTR_IHVNETWORK_ID, ...)`, and every component mask
except `DEFAULT` starts at zero — so the INFO and WARN lines are
emitted and then dropped before anything can see them. A driver that
appears completely mute is almost always this, not a driver that never
ran. `guest-debug-setup.ps1` sets the mask; to do it by hand:

```powershell
$k = 'HKLM:\SYSTEM\CurrentControlSet\Control\Session Manager\Debug Print Filter'
New-Item -Path $k -Force | Out-Null
New-ItemProperty -Path $k -Name IHVNETWORK -Value 0xFFFFFFFF -PropertyType DWord -Force
# reboot, or from a live kernel debugger: ed nt!Kd_IHVNETWORK_Mask 0xFFFFFFFF
```

### Kernel debugging with WinDbg over a QEMU named pipe

Add to the QEMU command line:
```
-serial pipe:\\.\pipe\windbg-vwifi
```
Inside the Windows VM, enable kernel debugging over COM1 (this is what
`guest-debug-setup.ps1 -KernelDebug` does):
```
bcdedit /debug on
bcdedit /dbgsettings SERIAL DEBUGPORT:1 BAUDRATE:115200
```
Reboot. Then on the host:
```
windbg -k com:pipe,port=\\.\pipe\windbg-vwifi,resets=0,reconnect
```

Point the debugger at your build so symbols and source resolve:
```
.sympath+ C:\src\qemu-vwifi\drivers\vwifi\windows\x64\Debug\vwifi
.srcpath+ C:\src\qemu-vwifi\drivers\vwifi\windows\src
.reload /f vwifi.sys
```

Break in before the driver initialises — the interesting failures all
happen during `MiniportInitializeEx`, which runs too early to catch by
hand:
```
sxe ld vwifi.sys          ; break when the image loads
bp vwifi!VwifiMiniportInitializeEx
bp vwifi!VwifiCtrlSendSync
bp vwifi!VwifiMessageIsr
```
The lifecycle is linear enough that stepping through those three once
shows you everything.

That recipe assumes a Windows host. For a Linux host see the section
below, which is a different technique rather than a translation of this
one — and the better one when the guest dies without bugchecking.

Useful commands once it's loaded:

| Command | What it tells you |
|---|---|
| `!ndiskd.miniports` | is the miniport registered, and in what state |
| `!ndiskd.miniport <handle>` | its NDIS state, and why it halted |
| `lm m vwifi` | did the image load, and did symbols resolve |
| `!poolused 4 fiWv` | driver allocations by tag (`'fiWv'` = `VWIFI_POOL_TAG`) |
| `!analyze -v` | after a bugcheck, before anything else |

### The WDI adapter bring-up sequence

Worth having written down, because every failure so far has been a step
in it going unanswered, and the failures look identical from outside:
the adapter opens, a little happens, and it closes again.

After `MiniportWdiOpenAdapter` completes, the Microsoft WLAN component
does this, in this order:

| Step | What it is |
|---|---|
| `MiniportWdiTalTxRxInitialize` | hand over the data-path handler table |
| `OID_WDI_GET_ADAPTER_CAPABILITIES` | read what the radio can do |
| `OID_WDI_SET_ADAPTER_CONFIGURATION` | push MAC and firmware settings down |
| `OID_WDI_TASK_SET_RADIO_STATE` | *only if* the radio is not already in the expected state |
| `MiniportWdiTalTxRxStart` | start the data path |
| `OID_WDI_TASK_CREATE_PORT` | create the first port |

`OID_WDI_TASK_OPEN` is always first, but it does not arrive as an OID —
it is delivered as the `OpenAdapterHandler` callback. Beyond that, the
order of the rest is not guaranteed.

Two things follow from having this table:

- **Where the trace stops is the diagnosis.** Stopping after
  `SET_ADAPTER_CONFIGURATION` with `SET_RADIO_STATE` skipped means the
  component could not get as far as starting the data path.
- **`SET_RADIO_STATE` being absent is normal**, not a missing step, as
  long as the capabilities report `HardwareRadioState` and
  `SoftwareRadioState` as TRUE. Those are "is the radio *enabled*",
  so TRUE means on and there is nothing for the OS to change.

### "Optional" in WABIModel.xml does not mean optional

The model file describes the wire format, not the requirements. A
container marked `optional="true"` may be omitted and the message will
still generate and parse cleanly — and the adapter will still not work.
This has now cost two rounds:

| Container | Marked | Actually |
|---|---|---|
| `BandInfo`, `PhyInfo` | optional | without them the adapter has no channels; the component reads the capabilities and closes |
| `DatapathAttributes` | optional | without it there is no `MaxNumPeers` to build a `WDI_TXRX_TARGET_CONFIGURATION` from, so `MiniportWdiTalTxRxStart` is never called |

`FirmwareVersion` is the opposite trap and easier to spot: genuinely
mandatory, and leaving it as a zeroed `ArrayOfElements` fails the
generate outright with `NDIS_STATUS_INVALID_DATA` (`0xc0010015`), which
at least says something.

The rule that actually holds: if a container describes something the
adapter *is*, fill it in, whatever the model says. Only omit containers
describing features the device genuinely does not have.

### Recovering a guest that hangs at boot

Once the package is installed, PnP starts the driver during boot. If it
locks the machine there, the guest hangs at the Windows logo on every
boot and there is no desktop to uninstall from.

The INF sets `ErrorControl = 1` (`SERVICE_ERROR_NORMAL`), which means
Windows logs a failure to load and carries on booting. So the fix is to
make the driver fail to load — take the `.sys` away from outside:

```bash
sudo modprobe nbd max_part=8
sudo qemu-nbd --connect=/dev/nbd0 /path/to/win11.qcow2
sudo mkdir -p /mnt/win && sudo mount /dev/nbd0p3 /mnt/win   # the Windows partition
sudo mv /mnt/win/Windows/System32/drivers/vwifi.sys \
        /mnt/win/Windows/System32/drivers/vwifi.sys.bak
sudo umount /mnt/win && sudo qemu-nbd --disconnect /dev/nbd0
```

Boot, and the adapter comes up with a yellow bang instead of a hang.
Then remove the package properly:

```
pnputil /enum-drivers | findstr /i vwifi
pnputil /delete-driver oemNN.inf /uninstall /force
```

The VM must be shut down before connecting `qemu-nbd`; two writers on
one image will corrupt it. Check `lsblk /dev/nbd0` if you are unsure
which partition is Windows — it is the large NTFS one, not the small
recovery or EFI partitions.

If you would rather stay inside the guest, force-power-cycle it twice
mid-boot to trigger Automatic Repair, then Advanced options → Startup
Settings → Safe Mode (`4`). Network miniports do not start in Safe
Mode, so `pnputil /delete-driver` works there. That is slower and
depends on WinRE being intact, which is why the offline route is
listed first.

### Debugging from a Linux host

A guest that resets without a bugcheck cannot be debugged from inside
itself. There is no stop screen and no `MEMORY.DMP` because Windows
never ran its crash path — the CPU faulted while already handling a
fault, or never came back at all. Everything below watches from the
host, where the evidence still exists.

This is not a workaround for having no WinDbg. For this class of
failure it is strictly better, because QEMU sees the reset itself.

**0. Capture the driver's own trace, on the host.** Do this before
anything else — it is the only sink that survives the guest dying, and
it usually answers the question on its own.

```
-debugcon file:/tmp/vwifi-boot.log
```

Every `VWIFI_INFO`/`WARN`/`ERR` line is mirrored to I/O port 0xE9 in
Debug builds, and QEMU appends each byte to that file as the guest
writes it. Unlike DebugView, whose buffer dies with the VM, and unlike
DbgPrintEx, which needs an attached debugger and produces nothing
during boot, this file is complete right up to the last instruction the
driver executed. **The last line in it is where the machine froze.**

This works at boot, which DebugView cannot do at all, so it is the only
way to see a driver that hangs the guest before there is a desktop.

**1. Stop the machine from resetting.** The single most useful flag:

```
-no-reboot -no-shutdown
```

A triple fault now leaves the VM stopped with its registers intact
instead of vanishing into a reboot. Everything else here depends on the
corpse still being warm.

**2. Log what QEMU sees.**

```
-d int,cpu_reset,guest_errors -D /tmp/vwifi-qemu.log
```

- `cpu_reset` dumps every register, `RIP` included, at the moment of
  reset. On a triple fault that dump *is* the crash report.
- `int` traces interrupt delivery. An interrupt storm is unmistakable:
  the same vector repeating thousands of times per second, and a log
  that grows without bound.
- `guest_errors` catches the device model complaining — a bad MMIO
  offset, a DMA to an unbacked address.

`-d int` is very verbose. Turn it on for the run that reproduces the
crash and off again afterwards.

**3. Attach a debugger to the vCPUs.**

```
-gdb tcp::1234        # or just -s
```

When it locks, from another terminal:

```
gdb -ex 'target remote :1234'
(gdb) info threads          # one per vCPU
(gdb) thread apply all x/i $pc
(gdb) thread apply all p/x $rip
```

All vCPUs spinning at the same address is a deadlock or a storm. One
faulting and the rest idle is a plain crash.

The QEMU monitor answers two questions gdb cannot:

```
(qemu) info irq             # interrupt counts -- a storm shows here
(qemu) info registers -a
```

**4. Turn a RIP into a source line.** `DriverEntry` prints where the
driver landed:

```
vwifi: image base FFFFF8021A340000 size 0x1e000  (RVA = RIP - base)
```

The build emits `x64\Debug\vwifi.map` beside the `.sys`. On the host:

```
./rip2sym.py --map x64/Debug/vwifi.map \
             --base 0xfffff8021a340000 --size 0x1e000 \
             0xfffff8021a34c1f3
```

```
0xfffff8021a34c1f3  RVA 0xc1f3   VwifiOidRequest+0x2b3   [oids.obj]
```

It also takes several addresses at once, or `--stdin` to scrape them
out of a QEMU log. An address outside the image is reported as such
rather than resolved to a bogus symbol — "another module" is an answer.

None of this needs a `.pdb` or any Windows tooling.

**What the outcomes mean.** With `AutoReboot` off and a kernel dump
configured (`guest-debug-setup.ps1` step 5/5), the three cases separate
cleanly:

| What you see | What it is |
|---|---|
| Stop screen stays up, `MEMORY.DMP` written | an ordinary bugcheck. Windows was healthy enough to report it; the dump has the stack. |
| No screen, no dump, `-d cpu_reset` shows a reset | triple fault. The register dump has the `RIP` — feed it to `rip2sym.py`. |
| No screen, no dump, no reset logged, vCPUs pegged | a true hard lock. Check `info irq` for a storm, and `thread apply all x/i $pc` for where they are stuck. |

In all three cases read `/tmp/vwifi-boot.log` first. A bugcheck tells
you where Windows noticed; the 0xE9 trace tells you what the driver was
doing, which is usually a more direct answer and never costs a round
trip to collect.

**If you want full WinDbg anyway**, the serial transport is the awkward
part on Linux, so use the network one instead. KDNET needs a NIC
Windows' debug transport supports — QEMU's `e1000e` (Intel 82574L)
qualifies, `virtio-net` does not:

```
-device e1000e,netdev=kd -netdev user,id=kd,hostfwd=udp::50000-:50000
```

In the guest:

```
bcdedit /debug on
bcdedit /dbgsettings net hostip:<the machine running WinDbg> port:50000 key:1.2.3.4
```

Then run WinDbg on the Windows build machine against that port. Worth
setting up once the crash is understood and you want to step through
the WDI handshake; overkill for finding out why the box resets.

### Reading a failed load

If Device Manager shows a yellow bang instead of the adapter, the
error code narrows it fast:

| Code | Meaning | Usual cause |
|---|---|---|
| 52 | signature not verified | test-signing off, Secure Boot on, or memory integrity on |
| 39 | driver corrupt or missing | `.sys` not copied to `System32\drivers`, or unsigned `.cat` |
| 31 | device not working properly | `DriverEntry` or `MiniportInitializeEx` returned a failure status — check the debug output |
| 10 | device cannot start | resource assignment failed; look at `hardware.c`'s BAR/MSI-X parsing. If the log says `got a line-based interrupt`, see below — the driver is refusing to start on purpose. |

`%windir%\inf\setupapi.dev.log` records what PnP did with the INF and
why it rejected it, which covers the install-time failures that never
reach the driver at all.

### The whole VM freezes and reboots during install

Not a bugcheck — no minidump, no Event 1001, nothing in the log,
because the guest never got far enough to write one. That signature is
an interrupt storm, and there was exactly one way to produce it here:
the device running on a **line-based INTx interrupt instead of MSI-X**.

`vwifi-virt` has no working INTx path. Its `raise_irq` backend asserted
the line with `pci_set_irq(pdev, 1)` and nothing ever lowered it —
`irq_status` is cleared by a ring-head MMIO write, which the assert
path never sees. On a level-triggered line that re-enters the ISR
forever and takes the host VM with it. The Linux driver asks for
`PCI_IRQ_MSIX` and refuses anything else, so the path had never been
exercised by anyone.

Getting MSI-X takes **both** of these, and either one alone leaves you
on INTx:

- the INF's `[Vwifi.ndi.NT.HW]` section, which sets `MSISupported = 1`
  under `Interrupt Management` in the device's *hardware* key. Windows
  does not give a PCI device message-signaled interrupts unless its INF
  asks. Note `.HW` — `AddReg` in the plain DDInstall section writes to
  the driver key, where the PCI driver never looks.
- `irq_chars.MsiSupported = TRUE` before `NdisMRegisterInterruptEx`.
  This is the request, not a statement about the hardware; leave it
  `FALSE` and NDIS connects a line interrupt regardless of what the INF
  granted.

`VwifiHwStart` now checks `irq_chars.InterruptType` afterwards and
refuses to enable the device unless it is `NDIS_CONNECT_MESSAGE_BASED`,
so a regression here is a Code 10 with an explanation rather than a
frozen host. The device side no longer asserts INTx at all; it warns
once on the QEMU console and drops the interrupt.

To confirm MSI-X is really in use: Device Manager → the adapter →
Properties → Resources, where message-signaled interrupts show as
**negative** IRQ numbers. The driver also logs `MSI-X connected, 4
messages`.

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
  MCS field, so an HT/VHT frame reports a data rate of 0 rather than a
  legacy-rate equivalent. A capture showing 0 Mb/s is obviously
  incomplete; one showing 6 Mb/s for every 802.11n frame looks right
  and gets believed.

## Status

The phase-by-phase plan in `docs/vwifi-virt-development-plan.md`
describes the intended order of work. In this tree the source covers
Phase 1 (bring-up), 1.5 (monitor), 2 (scan), 3 (connect) and 4 (keys) —
`wdi_scan.c`, `wdi_connect.c`, `wdi_keys.c` and `monitor.c` are all
implemented. What has **not** happened is a build against the WDK or a
run in a Windows guest, so treat "implemented" as "written and
reviewed", not "working".

## Parity with the Linux driver and the device

The Linux driver and the QEMU device have moved since this driver was
written. Changes made here to keep the two guests behaving the same:

| Change | Why it matters here |
|---|---|
| Arm the **whole** ctrl-rsp ring at init (`VwifiRingsArmCtrlRsp`) | The ring was armed one slot per outgoing request, at the request's index. The device picks slots with a producer index of its own that also advances for every asynchronous event, so a single `BSS_FOUND` desynchronised the two: some slots were unarmed when the device needed them (events dropped) and a later preload could overwrite a slot the device had already filled. Both failures are silent. |
| Fill `channel_mask_5` from the device's capabilities | The TLV shim hardcoded "2.4 GHz only, the device has no 5 GHz yet". It does — a Windows guest simply never scanned above channel 14. |
| Stale-scan backstop in `VwifiHandleTaskScan` | A lost `SCAN_COMPLETE` left the task `Active` forever, and every later scan was rejected. Linux arms a timer; here the next scan request completes the stale one first. |
| Report data rate 0 for HT/VHT codes | See the monitor-mode caveat above. |

Three changes needed nothing on this side, because they were fixed in
the device where both drivers benefit:

- A **zero-length SSID** in a scan request is the wildcard, not a
  literal empty name. WDI sends one in essentially every scan, exactly
  as cfg80211 does, and the device used to match it literally — which
  reported hidden APs only and looked like an empty medium.
- Association Requests now carry a **Supported Rates** element. Without
  it hostapd rejects the association with a bare `status=1`.
- A connect request with **no channel** now resolves the channel from
  the device's BSS table. WDI's connect parameters carry no channel at
  all, so this was not an edge case on Windows — it was every connect.

## Not done

- No build against the WDK, and no run in a guest. Everything above is
  unverified on Windows.
- TX power reporting (`OID_WDI_TASK_SET_RADIO_STATE` and friends) has
  no counterpart to the Linux driver's `get_tx_power`/`set_tx_power`.
  Nothing has asked for it yet.
- The scan request ignores WDI's `BandChannelList`; it scans everything
  the device supports and lets the OS filter. Legal, just slower.
