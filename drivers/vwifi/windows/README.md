# vwifi — Windows WDI miniport for qemu-vwifi

Phase 1 skeleton of the Windows kernel driver that binds to the
`vwifi-virt` QEMU PCI device and presents a WDI miniport to Windows.

Status: **loads, scans, and connects.** The adapter appears in the
Windows network list, `netsh wlan show networks` finds the AP, and a
connect runs end to end:

```
OID: method 0xe4400006 WDI_TASK_CONNECT
connect parsed: bssid 02:11:22:33:44:01 ssid='vwifi-open'
indicating ASSOCIATION_RESULT status=0 aid=1 ies=25
indicating CONNECT_COMPLETE (0x0)
IND: LINK_STATE connected on ndisport 0
alive: beat 3, scan 0, conn 0x0, assoc 1, port 1, opmode 1
```

**No traffic yet.** The association holds, but
`NdisWdiPeerCreateIndication`/`PeerDeleteIndication` are never called
and the TAL per-frame TX/RX path is still stubbed, so nothing flows over
the link. That is the next piece of work, and it is a build rather than
a debugging round. Monitor-mode RX indications are Phase 1.5.

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

### A task's results message is not its completion indication

`WABIModel.xml` describes both, near-identically named, and reading the
wrong one produces a completion the component discards without
comment. For create-port:

```xml
<message commandId="WDI_TASK_CREATE_PORT" type="WDI_TASK_CREATE_PORT_RESULTS"
         description="No TLV data needed, header is sufficient" direction="FromIhv" />

<message commandId="WDI_INDICATION_CREATE_PORT_COMPLETE" ... direction="FromIhv">
  <containerRef id="WDI_TLV_PORT_ATTRIBUTES" name="PortAttributes"
                type="PortAttributesContainer" optional="false" />
</message>
```

The first is the M2 — the task was accepted. The **second** is what
`NdisMIndicateStatusEx` carries, and its `PortAttributes` is mandatory.
Sent as a bare header, the component reads a create-port completion
naming no port and tears the adapter down.

Search for `commandId="WDI_INDICATION_<name>_COMPLETE"`, not for the
task. Of the completions this driver sends, only create-port needs a
payload; delete-port, scan, connect and change-operation-mode really
are header-only.

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

### Establish which layer is failing before testing theories inside one

`OID_WDI_TASK_CONNECT` has never reached this driver. What the traces
show instead is `wdiwifi.sys` accepting `OID_DOT11_CONNECT_REQUEST`,
returning success, and about two milliseconds later sending
`NDIS_STATUS_DOT11_CONNECTION_COMPLETION` back up with
`STATUS_NETWORK_UNREACHABLE` / `DOT11_ASSOC_STATUS_UNREACHABLE`.

Several rounds went into the BSS entry on the assumption that the port
driver was examining it and finding it wanting: the beacon frame, the
age timestamp, the algorithm pairs, the band and channel, who owns the
BSS list. Each of those found a real bug. None of them was this bug.

That reading was WRONG, and how it was wrong is the more useful lesson
-- see below. The measurement that seemed to settle it took one
command:

```
trace-wdi.cmd -Ssid foobar -Wait 45
```

Connecting to an SSID that exists nowhere fails **identically** to
connecting to the one that is on the air — same `0xC000023C`, same
`assocStatus 0x2`, same two milliseconds, no scan in between. The OID
sequences are the same OIDs in the same order with the same counts,
from the dot11 reset through `OID_DOT11_CONNECT_REQUEST`. The two cases
are indistinguishable at the driver boundary, which means the refusal
happens before the port driver considers any BSS at all.

So a whole class of theory — anything about what is *in* the BSS entry
— is ruled out, and was rulable out from the start. The lesson is not
about WDI: when a component you cannot see into rejects something,
first find a control case that isolates *which layer* is refusing.
Testing hypotheses inside a layer that turns out not to be involved
produces real fixes and no progress, and it is slow to notice because
each fix is defensible on its own.

### The control case that could not tell two answers apart

The `foobar` result above was over-read. Identical failures are
consistent with "no candidate was found" AND with "a candidate was
found and rejected", because both end the connect job instantly with
the same status. The control separated *timing*, not *cause*, and the
conclusion drawn from it -- that no BSS is ever examined -- sent several
rounds looking at port and adapter state that was fine all along.

What actually settled it was wdiwifi's own record, read out of
`CPort::m_pRoamTraceLoggingData` with `dx`:

    bssCandidateCount        : 1
    roamAPRankIndex          : 0xffffffff
    bestCandidateRank        : 0
    roamWabiReason           : WDI_ASSOC_STATUS_FAILURE
    connectJobStartTime      : equal to connectJobEndTime
    connectRoamTaskStartTime : 0

One candidate, ranked unusable, connect job over in zero elapsed time,
and the task that would have issued `OID_WDI_TASK_CONNECT` never
started. The BSS entry is examined and rejected -- the opposite of what
the control appeared to show.

A control case only rules out what its two outcomes would actually
differ on. Check that before trusting it.

Ruled out for the connect refusal, each against the generated headers
or `WABIModel.xml` rather than by inference: BSS entry completeness and
field semantics; `{OPEN, NONE}` in the advertised algorithm pairs;
band, channel and PHY; the transaction id on our indication (the header
documents `WDI_TRANSACTION_ID_UNSOLICIT` for every indication except
*task* completions, and a GET is not one); `WDI_PORT_ID_ADAPTER`
(`0xFFFF`) on the create-port request; the create-port completion's
contents; `WDI_OPERATION_MODE_STA` being `0x01`; and station and
interface capabilities field by field.

### One breakpoint hit is a sample, not a rule

Everything still open about the connect failure is a runtime fact, and
local kernel debugging (`kd -kl`) reads and disassembles but cannot stop
the machine. QEMU's `-gdb tcp::1234` can. Use `hbreak`, not `break` -- a
software breakpoint writes `0xCC` into wdiwifi's code, which is exactly
the kernel modification PatchGuard bugchecks for.

The first harness broke on `wdiwifi!CScanJob::FinishJob`, printed one
hit, and detached:

    gate 1  status        = 0x40230001
    gate 2  m_bCancelled  = 0x0
    gate 3  port id       = 0x0

`0x40230001` is `STATUS_NDIS_INDICATION_REQUIRED`, the value every task
handler here returns from its OID, and `FinishJob` writes
`WfcPortPropertyGoodScanStartTime` only when its status argument is
zero. That much is real: the OID's return value does reach that gate.

The conclusion drawn from it -- "so the property is never written, which
is why the connect is refused" -- was not. One hit cannot establish
"never": `FinishJob` runs more than once per job, and the call driven by
our `SCAN_COMPLETE` indication, the one that would carry status 0, was
not in the sample. **A single-shot breakpoint answers "does this
happen", never "is this all that happens".**

Completing the OID with `NDIS_STATUS_SUCCESS` instead settled it, and
settled it against. Same guest, same profile, one value different:

| OID return | driver's own debugcon trace |
| --- | --- |
| `INDICATION_REQUIRED` | one `GET_BSS_ENTRY_LIST` after `SCAN_COMPLETE`; `SCAN_COMPLETE` "for 3 task(s)" -- merged |
| `SUCCESS` | **ten** `GET_BSS_ENTRY_LIST` in 80 ms, all *before* `SCAN_COMPLETE`, every one answered "cache is empty"; `SCAN_COMPLETE` "for 1 task(s)", never merged |

`SUCCESS` makes wdiwifi believe the scan finished the moment the OID
completed: it polls for results that do not exist yet and stops holding
scan tasks open. Our M3 then arrives 1.6 s later for a job the port
driver has already closed. And the connect still failed -- so it fixed
nothing and broke the scan job's lifetime.

Which makes the result more interesting than a plain no. Under `SUCCESS`
gate 1 reads 0 and gates 2 and 3 had already passed, so the good-scan
property should have been written and the connect should have gone
ahead. It did not. Either the property is still not written, or it is
not the only thing the connect job is waiting on.

`scripts/gdb-wdi-connect.sh` replaces the single-shot script. It stays
attached and prints every hit at four points: `CScanJob::FinishJob`, the
store inside `CConnectJob::CheckAndUpdateCandidates` that discards the
candidate list, the compare in `CheckAndStartConnectProcess` that reads
the count, and `StartConnectRoamTask` itself. Four is also the ceiling
-- hardware breakpoints use the four x86 debug registers.
`dump-wdi-state.cmd` prints the whole command line with the addresses
filled in; they are valid for one boot, since wdiwifi is relocated on
every reboot.

Over one failed connect, with the task OIDs back on
`INDICATION_REQUIRED`, it printed the entire chain:

    [1] CScanJob::FinishJob  status=0x40230001 cancelled=0x0 portid=0x0
    [2] CheckAndUpdateCandidates+0x317  DISCARDING CANDIDATES  1 -> 0
        landed on: mov %r13d,0x26c(%rbx)
    [3] CheckAndStartConnectProcess+0x1a3  candidates=0
        landed on: cmp %r13d,0x26c(%rbx)
    [1] CScanJob::FinishJob  status=0x40230001
    [1] CScanJob::FinishJob  status=0x40230001
        [4] StartConnectRoamTask -- never reached

Both offset-based breakpoints landed on exactly the instructions the
disassembly named, so the numbers beside them can be trusted. Every scan
job in the window finished with `0x40230001` and none with `0`; one
`FinishJob` per scan completion, so there is no later call carrying the
real outcome. A candidate list of exactly **1** -- our AP, matched and
ranked -- is thrown away, and the connect task is never started.

So the job's completion status *is* the OID's, and a task's OID has to
end up completed with zero without being completed early. Only
`NDIS_STATUS_PENDING` does both: NDIS keeps the request outstanding, so
the scan job stays open and tasks still merge, and
`NdisMOidRequestComplete` supplies the final status later. That is what
`OID_WDI_TASK_SCAN` now returns. The other twelve task handlers still
return `INDICATION_REQUIRED` -- the scan job is the one that was
measured, and changing all of them at once would make the next result
unattributable again.

A pending OID is owed a completion on **every** path, not just the happy
one. Each merged requester's `NDIS_OID_REQUEST` is claimed with an
interlocked exchange so exactly one of the completion path, the teardown
path, and the late-merge recovery can complete it: completing twice is a
use after free, and never completing leaves the WLAN component blocked
on a request that can no longer be answered -- the same shape as the
disconnect-task bug that used to hang adapter removal.

The same four breakpoints then confirmed the fix, and moved the
question:

| | `INDICATION_REQUIRED` | `PENDING` + complete |
| --- | --- | --- |
| `[1]` FinishJob status | `0x40230001` ×3 | **`0x0`** ×3 |
| `[2]` candidates discarded | `1 -> 0` | never fires |
| `[3]` candidates at the decision | `0` | **`1`** |
| `[4]` StartConnectRoamTask | never | **CONNECT TASK GOING OUT** |

So the good-scan chain is closed. `WfcPortPropertyGoodScanStartTime` is
recorded, the candidate list survives, and the connect job starts its
connect task -- the first time this driver has got that far.

`OID_WDI_TASK_CONNECT` still does not arrive. The debugcon trace for the
same attempt runs `DOT11_RESET`, `SET_PRIVACY_EXEMPTION_LIST`, three
`GET_STATISTICS`, a full `TASK_SCAN`, one more `GET_BSS_ENTRY_LIST` --
and stops. `StartConnectRoamTask` is entered and gives up somewhere
between there and the OID.

That is the same position the good-scan chain was in one round earlier,
and it yields to the same pair of tools: `uf` from local KD for the
branches, and a host-side trace for which branch ran. `dump-wdi-state`
pass five writes the disassembly of the connect-task chain and prints a
`--at` command line for its entry points; `--at LABEL=ADDRESS` is
`gdb-wdi-connect.sh`'s generic breakpoint, which assumes nothing about
the function and just reports that it was entered, with `rcx`/`rdx`/`r8`/`r9`.
The last label printed is how far the chain got.

Four `--at` breakpoints is the maximum, and asking for a fifth now fails
loudly. gdb accepts it, the target silently never stops there, and the
output reads exactly like "that function was never called" -- a false
negative in the shape of a result.

That pass answered half the question:

    [StartConnectRoamTask]          rcx=job rdx=1
    [GenerateConnectTaskTlv]        rcx=job
    [FillConnectRoamTaskParameters] rcx=job
    [GenerateRoamTaskTlv]           never -- correct, this is a connect

The chain is entered, three deep, and the radio-state checks at
`StartConnectRoamTask+0x135` and `+0x164` (both of which return
`STATUS_NDIS_DOT11_POWER_STATE_INVALID`) are behind it.

### Entry points say a function ran; they cannot say what it returned

That is the whole limitation of the pass above, and the `uf` output says
where to put the return-side breakpoints instead.
`CConnectJob::GenerateConnectTaskTlv` is three calls in a row, each
tested for zero:

    +0x70   call CConnectJob::FillConnectRoamTaskParameters
    +0xc8   mov  r8,[rdi+200h]        <- reached only if it returned 0
    +0xf8   call GenerateWdiTaskConnectToIhv
    +0x12d  mov  rcx,[rdi+1F0h]       <- reached only if it returned 0
    +0x16c  call CMessageHelper::FitMessageToBufferSize
    +0x1ee  mov  rax,[rsp+50h]        <- reached only if it returned 0

Breaking on the three *landing* instructions turns "how far did it get"
into a ladder: the last rung that fires names the call that failed, with
no need to know what any of them do. That is `--tlv-stages`.

`--roam-status` is the fourth, at `StartConnectRoamTask`'s epilogue
(`mov eax,ebx`), printing the status the function is about to return --
zero, or whichever of `Task::Initialize`, `get_TaskDeviceCommand`,
`DeviceCommand::Initialize` and `CJobBase::StartTask` produced it.
`CJobBase::StartTask` is the call that actually posts the OID.

Which of the three TLV calls fails matters, because they fail on
different things. `FillConnectRoamTaskParameters` calls
`CDot11ToWabiConverter::MapAuthAlgorithm`, `MapCipherAlgorithm`,
`CConnectHelpers::GetSupportedAssociationMethods` and
`FillConnectBSSEntryTLV` -- all of which read what this driver
advertises, so a failure there is ours. `FitMessageToBufferSize` reads
adapter property `0x10` as the size limit and defaults it to `-1` when
unpopulated, so that one should be unfailable, and if it is not, the
assumption is wrong.

The assumption is wrong. The ladder ran and stopped on the third rung:

    [tlv 1/3] FillConnectRoamTaskParameters returned 0
    [tlv 2/3] GenerateWdiTaskConnectToIhv returned 0 -- msglen=368
    [tlv 3/3] never fired
    [ret]     StartConnectRoamTask returns 0xc0010015

`0xc0010015` is `NDIS_STATUS_INVALID_DATA` -- the same status this
driver's own TLV generator returns when a mandatory container is left
empty, and not a size complaint. The connect message **is** built: the
parameters are filled from our capabilities and the BSS entry, the
serialiser turns them into 368 bytes, and then
`CMessageHelper::FitMessageToBufferSize` refuses the result.

Which is the first status in this whole investigation that points back
at the message rather than at a state machine. Something in what this
driver advertises produces a connect message wdiwifi's own helper will
not accept -- so `FitMessageToBufferSize`'s code, and the name of
adapter property `0x10`, are the next two things to read.

`--tlv-limit` prints that property's value at the instruction after it
is read. `0xFFFFFFFF` there means unpopulated, the limit is infinite,
and the objection is to the message's contents rather than its length.

### A nested optional container has two presence bits

    [tlv limit] adapter property 0x10 = 0x0 (0)  msglen=368

Not `0xFFFFFFFF`. `_WFC_ADAPTER_PROPERTY_NAME` names index 16 as
`WfcAdapterPropertyMaxCommandSize`, and the default handed to
`GetPropertyULongOrDefault` is `0xFFFFFFFF`, so the property is not
missing -- it is **present and zero**. 368 bytes into a limit of 0 is
`NDIS_STATUS_INVALID_DATA`, and that is the whole failure.

This driver advertises `MaxCommandSize = 2048`. The value never
arrived, because `WABIModel.xml` nests two optional containers:

```xml
<aggregateContainer name="CommunicationAttributesContainer">
  <containerRef id="WDI_TLV_COMMUNICATION_CAPABILITIES"
                name="CommunicationCapabilities"
                type="CommunicationCapabilitiesContainer"
                optional="true" />
```

so `WDI_COMMUNICATION_ATTRIBUTES_CONTAINER` has an `Optional` bitfield
of its own, holding `CommunicationCapabilities_IsPresent`. The driver
set `params.Optional.CommunicationAttributes_IsPresent` and stopped
there, which emits the outer container with nothing inside it.

**Setting the outer presence bit does not make the inner one true.**
Every other nested container in the capabilities response gets both --
`StationAttributes.Optional.UnicastAlgorithms_IsPresent`,
`DatapathAttributes.Optional.DataPathCapabilities_IsPresent` -- and this
was the one that was missed. When adding an optional TLV, check whether
its parent is itself optional, and set every bit on the path.

Worth noting how the failure presented: a capability the driver believed
it had advertised, silently replaced by zero, surfacing four function
calls away as a status about the *message* rather than about the
capability. Nothing in any log named `MaxCommandSize`. The only thing
that found it was reading the value the consumer actually had, at the
instruction where it read it.

An absent property would have been *safer* than a half-filled one: the
default is "no limit". Half-filling a container is worse than omitting
it.

With the inner bit set, `OID_WDI_TASK_CONNECT` arrives for the first
time in this project's life:

    OID: method 0xe4400006 WDI_TASK_CONNECT
    OID M1: txn 20 wdiport 0x0000 ndisport 0 ... in 336 out 2046 bytes
    connect parsed: bssid 02:11:22:33:44:01 ssid='vwifi-open'
                    auth=0 akm=0 cipher=0 assoc_ies=0

### The same trap, one message later: ActivePhyTypeList

The connect ran, the device associated, and then:

    ASSOCIATION_RESULT TLV generate failed
    indicating CONNECT_COMPLETE (0xc0000001)

`WDI_ASSOCIATION_RESULT_CONTAINER` has five optional members with
presence bits and two mandatory ones -- and a third that is mandatory
only sometimes. `WABIModel.xml` lists it twice:

```xml
<containerRef id="WDI_TLV_PHY_TYPE_LIST" name="ActivePhyTypeList"
              type="PhyTypeListContainer" versionAdded="WDI_VERSION_1_1_4" />
<containerRef id="WDI_TLV_PHY_TYPE_LIST" name="ActivePhyTypeList"
              type="PhyTypeListContainer" optional="true"
              versionRemoved="WDI_VERSION_1_1_4" />
```

Optional up to 1.1.4, **required from 1.1.4 on**. This peer reports
`0x0001010a` -- 1.1.10 -- so it is required, and the driver left it as
a zeroed `ArrayOfElements`. Same failure as the empty `FirmwareVersion`
that once stopped adapter bring-up: the generator rejects the whole
message with `NDIS_STATUS_INVALID_DATA`.

**A `versionAdded` attribute can make a field mandatory that the
same file also describes as optional.** Read both rows, and check the
peer version before deciding which applies.

Three neighbouring fields were also wrong in a way worth stating,
because the values look plausible and are not: `AuthAlgorithm`,
`BandID` and `DSInfo` were all left at zero, and zero is not a valid
enumerator for any of them --- `WDI_AUTH_ALGO_80211_OPEN` is 1,
`WDI_BAND_ID_2400` is 1, and `WDI_DS_INFO` runs `CHANGED` 1,
`UNCHANGED` 2, `UNKNOWN` 3 with no zero at all. Only
`WDI_CIPHER_ALGO_NONE` is genuinely 0. A zeroed struct is a valid
starting point only for the fields whose zero means something.

With that, the connect completes:

    OID: method 0xe4400006 WDI_TASK_CONNECT
    connect parsed: bssid 02:11:22:33:44:01 ssid='vwifi-open'
    indicating ASSOCIATION_RESULT status=0 aid=1 ies=25
    IND: 0x4005004c wdiport 0x0000 ndisport 0 txn 0 status 0x0, 119 bytes
    indicating CONNECT_COMPLETE (0x0)
    IND: LINK_STATE connected on ndisport 0
    alive: beat 3, scan 0, conn 0x0, assoc 1, port 1, opmode 1

and holds, across heartbeats, with no disconnect.

### What is left, and one thing left deliberately unfixed

The association carries no traffic yet. Peers now exist -- see below --
but the TAL per-frame TX/RX path is still stubbed, so the link is up and
empty.

### Nothing can be sent to a port. Only to a peer

WDI does not address data by port. It addresses it by **(port, peer,
TID)**: `MiniportWdiTxDataSend` takes all three, RX indications carry
the peer id, and `WDI_TX_METADATA` names the peer on every frame. Until
the miniport says a peer exists, the component holds its TX queues
paused, and `dot11wdi.h` names the reason:

```c
WDI_TX_PAUSE_REASON_PEER_CREATE = 0x00000002
```

Which explains something that had been sitting in every trace unread:
`TxDataSend` had *never fired*, not once, in any log this project has
collected. That looked like "the component has nothing to send". It was
"the component has nobody to send it to".

A station has exactly one peer -- the AP -- created at association and
deleted at disconnect. The id is the **miniport's** to assign:
`NdisWdiPeerCreateIndication` takes `WDI_PEER_ID` as an input and the
component uses it verbatim afterwards. Its only constraint is
`MaxNumPeers` from `TalTxRxStart` (eight here), which the component
sizes its own per-peer state from.

Create is synchronous and delete is not. `PeerCreateIndication` returns
`void` with no status: once it returns, the peer exists.
`PeerDeleteIndication` has an `_Out_ NDIS_STATUS` *and* a matching
`TalTxRxPeerDeleteConfirmHandler` callback, so a slot is not reusable
until the confirmation arrives. Read the return type before assuming a
handshake has one half.

The create runs **before** `CONNECT_COMPLETE`, so the component has
somewhere to send before it is told it may; the delete runs **before**
the disassociation, for the mirror-image reason.

The peer indication itself works. The proof is not the driver's own log
line -- that only says we made the call -- but what the component did
differently afterwards:

    TAL TxAbort: port 0 peer 65535     <- before, always
    TAL TxAbort: port 0 peer 0         <- after the peer was created

`65535` is `WDI_PEER_ANY`. The component started naming the peer, so it
had accepted it. **Look for the change in what the other side does, not
for your own trace saying you did it.**

### An open network's port is authorized the moment it associates

Peers existed and still nothing flowed: no `TalTxRxPeerConfigHandler`,
no `TxDataSend`, on an open network with a link the OS believed was up.

`WDI_ASSOCIATION_RESULT_PARAMETERS.PortAuthorized` was hard-coded
`FALSE`. WABIModel: *"Specifies whether port authorization has been
performed"* -- the dot11 controlled-port flag. An unauthorized port
passes EAPOL and nothing else.

On a network with an AKM that is correct at association time: the
four-way handshake runs afterwards over EAPOL and authorizes the port
when it completes. On an **open** network there is no handshake to wait
for and nothing that will ever come along to flip the bit, so `FALSE`
leaves the port permanently shut. The AKM is the discriminator, not the
cipher and not the auth algorithm -- WPA2-PSK authenticates over the air
as Open System, so auth alone cannot tell them apart.

### In WDI, NDIS is not the data path

`MiniportSendNetBufferLists` and `NdisMIndicateReceiveNetBufferLists`
are the whole data path for an ordinary NDIS miniport. For station
traffic under WDI they are neither end of it. `wdiwifi.sys` owns both
directions: NDIS hands sends to **it**, its TxMgr queues them per
(port, peer, TID), and the miniport is expected to come and take them
through `NdisWdiTxDequeueIndication`. Receives go the other way, into
its RxMgr via `NdisWdiRxInorderDataIndication`.

So an implemented `VwifiTxDataFrame`, a working RX ring drain, a
created peer and a connected link still moved nothing — the frames were
never offered to one and the other put them where nobody was listening.
`wdi_data.c` is the plumbing between the component and the rings the
driver already had.

Two shapes worth knowing, because they are not symmetrical:

- **TX is a pull.** `TxDataSend` and `TxPeerBacklog` are only
  *notifications* that frames are waiting. Nothing arrives with them.
- **RX is a two-step push.** `RxInorderDataIndication` says frames exist
  for a (peer, TID); the component decides when to take them and calls
  back through `RxGetMpdusHandler`. They have to be parked in between.

### Changing five things at once costs a round, every time

The build that landed the data path also raised `MaxOutstandingTransfers`
from 1 to 64, on the reading that 1 was throttling TX — the previous
trace had shown `TxTargetDescInit`, then `TxPeerBacklog(backlogged=1)`,
then nothing, which looks like a component that prepared a frame and
found the miniport full.

That build regressed: `TalTxRxPeerConfigHandler`, which had been
arriving in the same millisecond as the connect completion, stopped
arriving at all — and with it the receive-filter widening, the multicast
updates and `TxTargetDescInit`. An associated link that Windows would no
longer build on.

The log isolated it without another round, but only by luck. Five things
changed and **four of them provably did not execute**: no TX
notification arrived, so the pump never ran; no data frame was received,
so the RX routing never ran and the queue was never non-empty; the fifth
was a log line. `MaxOutstandingTransfers` is set in `TalTxRxStart`, at
adapter init, long before a peer exists — the only change on the path
that mattered.

So it is back to 1. Not because 1 is known right, but because 64 is the
only candidate the evidence leaves. And 1 may not be a throttle at all
now: a transfer here is a memcpy into a ring slot and completes before
the handler returns, so the component can never find more than one
outstanding, and `TxPeerBacklog` drives the pump directly.

**Land one variable at a time, or be able to prove which one ran.**

### Creating a peer pauses its TX, and only the miniport can unpause it

With the port authorized, everything the WLAN service does after a
connect finally happened — `TalTxRxPeerConfigHandler`, the receive
filter widening to `0x0004000b`, the IPv6 multicast groups,
`TxTargetDescInit` — and hostapd confirmed the association from the
other side:

    hostapd: STA 52:54:00:8b:c7:82 IEEE 802.11: associated (aid 1)
    hostapd: AP-STA-CONNECTED 52:54:00:8b:c7:82 auth_alg=open

And still not a single frame. `TxDataSend` never fired; the only thing
that arrived was `TxPeerBacklog: port 0 peer 0 backlogged 1` — the
component holding frames it would not hand over.

`dot11wdi.h` names the reason and nothing else in the API can clear it:

```c
WDI_TX_PAUSE_REASON_PEER_CREATE = 0x00000002
```

Pause and restart are the **target's** to declare — both are
`NDIS_WDI_TX_SEND_*_IND`, miniport to component. So creating a peer
pauses its TX, and it stays paused until the miniport says the target is
ready for it. This driver created peers and never restarted them.

The restart goes in the *peer-config* callback rather than straight
after `PeerCreateIndication`: config is the component saying it has
finished setting the peer up, and declaring a peer ready that the
component has not finished with is answering a question it has not asked
yet.

### Two different pulls, and the header does not say which

`TxDequeueIndication` takes no peer. `TxReleaseFrameIndication` takes a
port, a peer and a TID bitmask. They are different questions — "what
should the target send next", scheduled by the component across
everything it holds, versus "release what is queued for *this* peer" —
and a backlog notification is about one peer.

Rather than guess, the pump asks the general one first and falls back to
the specific one, and **logs which produced the frames**. That is not
hedging: it is one round instead of two, and the answer ends up in the
trace instead of in an argument.

The same pass fixed a hole in that loop's tracing. It logged only when
it sent something, so a pump that dequeued nothing looked exactly like a
pump that was never called — which is precisely the state that had to be
diagnosed. **A loop that only logs its successes cannot tell you it ran.**

### Two clocks are worth more than two theories

`ASSOCIATION_RESULT status=16` came back twice, intermittently, from
builds that had connected minutes earlier. Two rounds went into reading
it from inside the guest — was the AP down, was the medium dropping
beacons, was a stale association confusing hostapd — and none of them
could have worked, because everything that mattered had already happened
somewhere the guest cannot see.

Lining the driver's trace up against hostapd's settled it in one pass:

```
driver   [84130.577]  WDI_TASK_CONNECT
driver   [84131.577]  ASSOCIATION_RESULT status=16 (AUTH_TIMEOUT)   <- 1.000 s

hostapd  18:27:41     authenticated
hostapd  18:27:43     associated (aid 1)                            <- 2 s
hostapd  18:27:43     AP-STA-CONNECTED
```

**The AP answered and the association succeeded.** The vwifi device's
`VWIFI_CONN_TIMEOUT_MS` was 1000, armed per stage, and it gave up while
hostapd was still working through an exchange that took about two
seconds. The station reported failure for a connection the AP considered
up — the guest saw "the AP never answered" and the AP saw an associated
station.

That is also why it looked intermittent across identical driver builds:
it is a race against however long the AP happens to take, and hostapd is
not always slow. **An intermittent failure with a suspiciously round
interval is a timeout, and the thing that timed out is rarely the thing
reporting it.**

Raised to 5000. A virtual medium carrying frames through a userspace
controller to another VM has no business being held to on-air timings;
that timer exists to fail eventually, not quickly. The trace now names
which stage expired, too — no Auth Response means the AP never engaged,
no Assoc Response means it did and then went quiet, and "timeout in
state 1" required the enum to hand.

### The contract that is not in the header, and how to act on one anyway

Finishing the per-frame path needs one fact that `dot11wdi.h` does not
state. A TX NBL arrives from `NdisWdiTxDequeueIndication` carrying a
`WDI_FRAME_METADATA` -- the struct is defined, it has a `pNBL`
back-pointer, and it holds the `WDI_FRAME_ID` that
`NdisWdiTxSendCompleteIndication` is fed. Nothing says **where on the
NBL to find it**.

The convention in IHV samples is `MiniportReserved[0]`. But
`MiniportReserved` is by definition the *miniport's* scratch space,
which argues the port driver would not keep its own bookkeeping there.
That is an argument, not a fact, and arguments about undocumented
contracts are what cost this project the `INDICATION_REQUIRED` build and
the `MaxCommandSize` round before it.

`wdiwifi.sys` knows: it is both the code that attaches the metadata and
the code that reads it back at send-complete, so the offset is a
constant in its disassembly. The symbols are now named:

```
wdiwifi!AdapterTxDequeueInd        the NDIS_WDI_DATA_API thunk
wdiwifi!CTxMgr::TxDequeueInd       where the NBL chain is built
wdiwifi!CTxMgr::TxSendCompleteInd  where FrameIDs are resolved back
wdiwifi!AdapterAllocateWifiFrameMetadata
```

and `dt` gives the layout, which is what makes acting on the answer safe
rather than another guess:

```
+0x000 Linkage : _LIST_ENTRY
+0x010 pNBL    : Ptr64 _NET_BUFFER_LIST
+0x018 FrameID : Uint2B
+0x020 u       : tx/rx union
```

A candidate pointer can be *checked* before it is trusted: read it,
follow `+0x10`, and see whether it points back at the NBL it came from.

And the **RX direction settles the argument**. The miniport creates
those NBLs, so it must attach the metadata somewhere the port driver
will look — and the only field of a `NET_BUFFER_LIST` a miniport is
permitted to write is `MiniportReserved`. There is nowhere else it
could be, and TX is the same contract read the other way.

That is still an argument, and this driver has been wrong about
arguments twice. So it is not trusted, it is **checked at runtime**:
`VwifiTxMetadata` confirms the candidate is a plausible kernel address,
confirms it is readable, and confirms that following its `pNBL` leads
back to the NBL it came from. A metadata that fails is not used, and the
frame is *still sent and still completed* — the transfer completion
takes the NBL, not the metadata — so a wrong guess degrades to "no
send-completion, said out loud in the log" rather than to a bugcheck.

**When a contract cannot be read, make the code prove it at runtime and
degrade safely when the proof fails.** That is available far more often
than it looks: `pNBL` exists precisely so the link can be verified from
either end.

### The line you leave in the log because it looks harmless

This one was in the log for weeks:

    device rejected SCAN: 0xc0000001 (a connect is in flight; the
    device will not sweep while associating)

wdiwifi scans as part of its own connect flow and the device refuses to
sweep while associating — it shares the radio and the one-shot timer
with the auth/assoc exchange. It looked harmless: the connect ran all
the way to a successful association result with two of these in the
middle of it. The note here said so, and added that the rejection
returns a failure status *synchronously*, that a task OID's return
value is what `CScanJob::FinishJob` reads as the job's outcome, and
that if connects ever started failing every other attempt this was the
first place to look.

Connects did not start failing. Something better disguised did.

The symptom was a link that came up perfectly — association result 0,
`LINK_STATE connected`, the AP logging `AP-STA-CONNECTED` — and then
sat at "Unidentified network" because DHCP had nowhere to go.
`TalTxRxPeerConfig` never arrived, so the peer was never configured, so
TX stayed paused on `WDI_TX_PAUSE_REASON_PEER_CREATE` for the life of
the association.

What made it findable was that it was **intermittent**, and the thing
it correlated with was not obvious. Across five captured runs:

| connect → association | scan during connect | `TalTxRxPeerConfig` |
| --------------------- | ------------------- | ------------------- |
| 0 ms                  | accepted            | **arrived**         |
| 0 ms                  | accepted            | **arrived**         |
| 62 ms                 | refused ×2          | never               |
| 79 ms                 | refused ×2          | never               |
| 3437 ms               | refused ×2          | never               |

Five for five, and nothing else differed. When the association happened
to finish before wdiwifi got its scan in, the device had already
returned to idle, the scan was accepted, and the peer was configured.
When it did not, the scan was refused, the scan *job* failed, and the
post-connect sequence that ends in `TalTxRxPeerConfig` was aborted
along with it.

So the failing task did not break the connect. It broke everything
downstream of the connect, while leaving the connect itself looking
perfect — which is why it survived so long as a line worth ignoring.

The fix is not to make the device sweep during association; that
constraint is real. It is to stop reporting "I could not do this right
now" as "this failed".

The obvious correction — answer it immediately as a scan that completed
and found what we already knew — was tried, and it failed *worse*.

Finishing the scan job synchronously is what drives
`CheckAndUpdateCandidates`, and a refreshed candidate list starts a
**new connect task while the first one is still running**. One connect
at a time, so the second is refused with
`NDIS_STATUS_REQUEST_ABORTED` — and that is the job wlansvc is
watching. The device associated anyway and the link came up, so the
driver log read as a clean success while the OS put up "Can't connect
to this network". Two lines that had never appeared in any earlier
trace showed up together:

    OID: method 0xe4400006 WDI_TASK_CONNECT     <- 0 ms after the first
    connect task already active

So both obvious answers are wrong, and they are wrong in opposite
directions:

| refusal answered as | what breaks |
| ------------------- | ----------- |
| failure             | scan job fails → post-connect sequence aborts → no peer config |
| immediate success   | scan job finishes → second connect task → refused → job fails |

What actually works was in the traces the whole time. In the two runs
that reached `TalTxRxPeerConfig`, the device *accepted* the scan, swept
for ~1.6 s, and its `SCAN_COMPLETE` landed **after** the connect had
completed. The ordering is the entire difference — and it was decided
by whether the AP happened to answer inside one timer tick.

So the refusal is now **deferred**: the scan task is held, Active with
no sweep behind it, and drained from `VwifiIndicateConnectComplete` on
every outcome. A second refused scan merges into it through the
ordinary merge path and gets its own `SCAN_COMPLETE` from the same
drain. That reproduces the known-good ordering deliberately instead of
leaving it to timing.

**A task OID's return value is not a status, it is a verdict on the
job.** "Busy" and "failed" are the same value to the caller, and the
caller acts on the verdict, not on your reason for it. Anywhere a
driver returns a failure for something it merely could not do *yet*,
the cost lands somewhere downstream and looks nothing like a scan.

**And "not now" has a third answer besides yes and no: later.** Both
synchronous answers were wrong here because both ended the job at a
moment the rest of the system was not ready for. The question a task
completion answers is not only *what* happened but *when* the caller
is told — and when a fast path and a slow path disagree about the
outcome, the ordering is usually the variable, not the result.

And the meta-lesson, which is the expensive one: **a known-suspicious
line you decided not to act on needs a prediction attached that could
be wrong in more than one way.** The prediction here was "connects will
fail every other attempt". Connects did not fail, so the note read as
disproved every time it was reviewed — while the actual damage went to
a subsystem the note never mentioned.

### The data path moved, and two of its own log lines were lying

Deferring the scan got the connect all the way through, and the TX path
came alive on its first try:

    TAL TxTargetDescInit (first call)
    TAL TxPeerBacklog: port 0 peer 0 backlogged 1
    TAL tx: sent 1 frame(s) via release, 1 with a frame id

Two things were settled by that one line. **`via release`** — so
`TxReleaseFrameIndication`, the per-peer pull, is what produces frames;
`TxDequeueIndication` returned nothing every time. And **`1 with a
frame id`** — so the `MiniportReserved[0]` metadata contract, the one
that could only ever be argued for and never read, is right. The
runtime round-trip check passed on a real frame.

DHCP still failed, and reading the same log harder turned up two places
where the trace was flattering itself.

**"sent" was counting attempts.** `nFrames++` ran once per NBL handled,
before anything checked what `VwifiTalTxOneNbl` returned. A frame
refused by `VwifiTxDataFrame` — not associated, bad length, ring full —
incremented it exactly like a frame that went out. The line now reads
`N frame(s) via release, N accepted by the device, N refused`. (In this
particular trace the frames really were accepted: the per-frame warning
inside `VwifiTalTxOneNbl` fires on every failure and none appeared. But
that was luck, not evidence — the summary line never checked.)

**Eight of everything, then nothing.** The TX log stopped after eight
sends while `TxPeerBacklog` kept firing 67 times. That reads exactly
like a stalled pump, and it was `VWIFI_TAL_FIRST(8, …)` — the rate
limiter put there to keep the trace readable. A rate-limited line and a
line that stopped happening are indistinguishable in the output, which
makes the limiter a trap laid for the person reading it later.

**A log line's job is to be falsifiable.** "sent" that counts attempts,
and a counter that stops printing without saying so, both survive
review because they read as true — the way to catch them is to ask what
each line would look like if the thing it describes had failed, and
make sure that looks different.

### Two bugs that only a hex dump and a counter could tell apart

TX looked healthy for several rounds — frames pulled, accepted by the
ring, no failures — while RX stayed at *exactly zero frames*. That
combination has an obvious reading (nothing is coming back from the AP)
and it was right, but not for the reason it looked like.

**The dump settled it in one line:**

    TAL tx: first frame 360 bytes: 08 01 00 80 02 11 | 22 33 44 01
    52 54 | 00 8b | c7 82 ff ff ff ff ff ff -- NOT 802.3

Frame control `08 01` is a data frame with ToDS set; addr1 is the
BSSID, addr2 our station MAC, addr3 broadcast; then LLC/SNAP and a
328-byte DHCP DISCOVER. 24 + 8 + 328 = 360. **wdiwifi hands the
miniport complete MPDUs** — it owns the 802.11 MAC state, so frame
control, duration, addresses and sequence number are set before the
miniport ever sees the frame.

The device assumed 802.3 and built a second header on top, producing a
frame whose "destination MAC" was the first six bytes of the original
802.11 header. It went out, the AP dropped it, nothing reported an
error. Fixed with `VWIFI_TX_F_80211` — deliberately not
`VWIFI_TX_F_INJECT`, which also skips the association check and the
cipher.

**Then the AP answered, and the second bug appeared:**

    rx(sta): descriptor 0: 342 bytes flags=0x0000 freq=2462 rssi=-30
    rx(sta): NBL alloc failed

342 bytes is the DHCP OFFER. It arrived, and the driver could not build
an NBL for it — nor for any frame after it. The pool was created with
`ContextSize = 0` while both RX paths asked
`NdisAllocateNetBufferAndNetBufferList` for `sizeof(VWIFI_RX_NBL_CONTEXT)`
bytes of context. That is **four** bytes, and an NBL context must be a
multiple of `MEMORY_ALLOCATION_ALIGNMENT` — sixteen on x64. Every
allocation returned NULL. Fixed by sizing the *pool's* context once,
aligned, and asking for none per allocation.

**Two failures in series look exactly like one failure.** Before the
format fix there was nothing to receive, so a broken receive path was
indistinguishable from a working one with no traffic. Neither bug could
have been found while the other was live — which is the argument for
diagnostics that make a path *say what it did* rather than for reading
harder.

**And the shape of the second one is worth keeping:** it was not a
resource shortage, a race, or a lifetime bug. It was a size argument
that had to be a multiple of sixteen and was four, failing identically
every time since the day it was written. The trace said "alloc failed",
which reads as memory pressure and is the one thing it never was.

### The OWN bit is not "is there a frame here"

Fixing the NBL context made RX work — `RxGetMpdus` and `RxReturnFrames`
balanced, frames going up, the AP answering. The same build hung the
guest hard enough to need a reset, with the 0xE9 log stopping
mid-flight.

A hard hang with the debug console silent is a short list: almost
always an unbounded loop or a deadlock at `DISPATCH_LEVEL`, because
neither ever returns to anything that could log. And the only newly
live code was everything downstream of an RX NBL that now allocates.

Both RX drains decide "is there a frame in this slot?" from the
descriptor's `VWIFI_DESC_F_OWN` bit alone:

```c
if (d->flags & VWIFI_DESC_F_OWN) break;   /* device still owns it */
```

That is sound only while every consumed slot is re-armed before the
consumer index laps it. It is not. A slot deliberately stays un-armed
until its NBL comes back through `VwifiMiniportReturnNetBufferLists` —
that is the whole point of the design, since the NBL's MDL points into
the slot's DMA buffer — and the return can be arbitrarily later.

Let enough frames arrive before the returns catch up, and the consumer
index wraps onto a slot it already consumed: `OWN` clear, `frame_len`
still set from last time. Indistinguishable from a fresh frame. It gets
consumed again, and so does the next, all the way round a ring in which
nothing is armed — an unbounded loop allocating NBLs and MDLs at
`DISPATCH_LEVEL`.

It could not happen while the NBL allocation was failing, because
nothing was ever outstanding. **The alignment bug was hiding it.**

Two stops now, deliberately redundant:

- `RxOutstanding` counts slots held by an NBL, and the drain stops one
  short of the ring, dropping frames and re-arming their slots instead
  — safe precisely because no NBL has taken those. This keeps the lap
  unreachable.
- A `guard` counter bounds each pass to one lap regardless, and says so
  loudly. This makes the loop terminate even if the reasoning above is
  wrong.

**A "can't happen" that depends on a rate is a race, not an invariant.**
The lap needed 256 frames to arrive before their returns landed; at
DHCP-burst rates that is a fraction of a second, and it had simply
never been reachable before because the receive path had never once
succeeded.

**And once again the rate limiter lied.** `rx(sta): descriptor N` is
`VWIFI_TAL_FIRST(8, …)`, so the log falling silent after eight frames
says nothing whatever about whether RX stopped — which is exactly the
trap written up two sections above, walked into again one build later.
When a trace goes quiet, check what is rate-limited *before* concluding
anything went quiet.

### When the driver log is silent, the problem is above the driver

The 0xE9 trace records everything the driver is asked to do. That makes
its silence informative in its own right: an operation that produces no
driver output never reached the driver.

Three that have behaved this way:

- clicking Connect, which sits at "Checking network requirements"
- disabling the network interface, which succeeds
- uninstalling the driver from Device Manager, which hangs

None of them logged a line. So the WLAN service and the Microsoft WDI
component are doing something — or waiting for something — before they
issue an OID, and no amount of driver-side logging will show it. That
layer has its own logs:

**Event Viewer** — Applications and Services Logs → Microsoft →
Windows → **WLAN-AutoConfig → Operational**. Connect attempts appear
here with a reason for each failure. This is the first place to look
when a connect never reaches the driver.

**A full WLAN report**, which is the same data rendered readably:

```
netsh wlan show wlanreport
```
writes `C:\ProgramData\Microsoft\Windows\WlanReport\wlan-report-latest.html`.

**What the service currently believes**, which is worth comparing
against what the driver logged it indicated:

```
netsh wlan show interfaces
netsh wlan show networks mode=bssid
```

If `show networks` lists the SSID with a sensible signal percentage,
the BSS entries are getting through intact. If it shows the network
with 0% signal, or not at all, the fault is in what
`VwifiTlvGenerateBssEntryList` reports rather than in the connect path.

**An ETW trace of the component itself**, when the above is not
specific enough:

```
netsh trace start scenario=wlan tracefile=C:\wlan.etl
  ... reproduce ...
netsh trace stop
```

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

## The 802.3 audit

Three separate bugs in this driver had the same shape: a piece of code
that was correct while the device did the 802.3 ↔ 802.11 conversion,
and stayed in place after `VWIFI_CTRL_RX_80211` and `VWIFI_TX_F_80211`
moved that conversion above the miniport. Each was found the same way
— a frame going missing with nothing reporting an error — and fixed
one at a time. This is the sweep for the rest of them.

**What was found and fixed**

| Site | The assumption | Consequence |
|---|---|---|
| `data.c`, `VwifiTxDataFrame` | `FrameLen < 14` — the 802.3 minimum — on a path that now carries MPDUs | A 14-to-23-byte frame passed the check and reached a device that would read an `addr3` that was never sent. Now `24` when `VWIFI_TX_F_80211` is set. |
| `driver.c`, `MiniportSendNetBufferLists` STA branch | Passed flags of `0`, i.e. "this is 802.3", under a comment saying the device would encapsulate it | Dormant — wdiwifi owns the send path for station data and this handler is never called for it. Live and wrong all the same. Now passes `VWIFI_TX_F_80211` and logs once if it ever fires, because it firing at all would mean something about the send path is not what this driver believes. |
| `wdi_data.c`, TX shape dump | Tested bytes 12-13 for an EtherType *first*, and only fell through to the 802.11 test if that failed | Bytes 12-13 of an MPDU are the middle of `addr1`. The dump could print "802.3: the device's assumption holds" about a frame that was nothing of the kind. Now decoded as 802.11 first, with the EtherType read from LLC/SNAP at a computed offset. |
| `abi/vwifi_abi.h`, `VWIFI_RX_F_RAW` | Documented as "monitor-mode capture" | It means "the payload is 802.11, not 802.3", and since `VWIFI_CTRL_RX_80211` it is set on ordinary station traffic too. The Linux driver still routes on it as though it meant monitor mode, which is safe only because that driver never asks for MPDUs — noted in place. |

**The actual root**

None of the above is the root. The root is that the frame-format
contract was *written down in five places* — the file headers of
`data.c` and `wdi_data.c`, the RX section header, the ABI flag
comments, and this README — and when the contract changed, three of
those copies did not. Every one of the bugs above was written or left
standing by someone reading a stale copy, and at least two debugging
sessions were spent reasoning from a comment instead of from the wire.

So there is now exactly one authoritative statement of it, next to the
`VwifiTxDataFrame` declaration in `vwifi_drv.h`, and the other sites
point at it rather than restating it. A contract that exists once
cannot disagree with itself.

## Status

The phase-by-phase plan in `docs/vwifi-virt-development-plan.md`
describes the intended order of work. In this tree the source covers
Phase 1 (bring-up), 1.5 (monitor), 2 (scan), 3 (connect) and 4 (keys) —
`wdi_scan.c`, `wdi_connect.c`, `wdi_keys.c` and `monitor.c` are all
implemented. It builds against the WDK and runs in a Windows 10 (19041)
guest: it scans, associates on an open network, and gets a DHCP lease.
WPA2 associates but the 4-way handshake does not complete.

(This paragraph used to say no build and no guest run had happened,
long after both had. It is corrected here for the same reason the
802.3 audit above exists — a stale statement of fact in a document
people reason from is not a harmless stale statement.)

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

- The WPA2 4-way handshake. Association succeeds and EAPOL M1 is
  indicated and accepted, but nothing emerges above the 802.11-to-802.3
  converter and the supplicant never reports a result.

  One cause of this is now identified and fixed, and it was not in the
  receive path at all. Anchoring a medium capture to the driver log —
  three points, association, probe response and deauthentication, all
  agreeing to within 15 ms — shows what actually happened:

  | driver | medium | |
  |---|---|---|
  | 327.99 | 13.54 | scan starts sweeping 13 channels at 100 ms each |
  | 328.05 | 13.60 | AP transmits EAPOL M1 (replay counter 1) — **station is off channel, frame lost** |
  | 329.05 | 14.60 | AP retries M1 (replay counter 2) — lands in the one moment the sweep is back on channel 11 |
  | 329.60 | 15.15 | station deauthenticates |

  wlansvc issued `OID_WDI_TASK_SCAN` 0 ms after `OID_WDI_TASK_CONNECT`.
  The device accepted it, because its only rule was "don't sweep during
  an auth/assoc exchange" and the association was already over. The
  4-way handshake is the same kind of exchange and was not covered.
  The single M1 that ever reached the driver was the retry, and it
  reached it *during* the sweep — so the converter that dropped it was
  being handed a frame while the OS believed the radio was elsewhere.

  Fixed by making the device refuse a scan while associated on a secure
  BSS with no pairwise key yet, and by holding the driver's deferred
  scan past `CONNECT_COMPLETE` until the key arrives. Whether that is
  also why the converter dropped the retry is not established.

  Worth recording as a method note: this was invisible in the driver log
  alone, which showed one EAPOL arriving and being indicated
  successfully. It took correlating two clocks to see that the
  interesting frame was the one that never appeared.
- Airplane mode. `OID_WDI_TASK_SET_RADIO_STATE` is accepted and not
  acted on, so the radio comes back on about 30 seconds after it is
  turned off.
- TX power reporting (`OID_WDI_TASK_SET_RADIO_STATE` and friends) has
  no counterpart to the Linux driver's `get_tx_power`/`set_tx_power`.
  Nothing has asked for it yet.
- The scan request ignores WDI's `BandChannelList`; it scans everything
  the device supports and lets the OS filter. Legal, just slower.
