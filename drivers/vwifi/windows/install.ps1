<#
.SYNOPSIS
    Install the vwifi driver in the Windows guest, removing every older
    copy first.

.DESCRIPTION
    Run elevated, from the folder holding vwifi.sys / vwifi.inf /
    vwifi.cat (that is x64\Debug\vwifi\ as assembled by sign.cmd).

    THE WAY TO USE THIS: boot the guest with the vwifi-virt device OFF
    the QEMU command line, run this, shut down, put the device back.

        -chardev socket,id=medium,...           <- drop for the install
        -device vwifi-virt,chardev=medium,...   <- drop for the install

    With no device present there is nothing for a removal to stop, so
    every step below completes instantly. With the device present and a
    driver bound to it, removal has to stop a device the WLAN service is
    holding, and that has been observed to hang hard enough that the VM
    had to be forced off -- so this refuses to try unless you pass
    -Force.

    Why removal is not optional: an earlier version of this script only
    staged the package, on the theory that PnP ranks packages by
    DriverVer and build.cmd makes every build outrank the last. Ranking
    is real, but it only runs when a device is INSTALLED. A devnode that
    already has a driver keeps it -- the binding is recorded in the
    device's own Driver key and survives reboots -- so a newer package
    sitting in the store is never consulted. The visible symptom is an
    install that reports success and a debug log that still shows the
    old build's version. Removing the old package is what forces PnP to
    choose again.

.PARAMETER Path
    Folder holding the driver package. Defaults to the script's own
    folder, then to the build output under it.

.PARAMETER Force
    Remove the old packages even though the device is present and
    started. This is the operation that hangs. Prefer booting without
    the device.

.PARAMETER KeepOld
    Skip removal and only stage the new package. The old behaviour, kept
    for the case where the device has no driver bound at all and there
    is genuinely nothing to replace.
#>
[CmdletBinding()]
param(
    [string] $Path,
    [switch] $Force,
    [switch] $KeepOld
)

$ErrorActionPreference = 'Stop'

$HwId       = 'PCI\VEN_1AF4&DEV_0E00'
$InstalledSys = Join-Path $env:SystemRoot 'System32\drivers\vwifi.sys'

Write-Host 'vwifi driver install'

# Resolve the package folder.
#
# Not `param([string] $Path = $PSScriptRoot)`: that default came back
# empty under `powershell -File`, and the first thing to touch it was a
# Join-Path, which reports it as its own parameter binding error and
# names no useful cause.
if (-not $Path) { $Path = $PSScriptRoot }
if (-not $Path) { $Path = Split-Path -Parent $MyInvocation.MyCommand.Path }
if (-not $Path) { $Path = (Get-Location).Path }

# This script lives beside the sources as well as inside the package
# sign.cmd assembles, so running it from the repo folder on the build
# machine is an easy mistake. Look in the build output before giving up.
if (-not (Test-Path (Join-Path $Path 'vwifi.inf'))) {
    foreach ($sub in @('x64\Debug\vwifi', 'x64\Release\vwifi')) {
        $alt = Join-Path $Path $sub
        if (Test-Path (Join-Path $alt 'vwifi.inf')) {
            Write-Host "No driver package in $Path; using $alt"
            $Path = $alt
            break
        }
    }
}

$isAdmin = ([Security.Principal.WindowsPrincipal] `
    [Security.Principal.WindowsIdentity]::GetCurrent()
    ).IsInRole([Security.Principal.WindowsBuiltInRole]::Administrator)
if (-not $isAdmin) {
    Write-Host ''
    Write-Host 'ERROR: not elevated. pnputil needs administrator rights.'
    Write-Host '  Open an admin command prompt and re-run install.cmd there.'
    exit 1
}

$inf = Join-Path $Path 'vwifi.inf'
$sys = Join-Path $Path 'vwifi.sys'
$cat = Join-Path $Path 'vwifi.cat'

if (-not (Test-Path $inf)) { throw "vwifi.inf not found in $Path" }
if (-not (Test-Path $sys)) { throw "vwifi.sys not found in $Path" }
if (-not (Test-Path $cat)) {
    Write-Warning 'No vwifi.cat -- run sign.cmd. Without it the package is unsigned.'
}

# The DriverVer actually being installed, so a stale package is obvious.
$driverVer = (Select-String -Path $inf -Pattern '^\s*DriverVer\s*=' |
              Select-Object -First 1).Line
if ($driverVer) { Write-Host "Installing: $($driverVer.Trim())" }
Write-Host "Package SHA256: $((Get-FileHash $sys -Algorithm SHA256).Hash)"

# --- helpers ----------------------------------------------------------

# Every vwifi package currently in the driver store, as its published
# oemNN.inf name.
#
# Parsed out of `pnputil /enum-drivers` by splitting the output into
# blank-line-separated blocks and keeping the ones that mention
# vwifi.inf. Deliberately not matching on field labels: those are
# localised and this has to work on whatever guest is to hand. The
# values -- oem21.inf, vwifi.inf -- are not.
function Get-VwifiStorePackages {
    $text = (& pnputil /enum-drivers | Out-String)
    $out = @()
    foreach ($block in ($text -split "(\r?\n){2,}")) {
        if ($block -notmatch 'vwifi\.inf') { continue }
        if ($block -match '(oem\d+\.inf)') { $out += $Matches[1] }
    }
    return ($out | Select-Object -Unique)
}

# Devnodes for the vwifi device, present or not. Get-PnpDevice reports
# both; a device that is not plugged in comes back with Status
# 'Unknown', and that phantom is exactly what holds the stale driver
# binding across a reboot.
function Get-VwifiDevices {
    Get-PnpDevice -ErrorAction SilentlyContinue |
        Where-Object { $_.InstanceId -like "$HwId*" }
}

# --- 1. what is here now ----------------------------------------------
Write-Host ''
Write-Host '[1/5] Current state'

$oldPackages = @(Get-VwifiStorePackages)
if ($oldPackages.Count -eq 0) {
    Write-Host '  driver store: no vwifi package installed'
} else {
    Write-Host "  driver store: $($oldPackages -join ', ')"
}

$devices = @(Get-VwifiDevices)
if ($devices.Count -eq 0) {
    Write-Host '  device: not present (this is the safe way to install)'
} else {
    foreach ($d in $devices) {
        Write-Host "  device: $($d.InstanceId)  Status=$($d.Status) Problem=$($d.ProblemCode)"
    }
}

if (Test-Path $InstalledSys) {
    Write-Host "  installed binary: $((Get-FileHash $InstalledSys -Algorithm SHA256).Hash)"
} else {
    Write-Host '  installed binary: none'
}

# Started means a driver is loaded and the WLAN service is holding it.
# That is the state in which every removal route has hung.
$started = @($devices | Where-Object { $_.Status -eq 'OK' })

# --- 2. remove every old package --------------------------------------
Write-Host ''
if ($KeepOld) {
    Write-Host '[2/5] -KeepOld: leaving existing packages in place'
} elseif ($started.Count -gt 0 -and -not $Force) {
    Write-Host '[2/5] REFUSING to remove: the device is started.'
    Write-Host '      Removing a package that a started device is using has to'
    Write-Host '      stop that device, and that is the step that hangs badly'
    Write-Host '      enough to need the VM forced off.'
    Write-Host ''
    Write-Host '      Shut the guest down, drop these two lines from the QEMU'
    Write-Host '      command line, boot, and run this again:'
    Write-Host '        -chardev socket,id=medium,...'
    Write-Host '        -device vwifi-virt,chardev=medium,...'
    Write-Host ''
    Write-Host '      Or re-run with -Force to try anyway.'
    exit 1
} elseif ($oldPackages.Count -eq 0 -and $devices.Count -eq 0) {
    Write-Host '[2/5] Nothing to remove'
} else {
    Write-Host '[2/5] Removing old packages and device nodes ...'

    # Devnodes first. Removing the devnode drops its Driver key, which
    # is the binding that would otherwise survive into the next boot and
    # keep the old driver in use no matter what is in the store.
    foreach ($d in $devices) {
        Write-Host "  removing device node $($d.InstanceId)"
        $removed = $false
        # pnputil /remove-device is the supported route, and is present
        # on Windows 10 2004 and later.
        & pnputil /remove-device "$($d.InstanceId)" 2>&1 | Out-Null
        if ($LASTEXITCODE -eq 0) { $removed = $true }
        if (-not $removed) {
            # Older guests: the cmdlet, where it exists.
            try {
                Remove-PnpDevice -InstanceId $d.InstanceId -Confirm:$false `
                                 -ErrorAction Stop
                $removed = $true
            } catch { }
        }
        if (-not $removed) {
            Write-Warning ("  could not remove $($d.InstanceId) -- the package " +
                           "removal below still unbinds it")
        }
    }

    # Then the packages. /uninstall removes the driver from any device
    # still using it; /force proceeds even if one is.
    foreach ($oem in $oldPackages) {
        Write-Host "  deleting $oem"
        & pnputil /delete-driver $oem /uninstall /force 2>&1 |
            ForEach-Object { Write-Host "    $_" }
    }

    # And the binary itself. PnP copies vwifi.sys out of the store into
    # System32\drivers, and a file left behind there is a file the
    # service can still load. This is the one that makes an install look
    # like it worked and a debug log show the previous build.
    if (Test-Path $InstalledSys) {
        Write-Host "  deleting $InstalledSys"
        try {
            Remove-Item $InstalledSys -Force -ErrorAction Stop
        } catch {
            Write-Warning ("  could not delete it: $($_.Exception.Message)")
            Write-Warning '  something still has the driver loaded; reboot and re-run.'
        }
    }
}

# --- 3. stage and install the new package -----------------------------
Write-Host ''
Write-Host '[3/5] Installing the new package ...'
Write-Host "  $inf"

# /install, not bare /add-driver. Staging alone puts the package in the
# store and stops; /install also offers it to matching devices that have
# no driver, which after step 2 is what the vwifi device is.
& pnputil /add-driver $inf /install
if ($LASTEXITCODE -ne 0) {
    Write-Host ''
    Write-Host 'ERROR: pnputil failed. The usual causes:'
    Write-Host '  * test-signing off, or Secure Boot / memory integrity on'
    Write-Host '  * the test certificate not in Root AND TrustedPublisher'
    Write-Host '  * vwifi.cat stale -- re-run sign.cmd after every build'
    Write-Host '  %windir%\inf\setupapi.dev.log records what PnP actually did.'
    exit 1
}

# --- 4. let PnP pick it up --------------------------------------------
Write-Host ''
Write-Host '[4/5] Rescanning ...'
& pnputil /scan-devices | Out-Null
Start-Sleep -Seconds 2

# --- 5. verify --------------------------------------------------------
#
# By file hash, not by version string. The driver carries no version
# resource, so the only honest way to answer "is the binary on disk the
# one I just built" is to compare the bytes.
Write-Host ''
Write-Host '[5/5] Result'

$nowPackages = @(Get-VwifiStorePackages)
Write-Host "  driver store: $(if ($nowPackages) { $nowPackages -join ', ' } else { 'none' })"
if (-not $KeepOld -and $nowPackages.Count -gt 1) {
    Write-Warning ('  more than one vwifi package is still installed -- ' +
                   'PnP may pick either one')
}

$devices = @(Get-VwifiDevices)
if ($devices.Count -eq 0) {
    Write-Host '  device: not present.'
    Write-Host '          Expected if you booted without -device vwifi-virt.'
    Write-Host '          Shut down, put it back on the QEMU command line, boot.'
} else {
    foreach ($d in $devices) {
        Write-Host "  device: $($d.FriendlyName)"
        Write-Host "          Status=$($d.Status) Problem=$($d.ProblemCode) $($d.ProblemDescription)"
    }
}

if (Test-Path $InstalledSys) {
    $want = (Get-FileHash $sys -Algorithm SHA256).Hash
    $have = (Get-FileHash $InstalledSys -Algorithm SHA256).Hash
    if ($want -eq $have) {
        Write-Host '  installed binary MATCHES the package just built'
    } else {
        Write-Host '  installed binary DOES NOT MATCH the package just built:'
        Write-Host "    package:   $want"
        Write-Host "    installed: $have"
        Write-Warning ('  the device is still bound to an older package. ' +
                       'Boot without the vwifi-virt device and re-run.')
    }
} else {
    Write-Host '  installed binary: none yet -- PnP copies it when the device'
    Write-Host '                    is present and installs. Expected if you'
    Write-Host '                    booted without the device.'
}

Write-Host ''
Write-Host 'The driver''s own account of what happened is on the HOST, not here:'
Write-Host '  QEMU -debugcon file:/tmp/vwifi-boot.log'
Write-Host 'It is written as the guest emits it, so it survives a hang, a'
Write-Host 'bugcheck and a reset -- and unlike DebugView it captures boot.'
Write-Host 'The first line names the build: "DriverEntry  build 1.0.MMdd.HHmm".'
Write-Host 'If that does not match the DriverVer above, the old driver is'
Write-Host 'still loaded and nothing in this run took effect.'
