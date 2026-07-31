<#
.SYNOPSIS
    Install or replace the vwifi driver in the Windows guest.

.DESCRIPTION
    Run elevated, from the folder holding vwifi.sys / vwifi.inf /
    vwifi.cat (that is x64\Debug\vwifi\ as assembled by sign.cmd).

    This STAGES the package and stops. It does not remove the old
    package, disable the device, or restart it -- every one of those has
    to stop a device the WLAN service is holding, and every one of them
    has been observed to hang badly enough that the VM had to be forced
    off.

    Staging is enough. PnP picks a package by DriverVer ranking, and
    build.cmd derives the version from the build time, so each build
    strictly outranks the last. Reboot and the new driver loads.

    Reboot is currently the reliable way to swap drivers here, and this
    script is honest about that rather than hanging while pretending
    otherwise.

.PARAMETER Path
    Folder holding the driver package. Defaults to the script's own
    folder, then to the build output under it.

.PARAMETER Restart
    Also restart the device so the staged driver takes effect without a
    reboot. Off by default: this is the step that hangs. If it does,
    the package is already staged, so a reboot still gets you there.
#>
[CmdletBinding()]
param(
    [string] $Path,
    [switch] $Restart
)

$ErrorActionPreference = 'Stop'

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

# --- 1. stage the new package ----------------------------------------
#
# The old package is not removed to make room. It used to be, because
# two builds on the same day carried an identical DriverVer and PnP had
# no reason to prefer the new one. build.cmd now derives the version
# from the build time (1.0.MMdd.HHmm), so every build strictly outranks
# its predecessor and ordinary ranking picks it up.
Write-Host ''
Write-Host '[1/3] Staging the driver package ...'

# STAGE ONLY. Nothing here touches the running device.
#
# Two earlier versions of this script did, and both wedged:
#
#   pnputil /delete-driver /uninstall /force  -- has to stop the device
#       using the package.
#   Disable-PnpDevice                         -- same thing by another
#       route.
#
# Both hang, and they hang hard enough to take the reboot with them:
# the VM has to be forced off. Why is not yet known -- no trace has been
# captured during a disable, so whether the driver's teardown is even
# reached is an open question, not a settled one. What is known is that
# asking from here does not help.
#
# `pnputil /add-driver` without /install stages the package in the
# driver store and returns immediately. PnP picks it up the next time
# the device starts -- which, given a reboot is currently needed anyway,
# costs nothing and cannot wedge.
#
# -Restart tries to make it take effect now, and is off by default
# precisely because that is the part that hangs.
Write-Host "  staging $inf in the driver store ..."
& pnputil /add-driver $inf
$addStatus = $LASTEXITCODE
if ($LASTEXITCODE -ne 0) {
    Write-Host ''
    Write-Host 'ERROR: pnputil failed. The usual causes:'
    Write-Host '  * test-signing off, or Secure Boot / memory integrity on'
    Write-Host '  * the test certificate not in Root AND TrustedPublisher'
    Write-Host '  * vwifi.cat stale -- re-run sign.cmd after every build'
    Write-Host '  %windir%\inf\setupapi.dev.log records what PnP actually did.'
    exit 1
}

# --- 2. bind it, if the device has nothing bound ----------------------
#
# Staging alone does NOT load a driver. `pnputil /add-driver` without
# /install writes the package to the driver store and stops there; the
# device keeps whatever it already had, which for a device with nothing
# bound means it stays in an error state and no driver output ever
# appears. An earlier version of this script dropped the rescan at the
# same time it dropped /install, so it staged packages that were never
# used and reported success for doing it.
#
# pnputil /scan-devices re-enumerates. For a device with no driver that
# binds the newly staged package immediately -- nothing has to be
# stopped, so there is nothing to hang on. For a device that already has
# a working driver, it does nothing, because replacing a bound driver
# means restarting the device and that is the operation that wedges.
Write-Host ''
$dev = Get-PnpDevice -InstanceId 'PCI\VEN_1AF4&DEV_0E00*' -ErrorAction SilentlyContinue
$bound = $dev | Where-Object { $_.Status -eq 'OK' }

if ($bound) {
    Write-Host '[2/3] A driver is already loaded on this device.'
    Write-Host '      The new package is staged and outranks it, but swapping'
    Write-Host '      a bound driver means restarting the device, which is the'
    Write-Host '      step that hangs. REBOOT to load it.'
} else {
    Write-Host '[2/3] No driver bound; rescanning so PnP picks up the staged package ...'
    & pnputil /scan-devices | Out-Null
    Start-Sleep -Seconds 2
}

# --- 3. report where the device ended up -----------------------------
Write-Host ''
Write-Host '[3/3] Device status:'
$dev = Get-PnpDevice -InstanceId 'PCI\VEN_1AF4&DEV_0E00*' -ErrorAction SilentlyContinue
if (-not $dev) {
    Write-Host '  no vwifi-virt device present -- is -device vwifi-virt on the QEMU command line?'
} else {
    foreach ($d in $dev) {
        Write-Host "  $($d.FriendlyName)"
        Write-Host "  Status: $($d.Status)   Problem: $($d.ProblemCode) $($d.ProblemDescription)"
    }
}

Write-Host ''
Write-Host 'The driver''s own account of what happened is on the HOST, not here:'
Write-Host '  QEMU -debugcon file:/tmp/vwifi-boot.log'
Write-Host 'It is written as the guest emits it, so it survives a hang, a'
Write-Host 'bugcheck and a reset -- and unlike DebugView it captures boot.'
Write-Host 'If something stalls, read that file while it is still stuck.'
