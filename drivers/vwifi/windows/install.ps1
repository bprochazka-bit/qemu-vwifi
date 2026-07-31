<#
.SYNOPSIS
    Install or replace the vwifi driver in the Windows guest.

.DESCRIPTION
    Run elevated, from the folder holding vwifi.sys / vwifi.inf /
    vwifi.cat (that is x64\Debug\vwifi\ as assembled by sign.cmd).

    Replacing a driver is not the same as installing one.
    `pnputil /add-driver` adds a package, but PnP keeps the one already
    bound to the device unless the new package ranks better -- and
    ranking is on DriverVer, which stampinf writes as <date>,<version>.
    build.cmd derives the version from the build time, so every build
    strictly outranks the last and ranking alone picks the new one.

    The old package is therefore left alone by default. Removing it
    means `pnputil /delete-driver /uninstall`, which has to stop the
    device using it -- and once the driver works, that is a live adapter
    to unwind. If the teardown stalls, so does the uninstall, and the
    install never happens. Pass -RemoveOld to clean up superseded
    packages; it runs last, after the new driver is in.

.PARAMETER Path
    Folder holding the driver package. Defaults to the script's own
    folder, then to the build output under it.

.PARAMETER RemoveOld
    After installing, remove superseded vwifi packages. Off by default:
    it has to stop whatever is using them, and a stall there used to
    take the whole install with it.
#>
[CmdletBinding()]
param(
    [string] $Path,
    [switch] $RemoveOld
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

# --- 1. add and install the new package ------------------------------
#
# Installed first, and the old package is NOT removed to make room.
#
# It used to be, because two builds on the same day carried an identical
# DriverVer and PnP had no reason to prefer the new one. build.cmd now
# derives the version from the build time (1.0.MMdd.HHmm), so every
# build strictly outranks its predecessor and ordinary ranking picks it.
#
# Removing first also stopped being harmless the moment the driver
# started working. `pnputil /delete-driver /uninstall /force` has to
# stop the device that is using the package, which means unwinding a
# live adapter -- port teardown, close, halt -- and if any of that
# stalls, the uninstall stalls with it and takes the install with it.
# That is a bad place for a bring-up script to wedge: nothing is
# installed yet, and the reason for the stall is in the driver you were
# about to replace.
Write-Host ''
Write-Host '[1/4] Installing ...'

# Disable the device first, and re-enable it afterwards.
#
# Once the driver works, the WLAN service holds the adapter open, and
# pnputil cannot replace a driver whose device is in use -- it fails,
# and the only way anyone found back out was a reboot. Disabling drops
# the service's hold and takes the adapter through its own teardown,
# which is a far better place to unwind than half way through an
# uninstall.
#
# Best-effort on purpose: on the very first install there is no device
# to disable, and a device stuck in a bad state may refuse. Neither is
# a reason not to try the install.
$dev = Get-PnpDevice -InstanceId 'PCI\VEN_1AF4&DEV_0E00*' -ErrorAction SilentlyContinue |
       Where-Object { $_.Status -ne 'Unknown' }
$disabled = $false
if ($dev) {
    Write-Host '  disabling the adapter so pnputil is not fighting it ...'
    try {
        $dev | Disable-PnpDevice -Confirm:$false -ErrorAction Stop
        $disabled = $true
    } catch {
        Write-Host "    (could not disable: $($_.Exception.Message))"
        Write-Host '    continuing; the install may fail with the device in use.'
    }
}

Write-Host "  adding $inf ..."
& pnputil /add-driver $inf /install
$addStatus = $LASTEXITCODE

if ($disabled) {
    Write-Host '  re-enabling the adapter ...'
    try {
        Get-PnpDevice -InstanceId 'PCI\VEN_1AF4&DEV_0E00*' -ErrorAction Stop |
            Enable-PnpDevice -Confirm:$false -ErrorAction Stop
    } catch {
        Write-Host "    (could not re-enable: $($_.Exception.Message))"
        Write-Host '    enable it by hand in Device Manager.'
    }
}

$global:LASTEXITCODE = $addStatus
if ($LASTEXITCODE -ne 0) {
    Write-Host ''
    Write-Host 'ERROR: pnputil failed. The usual causes:'
    Write-Host '  * test-signing off, or Secure Boot / memory integrity on'
    Write-Host '  * the test certificate not in Root AND TrustedPublisher'
    Write-Host '  * vwifi.cat stale -- re-run sign.cmd after every build'
    Write-Host '  %windir%\inf\setupapi.dev.log records what PnP actually did.'
    exit 1
}

# --- 2. make PnP re-evaluate the device ------------------------------
Write-Host ''
Write-Host '[2/4] Rescanning for devices ...'
& pnputil /scan-devices | Out-Null

# --- 3. clean up superseded packages, if asked -----------------------
#
# Last, and opt-in. By this point the device is bound to the new
# package, so removing the old ones should not have to stop anything --
# but "should not" is what the ordering above is defending against, and
# a stall here costs nothing: the driver is already installed and Ctrl-C
# is safe.
Write-Host ''
if (-not $RemoveOld) {
    Write-Host '[3/4] Leaving superseded packages in place (-RemoveOld to clean up).'
} else {
    Write-Host '[3/4] Removing superseded vwifi packages ...'

    # pnputil /enum-drivers emits blank-line-separated records; find the
    # published oemNN.inf name of any whose original name is vwifi.inf.
    $records = ((pnputil /enum-drivers) -join "`n") -split "`r?`n`r?`n"
    $old = foreach ($r in $records) {
        if ($r -match '(?im)^\s*Original Name:\s*vwifi\.inf\s*$' -and
            $r -match '(?im)^\s*Published Name:\s*(oem\d+\.inf)\s*$') {
            $Matches[1]
        }
    }

    if (-not $old) {
        Write-Host '  none installed.'
    } else {
        foreach ($o in $old) {
            Write-Host "  removing $o (Ctrl-C is safe -- the new driver is in)"
            & pnputil /delete-driver $o /uninstall | Out-Null
            if ($LASTEXITCODE -ne 0) {
                Write-Host "    (pnputil returned $LASTEXITCODE; continuing)"
            }
        }
    }
}

# --- 4. report where the device ended up -----------------------------
Write-Host ''
Write-Host '[4/4] Device status:'
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
