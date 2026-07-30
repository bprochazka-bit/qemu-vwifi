<#
.SYNOPSIS
    Install or replace the vwifi driver in the Windows guest.

.DESCRIPTION
    Run elevated, from the folder holding vwifi.sys / vwifi.inf /
    vwifi.cat (that is x64\Debug\vwifi\ as assembled by sign.cmd).

    Replacing a driver is not the same as installing one.
    `pnputil /add-driver` adds a package, but PnP keeps the one already
    bound to the device unless the new package ranks better — and
    ranking is on DriverVer, which stampinf writes as <date>,<version>.
    Two builds on the same day with the same version are
    indistinguishable, so the old package wins and you keep testing the
    .sys you thought you replaced. This removes the old package
    outright rather than relying on ranking.

.PARAMETER Path
    Folder holding the driver package. Defaults to the script's folder.
#>
[CmdletBinding()]
param(
    [string] $Path = $PSScriptRoot
)

$ErrorActionPreference = 'Stop'

$isAdmin = ([Security.Principal.WindowsPrincipal] `
    [Security.Principal.WindowsIdentity]::GetCurrent()
    ).IsInRole([Security.Principal.WindowsBuiltInRole]::Administrator)
if (-not $isAdmin) { throw 'Run this elevated (Run as Administrator).' }

$inf = Join-Path $Path 'vwifi.inf'
$sys = Join-Path $Path 'vwifi.sys'
$cat = Join-Path $Path 'vwifi.cat'

if (-not (Test-Path $inf)) { throw "vwifi.inf not found in $Path" }
if (-not (Test-Path $sys)) { throw "vwifi.sys not found in $Path" }
if (-not (Test-Path $cat)) {
    Write-Warning 'No vwifi.cat — run sign.cmd. Without it the package is unsigned.'
}

# The DriverVer actually being installed, so a stale package is obvious.
$driverVer = (Select-String -Path $inf -Pattern '^\s*DriverVer\s*=' |
              Select-Object -First 1).Line
if ($driverVer) { Write-Host "Installing: $($driverVer.Trim())" }

# --- 1. remove every previously installed vwifi package --------------
Write-Host ''
Write-Host '[1/4] Removing previously installed vwifi packages ...'

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
        Write-Host "  removing $o"
        & pnputil /delete-driver $o /uninstall /force | Out-Null
        if ($LASTEXITCODE -ne 0) {
            Write-Host "    (pnputil returned $LASTEXITCODE; continuing)"
        }
    }
}

# --- 2. add and install the new package ------------------------------
Write-Host ''
Write-Host "[2/4] Installing $inf ..."
& pnputil /add-driver $inf /install
if ($LASTEXITCODE -ne 0) {
    Write-Host ''
    Write-Host 'ERROR: pnputil failed. The usual causes:'
    Write-Host '  * test-signing off, or Secure Boot / memory integrity on'
    Write-Host '  * the test certificate not in Root AND TrustedPublisher'
    Write-Host '  * vwifi.cat stale — re-run sign.cmd after every build'
    Write-Host '  %windir%\inf\setupapi.dev.log records what PnP actually did.'
    exit 1
}

# --- 3. make PnP re-evaluate the device ------------------------------
Write-Host ''
Write-Host '[3/4] Rescanning for devices ...'
& pnputil /scan-devices | Out-Null

# --- 4. report where the device ended up -----------------------------
Write-Host ''
Write-Host '[4/4] Device status:'
$dev = Get-PnpDevice -InstanceId 'PCI\VEN_1AF4&DEV_0E00*' -ErrorAction SilentlyContinue
if (-not $dev) {
    Write-Host '  no vwifi-virt device present — is -device vwifi-virt on the QEMU command line?'
} else {
    foreach ($d in $dev) {
        Write-Host "  $($d.FriendlyName)"
        Write-Host "  Status: $($d.Status)   Problem: $($d.ProblemCode) $($d.ProblemDescription)"
    }
}

Write-Host ''
Write-Host 'If it did not start, capture the driver''s own account of why:'
Write-Host '  DebugView as admin, Capture Kernel + Enable Verbose Kernel Output,'
Write-Host '  then disable and re-enable the device to re-run init with the'
Write-Host '  capture already attached.'
