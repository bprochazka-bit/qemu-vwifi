<#
.SYNOPSIS
    Prepare a Windows guest to load and debug the vwifi test driver.

.DESCRIPTION
    Run once, as Administrator, inside the Windows VM. Reboot afterwards.

    Does four things, each of which is a separate reason a test driver
    "doesn't load" or "prints nothing":

      1. Enables test-signing, so an unsigned-by-Microsoft .sys loads.
      2. Imports the test certificate into Root + TrustedPublisher, so
         pnputil installs without prompting.
      3. Raises the DbgPrintEx filter mask for the IHVNETWORK component.
         Without this the driver's VWIFI_INFO/WARN lines are compiled
         in, emitted, and silently discarded before they reach
         DebugView or the kernel debugger. This is the single most
         common "my driver is mute" cause.
      4. Optionally turns on kernel debugging over COM1, which pairs
         with QEMU's  -serial pipe:\\.\pipe\windbg-vwifi.

.PARAMETER CertPath
    Path to vwifi-test-cert.cer as exported by sign.cmd on the build
    machine. Omit to skip the certificate import.

.PARAMETER KernelDebug
    Also enable kernel debugging over COM1 at 115200 baud.

.PARAMETER Verifier
    Also enable Driver Verifier's standard checks against vwifi.sys.
    Slower, but turns silent corruption into an immediate, attributable
    bugcheck. Worth it for the first few boots.

.EXAMPLE
    .\guest-debug-setup.ps1 -CertPath .\vwifi-test-cert.cer -KernelDebug
#>
[CmdletBinding()]
param(
    [string] $CertPath,
    [switch] $KernelDebug,
    [switch] $Verifier
)

$ErrorActionPreference = 'Stop'

$isAdmin = ([Security.Principal.WindowsPrincipal] `
    [Security.Principal.WindowsIdentity]::GetCurrent()
    ).IsInRole([Security.Principal.WindowsBuiltInRole]::Administrator)
if (-not $isAdmin) {
    throw 'Run this script from an elevated PowerShell (Run as Administrator).'
}

# --- 1. test-signing -------------------------------------------------
Write-Host '[1/4] Enabling test-signing ...'
bcdedit /set testsigning on | Out-Null

# --- 2. trust the test certificate -----------------------------------
if ($CertPath) {
    if (-not (Test-Path $CertPath)) { throw "Certificate not found: $CertPath" }
    Write-Host "[2/4] Importing $CertPath into Root and TrustedPublisher ..."
    certutil -addstore Root             $CertPath | Out-Null
    certutil -addstore TrustedPublisher $CertPath | Out-Null
} else {
    Write-Host '[2/4] No -CertPath given; skipping certificate import.'
}

# --- 3. let DbgPrintEx through ---------------------------------------
# DPFLTR_IHVNETWORK_ID is what vwifi_drv.h's VWIFI_DBG() logs under.
# 0xFFFFFFFF = every level, including DPFLTR_INFO_LEVEL.
Write-Host '[3/4] Raising the IHVNETWORK debug print filter mask ...'
$filterKey = 'HKLM:\SYSTEM\CurrentControlSet\Control\Session Manager\Debug Print Filter'
if (-not (Test-Path $filterKey)) { New-Item -Path $filterKey -Force | Out-Null }
New-ItemProperty -Path $filterKey -Name 'IHVNETWORK' `
                 -Value 0xFFFFFFFF -PropertyType DWord -Force | Out-Null
# DEFAULT covers anything logging under DPFLTR_DEFAULT_ID, e.g. NDIS itself.
New-ItemProperty -Path $filterKey -Name 'DEFAULT' `
                 -Value 0xFFFFFFFF -PropertyType DWord -Force | Out-Null

# --- 4. kernel debugging / verifier ----------------------------------
if ($KernelDebug) {
    Write-Host '[4/4] Enabling kernel debugging over COM1 @ 115200 ...'
    bcdedit /debug on | Out-Null
    bcdedit /dbgsettings SERIAL DEBUGPORT:1 BAUDRATE:115200 | Out-Null
} else {
    Write-Host '[4/4] Skipping kernel debugging (-KernelDebug to enable).'
}

if ($Verifier) {
    Write-Host '      Enabling Driver Verifier (standard flags) on vwifi.sys ...'
    verifier /standard /driver vwifi.sys | Out-Null
}

Write-Host ''
Write-Host 'Done. Two things this script cannot do for you:'
Write-Host '  * Memory integrity (Core Isolation) blocks test-signed drivers.'
Write-Host '    Turn it off: Windows Security > Device security > Core isolation.'
Write-Host '  * Secure Boot must be off in the VM firmware for test-signing.'
Write-Host ''
Write-Host 'Reboot now, then: pnputil /add-driver vwifi.inf /install'
