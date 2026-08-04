<#
.SYNOPSIS
    Dump what the WDI port driver thinks our adapter and port are, using
    !ndiskd in a local kernel debugger.

.DESCRIPTION
    OID_WDI_TASK_CONNECT has never reached this miniport. wdiwifi.sys
    accepts OID_DOT11_CONNECT_REQUEST, returns success, and about two
    milliseconds later reports STATUS_NETWORK_UNREACHABLE back up the
    stack. Connecting to an SSID that exists nowhere fails identically
    -- same status, same timing, the same OIDs in the same order -- so
    the refusal happens before any BSS is considered, and nothing about
    the BSS entry can explain it.

    What is left is state we cannot see: how wdiwifi has recorded this
    adapter, its ports, and the capabilities we reported. !ndiskd prints
    exactly that, and it prints it statically -- no reproduction to time,
    no race to catch.

    Two passes, because the interesting commands need a handle that only
    the first pass can tell us. Pass one lists miniports; the handle for
    the adapter matching -Adapter is pulled out of that output and fed
    to pass two.

    ---------------------------------------------------------------
    !ndiskd NEEDS ndis.sys SYMBOLS. It walks NDIS structures by symbol
    name, and without them it prints errors instead of state. On a
    machine whose only network adapter is the one being debugged there
    is no route to the symbol server, so use -ShowSymbolKeys to get the
    exact URLs, fetch the PDBs anywhere else, and pass the directory as
    -SymbolPath. Anything else is guesswork dressed up as output.
    ---------------------------------------------------------------

.PARAMETER Enable
    Turn on local kernel debugging (bcdedit) and stop. Needs a reboot.

.PARAMETER SymbolPath
    Directory holding pre-fetched PDBs (at minimum ndis.sys's). Used
    instead of the Microsoft symbol server.

.PARAMETER ShowSymbolKeys
    Print the symbol-server URLs for the modules this needs, then exit.
    Run this first if the machine has no internet.

.PARAMETER Adapter
    Substring matched against the miniport list. Default 'vwifi'.

.PARAMETER OutDir
    Where the logs go. Default the current directory.

.EXAMPLE
    .\dump-wdi-state.cmd -ShowSymbolKeys

.EXAMPLE
    .\dump-wdi-state.cmd -Enable
    :: reboot, then:
    .\dump-wdi-state.cmd -SymbolPath C:\symbols
#>
[CmdletBinding()]
param(
    [switch] $Enable,
    [switch] $ShowSymbolKeys,
    [string] $SymbolPath,
    [string] $Adapter = 'vwifi',
    [string] $OutDir  = '.'
)

$ErrorActionPreference = 'Stop'
. (Join-Path $PSScriptRoot 'wdk-symbols.ps1')

# ndis.sys is what !ndiskd actually needs; the other two are here so the
# stack is readable end to end if the first pass raises new questions.
$modules = @(
    "$env:SystemRoot\System32\drivers\ndis.sys",
    "$env:SystemRoot\System32\drivers\wdiwifi.sys",
    "$env:SystemRoot\System32\drivers\vwifi.sys"
) | Where-Object { Test-Path $_ }

function Assert-Admin {
    $p = [Security.Principal.WindowsPrincipal] [Security.Principal.WindowsIdentity]::GetCurrent()
    if (-not $p.IsInRole([Security.Principal.WindowsBuiltInRole]::Administrator)) {
        throw 'Run this from an elevated PowerShell (Run as Administrator).'
    }
}

function Show-SymbolKeys {
    Write-Host 'Symbol-server URLs for this machine''s binaries:'
    Write-Host ''
    foreach ($m in $modules) {
        $cv = Get-PeCodeViewInfo $m
        if ($cv) {
            Write-Host ("  {0}" -f (Split-Path $m -Leaf))
            Write-Host ("    {0}" -f $cv.Url)
        } else {
            Write-Warning "no CodeView record in $m"
        }
    }
    Write-Host ''
    Write-Host 'Download those, put the .pdb files in one directory, and pass it'
    Write-Host 'as -SymbolPath. ndis.sys is the one !ndiskd cannot work without.'
}

function Enable-LocalKd {
    Assert-Admin
    Write-Host 'Enabling local kernel debugging'
    & bcdedit /dbgsettings local | ForEach-Object { Write-Host "  $_" }
    & bcdedit /debug on          | ForEach-Object { Write-Host "  $_" }
    Write-Host ''
    Write-Host 'Reboot, then re-run without -Enable.'
    Write-Host ''
    Write-Host 'If local KD still refuses after the reboot, the usual cause is'
    Write-Host 'virtualisation-based security holding the debugger off. Check'
    Write-Host '  msinfo32 -> "Virtualization-based security"'
    Write-Host 'and turn off Memory Integrity (Core Isolation) if it is running.'
}

function Test-LocalKdEnabled {
    $dbg = (& bcdedit /enum '{current}' 2>&1 | Out-String)
    $set = (& bcdedit /dbgsettings 2>&1 | Out-String)
    $on    = $dbg -match '(?im)^\s*debug\s+Yes\s*$'
    $local = $set -match '(?im)debugtype\s+Local'
    return @{ DebugOn = $on; TypeLocal = $local }
}

function Get-SymbolPathArg {
    if ($SymbolPath) {
        if (-not (Test-Path $SymbolPath)) { throw "no such path: $SymbolPath" }
        $full = (Resolve-Path $SymbolPath).Path
        $n = @(Get-ChildItem $full -Filter '*.pdb' -Recurse -ErrorAction SilentlyContinue).Count
        Write-Host "      Symbols: $full ($n pdb)"
        if ($n -eq 0) { Write-Warning 'that directory contains no .pdb files' }
        # No srv* fallback appended on purpose. If the supplied symbols
        # are wrong we want !ndiskd to fail visibly, not to half-resolve
        # from a server this machine probably cannot reach anyway.
        return $full
    }
    $p = "srv*$env:SystemDrive\symbols*https://msdl.microsoft.com/download/symbols"
    Write-Host "      Symbols: $p"
    Write-Warning 'no -SymbolPath given; this needs internet. Use -ShowSymbolKeys if there is none.'
    return $p
}

function Invoke-Kd {
    param(
        [Parameter(Mandatory)][string] $Kd,
        [Parameter(Mandatory)][string] $Sym,
        [Parameter(Mandatory)][string] $Commands,
        [Parameter(Mandatory)][string] $LogFile
    )
    # -kl local kernel debug, -logo overwrite log, and a trailing q so
    # the session exits instead of sitting at a prompt no one is at.
    & $Kd -kl -y "$Sym" -logo "$LogFile" -c "$Commands;q" 2>&1 | Out-Null
    if (Test-Path $LogFile) { return Get-Content $LogFile }
    return @()
}

function Find-MiniportHandle {
    param([string[]] $Lines)

    # !ndiskd.miniports prints a table whose rows carry the miniport
    # handle and the adapter's friendly name. Match the row by name,
    # then take the first pointer-width hex token on it.
    foreach ($l in $Lines) {
        if ($l -notmatch [regex]::Escape($Adapter)) { continue }
        $m = [regex]::Match($l, '\b([0-9a-fA-F]{8,16})\b')
        if ($m.Success) { return $m.Groups[1].Value }
    }
    return $null
}

# ---------------------------------------------------------------- main
if ($ShowSymbolKeys) { Show-SymbolKeys; return }
if ($Enable)         { Enable-LocalKd;  return }

Assert-Admin

$kd = Find-WdkTool 'kd.exe'
if (-not $kd) {
    throw @'
kd.exe not found. It ships with Debugging Tools for Windows, which the
EWDK does not include -- install just that component from the Windows
SDK installer (winsdksetup.exe), or copy a Debuggers\x64 directory onto
this machine and put it on PATH.
'@
}
Write-Host "[1/4] kd: $kd"

$state = Test-LocalKdEnabled
if (-not $state.DebugOn -or -not $state.TypeLocal) {
    throw "local kernel debugging is not enabled (debug=$($state.DebugOn), local=$($state.TypeLocal)). Run with -Enable, then reboot."
}

New-Item -ItemType Directory -Force -Path $OutDir | Out-Null
$OutDir = (Resolve-Path $OutDir).Path
$sym    = Get-SymbolPathArg

# Pass one: what miniports exist, and does ndiskd work at all.
$log1 = Join-Path $OutDir 'wdi-state-miniports.txt'
Write-Host '[2/4] Listing miniports'
$out1 = Invoke-Kd -Kd $kd -Sym $sym -LogFile $log1 -Commands @'
.load ndiskd
!ndiskd.help
!ndiskd.miniports
'@

if ($out1.Count -eq 0) { throw "kd produced no output; see $log1" }

# ndiskd failing to resolve NDIS symbols is the single most common way
# this ends up with pages of confident nonsense, so it is checked rather
# than hoped for.
if ($out1 -match 'Unable to (?:load|resolve)|symbol.*not found|\*\*\* ERROR') {
    Write-Warning 'ndiskd reported symbol errors -- output below is unreliable.'
    Write-Warning 'Fetch ndis.sys symbols (-ShowSymbolKeys) and pass -SymbolPath.'
}

$handle = Find-MiniportHandle -Lines $out1
if (-not $handle) {
    Write-Warning "no miniport matching '$Adapter' in $log1"
    Write-Host   'Open that file, find the adapter row, and re-run with -Adapter <substring>.'
    return
}
Write-Host "      miniport handle: $handle"

# Pass two: everything worth knowing about that miniport. Each command
# is independent, so one that this ndiskd build does not have costs its
# own output and nothing else.
$log2 = Join-Path $OutDir 'wdi-state-detail.txt'
Write-Host '[3/4] Dumping adapter, port and WDI state'
$cmds = @(
    "!ndiskd.miniport $handle",
    "!ndiskd.wdiminidriver $handle",
    "!ndiskd.netadapter $handle",
    '!ndiskd.miniports -wdi',
    '!ndiskd.oid',
    '!ndiskd.pendingoids',
    'lm vm ndis',
    'lm vm wdiwifi',
    'lm vm vwifi'
) -join ';'
$out2 = Invoke-Kd -Kd $kd -Sym $sym -LogFile $log2 -Commands $cmds

Write-Host '[4/4] Done'
Write-Host "      $log1 ($($out1.Count) lines)"
Write-Host "      $log2 ($($out2.Count) lines)"
Write-Host ''
Write-Host '      Send wdi-state-detail.txt. The port state and the capabilities'
Write-Host '      wdiwifi recorded are what this exists to show.'
