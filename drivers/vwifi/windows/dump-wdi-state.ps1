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

.PARAMETER MiniportHandle
    Skip discovery and dump this handle. Read it out of
    wdi-state-miniports.txt when the -Adapter match does not find the
    row; that file always has the answer even when the matching does
    not.

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
    [string] $MiniportHandle,
    [string] $OutDir  = '.'
)

$ErrorActionPreference = 'Stop'
. (Join-Path $PSScriptRoot 'wdk-symbols.ps1')

# Microsoft binaries, whose PDBs come from the symbol server. ndis.sys
# is what !ndiskd actually needs; wdiwifi.sys is here so the stack is
# readable end to end if the first pass raises new questions.
$msModules = @(
    "$env:SystemRoot\System32\drivers\ndis.sys",
    "$env:SystemRoot\System32\drivers\wdiwifi.sys"
) | Where-Object { Test-Path $_ }

# Ours. Deliberately NOT in the list above: vwifi.pdb is produced by our
# own build, so Microsoft's symbol server has never heard of it and asking
# for it returns 404. The build writes it next to the .sys under
# x64\<cfg>\, and those directories go on the symbol path directly.
$localSymbolDirs = @(
    (Join-Path $PSScriptRoot 'x64\Debug'),
    (Join-Path $PSScriptRoot 'x64\Release')
) | Where-Object { Test-Path $_ }

function Assert-Admin {
    $p = [Security.Principal.WindowsPrincipal] [Security.Principal.WindowsIdentity]::GetCurrent()
    if (-not $p.IsInRole([Security.Principal.WindowsBuiltInRole]::Administrator)) {
        throw 'Run this from an elevated PowerShell (Run as Administrator).'
    }
}

function Show-SymbolKeys {
    Write-Host 'Fetch these from Microsoft''s symbol server:'
    Write-Host ''
    foreach ($m in $msModules) {
        $cv = Get-PeCodeViewInfo $m
        if ($cv) {
            Write-Host ("  {0}" -f (Split-Path $m -Leaf))
            Write-Host ("    {0}" -f $cv.Url)
        } else {
            Write-Warning "no CodeView record in $m"
        }
    }
    Write-Host ''
    Write-Host 'Put those .pdb files in one directory and pass it as -SymbolPath.'
    Write-Host 'ndis.sys is the one !ndiskd cannot work without; wdiwifi.sys only'
    Write-Host 'makes its frames readable if the first pass raises new questions.'
    Write-Host ''

    # vwifi.pdb is ours. It is called out rather than silently omitted
    # because an earlier version listed it alongside the Microsoft
    # binaries, which sent people to a symbol-server URL that can only
    # ever 404 -- the server has never seen a driver we built.
    Write-Host 'vwifi.sys is NOT on the symbol server -- it is our own build, so'
    Write-Host 'that URL would 404. Its PDB is already on this machine:'
    if ($localSymbolDirs.Count -eq 0) {
        Write-Host '  (no x64\Debug or x64\Release here yet -- build first)'
    } else {
        foreach ($d in $localSymbolDirs) {
            $pdb = Join-Path $d 'vwifi.pdb'
            $mark = if (Test-Path $pdb) { 'vwifi.pdb' } else { '(no vwifi.pdb yet)' }
            Write-Host ("  {0}  {1}" -f $d, $mark)
        }
        Write-Host '  These are added to the symbol path automatically.'
    }
}

function Enable-LocalKd {
    Assert-Admin
    Write-Host 'Enabling local kernel debugging'
    (Invoke-Native 'bcdedit' @('/dbgsettings','local')).Output | ForEach-Object { Write-Host "  $_" }
    (Invoke-Native 'bcdedit' @('/debug','on')).Output          | ForEach-Object { Write-Host "  $_" }
    Write-Host ''
    Write-Host 'Reboot, then re-run without -Enable.'
    Write-Host ''
    Write-Host 'If local KD still refuses after the reboot, the usual cause is'
    Write-Host 'virtualisation-based security holding the debugger off. Check'
    Write-Host '  msinfo32 -> "Virtualization-based security"'
    Write-Host 'and turn off Memory Integrity (Core Isolation) if it is running.'
}

function Test-LocalKdEnabled {
    $dbg = ((Invoke-Native 'bcdedit' @('/enum','{current}')).Output -join "`n")
    $set = ((Invoke-Native 'bcdedit' @('/dbgsettings')).Output -join "`n")
    $on    = $dbg -match '(?im)^\s*debug\s+Yes\s*$'
    $local = $set -match '(?im)debugtype\s+Local'
    return @{ DebugOn = $on; TypeLocal = $local }
}

function Get-SymbolPathArg {
    # Our own build output always goes on, whatever else does. These are
    # local directories, not a server, so they cannot cause the
    # half-resolve problem the srv* fallback would, and without them
    # `lm vm vwifi` has nothing to say about our own driver.
    $parts = @()

    if ($SymbolPath) {
        if (-not (Test-Path $SymbolPath)) { throw "no such path: $SymbolPath" }
        $full = (Resolve-Path $SymbolPath).Path
        $pdbs = Get-PdbFiles $full
        Write-Host "      Symbols: $full ($($pdbs.Count) pdb)"
        if ($pdbs.Count -eq 0) { Write-Warning 'that directory contains no .pdb files' }
        if (-not ($pdbs | Where-Object { $_.Name -ieq 'ndis.pdb' })) {
            Write-Warning 'no ndis.pdb there -- !ndiskd needs it and will print errors instead of state'
        }

        # Both forms, because a symbol-server download is laid out as
        # <name>.pdb\<guid+age>\<name>.pdb while a hand-assembled folder
        # is usually flat, and which one this is depends on how the
        # files were fetched. The bare path covers flat; srv*<path> with
        # no upstream tells dbghelp to read the same directory as a
        # symbol store. Neither reaches the network.
        $parts += $full
        $parts += "srv*$full"
        # No upstream server on purpose. If the supplied symbols are
        # wrong we want !ndiskd to fail visibly, not to half-resolve
        # from a server this machine probably cannot reach anyway.
    } else {
        $parts += "srv*$env:SystemDrive\symbols*https://msdl.microsoft.com/download/symbols"
        Write-Warning 'no -SymbolPath given; this needs internet. Use -ShowSymbolKeys if there is none.'
    }

    $parts += $localSymbolDirs
    $p = ($parts -join ';')
    Write-Host "      Symbol path: $p"
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
    (Invoke-Native $Kd @('-kl','-y',$Sym,'-logo',$LogFile,'-c',"$Commands;q")) | Out-Null
    if (Test-Path $LogFile) { return Get-Content $LogFile }
    return @()
}

function Find-MiniportHandle {
    param([string[]] $Lines)

    # !ndiskd.miniports prints a table whose rows carry the miniport
    # handle and the adapter's friendly name. Match the row by name,
    # then take the first pointer-width hex token on it.
    #
    # ndiskd emits its handles as DML links, so in a log the row can
    # read `<link cmd="...">ffffe00...</link>`; the hex is still there,
    # which is why this scrapes for hex rather than parsing columns.
    foreach ($l in $Lines) {
        if ($l -notmatch [regex]::Escape($Adapter)) { continue }
        $m = [regex]::Match($l, '\b([0-9a-fA-F]{8,16})\b')
        if ($m.Success) { return $m.Groups[1].Value }
    }
    return $null
}

# Say why the match failed, using the output we already have.
#
# "No miniport matching vwifi" is useless on its own: it does not
# distinguish a name this script guessed wrongly from ndiskd never
# having run. Both look the same from outside and need opposite fixes.
function Show-MiniportDiagnosis {
    param([string[]] $Lines, [string] $LogFile)

    $loadFailed = $Lines | Select-String -Pattern 'Unable to load|No export|not found|cannot find|is not a valid'
    $anyRows    = $Lines | Select-String -Pattern '\b[0-9a-fA-F]{12,16}\b'

    Write-Host ''
    if (-not $Lines -or $Lines.Count -lt 5) {
        Write-Warning "kd produced almost nothing. Check $LogFile -- local KD may not be active."
    } elseif ($loadFailed) {
        Write-Warning 'ndiskd did not load. It ships as ndiskd.dll beside kd.exe:'
        Write-Warning 'copying kd.exe alone is not enough, the whole Debuggers\x64 folder is needed.'
        $loadFailed | Select-Object -First 3 | ForEach-Object { Write-Host "      $_" }
    } elseif (-not $anyRows) {
        Write-Warning 'ndiskd loaded but listed no adapters, which is what it does'
        Write-Warning 'without ndis.sys symbols. Re-run with -SymbolPath pointing at ndis.pdb.'
    } else {
        Write-Warning "ndiskd listed adapters, but none matched '$Adapter'."
        Write-Host   '      Rows that carry a handle:'
        $anyRows | Select-Object -First 15 | ForEach-Object { Write-Host "      $($_.Line.Trim())" }
        Write-Host ''
        Write-Host   '      Pick the vwifi row and re-run with either:'
        Write-Host   '        -Adapter <substring of its name>'
        Write-Host   '        -MiniportHandle <the hex handle>'
    }
    Write-Host ''
    Write-Host "      Full output: $LogFile"
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

if ($MiniportHandle) {
    # Explicit handle: skip discovery entirely. The point is that
    # reading the table yourself always works, whatever this script's
    # matching does or does not manage.
    $handle = $MiniportHandle -replace '^0x', ''
    Write-Host "      miniport handle (given): $handle"
} else {
    $handle = Find-MiniportHandle -Lines $out1
    if (-not $handle) {
        Show-MiniportDiagnosis -Lines $out1 -LogFile $log1
        return
    }
    Write-Host "      miniport handle: $handle"
}
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
