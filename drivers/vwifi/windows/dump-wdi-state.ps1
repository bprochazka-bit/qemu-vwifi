<#
.SYNOPSIS
    Dump what the WDI port driver thinks our adapter and port are, using
    !ndiskd in a local kernel debugger.

.DESCRIPTION
    OID_WDI_TASK_CONNECT has never reached this miniport. wdiwifi.sys
    accepts OID_DOT11_CONNECT_REQUEST, returns success, and about two
    milliseconds later reports STATUS_NETWORK_UNREACHABLE back up the
    stack.

    Connecting to an SSID that exists nowhere fails identically -- same
    status, same timing, the same OIDs in the same order. That was once
    read here as proof that the refusal happens before any BSS is
    considered. It is not: the control separated the two cases in
    TIMING, not in CAUSE, and _ROAM_TRACELOGGING_DATA later showed
    bssCandidateCount 1. A BSS is considered, matched, and the connect
    job ends anyway.

    So what this collects is the state we cannot otherwise see: how
    wdiwifi has recorded this adapter and its ports, what it parsed out
    of the frames we handed it, and -- since public PDBs carry function
    names even without type layout -- the code of the routines that
    decide. All of it static: no reproduction to time, no race to catch.

    That method has now found and closed one link of the chain. The scan
    job's completion status was the task OID's return value, so
    WfcPortPropertyGoodScanStartTime was never recorded and the connect
    job discarded a candidate list of exactly one. With
    OID_WDI_TASK_SCAN returning NDIS_STATUS_PENDING and completed when
    the sweep really ends, the property is written, the candidate
    survives, and CConnectJob::StartConnectRoamTask runs.

    OID_WDI_TASK_CONNECT still does not arrive, so the same two tools
    now point one function further in: pass five disassembles the
    connect-task chain and prints the host-side trace command for its
    entry points.

    Pass one lists miniports, because everything after needs a handle
    only it can supply. Pass two dumps the miniport. Pass three dumps
    wdiwifi's own CAdapter, whose pointer only pass two prints. Pass
    four resolves the breakpoint addresses for the scan and candidate
    path. Pass five does the same for the connect task, with the
    disassembly beside it.

    There is no WPP pass. wdiwifi's WPP is compiled in -- every routine
    is threaded with WPP_RECORDER_INITIALIZED checks and
    WPP_RECORDER_SF_* calls -- but rcdrkd reports "<Empty log>" for it,
    both by module name and by file name, on a machine that had just
    failed a connect. The recorder buffer is never written, so those
    checks are taking the skip branch at runtime and there is nothing
    to read. Do not spend another round on it.

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

# ndiskd.dll does not sit beside kd.exe -- Debugging Tools puts its
# extensions in subdirectories (winxp, winext, w2k), and ndiskd lives in
# winxp. `.load ndiskd` by bare name relies on the debugger's extension
# search path being right, which is exactly the thing that fails when
# the tools have been copied around rather than installed. Finding the
# DLL and loading it by full path removes the guess.
function Find-KdExtension {
    param(
        [Parameter(Mandatory)][string] $KdPath,
        [Parameter(Mandatory)][string] $Name
    )
    # Recursive, because the extensions do not sit next to kd.exe -- the
    # run that found no ndiskd.dll was looking only in kd's own folder,
    # and it lives one level down in winxp\.
    $root = Split-Path $KdPath -Parent
    $hit = Get-ChildItem -Path $root -Filter $Name -Recurse -File -ErrorAction SilentlyContinue |
           Sort-Object FullName | Select-Object -First 1
    if ($hit) { return $hit.FullName }
    return $null
}

# 8.3 form, so a path under "Program Files (x86)" carries no spaces into
# kd's -c string. Quoting inside that string has to survive PowerShell's
# native-argument handling, which is not worth relying on when the short
# name sidesteps it entirely.
function Get-ShortPath {
    param([Parameter(Mandatory)][string] $Path)
    try {
        $fso = New-Object -ComObject Scripting.FileSystemObject
        $sp  = $fso.GetFile($Path).ShortPath
        if ($sp -and $sp -notmatch '\s') { return $sp }
    } catch { }
    return $Path
}

# One native argument, quoted so the CRT puts it back together as one.
#
# Start-Process -ArgumentList takes EITHER an array, which it joins with
# spaces and does NOT quote, or a single string used verbatim. The array
# form turned
#
#   -c ".load ...;.chain;!ndiskd.miniports"
#
# into a bare `-c .load ...` followed by loose words, and kd answered
# with its usage text. PowerShell's call operator (& exe @args) quotes
# each element itself, which is why the previous version worked and why
# this one has to do the quoting by hand.
function ConvertTo-QuotedArg {
    param([Parameter(Mandatory)][AllowEmptyString()][string] $Value)
    # A trailing backslash immediately before the closing quote is read
    # as an escaped quote, which would swallow it and everything after.
    $v = $Value -replace '\\+$',''
    if ($v -match '"') {
        throw "cannot pass an argument containing a double quote: $v"
    }
    return '"' + $v + '"'
}

function Invoke-Kd {
    param(
        [Parameter(Mandatory)][string] $Kd,
        [Parameter(Mandatory)][string] $Sym,
        [Parameter(Mandatory)][string] $Commands,
        [Parameter(Mandatory)][string] $LogFile,
        [int] $TimeoutSec = 180,
        [switch] $Append
    )
    # -kl local kernel debug, and a trailing q so the session exits
    # instead of sitting at a prompt no one is at.
    #
    # -logo truncates, -loga appends. Appending is what lets one pass be
    # split across several kd invocations into one readable file.
    $logSwitch = if ($Append) { '-loga' } else { '-logo' }

    $argLine = @(
        '-kl',
        '-y',       (ConvertTo-QuotedArg $Sym),
        $logSwitch, (ConvertTo-QuotedArg $LogFile),
        '-c',       (ConvertTo-QuotedArg "$Commands;q")
    ) -join ' '

    # kd's console output goes to a file beside the log rather than the
    # screen: it duplicates the log and would bury the progress lines.
    # It is kept because it is the ONLY place a failure to start shows
    # up -- kd printing its usage text writes no log at all, so an empty
    # log plus a console capture is how that gets diagnosed instead of
    # looking like a debugger that found nothing.
    $conFile = "$LogFile.console.txt"

    # Under a timeout, because kd does not always take the trailing q.
    # One malformed command is enough: `x ndis!*Wdi*` left the session
    # echoing the whole -c string back as an unresolved symbol and
    # waiting at a prompt forever, with the script blocked on a process
    # that was never going to exit. A dump tool that can hang the
    # machine it is diagnosing is worse than one that returns nothing.
    $proc = Start-Process -FilePath $Kd -PassThru -NoNewWindow `
                          -RedirectStandardOutput $conFile `
                          -ArgumentList $argLine
    if (-not $proc.WaitForExit($TimeoutSec * 1000)) {
        Write-Warning "kd did not exit within ${TimeoutSec}s -- killing it."
        Write-Warning "  command: $Commands"
        Write-Warning '  whatever it managed to write is still in the log.'
        try { $proc.Kill() } catch { }
        try { $proc.WaitForExit(5000) | Out-Null } catch { }
    }

    if ((-not (Test-Path $LogFile)) -or ((Get-Item $LogFile).Length -eq 0)) {
        Write-Warning 'kd wrote no log. Its console output was:'
        if (Test-Path $conFile) {
            Get-Content $conFile -TotalCount 15 | ForEach-Object { Write-Warning "  $_" }
        } else {
            Write-Warning '  (nothing)'
        }
        Write-Warning "  argument line was: $argLine"
        return @()
    }
    return Get-Content $LogFile
}

# Run commands one kd invocation at a time, appending to one log.
#
# Slower than a single ';'-joined string, and worth it wherever a
# command might not come back: one that hangs or derails the parser
# takes down only itself, and every command before it is already on
# disk. Used for exploratory passes; settled ones stay batched.
#
# EVERY invocation gets $Prologue, and for wdiwifi that has to be
# `.reload /f wdiwifi.sys`. A local kernel session does not have the
# module in its list until something forces it -- `lm vm wdiwifi`
# printed its header and no rows, and `x wdiwifi!CScanJob::FinishJob`
# answered "Couldn't resolve 'x wdiwifi'" with the caret on the `!`,
# because the module name resolved to nothing. Batched passes got the
# reload once at the front and every later command rode on it; splitting
# them up took that away from all of them.
#
# The parse error is also why the timeout fired: kd never reached the
# trailing q, so it sat at the prompt for the full 60 seconds and had to
# be killed, four times over.
function Invoke-KdEach {
    param(
        [Parameter(Mandatory)][string]   $Kd,
        [Parameter(Mandatory)][string]   $Sym,
        [Parameter(Mandatory)][string[]] $Commands,
        [Parameter(Mandatory)][string]   $LogFile,
        [string] $Prologue = '',
        [int] $TimeoutSec = 60
    )
    Remove-Item $LogFile -ErrorAction SilentlyContinue
    $first = $true
    foreach ($c in $Commands) {
        $full = if ($Prologue) { "$Prologue;$c" } else { $c }
        Write-Host "      kd: $c"
        (Invoke-Kd -Kd $Kd -Sym $Sym -LogFile $LogFile -TimeoutSec $TimeoutSec `
                   -Commands $full -Append:(-not $first)) | Out-Null
        $first = $false
    }
    if (Test-Path $LogFile) { return Get-Content $LogFile }
    return @()
}

function Find-MiniportHandle {
    param([string[]] $Lines)

    # The table is
    #
    #     Driver             NetAdapter          Name
    #     ffffd008e41f0020   ffffd008db5e11a0    vwifi virtual Wi-Fi adapter
    #
    # so there are TWO kernel pointers per row and they are different
    # objects. Taking the first one handed the Driver to commands that
    # validate a NetAdapter, which answered "is not a valid NetAdapter"
    # -- a message that sounds like the adapter is broken when in fact
    # the wrong column was read.
    #
    # The column order is taken from the header rather than assumed,
    # since being wrong about it is silent and looks like a device fault.
    $netIdx = 1
    foreach ($l in $Lines) {
        if ($l -match 'Driver' -and $l -match 'NetAdapter') {
            $netIdx = if ($l.IndexOf('NetAdapter') -lt $l.IndexOf('Driver')) { 0 } else { 1 }
            break
        }
    }

    foreach ($l in $Lines) {
        if ($l -notmatch [regex]::Escape($Adapter)) { continue }
        $ptrs = @([regex]::Matches($l, '\b(f{4}[0-9a-fA-F]{12})\b') | ForEach-Object { $_.Groups[1].Value })
        if ($ptrs.Count -eq 0) { continue }
        $net = if ($ptrs.Count -gt $netIdx) { $ptrs[$netIdx] } else { $ptrs[-1] }
        $drv = if ($ptrs.Count -gt 1) { $ptrs[1 - $netIdx] } else { $ptrs[0] }
        return [pscustomobject]@{ NetAdapter = $net; Driver = $drv }
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

    # `.chain` lists the loaded extension DLLs, so whether ndiskd is
    # actually in the debugger is a fact in the output rather than an
    # inference from error text. That distinction matters: the previous
    # version keyed off "Unable to load" and blamed a missing extension
    # for what was really a malformed -c string.
    $chainHasNdiskd = $Lines | Select-String -Pattern 'ndiskd' -SimpleMatch
    $loadFailed     = $Lines | Select-String -Pattern 'Unable to load|No export|is not a valid'
    $anyRows        = $Lines | Select-String -Pattern '\b[0-9a-fA-F]{12,16}\b'

    Write-Host ''
    if (-not $Lines -or $Lines.Count -lt 5) {
        Write-Warning "kd produced almost nothing. Check $LogFile -- local KD may not be active."
    } elseif ($loadFailed -or -not $chainHasNdiskd) {
        Write-Warning 'ndiskd is not loaded in the debugger.'
        Write-Warning 'It ships in a subdirectory beside kd.exe (Debuggers\x64\winxp\ndiskd.dll),'
        Write-Warning 'so copying kd.exe on its own is not enough -- the whole folder is needed.'
        if ($loadFailed) { $loadFailed | Select-Object -First 3 | ForEach-Object { Write-Host "      $_" } }
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
Write-Host "[1/6] kd: $kd"

$state = Test-LocalKdEnabled
if (-not $state.DebugOn -or -not $state.TypeLocal) {
    throw "local kernel debugging is not enabled (debug=$($state.DebugOn), local=$($state.TypeLocal)). Run with -Enable, then reboot."
}

New-Item -ItemType Directory -Force -Path $OutDir | Out-Null
$OutDir = (Resolve-Path $OutDir).Path
$sym    = Get-SymbolPathArg

# Pass one: what miniports exist, and does ndiskd work at all.
$log1 = Join-Path $OutDir 'wdi-state-miniports.txt'
Write-Host '[2/6] Listing miniports'
# Semicolons, not newlines. kd's -c takes ONE command string with ';'
# separators; a multi-line string is not parsed as successive commands,
# which is why the first version of this loaded nothing and then blamed
# a missing extension for it.
$ndiskd = Find-KdExtension -KdPath $kd -Name 'ndiskd.dll'
$load   = if ($ndiskd) {
    Write-Host "      ndiskd: $ndiskd"
    ".load $(Get-ShortPath $ndiskd)"
} else {
    Write-Warning 'ndiskd.dll not found near kd.exe; relying on the extension search path'
    '.load ndiskd'
}

$out1 = Invoke-Kd -Kd $kd -Sym $sym -LogFile $log1 `
            -Commands (@($load, '.chain',
                         '!ndiskd.miniports',
                         '!ndiskd.netadapter') -join ';')

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
    # The Driver is a different object and must still be looked up. It
    # used to default to $handle, which meant !ndiskd.minidriver was
    # handed a NetAdapter and printed a handler table decoded from the
    # wrong struct -- entries like "SendNetBufferListsHandler
    # ndis!NetDmaDeregisterProvider" and "PauseHandler 00000001", which
    # look like a broken driver rather than a bad argument.
    $found  = Find-MiniportHandle -Lines $out1
    $driver = if ($found) { $found.Driver } else { $null }
    if ($driver) {
        Write-Host "      NetAdapter (given): $handle   Driver: $driver"
    } else {
        Write-Warning "NetAdapter $handle given, but no Driver found in pass one -- skipping -handlers"
    }
} else {
    $found = Find-MiniportHandle -Lines $out1
    if (-not $found) {
        Show-MiniportDiagnosis -Lines $out1 -LogFile $log1
        return
    }
    $handle = $found.NetAdapter
    $driver = $found.Driver
    Write-Host "      NetAdapter: $handle   Driver: $driver"
}
# Pass two: everything worth knowing about that miniport. Each command
# is independent, so one that this ndiskd build does not have costs its
# own output and nothing else.
$log2 = Join-Path $OutDir 'wdi-state-detail.txt'
Write-Host '[3/6] Dumping adapter, port and WDI state'
# Two commands were removed after a run showed them to be inventions
# of mine rather than things this ndiskd has: `!ndiskd.miniports -wdi`
# answered "Unknown parameter wdi", and `!ndiskd.pendingoids` answered
# "ndiskd has no pendingoids export". `!ndiskd.oid` is the real one and
# does report pending and queued OIDs.
#
# !ndiskd.wdiminidriver is kept but is not expected to work: it
# dereferences wdiwifi!CMiniportDriver, a C++ type that only exists in
# private symbols, and Microsoft's public PDB has no type information.
# It stays because its failure is informative and costs one line.
$cmds = @(
    "!ndiskd.netadapter $handle",
    "!ndiskd.miniport $handle -ports",
    $(if ($driver) { "!ndiskd.minidriver $driver -handlers" } else { '.echo no-driver-handle' }),
    "!ndiskd.miniport $handle -log",
    "!ndiskd.oid -miniport $handle",
    'lm vm ndis',
    'lm vm wdiwifi',
    'lm vm vwifi'
) -join ';'
$out2 = Invoke-Kd -Kd $kd -Sym $sym -LogFile $log2 -Commands $cmds

# Pass three: wdiwifi's own per-adapter object.
#
# !ndiskd.netadapter prints "WDI state <ptr>", which is a
# wdiwifi!CAdapter -- the port driver's own record of this adapter, and
# the closest thing to the state that decides whether a connect is
# attempted. It is only knowable after pass two has run, hence a third
# pass rather than more commands in the second.
#
# Treat what comes back with suspicion. Public PDBs carry no C++ type
# layout, and pass two already showed what that looks like:
# !ndiskd.wdiminidriver, handed the handle ndiskd itself suggests,
# reported DRIVER_OBJECT 0, "Ndis API version v0.0", "WDI API version
# 0.0.0" and a "WDI adapter" pointer that is really the VirtIO Ethernet
# driver's handle. Those are not the port driver's real values; they are
# what reading a struct with no layout information produces.
$wdiState = $null
foreach ($l in $out2) {
    $m = [regex]::Match($l, 'WDI state\s+([0-9a-fA-F]{8,16})')
    if ($m.Success) { $wdiState = $m.Groups[1].Value; break }
}

$log3 = Join-Path $OutDir 'wdi-state-adapter.txt'
if ($wdiState) {
    Write-Host "      WDI state (wdiwifi!CAdapter): $wdiState"
    # -r1, not -r2. CAdapter carries m_DebugHungDeviceCommand[184] and
    # m_DebugHungTask[472]; at depth 2 dx prints those byte by byte and
    # the output runs out before reaching the members after them -- which
    # is where the port list lives. Depth 1 names every member, then the
    # interesting ones are expanded individually.
    #
    # dt is gone: it answered "Symbol wdiwifi!CAdapter not found" while
    # dx resolved the same type happily, so dx is what the public PDB
    # actually supports.
    $a = "(wdiwifi!CAdapter*)0x$wdiState"
    # Flink MINUS 8. dt says CBSSEntry puts __VFN_table at +0x000 and
    # m_BssListEntry at +0x008, and _CPP_LIST_ENTRY leads with its
    # LIST_ENTRY, so the list threads through +0x008 and the object
    # starts eight bytes earlier. Casting Flink straight to CBSSEntry*
    # shifted every field by eight and produced a confident, entirely
    # fictional entry -- Rssi 6226015, unreadable frame pointers,
    # channel 0. Cast through unsigned char* so the subtraction is in
    # bytes; subtracting from a _LIST_ENTRY* would step 16 per unit.
    $e = "(wdiwifi!CBSSEntry*)((unsigned char*)(($a)->m_ExtStaBSSList.m_BSSEntryList.Flink) - 8)"
    # .reload /f forces wdiwifi's symbols in before anything asks dx for
    # its types. Module symbols load lazily and dx does a qualified type
    # lookup that does NOT trigger the load -- it just answers "Unable to
    # find module 'wdiwifi' for qualified type lookup". An earlier run
    # only worked by accident: it happened to run `dt wdiwifi!CAdapter`
    # first, which failed with "Symbol not found" but pulled the symbols
    # in as a side effect. Dropping the dt removed the side effect and
    # every dx failed.
    $out3 = Invoke-Kd -Kd $kd -Sym $sym -LogFile $log3 -Commands (@(
        $load,
        '.reload /f wdiwifi.sys',
        'lm vm wdiwifi',
        "dx -r1 $a",

        # The BSS entry as wdiwifi parsed it. Every field it extracted
        # from our beacon is right: BSSID, Rssi -30, channel 11,
        # WDI_BAND_ID_2400, centre 2462, ESS set, capability 0x421,
        # BlockedReasons 0, both frame bodies present. So the entry is
        # not malformed and this dump is no longer the open question --
        # it is here only to confirm the same entry is still the one
        # being talked about.
        "dx -r1 ($a)->m_ExtStaBSSList",
        "dx -r2 $e",

        # The bytes themselves, which no dx of a parsed struct can
        # stand in for. m_BeaconFrameBody and m_ProbeResponseFrameBody
        # are what wdiwifi's own IE walker reads; if an IE is
        # mis-ordered, over-long, or missing something ranking needs,
        # it shows here and nowhere else. 256 bytes covers both bodies
        # (0x65 and 0x5f) with room to see where they stop.
        "db @@c++(($e)->m_BeaconFrameBody.pBuffer) L0n256",
        "db @@c++(($e)->m_ProbeResponseFrameBody.pBuffer) L0n256",

        # What the port driver kept from OID_WDI_GET_ADAPTER_CAPABILITIES
        # and OID_WDI_TASK_CREATE_PORT. m_CachedRank on the entry is
        # 0x64, so ranking itself works -- which points away from the
        # entry and towards the profile it is being ranked against.
        "dx -r2 ($a)->m_AdapterPropertyCache",
        "dx -r2 ($a)->m_DatapathCapabilities",
        "dx -r1 ($a)->m_pPortList[0]",
        "dx -r2 ($a)->m_pPortList[0]->m_PortPropertyCache",

        # Connect history, in case something here is refusing the BSSID
        # before ranking ever runs. m_DisallowedBssidCount is 0 and
        # m_BlockedInfo.BlockedReasons is 0, so if a refusal is
        # remembered anywhere it is in one of these two.
        "dx -r2 ($a)->m_pPortList[0]->m_pConnectHistory",
        "dx -r2 ($a)->m_pPortList[0]->m_pBssidConnectHistory",
        "dx -r1 ($a)->m_pPortList[0]->m_pRoamTraceLoggingData",

        # wdiwifi's own vocabulary for the verdict. roamDebugCode came
        # back WdiRoamDebugCodeNotSet and roamConfigFlags 0x2021, and
        # neither means anything without the enumerator lists -- which
        # the public PDB does carry, because dx printed the names.
        'dt wdiwifi!_WDI_ROAM_DEBUG_CODE',
        'dt wdiwifi!_WDI_ROAM_CONFIGURATION_FLAGS',
        'dt wdiwifi!_WFC_PORT_DOT11_STATE',
        'dt wdiwifi!_WFC_ROAM_CONNECT_TRIGGER',
        'dt wdiwifi!_WDI_ASSOC_STATUS',

        # Function symbols, which public PDBs do have even though they
        # carry no C++ type layout. Kept because the names are how pass
        # four knows what to disassemble, and because a wdiwifi update
        # can rename or split these.
        'x wdiwifi!*Candidate*',
        'x wdiwifi!*Rank*',
        'x wdiwifi!*RoamReconnect*'
    ) -join ';')
    Write-Host "      $log3 ($($out3.Count) lines)"

    # Pass four: what happens after the candidate is chosen.
    #
    # Candidate selection is now read end to end and it WORKS.
    #
    # FindMatchingBSSEntriesForConnect has exactly two failure returns:
    # 0xC001001B when no port in m_pPortList[0..4] has a m_WfcPortType
    # matching the list manager's m_PortType, and 0xC000000D when the
    # criteria or the count pointer is null. Both leave before *pCount
    # is ever written. Every other path falls through to
    #
    #     mov [r15],ebx        ; *pCount = matched
    #     ...
    #     mov eax,r12d         ; r12d still 0
    #
    # and returns SUCCESS. So bssCandidateCount 1 is proof the matcher
    # succeeded: the count is only written on the path that returns 0.
    # CBSSEntry::IsMatchingBssEntryForConnect accepted our entry,
    # CalculateRank scored it, qsort ordered it.
    #
    # Which also disposes of the two fields this was chasing.
    # bestCandidateRank is written at CheckAndUpdateCandidates+0x4c0 and
    # roamAPRankIndex on the roam paths, both inside a block guarded by
    # roamConfigFlags bit 8, RC_CHECK_GOOD_ENOUGH_AP. Our flags are
    # 0x2021, bit 8 clear, so the block is skipped and both fields stay
    # at their initial 0 and 0xffffffff. They were never symptoms; a
    # first-time connect simply does not fill them in.
    #
    # roamOccured false is not evidence either. It is set on the success
    # path, but every dump is taken well after the attempt and a later
    # CRoamReconnectJob::Initialize re-stamps the struct -- which is why
    # connectTrigger and roamConfigFlags are populated in a dump whose
    # timestamps predate the scan sitting next to them.
    #
    # The connect job discards its own candidate list.
    #
    # CheckAndStartConnectProcess, at +0x1a3, immediately after
    # CheckAndUpdateCandidates returns success:
    #
    #     cmp  dword ptr [rbx+26Ch],r13d     ; job candidateCount vs 0
    #     jbe  +0x65f                        ; zero -> skip the connect
    #   +0x1b0:
    #     inc  dword ptr [rbx+6E8h]          ; attempt++
    #     mov  dword ptr [r12],r13d          ; *pAssocStatus = SUCCESS
    #     ...
    #   +0x5d9:
    #     call CConnectJob::StartConnectRoamTask   ; OID_WDI_TASK_CONNECT
    #
    # +0x65f, with roamConfigFlags bit 3 clear, falls straight through to
    # the epilogue. So a zero candidate count means the connect task is
    # never started -- exactly what the driver sees. And *pAssocStatus is
    # still 6 from the entry check at +0xa1, WDI_ASSOC_STATUS_
    # CANDIDATE_LIST_EXHAUSTED, which is a much better description of
    # this failure than anything considered so far.
    #
    # But PickCandidates wrote 1 into that count. CheckAndUpdateCandidates
    # zeroes it, and with roamConfigFlags 0x2021 exactly one of its three
    # zeroing paths can be reached:
    #
    #   +0x244  test byte ptr [rbx+260h],1        ; RC_CONNECT, set
    #   +0x251  cmp  dword ptr [rbx+218h],0
    #           jne  +0x2af
    #   +0x277  now - port->m_ullLastNloDiscoverTime < 1s ?
    #           m_ullLastNloDiscoverTime is 0 in every dump, so no
    #   +0x2af  GetPropertyBuffer(portPropertyCache, 0x47, &buf, &len)
    #           cmp  eax,0C0000184h               ; STATUS_INVALID_DEVICE_STATE
    #           jne  +0x396                       ; property readable -> keep
    #   +0x2d7  <WPP id 0x9a>
    #   +0x317  mov  dword ptr [rbx+26Ch],r13d    ; candidateCount = 0
    #
    # So: port property 0x47 fails to read with STATUS_INVALID_DEVICE_
    # STATE, and wdiwifi throws away a perfectly good candidate list.
    # Everything else -- the beacon, the ranking, the matching -- was
    # never the problem.
    #
    # Property 0x47 is WfcPortPropertyGoodScanStartTime, and it is not
    # populated. The chain is now read end to end with nothing inferred:
    #
    #   GetPropertyEntryForPropertyName:
    #     if (!outPtr || name >= m_PropertyNameMax)  -> 0xC000000D
    #     entry = m_PropertyTable + name*0x38
    #     if (!requirePopulated)      -> return entry
    #     if (entry->IsPopulated)     -> return entry
    #     if (entry->pDefaultEntry)   -> return the default
    #     <WPP 0x14>  ebx = 0xC0000184   ; STATUS_INVALID_DEVICE_STATE
    #
    # and entry[0x47] reads IsPopulated false with pDefaultEntry 0 --
    # exactly the pair that produces 0xC0000184. GetPropertyBuffer hands
    # that back verbatim, CheckAndUpdateCandidates matches it and zeroes
    # the candidate count, and CheckAndStartConnectProcess skips
    # StartConnectRoamTask, so OID_WDI_TASK_CONNECT never goes out and
    # *pAssocStatus is left at WDI_ASSOC_STATUS_CANDIDATE_LIST_EXHAUSTED.
    #
    # Static reading has gone as far as it goes.
    #
    # PROVEN out of the binary: WfcPortPropertyGoodScanStartTime (port
    # property 71) is unpopulated -> CheckAndUpdateCandidates zeroes the
    # candidate count -> CheckAndStartConnectProcess skips
    # StartConnectRoamTask -> OID_WDI_TASK_CONNECT is never sent.
    #
    # Its only writer is CScanJob::FinishJob, behind three gates:
    #
    #   +0x98  test ebp,ebp                    ; status
    #          jne  skip
    #   +0xa6  cmp  byte ptr [rbx+78Ah],bpl    ; m_bCancelled
    #          jne  skip
    #   +0xaf  resolve the port property cache for [rbx+34h]
    #          null -> skip
    #
    # FinishJob demonstrably runs -- it clears m_ScanInProgress at
    # +0x55, ahead of every gate, and that reads false in every dump --
    # so the scan job completes and one of the three rejects it. WHICH
    # one is a runtime fact, and local kernel debugging cannot break.
    #
    # Ruled out along the way, so nobody re-runs them: the M1 addressing
    # (transaction ids and port ids echo exactly, per the driver's own
    # OID M1 trace); the M2 (now written, no change); and the Wdi_Ndis*
    # entry points (we call NdisMRegisterWdiMiniportDriver out of
    # ndis.lib, and the CREATE_PORT completion demonstrably drives the
    # port driver's state machine forward, so plain NdisMIndicateStatusEx
    # does reach the task machinery).
    #
    # ONE breakpoint was not enough, and the way it was not enough is
    # worth recording. The first host-side harness broke on FinishJob,
    # printed one hit and detached. It reported status 0x40230001,
    # STATUS_NDIS_INDICATION_REQUIRED -- the driver's own task return
    # value -- and that was read as "gate 1 fails, always". A single
    # sample cannot say "always": FinishJob runs more than once per job,
    # and the call driven by our SCAN_COMPLETE indication was never in
    # it. Acting on that reading cost a build and broke the scan job's
    # lifetime, without touching the connect failure.
    #
    # So this pass prints FOUR addresses, and the harness that consumes
    # them stays attached and reports every hit:
    #
    #   CScanJob::FinishJob                       is the property
    #                                             written, and on which
    #                                             call
    #   CConnectJob::CheckAndUpdateCandidates     +0x317, the store that
    #                                             discards the list
    #   CConnectJob::CheckAndStartConnectProcess  +0x1a3, the compare
    #                                             that reads the count
    #   CConnectJob::StartConnectRoamTask         did the connect task
    #                                             go out at all
    #
    # Four is also the ceiling: they have to be hardware breakpoints
    # (a software one writes 0xCC into wdiwifi and PatchGuard bugchecks
    # for that) and x86 has four debug registers.
    #
    # One command per kd invocation. The previous batched pass hung:
    # `x ndis!*Wdi*` derailed kd's parser, it echoed the whole -c string
    # back as an unresolved symbol, and sat at a prompt forever with the
    # script blocked behind it. Split, a bad command costs only itself.
    Write-Host '[4/6] Locating the breakpoints'
    $log4 = Join-Path $OutDir 'wdi-state-decide.txt'

    # Ordered, because the composed command line below reads them back
    # by name and the order it prints them in should be stable.
    $wanted = [ordered]@{
        'CScanJob::FinishJob'                      = '--finish-job'
        'CConnectJob::CheckAndUpdateCandidates'    = '--update-candidates'
        'CConnectJob::CheckAndStartConnectProcess' = '--start-connect'
        'CConnectJob::StartConnectRoamTask'        = '--roam-task'
    }

    # Built up rather than written as one parenthesised expression: a
    # comment between a line-trailing `+` and its operand is legal but
    # not worth relying on in a script nobody can syntax-check on the
    # host it is written on.
    $cmds4 = @('lm vm wdiwifi')
    foreach ($name in $wanted.Keys) { $cmds4 += "x wdiwifi!$name" }
    # The M3's own landing point. If FinishJob turns out never to be
    # called a second time, this is the function that received the
    # SCAN_COMPLETE and decided not to finish the job, and it is where
    # the next disassembly starts.
    $cmds4 += 'x wdiwifi!CScanJob::CompleteScanTask'

    $out4 = Invoke-KdEach -Kd $kd -Sym $sym -LogFile $log4 `
                          -Prologue '.reload /f wdiwifi.sys' -Commands $cmds4
    Write-Host "      $log4 ($($out4.Count) lines)"

    # kd prints `fffff804`505d02bc wdiwifi!Sym (private: void ...)`.
    # The tick is kd's 32-bit grouping separator and has to come out
    # before anything else can use the number.
    function Get-KdSymbol {
        param([string[]] $Lines, [string] $Name)
        $rx = '^([0-9a-f`]{8,})\s+wdiwifi!' + [regex]::Escape($Name) + '\b'
        foreach ($l in $Lines) {
            $m = [regex]::Match($l, $rx)
            if ($m.Success) { return '0x' + ($m.Groups[1].Value -replace '`','') }
        }
        return $null
    }

    # Say something either way. A silent no-op here reads exactly like a
    # pass that worked, which is how two rounds were lost.
    $found   = [ordered]@{}
    $missing = @()
    foreach ($name in $wanted.Keys) {
        $a = Get-KdSymbol -Lines $out4 -Name $name
        if ($a) { $found[$name] = $a } else { $missing += $name }
    }

    Write-Host ''
    if ($found.Count -gt 0) {
        Write-Host '      BREAKPOINT ADDRESSES (this boot only -- wdiwifi relocates)'
        foreach ($name in $found.Keys) {
            Write-Host ("        {0,-42} {1}" -f $name, $found[$name])
        }
        Write-Host ''
        Write-Host '      On the Linux host, with the guest started with -gdb tcp::1234:'
        Write-Host ''
        $parts = @()
        foreach ($name in $found.Keys) { $parts += ("{0} {1}" -f $wanted[$name], $found[$name]) }
        Write-Host ("        scripts/gdb-wdi-connect.sh " + ($parts -join ' '))
        Write-Host ''
        Write-Host '      Then attempt the connection in the guest. Ctrl-C, then'
        Write-Host '      "detach" and "quit", once it has failed.'
    }
    if ($missing.Count -gt 0) {
        Write-Warning ("not found in the pass-four output: " + ($missing -join ', '))
        Write-Warning "Look in $log4 for lines like"
        Write-Warning '  fffff804`505d02bc wdiwifi!CScanJob::FinishJob (private: void ...)'
        Write-Warning 'and pass those addresses to scripts/gdb-wdi-connect.sh by hand.'
        Write-Warning 'If instead it says "Couldnt resolve x wdiwifi", the module'
        Write-Warning 'symbols did not load and the .reload prologue is not working.'
    }

    # Pass five: the connect task itself.
    #
    # The four breakpoints above have done their job and the answer they
    # gave moved the question. With OID_WDI_TASK_SCAN returning
    # NDIS_STATUS_PENDING:
    #
    #   [1] CScanJob::FinishJob  status=0x0            (was 0x40230001)
    #   [2] CheckAndUpdateCandidates  never fired      (was 1 -> 0)
    #   [3] CheckAndStartConnectProcess  candidates=1  (was 0)
    #   [4] StartConnectRoamTask -- CONNECT TASK GOING OUT
    #
    # The good-scan chain is closed. But OID_WDI_TASK_CONNECT still does
    # not reach the driver: the debugcon trace for the same attempt runs
    # DOT11_RESET, SET_PRIVACY_EXEMPTION_LIST, three GET_STATISTICS, a
    # full TASK_SCAN, one more GET_BSS_ENTRY_LIST -- and stops. So
    # StartConnectRoamTask is entered and gives up somewhere between
    # there and the OID.
    #
    # This is exactly the position the good-scan chain was in one round
    # ago, and it yielded to the same two things together: the branches,
    # from a disassembly local KD can take without stopping the machine,
    # and which branch ran, from a host-side trace. So this pass writes
    # `uf` for the whole chain and prints a --at command line for its
    # entry points.
    #
    # Four entry points, because four is all the debug registers allow.
    # The last label the trace prints is how far the chain got; the
    # disassembly beside it says what the next branch tests.
    Write-Host '[5/6] Disassembling the connect task chain'
    $log5 = Join-Path $OutDir 'wdi-state-connecttask.txt'

    # Ordered by where each sits in the chain, so the printed --at line
    # reads in call order and the trace can be compared against it
    # directly.
    $chain = @(
        'CConnectJob::StartConnectRoamTask',
        'CConnectJob::FillConnectRoamTaskParameters',
        'CConnectJob::GenerateConnectTaskTlv',
        'CConnectJob::GenerateRoamTaskTlv'
    )

    # `x` before `uf`. uf's output starts with a bare label line and then
    # raw instructions, none of which carry the `ADDR wdiwifi!Name` shape
    # Get-KdSymbol matches -- so the disassembly alone would leave the
    # --at line with no addresses to print.
    $cmds5 = @()
    foreach ($name in $chain) { $cmds5 += "x wdiwifi!$name" }
    foreach ($name in $chain) { $cmds5 += "uf wdiwifi!$name" }
    # Read but not broken on: these are the rest of what the chain
    # touches, and an early return in the middle of one of them is the
    # kind of thing only the disassembly shows.
    $cmds5 += 'uf wdiwifi!CConnectJob::FillConnectionProfileParameters'
    $cmds5 += 'uf wdiwifi!CConnectJob::ReorderAssociationPhyTypesList'
    $cmds5 += 'uf wdiwifi!CConnectJob::CheckAndStartConnectProcess'
    # Whatever actually posts a task OID. The name is not known yet and
    # these three patterns are how to find it -- once it is known, it
    # becomes the fourth breakpoint and "did the OID even get built"
    # stops being a guess.
    $cmds5 += 'x wdiwifi!*DeviceCommand*'
    $cmds5 += 'x wdiwifi!*SendOid*'
    $cmds5 += 'x wdiwifi!*OidJob*'

    $out5 = Invoke-KdEach -Kd $kd -Sym $sym -LogFile $log5 `
                          -Prologue '.reload /f wdiwifi.sys' -Commands $cmds5 `
                          -TimeoutSec 120
    Write-Host "      $log5 ($($out5.Count) lines)"

    $chainFound = [ordered]@{}
    foreach ($name in $chain) {
        $a = Get-KdSymbol -Lines $out4 -Name $name
        if (-not $a) { $a = Get-KdSymbol -Lines $out5 -Name $name }
        if ($a) { $chainFound[$name] = $a }
    }

    Write-Host ''
    if ($chainFound.Count -gt 0) {
        Write-Host '      CONNECT TASK TRACE (this boot only)'
        Write-Host ''

        # The --at pass over these four entry points has already run, and
        # it said the chain IS entered: StartConnectRoamTask ->
        # GenerateConnectTaskTlv -> FillConnectRoamTaskParameters, with
        # GenerateRoamTaskTlv correctly skipped for a connect. Entry
        # points cannot say what any of them RETURNED, which is the
        # remaining question, so the command printed first is the
        # return-side one.
        $gen  = $chainFound['CConnectJob::GenerateConnectTaskTlv']
        $roam = $chainFound['CConnectJob::StartConnectRoamTask']
        if ($gen -and $roam) {
            Write-Host ("        scripts/gdb-wdi-connect.sh --tlv-stages {0} --roam-status {1}" -f $gen, $roam)
            Write-Host ''
            Write-Host '      Three rungs inside GenerateConnectTaskTlv, each on the path'
            Write-Host '      taken when one of its three calls returned zero, plus the'
            Write-Host '      status StartConnectRoamTask ends up returning. The last rung'
            Write-Host '      that fires names the call that failed. Four breakpoints, which'
            Write-Host '      is the whole budget.'
        }

        Write-Host ''
        Write-Host '      Entry points only, if the chain needs re-walking:'
        $atParts = @()
        foreach ($name in $chainFound.Keys) {
            # Short label: the class prefix is the same for all of them
            # and only makes the trace lines harder to scan.
            $short = ($name -split '::')[-1]
            $atParts += ("--at {0}={1}" -f $short, $chainFound[$name])
        }
        Write-Host ("        scripts/gdb-wdi-connect.sh " + ($atParts -join ' '))
        Write-Host ''
        Write-Host '      Attempt the connection, then send back the trace AND'
        Write-Host "      $log5"
    } else {
        Write-Warning 'none of the connect-task symbols resolved; see the log.'
    }
} else {
    Write-Warning 'no "WDI state" pointer in pass two; skipping the adapter dump'
    Write-Warning 'If -MiniportHandle was given: handles are reassigned every'
    Write-Warning 'time the driver reloads, so a handle from an earlier run'
    Write-Warning 'names a different (or dead) adapter. Drop it and let pass'
    Write-Warning 'one find the adapter itself.'
}

Write-Host '[6/6] Done'
Write-Host "      $log1 ($($out1.Count) lines)"
Write-Host "      $log2 ($($out2.Count) lines)"
Write-Host ''
Write-Host '      wdi-state-decide.txt holds the scan-path breakpoint addresses;'
Write-Host '      wdi-state-connecttask.txt holds the connect-task disassembly.'
Write-Host '      What is left is a runtime question and needs the guest stopped,'
Write-Host '      which local KD cannot do -- see scripts/gdb-wdi-connect.sh.'
