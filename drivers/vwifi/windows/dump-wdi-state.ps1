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

# ndiskd.dll does not sit beside kd.exe -- Debugging Tools puts its
# extensions in subdirectories (winxp, winext, w2k), and ndiskd lives in
# winxp. `.load ndiskd` by bare name relies on the debugger's extension
# search path being right, which is exactly the thing that fails when
# the tools have been copied around rather than installed. Finding the
# DLL and loading it by full path removes the guess.
function Find-NdiskdDll {
    param([Parameter(Mandatory)][string] $KdPath)
    $root = Split-Path $KdPath -Parent
    $hit = Get-ChildItem -Path $root -Filter 'ndiskd.dll' -Recurse -File -ErrorAction SilentlyContinue |
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
# Semicolons, not newlines. kd's -c takes ONE command string with ';'
# separators; a multi-line string is not parsed as successive commands,
# which is why the first version of this loaded nothing and then blamed
# a missing extension for it.
$ndiskd = Find-NdiskdDll -KdPath $kd
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
Write-Host '[3/4] Dumping adapter, port and WDI state'
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
        # carry no C++ type layout. Naming the routine that sets
        # roamAPRankIndex is the difference between guessing at the
        # rejection and disassembling it: local kernel debugging cannot
        # break, but it can read code, so `uf` on the right name in the
        # next round reads the conditions out of wdiwifi directly.
        'x wdiwifi!*Candidate*',
        'x wdiwifi!*Rank*',
        'x wdiwifi!*RoamReconnect*'
    ) -join ';')
    Write-Host "      $log3 ($($out3.Count) lines)"
} else {
    Write-Warning 'no "WDI state" pointer in pass two; skipping the adapter dump'
}

Write-Host '[4/4] Done'
Write-Host "      $log1 ($($out1.Count) lines)"
Write-Host "      $log2 ($($out2.Count) lines)"
Write-Host ''
Write-Host '      Send wdi-state-adapter.txt. That is the one that matters now:'
Write-Host '      the beacon and probe bytes wdiwifi holds, the cached profile it'
Write-Host '      ranks them against, and the names of the routines that decide.'
