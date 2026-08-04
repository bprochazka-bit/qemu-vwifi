<#
.SYNOPSIS
    Capture and decode a trace of the Windows WDI port driver, in one run.

.DESCRIPTION
    The WDI port driver (wdiwifi.sys, via WDILib) sits between wlansvc
    and this miniport. It translates OID_DOT11_* into OID_WDI_*, and it
    is the only participant in a connect whose reasoning we have never
    been able to see.

    That gap is not academic. In every trace so far the port driver
    accepts OID_DOT11_CONNECT_REQUEST, returns success, and about a
    millisecond later reports STATUS_NETWORK_UNREACHABLE back up the
    stack -- without ever issuing OID_WDI_TASK_CONNECT to us. The
    decision is entirely inside it, and nothing this driver logs can
    explain it.

    `netsh wlan set tracing mode=yes` does NOT cover it. That capture
    carries WPP from wlansvc, wlanmsm, wlansec, nwifi, vwififlt and onex
    -- confirmed by the .pdb names embedded in the ETL -- and nothing
    from the port driver.

    WDILib has its own provider:

        {21ba7b61-05f8-41f1-9048-c09493dcfe38}

    It is WPP rather than manifest-based, so tracerpt cannot read it:
    WPP swaps format strings for numeric ids at compile time and
    recovering the text needs TMF files built from the matching PDB.
    This script builds them.

    Everything happens on one machine, which removes the only hard part
    of WPP decoding. WPP message ids are assigned per build, so a PDB
    from a different patch level decodes to text that is plausible and
    wrong; tracing and decoding on the same box means the wdiwifi.sys
    being decoded IS the one that ran. The TMFs are cached and rebuilt
    only when that binary changes, so the symbol fetch happens once
    rather than once per attempt.

    Three files come out, next to the .etl:
        *-wdi.txt       decoded WPP -- the port driver's own reasoning
        *-events.xml    decoded manifest providers (NDIS/NWiFi/wlansvc)
        *-summary.txt   the connect window, small enough to paste

.PARAMETER Ssid
    Drive the connect automatically instead of waiting for you to click.
    Installs a temporary open-network profile, connects, waits for the
    attempt to resolve, then removes the profile again.

    Note this makes it a with-profile connect, where clicking the tray
    icon is a without-profile connect. The failure has been identical
    either way, but if you are chasing a difference, omit this and
    click.

.PARAMETER Wait
    Seconds to keep capturing after the trigger. Default 15.

.PARAMETER EtlPath
    Where to write the capture. Default C:\vwifi-wdi.etl.

.PARAMETER NoDecode
    Capture only. Decode later by re-running with -DecodeOnly.

.PARAMETER DecodeOnly
    Skip capture and decode an existing -EtlPath.

.NOTES
    Needs tracepdb.exe and tracefmt.exe, which only exist once an
    EWDK/WDK build environment is set up -- run LaunchBuildEnv first.
    It does NOT need symchk.exe: the EWDK carries the build tools but
    not the Debugging Tools, so the PDB is fetched from the symbol
    server directly, using the GUID and age read out of wdiwifi.sys.

    LaunchBuildEnv leaves you in cmd, so use the wrapper rather than
    remembering -File and an execution-policy override:

        trace-wdi.cmd -Ssid vwifi-open

.EXAMPLE
    # Fully automatic, elevated, from inside the build environment:
    .\trace-wdi.cmd -Ssid vwifi-open

.EXAMPLE
    # Capture while you click the tray icon yourself:
    .\trace-wdi.cmd

.EXAMPLE
    # Control experiment: connect to an SSID that does not exist.
    #
    # This is the one measurement that splits the remaining hypotheses.
    # If a network that is definitely not on the air fails the same way
    # the real one does -- same status, same timing, no scan first --
    # then the port driver is refusing before it ever considers a BSS,
    # and every theory about our BSS entry's contents is aimed at the
    # wrong thing. If it fails differently, our entry really is being
    # examined and rejected.
    #
    # Give it longer than the default: a real network fails in two
    # milliseconds, while an absent one is scanned for first.
    .\trace-wdi.cmd -Ssid foobar -Wait 45
#>
[CmdletBinding()]
param(
    [string] $Ssid,
    [int]    $Wait = 15,
    [string] $EtlPath = 'C:\vwifi-wdi.etl',
    [switch] $NoDecode,
    [switch] $DecodeOnly,
    [switch] $ListProviders,
    [string] $SymbolPath
)

$ErrorActionPreference = 'Stop'
$script:AutoProfile = $null
$session   = 'vwifi-wdi'
$wdiGuid   = '{21ba7b61-05f8-41f1-9048-c09493dcfe38}'
$wdiSys    = "$env:SystemRoot\System32\drivers\wdiwifi.sys"
$cacheRoot = Join-Path $env:LOCALAPPDATA 'vwifi-wdi-trace'

# The port driver, plus the layers either side of it, in one session.
# The question is always what the port driver did between the OID
# arriving from above and the status going back up, and that is
# unreadable across files with different clocks.
$manifestProviders = @(
    @{ Name = 'Microsoft-Windows-WLAN-AutoConfig'; Desc = 'wlansvc / MSM state machine' },
    @{ Name = 'Microsoft-Windows-NWiFi';           Desc = 'Native WiFi, dot11 OIDs' },
    @{ Name = 'Microsoft-Windows-NDIS';            Desc = 'OID request/complete' },
    @{ Name = 'Microsoft-Windows-OneX';            Desc = '802.1X (quiet on open networks)' }
)

function Assert-Admin {
    $p = [Security.Principal.WindowsPrincipal] [Security.Principal.WindowsIdentity]::GetCurrent()
    if (-not $p.IsInRole([Security.Principal.WindowsBuiltInRole]::Administrator)) {
        throw 'Run this from an elevated PowerShell (Run as Administrator).'
    }
}

function Find-Tool([string] $Name) {
    $cmd = Get-Command $Name -ErrorAction SilentlyContinue
    if ($cmd) { return $cmd.Source }

    # tracefmt/tracepdb live under bin\<ver>\<arch>; symchk, when it
    # exists at all, under Debuggers\<arch>. The EWDK is the common case
    # here and it carries the build tools but not the Debugging Tools,
    # so symchk is routinely absent -- see Get-SymbolFile, which is why
    # that is no longer fatal. WindowsSdkDir is set by LaunchBuildEnv.
    $roots = @(
        $env:WindowsSdkVerBinPath,
        $env:WindowsSdkDir,
        "${env:ProgramFiles(x86)}\Windows Kits\10\bin",
        "${env:ProgramFiles(x86)}\Windows Kits\10\Debuggers",
        "${env:ProgramFiles}\Windows Kits\10\bin",
        "${env:ProgramFiles}\Windows Kits\10\Debuggers"
    ) | Where-Object { $_ -and (Test-Path $_) }

    foreach ($r in $roots) {
        $hit = Get-ChildItem -Path $r -Filter $Name -Recurse -ErrorAction SilentlyContinue |
               Sort-Object FullName -Descending | Select-Object -First 1
        if ($hit) { return $hit.FullName }
    }
    return $null
}

# ------------------------------------------------------------- symbols
# Read the CodeView (RSDS) record out of a PE image.
#
# This is what symchk does before it talks to the symbol server, and
# doing it here means the EWDK's lack of Debugging Tools stops mattering.
# A symbol server request is fully determined by three things found in
# the binary itself: the PDB's base name, its GUID, and its age. The
# request is then just
#
#   .../download/symbols/<pdb>/<GUID-no-dashes-uppercase><age-hex>/<pdb>
#
# and it is exact -- the key identifies one specific build. That matters
# more than convenience here: WPP message ids are assigned per build, so
# a near-miss PDB decodes to text that is plausible and wrong.
function Get-PeCodeViewInfo([string] $Path) {
    $b = [IO.File]::ReadAllBytes($Path)
    if ($b.Length -lt 0x40 -or $b[0] -ne 0x4D -or $b[1] -ne 0x5A) { return $null }

    $peOff = [BitConverter]::ToInt32($b, 0x3C)
    if ($peOff -le 0 -or ($peOff + 24) -gt $b.Length) { return $null }
    if ([BitConverter]::ToUInt32($b, $peOff) -ne 0x00004550) { return $null }

    $coff      = $peOff + 4
    $nSections = [BitConverter]::ToUInt16($b, $coff + 2)
    $optSize   = [BitConverter]::ToUInt16($b, $coff + 16)
    $opt       = $coff + 20
    $magic     = [BitConverter]::ToUInt16($b, $opt)

    # Data directory 6 is Debug. Its offset differs between PE32 and
    # PE32+ because of the wider fields in the 64-bit optional header.
    $ddOff = if ($magic -eq 0x20B) { $opt + 112 } else { $opt + 96 }
    $dbgRva  = [BitConverter]::ToUInt32($b, $ddOff + 48)
    $dbgSize = [BitConverter]::ToUInt32($b, $ddOff + 52)
    if ($dbgRva -eq 0 -or $dbgSize -lt 28) { return $null }

    $secTab = $opt + $optSize
    $dbgOff = 0
    for ($i = 0; $i -lt $nSections; $i++) {
        $s    = $secTab + $i * 40
        $vsz  = [BitConverter]::ToUInt32($b, $s + 8)
        $va   = [BitConverter]::ToUInt32($b, $s + 12)
        $rsz  = [BitConverter]::ToUInt32($b, $s + 16)
        $praw = [BitConverter]::ToUInt32($b, $s + 20)
        $span = [Math]::Max($vsz, $rsz)
        if ($dbgRva -ge $va -and $dbgRva -lt ($va + $span)) {
            $dbgOff = $praw + ($dbgRva - $va); break
        }
    }
    if ($dbgOff -eq 0) { return $null }

    for ($i = 0; $i -lt [int]($dbgSize / 28); $i++) {
        $e = $dbgOff + $i * 28
        if (($e + 28) -gt $b.Length) { break }
        if ([BitConverter]::ToUInt32($b, $e + 12) -ne 2) { continue }  # CODEVIEW
        $raw = [BitConverter]::ToUInt32($b, $e + 24)
        if ($raw -eq 0 -or ($raw + 24) -ge $b.Length) { continue }
        if ([Text.Encoding]::ASCII.GetString($b, $raw, 4) -ne 'RSDS') { continue }

        $guid = [guid]::new([byte[]]$b[($raw + 4)..($raw + 19)])
        $age  = [BitConverter]::ToUInt32($b, $raw + 20)

        $p = $raw + 24; $q = $p
        while ($q -lt $b.Length -and $b[$q] -ne 0) { $q++ }
        $name = [IO.Path]::GetFileName([Text.Encoding]::ASCII.GetString($b, $p, $q - $p))

        return [pscustomobject]@{
            PdbName = $name
            Key     = ('{0}{1:X}' -f $guid.ToString('N').ToUpper(), $age)
        }
    }
    return $null
}

function Get-SymbolFile([string] $ImagePath, [string] $DestDir) {
    $cv = Get-PeCodeViewInfo $ImagePath
    if (-not $cv) {
        Write-Warning "could not read a CodeView record from $ImagePath"
        return $null
    }

    $dest = Join-Path $DestDir $cv.PdbName
    if (Test-Path $dest) { return $dest }
    New-Item -ItemType Directory -Force -Path $DestDir | Out-Null

    $url = 'https://msdl.microsoft.com/download/symbols/{0}/{1}/{0}' -f $cv.PdbName, $cv.Key
    Write-Host "      GET $url"

    try {
        # PowerShell 5.1 still defaults to TLS 1.0, which msdl refuses.
        [Net.ServicePointManager]::SecurityProtocol =
            [Net.SecurityProtocolType]::Tls12
        Invoke-WebRequest -Uri $url -OutFile $dest -UseBasicParsing `
            -UserAgent 'Microsoft-Symbol-Server/10.0.0.0' -TimeoutSec 120
    } catch {
        # Print the URL, not just the failure. This machine's only
        # network adapter is the one being debugged, so the download
        # failing is expected rather than exceptional -- and the URL is
        # everything needed to fetch the PDB from anywhere else and drop
        # it in with -SymbolPath.
        Write-Warning "symbol download failed: $($_.Exception.Message)"
        Write-Host ''
        Write-Host '      Fetch it from any machine with internet:'
        Write-Host "        $url"
        Write-Host "      then re-run with:  -SymbolPath <the-downloaded-.pdb>"
        Write-Host ''
        Remove-Item $dest -ErrorAction SilentlyContinue
        return $null
    }

    if (Test-Path $dest) { return $dest }
    return $null
}

# ---------------------------------------------------------------- TMFs
function Get-TmfDirectory {
    if (-not (Test-Path $wdiSys)) {
        Write-Warning "no $wdiSys; WPP records will stay as raw ids"
        return $null
    }

    # Key the cache on the binary's identity, so a Windows update that
    # replaces wdiwifi.sys invalidates it automatically instead of
    # silently decoding the new trace with the old message ids.
    $fi  = Get-Item $wdiSys
    $ver = $fi.VersionInfo.FileVersion
    if (-not $ver) { $ver = 'unknown' }
    $key = ('{0}_{1}' -f ($ver -replace '[^\w\.]', '-'), $fi.Length)

    $tmf = Join-Path $cacheRoot "tmf\$key"
    $sym = Join-Path $cacheRoot "sym"

    if ((Test-Path $tmf) -and (Get-ChildItem $tmf -Filter '*.tmf' -ErrorAction SilentlyContinue)) {
        Write-Host "      TMFs cached for wdiwifi.sys $ver"
        return $tmf
    }

    New-Item -ItemType Directory -Force -Path $tmf, $sym | Out-Null

    $tracepdb = Find-Tool 'tracepdb.exe'
    if (-not $tracepdb) {
        Write-Warning 'tracepdb.exe not found. Run this from an EWDK/WDK build environment (LaunchBuildEnv).'
        return $null
    }

    Write-Host "      Fetching symbols for wdiwifi.sys $ver (once; cached after this)"

    # Three ways to end up with a PDB, in order of preference:
    # one the caller supplies, symchk if it happens to be installed, or
    # a direct fetch. The EWDK carries the build tools but not the
    # Debugging Tools, so symchk is usually absent; and this machine's
    # only network adapter is the one being debugged, so the fetch
    # usually fails too. -SymbolPath is the one that always works.
    #
    # Supplied PDBs are copied rather than used in place, so the cache
    # keyed on this wdiwifi.sys keeps them and later runs need neither
    # the network nor the switch.
    if ($SymbolPath) {
        if (-not (Test-Path $SymbolPath)) { throw "no such path: $SymbolPath" }
        $src = Get-Item $SymbolPath
        $pdbFiles = if ($src.PSIsContainer) {
            Get-ChildItem $src.FullName -Filter '*.pdb' -Recurse
        } else { @($src) }
        foreach ($f in $pdbFiles) { Copy-Item $f.FullName -Destination $sym -Force }
        Write-Host ("      Using {0} supplied PDB(s)" -f @($pdbFiles).Count)
    }

    $symchk = Find-Tool 'symchk.exe'
    if ($SymbolPath) {
        # nothing to fetch
    } elseif ($symchk) {
        & $symchk /r $wdiSys /s "srv*$sym*https://msdl.microsoft.com/download/symbols" 2>&1 |
            Select-Object -Last 2 | ForEach-Object { Write-Host "        $_" }
    } else {
        (Get-SymbolFile -ImagePath $wdiSys -DestDir $sym) | Out-Null
    }

    $pdbs = @(Get-ChildItem -Path $sym -Filter '*.pdb' -Recurse -ErrorAction SilentlyContinue)
    if ($pdbs.Count -eq 0) {
        Write-Warning 'no PDB retrieved; WPP records will stay as raw ids'
        return $null
    }
    Write-Host "      Building TMFs from $($pdbs.Count) PDB(s)"
    foreach ($p in $pdbs) { & $tracepdb -f $p.FullName -p $tmf 2>&1 | Out-Null }

    $n = @(Get-ChildItem $tmf -Filter '*.tmf' -ErrorAction SilentlyContinue).Count
    if ($n -eq 0) {
        # Microsoft does not ship WPP metadata in every public PDB. Say
        # so plainly rather than producing an empty decode and letting
        # it look like the driver was silent.
        Write-Warning 'PDB carried no WPP metadata; the public symbol may be stripped of it'
        return $null
    }
    Write-Host "      $n TMF(s) ready"
    return $tmf
}

# Every WPP control GUID that wdiwifi.sys actually registers.
#
# tracepdb names each file it emits after the trace provider GUID it
# came from, so once the PDB has been turned into TMFs the filenames
# are the answer -- no guessing, no enumeration, and it works offline.
#
# This exists because the documented WDILib GUID
# ({21ba7b61-05f8-41f1-9048-c09493dcfe38}) produced a completely silent
# capture: 8820 decoded lines, all of them NDIS, WLAN-AutoConfig or
# NWiFi, and not one undecodable record either. WDILib is a helper
# library that IHV drivers link against rather than something inside
# the port driver, so on a miniport that does not use it there is
# nothing for that provider to say. The port driver's own GUIDs are
# these.
function Get-WppGuidsFromTmf([string] $TmfDir) {
    if (-not $TmfDir -or -not (Test-Path $TmfDir)) { return @() }
    $guids = Get-ChildItem $TmfDir -Filter '*.tmf' -ErrorAction SilentlyContinue |
             ForEach-Object { [IO.Path]::GetFileNameWithoutExtension($_.Name) } |
             Where-Object { $_ -match '^[0-9a-fA-F]{8}-([0-9a-fA-F]{4}-){3}[0-9a-fA-F]{12}$' } |
             Sort-Object -Unique
    return @($guids)
}

# Every trace GUID currently registered on the machine.
#
# tracelog ships with the EWDK, needs no network, and lists WPP control
# GUIDs as well as manifest providers. On its own it is not enough --
# it says which GUIDs exist but not which module owns them -- so it is
# only half of Get-WppGuidsFromImage below.
function Get-RegisteredTraceGuids {
    $tracelog = Find-Tool 'tracelog.exe'
    if (-not $tracelog) {
        Write-Warning 'tracelog.exe not found; cannot enumerate registered trace GUIDs'
        return @()
    }
    $rx = '([0-9a-fA-F]{8}-(?:[0-9a-fA-F]{4}-){3}[0-9a-fA-F]{12})'
    $g = foreach ($l in (& $tracelog -enumguid 2>&1)) {
        if ("$l" -match $rx) { $Matches[1] }
    }
    return @($g | Sort-Object -Unique)
}

# Which of those GUIDs wdiwifi.sys actually contains.
#
# The offline answer to "what are the port driver's control GUIDs",
# for when the PDB cannot be fetched -- which on this test machine is
# the normal case, since the only network adapter it has is the one
# being debugged.
#
# WPP compiles its control GUID into the image, so the bytes are in the
# binary; and tracelog lists the GUIDs the system has registered. Either
# set alone is useless -- the image contains many 16-byte sequences that
# merely look like GUIDs, and the registered list has no attribution --
# but a GUID that is in both is a trace provider this binary registered.
#
# Compared as hex over the whole file rather than by constructing a Guid
# per offset: a few hundred IndexOf calls against one string, instead of
# a hundred thousand object allocations.
function Get-WppGuidsFromImage([string] $ImagePath, [string[]] $Candidates) {
    if (-not (Test-Path $ImagePath) -or $Candidates.Count -eq 0) { return @() }

    $hex = [BitConverter]::ToString([IO.File]::ReadAllBytes($ImagePath)) -replace '-'
    $hit = foreach ($c in $Candidates) {
        # ToByteArray gives the on-disk little-endian layout, which is
        # how WPP stores it -- not the display order.
        $needle = [BitConverter]::ToString(([guid]$c).ToByteArray()) -replace '-'
        if ($hex.IndexOf($needle, [StringComparison]::OrdinalIgnoreCase) -ge 0) { $c }
    }
    return @($hit)
}

# ------------------------------------------------------------- capture
function Invoke-Capture([string[]] $WppGuids) {
    Assert-Admin

    # An interrupted run leaves the session registered, and logman
    # create then fails on the name rather than doing the obvious thing.
    logman stop $session -ets 2>$null | Out-Null

    Write-Host "[1/5] Starting session -> $EtlPath"

    # Flags 0xffffffff / level 0xff: every WPP flag bit, verbose. The
    # bits are per-component and undocumented for inbox drivers, so
    # asking for a subset means guessing which one carries the connect
    # path.
    #
    # `-f bincirc -max 64` is the circular 64 MB file on its own --
    # adding `-mode Circular` as well is redundant and some logman
    # builds reject the pair outright. The interesting window is
    # milliseconds wide but NDIS is chatty enough to wrap a small
    # buffer, hence 64 MB. Buffers stay modest (4 MB floor, 16 MB
    # ceiling) because they are non-paged and this runs in a VM.
    $create = @(
        'create', 'trace', $session, '-ow', '-o', $EtlPath,
        '-p', $wdiGuid, '0xffffffff', '0xff',
        '-nb', '64', '256', '-bs', '64',
        '-f', 'bincirc', '-max', '64',
        '-ets'
    )
    $out = & logman @create 2>&1
    if ($LASTEXITCODE -ne 0) {
        # Print what logman actually said. Swallowing it turns a typo in
        # a switch into an unexplained exit code.
        $out | ForEach-Object { Write-Host "      $_" }
        throw "logman create failed ($LASTEXITCODE)"
    }
    Write-Host '        + WDILib (documented, but silent unless the miniport links it)'

    # The port driver's real control GUIDs, recovered from its own PDB.
    # This is the part that matters; the WDILib GUID above is kept only
    # because it costs nothing and would carry an IHV driver that does
    # link the library.
    foreach ($g in $WppGuids) {
        if ($g -eq ($wdiGuid -replace '[{}]', '')) { continue }
        & logman update trace $session -p "{$g}" 0xffffffff 0xff -ets | Out-Null
        if ($LASTEXITCODE -eq 0) { Write-Host "        + wdiwifi {$g}" }
        else { Write-Warning "could not add {$g}" }
    }
    if ($WppGuids.Count -eq 0) {
        Write-Warning 'no wdiwifi control GUIDs known -- capture will miss the port driver'
    }

    foreach ($p in $manifestProviders) {
        & logman update trace $session -p $p.Name 0xffffffffffffffff 0xff -ets | Out-Null
        if ($LASTEXITCODE -eq 0) { Write-Host ("        + {0}" -f $p.Desc) }
        else { Write-Warning ("could not add {0}" -f $p.Name) }
    }

    Write-Host '[2/5] Triggering the connect'
    if ($Ssid) {
        Invoke-AutoConnect
    } else {
        Write-Host '      Connect to the network now, let it fail, then press Enter.'
        Read-Host  '      Press Enter when done'
    }

    Write-Host "[3/5] Capturing for $Wait more second(s)"
    Start-Sleep -Seconds $Wait

    Write-Host '[4/5] Stopping session'
    & logman stop $session -ets | Out-Null
    Remove-AutoConnectProfile

    if (-not (Test-Path $EtlPath)) { throw "no $EtlPath was produced" }
    $kb = [int]((Get-Item $EtlPath).Length / 1KB)
    Write-Host "      $EtlPath ($kb KB)"
}

function Invoke-AutoConnect {
    # netsh can only connect to a named profile, while clicking the tray
    # icon is a profile-less connect. A throwaway open profile is the
    # closest scriptable equivalent; it is removed again below so the
    # machine is left as it was found.
    $xml = @"
<?xml version="1.0"?>
<WLANProfile xmlns="http://www.microsoft.com/networking/WLAN/profile/v1">
  <name>$Ssid</name>
  <SSIDConfig><SSID><name>$Ssid</name></SSID></SSIDConfig>
  <connectionType>ESS</connectionType>
  <connectionMode>manual</connectionMode>
  <MSM><security>
    <authEncryption>
      <authentication>open</authentication>
      <encryption>none</encryption>
      <useOneX>false</useOneX>
    </authEncryption>
  </security></MSM>
</WLANProfile>
"@
    $tmp = Join-Path $env:TEMP "vwifi-$Ssid.xml"
    Set-Content -Path $tmp -Value $xml -Encoding UTF8

    try {
        & netsh wlan add profile filename="$tmp" 2>&1 | ForEach-Object { Write-Host "      $_" }
        Start-Sleep -Milliseconds 500
        & netsh wlan connect name="$Ssid" 2>&1 | ForEach-Object { Write-Host "      $_" }
        # Record it for Remove-AutoConnectProfile, which runs after the
        # capture window rather than here. Deleting a profile aborts any
        # attempt using it, and this used to do that five seconds in --
        # fine for a network that fails in two milliseconds, fatal for
        # one that does not exist, where Windows scans for far longer
        # than five seconds before giving up. That is precisely the
        # comparison this switch is useful for, so the teardown must not
        # be what ends it.
        $script:AutoProfile = $Ssid
    } finally {
        Remove-Item $tmp -ErrorAction SilentlyContinue
    }
}

function Remove-AutoConnectProfile {
    if (-not $script:AutoProfile) { return }
    & netsh wlan delete profile name="$script:AutoProfile" 2>&1 | Out-Null
    Write-Host "      Removed temporary profile $script:AutoProfile"
    $script:AutoProfile = $null
}

# -------------------------------------------------------------- decode
function Invoke-Decode {
    if (-not (Test-Path $EtlPath)) { throw "no such file: $EtlPath" }

    $base    = [IO.Path]::Combine(
                 [IO.Path]::GetDirectoryName((Resolve-Path $EtlPath)),
                 [IO.Path]::GetFileNameWithoutExtension($EtlPath))
    $wppOut  = "$base-wdi.txt"
    $evtOut  = "$base-events.xml"
    $sumOut  = "$base-summary.txt"

    Write-Host '[5/5] Decoding'
    $tmf = $script:TmfDir
    if (-not $tmf) { $tmf = Get-TmfDirectory }   # -DecodeOnly path

    # WPP and manifest events share the ETL but need different decoders,
    # and both read the same file, so their timestamps line up exactly.
    $tracefmt = Find-Tool 'tracefmt.exe'
    if ($tracefmt -and $tmf) {
        & $tracefmt $EtlPath -p $tmf -o $wppOut -nosummary 2>&1 |
            Select-Object -Last 3 | ForEach-Object { Write-Host "      $_" }
    } elseif ($tracefmt) {
        & $tracefmt $EtlPath -o $wppOut -nosummary 2>&1 | Out-Null
    } else {
        Write-Warning 'tracefmt.exe not found (WDK); skipping WPP decode'
    }

    & tracerpt $EtlPath -o $evtOut -of XML -y 2>&1 | Out-Null

    Write-Host ''
    Write-Host '      --- results ---'
    foreach ($f in @($wppOut, $evtOut)) {
        if (Test-Path $f) {
            $kb = [int]((Get-Item $f).Length / 1KB)
            Write-Host ("      {0} ({1} KB)" -f $f, $kb)
        }
    }

    Write-Summary -WppFile $wppOut -SummaryFile $sumOut
}

function Write-Summary([string] $WppFile, [string] $SummaryFile) {
    if (-not (Test-Path $WppFile)) { return }

    $lines = Get-Content $WppFile -ErrorAction SilentlyContinue
    if (-not $lines) { return }

    # Zero WDI lines is itself the finding: it means the provider never
    # attached, or the public PDB had no WPP metadata. Either way the
    # run needs a different approach, and saying so here saves a cycle
    # spent reading an empty file as "the driver did nothing".
    Write-Host ("      {0} decoded WPP line(s)" -f $lines.Count)
    if ($lines.Count -eq 0) {
        Write-Warning 'no WPP output -- provider did not attach, or symbols lack WPP metadata'
        return
    }

    # Which components actually spoke, before any filtering.
    #
    # This is the first thing to read, because a filtered excerpt cannot
    # distinguish "the port driver explained itself and none of it
    # matched my keywords" from "the port driver was never captured at
    # all" -- and the run that motivated this reported 8820 confident
    # lines while containing nothing whatsoever from the port driver.
    # tracefmt prefixes every line with the CPU number in brackets --
    # "[1]0F24.1E30::...[Microsoft-Windows-NDIS]..." -- so the first
    # bracketed token is a digit, not a provider. Take the first
    # non-numeric one instead of blindly taking match zero.
    $byProvider = $lines |
        ForEach-Object {
            foreach ($m in [regex]::Matches($_, '\[([A-Za-z0-9_.-]+)\]')) {
                $v = $m.Groups[1].Value
                if ($v -notmatch '^\d+$') { $v; break }
            }
        } |
        Group-Object | Sort-Object Count -Descending

    $known = @('Microsoft-Windows-NDIS', 'Microsoft-Windows-WLAN-AutoConfig',
               'Microsoft-Windows-NWiFi', 'Microsoft-Windows-OneX')
    $fromPortDriver = @($byProvider | Where-Object { $known -notcontains $_.Name })

    $pat = 'connect|assoc|bss|unreachable|fail|error|reject|refus|invalid|' +
           'denied|status|state|port|scan|c000023c|38002'
    $hits = $lines | Select-String -Pattern $pat -CaseSensitive:$false

    $out = @()
    $out += 'vwifi WDI port driver trace summary'
    $out += "source: $WppFile ($($lines.Count) lines)"
    $out += ''
    $out += 'providers seen:'
    $out += $byProvider | ForEach-Object { '  {0,8}  {1}' -f $_.Count, $_.Name }
    $out += ''
    if ($fromPortDriver.Count -eq 0) {
        $out += '*** NOTHING from the WDI port driver in this capture. ***'
        $out += '*** Only the manifest providers above responded, so the  ***'
        $out += '*** control GUIDs used did not match wdiwifi.sys.        ***'
        $out += '*** Re-run with -ListProviders to see what its PDB says. ***'
        $out += ''
    }
    $out += "matched $($hits.Count) line(s) on: $pat"
    $out += ''
    $out += $hits | ForEach-Object { $_.Line }
    Set-Content -Path $SummaryFile -Value $out -Encoding UTF8

    Write-Host ''
    foreach ($g in $byProvider) { Write-Host ('      {0,8}  {1}' -f $g.Count, $g.Name) }
    if ($fromPortDriver.Count -eq 0) {
        Write-Warning 'nothing from the WDI port driver -- the control GUIDs did not match wdiwifi.sys'
    }
    Write-Host ("      {0} ({1} matching line(s))" -f $SummaryFile, $hits.Count)
    Write-Host ''
    Write-Host '      Send the -summary.txt first; the full -wdi.txt if asked.'
}

# ---------------------------------------------------------------- main
#
# Symbols are prepared BEFORE the capture, not after. The TMFs are what
# name the port driver's control GUIDs, and the GUIDs have to be known
# at logman-create time -- preparing them afterwards, as this did while
# it was trusting the documented WDILib GUID, means every run captures
# the wrong providers and only finds out during decode.
$script:TmfDir = $null
if (-not $DecodeOnly -or $ListProviders) {
    Write-Host '[0/5] Preparing symbols'
    $script:TmfDir = Get-TmfDirectory
}

# Preferred: the PDB names the GUIDs outright, and the same TMFs then
# decode the result. Fallback: intersect the image with the registered
# trace GUIDs, which needs no network. The fallback gets the capture
# right even when decoding will only yield raw ids -- which is still
# the difference between a trace of the port driver and a trace of
# everything except the port driver.
$guids = Get-WppGuidsFromTmf $script:TmfDir
$source = 'wdiwifi.pdb'
if ($guids.Count -eq 0) {
    Write-Host '      No TMFs; falling back to image/registered-GUID intersection'
    $guids  = Get-WppGuidsFromImage -ImagePath $wdiSys -Candidates (Get-RegisteredTraceGuids)
    $source = 'wdiwifi.sys x tracelog -enumguid'
}

if ($ListProviders) {
    Write-Host ''
    Write-Host "WPP control GUIDs for wdiwifi.sys (via $source):"
    if ($guids.Count -eq 0) {
        Write-Host '  (none found)'
        Write-Host '  Without a PDB this needs tracelog.exe from the EWDK build environment.'
    } else {
        $guids | ForEach-Object { Write-Host "  {$_}" }
    }
    return
}

if (-not $DecodeOnly) { Invoke-Capture -WppGuids $guids }
if (-not $NoDecode)   { Invoke-Decode }
