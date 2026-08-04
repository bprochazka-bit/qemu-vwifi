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

.EXAMPLE
    # Fully automatic, elevated:
    .\trace-wdi.ps1 -Ssid vwifi-open

.EXAMPLE
    # Capture while you click the tray icon yourself:
    .\trace-wdi.ps1
#>
[CmdletBinding()]
param(
    [string] $Ssid,
    [int]    $Wait = 15,
    [string] $EtlPath = 'C:\vwifi-wdi.etl',
    [switch] $NoDecode,
    [switch] $DecodeOnly
)

$ErrorActionPreference = 'Stop'
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

    # tracefmt/tracepdb live under bin\<ver>\<arch>; symchk under
    # Debuggers\<arch>. Search both rather than guessing a version.
    $roots = @(
        "${env:ProgramFiles(x86)}\Windows Kits\10\bin",
        "${env:ProgramFiles(x86)}\Windows Kits\10\Debuggers",
        "${env:ProgramFiles}\Windows Kits\10\bin",
        "${env:ProgramFiles}\Windows Kits\10\Debuggers"
    ) | Where-Object { Test-Path $_ }

    foreach ($r in $roots) {
        $hit = Get-ChildItem -Path $r -Filter $Name -Recurse -ErrorAction SilentlyContinue |
               Sort-Object FullName -Descending | Select-Object -First 1
        if ($hit) { return $hit.FullName }
    }
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

    $symchk   = Find-Tool 'symchk.exe'
    $tracepdb = Find-Tool 'tracepdb.exe'
    if (-not $tracepdb) {
        Write-Warning 'tracepdb.exe not found (WDK); WPP records will stay as raw ids'
        return $null
    }

    if ($symchk) {
        Write-Host "      Fetching symbols for wdiwifi.sys $ver (once; cached after this)"
        & $symchk /r $wdiSys /s "srv*$sym*https://msdl.microsoft.com/download/symbols" 2>&1 |
            Select-Object -Last 2 | ForEach-Object { Write-Host "        $_" }
    } else {
        Write-Warning 'symchk.exe not found; relying on any PDB already in the symbol path'
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

# ------------------------------------------------------------- capture
function Invoke-Capture {
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
    Write-Host '        + WDILib (the WDI port driver)'

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
    } finally {
        Remove-Item $tmp -ErrorAction SilentlyContinue
        # Deferred: deleting the profile while the attempt is in flight
        # would abort it and trace the teardown instead of the failure.
        Start-Sleep -Seconds 5
        & netsh wlan delete profile name="$Ssid" 2>&1 | Out-Null
    }
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
    $tmf = Get-TmfDirectory

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

    $pat = 'connect|Connect|CONNECT|assoc|Assoc|ASSOC|bss|BSS|Bss|unreachable|UNREACHABLE|' +
           'fail|Fail|FAIL|error|Error|ERROR|reject|Reject|0xC000023C|c000023c|port|Port'
    $hits = $lines | Select-String -Pattern $pat

    $out = @()
    $out += "vwifi WDI port driver trace summary"
    $out += "source: $WppFile ($($lines.Count) lines)"
    $out += "matched: $($hits.Count)"
    $out += ''
    $out += $hits | ForEach-Object { $_.Line }
    Set-Content -Path $SummaryFile -Value $out -Encoding UTF8

    Write-Host ("      {0} ({1} matching line(s))" -f $SummaryFile, $hits.Count)
    Write-Host ''
    Write-Host '      Send the -summary.txt first; the full -wdi.txt if asked.'
}

# ---------------------------------------------------------------- main
if (-not $DecodeOnly) { Invoke-Capture }
if (-not $NoDecode)   { Invoke-Decode }
