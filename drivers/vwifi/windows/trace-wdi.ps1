<#
.SYNOPSIS
    Capture and decode a trace of the Windows WDI port driver.

.DESCRIPTION
    The WDI port driver (wdiwifi.sys, via WDILib) is the component
    between wlansvc and this miniport. It is where OID_DOT11_* is
    translated into OID_WDI_*, and it is the only participant in a
    connect whose reasoning we have never been able to see.

    That gap is not academic. In every trace so far the port driver
    accepts OID_DOT11_CONNECT_REQUEST, returns success, and about a
    millisecond later reports STATUS_NETWORK_UNREACHABLE back up the
    stack -- without ever issuing OID_WDI_TASK_CONNECT to us. The
    decision is entirely inside it, and nothing this driver logs can
    explain it.

    `netsh wlan set tracing mode=yes` does NOT cover it. That capture
    contains WPP from wlansvc, wlanmsm, wlansec, nwifi, vwififlt and
    onex -- verified by the .pdb names embedded in the ETL -- and
    stops there.

    WDILib has its own provider:

        {21ba7b61-05f8-41f1-9048-c09493dcfe38}

    It is WPP, not manifest-based, which has one consequence worth
    knowing before you start: tracerpt cannot decode it. WPP replaces
    format strings with numeric ids at compile time, and turning them
    back into text needs TMF files derived from the matching PDB. Hence
    the -Decode mode below.

.PARAMETER Capture
    Start the trace session, wait for you to reproduce, then stop it.

.PARAMETER Decode
    Turn a captured .etl into readable text. Needs the WDK's tracepdb
    and tracefmt, so run this on the build machine if the guest has no
    WDK.

.PARAMETER EtlPath
    Where the .etl lives. Default C:\vwifi-wdi.etl.

.PARAMETER SysPath
    For -Decode: the wdiwifi.sys the trace was captured against. The
    PDB is fetched from Microsoft's symbol server by matching this
    binary, so it must be the copy from the traced machine -- a
    different patch level yields TMFs whose message ids do not line up,
    and the output is worse than nothing because it is confidently
    wrong.

    Copy it off the guest with:
        C:\Windows\System32\drivers\wdiwifi.sys

.EXAMPLE
    # On the guest, elevated:
    .\trace-wdi.ps1 -Capture

.EXAMPLE
    # On the build machine, with the WDK:
    .\trace-wdi.ps1 -Decode -EtlPath .\vwifi-wdi.etl -SysPath .\wdiwifi.sys
#>
[CmdletBinding(DefaultParameterSetName = 'Capture')]
param(
    [Parameter(ParameterSetName = 'Capture')] [switch] $Capture,
    [Parameter(ParameterSetName = 'Decode')]  [switch] $Decode,
    [string] $EtlPath = 'C:\vwifi-wdi.etl',
    [Parameter(ParameterSetName = 'Decode')]  [string] $SysPath
)

$ErrorActionPreference = 'Stop'
$session = 'vwifi-wdi'

# The WDI port driver, and the layers either side of it. All in one
# session so the timestamps interleave -- the question is always what
# the port driver did between the OID arriving from above and the
# status going back up, and that is unreadable across two files.
$providers = @(
    @{ Name = '{21ba7b61-05f8-41f1-9048-c09493dcfe38}'; Desc = 'WDILib (the WDI port driver)' },
    @{ Name = 'Microsoft-Windows-WLAN-AutoConfig';      Desc = 'wlansvc / MSM state machine' },
    @{ Name = 'Microsoft-Windows-NWiFi';                Desc = 'Native WiFi, dot11 OIDs' },
    @{ Name = 'Microsoft-Windows-NDIS';                 Desc = 'OID request/complete' },
    @{ Name = 'Microsoft-Windows-OneX';                 Desc = '802.1X (quiet on open networks)' }
)

function Assert-Admin {
    $p = [Security.Principal.WindowsPrincipal] [Security.Principal.WindowsIdentity]::GetCurrent()
    if (-not $p.IsInRole([Security.Principal.WindowsBuiltInRole]::Administrator)) {
        throw 'Run this from an elevated PowerShell (Run as Administrator).'
    }
}

function Invoke-Capture {
    Assert-Admin

    # A previous run that was interrupted leaves the session behind, and
    # logman create then fails with a name collision rather than doing
    # the obvious thing.
    logman stop $session -ets 2>$null | Out-Null

    Write-Host "[1/4] Starting session -> $EtlPath"

    # Flags 0xffffffff / level 0xff: every WPP flag bit, verbose. WPP
    # flag bits are per-component and undocumented for inbox drivers,
    # so asking for a subset means guessing which bit carries the
    # connect path. Ask for all of them.
    #
    # 64 MB circular. The interesting window is a few milliseconds
    # around the connect, but NDIS is chatty enough that a small buffer
    # wraps past it while you are still reaching for the mouse.
    $args = @(
        'create', 'trace', $session, '-ow', '-o', $EtlPath,
        '-p', $providers[0].Name, '0xffffffff', '0xff',
        '-nb', '128', '640', '-bs', '128',
        '-mode', 'Circular', '-f', 'bincirc', '-max', '64',
        '-ets'
    )
    & logman @args | Out-Null
    if ($LASTEXITCODE -ne 0) { throw "logman create failed ($LASTEXITCODE)" }
    Write-Host ("        + {0}" -f $providers[0].Desc)

    foreach ($p in $providers[1..($providers.Count - 1)]) {
        & logman update trace $session -p $p.Name 0xffffffffffffffff 0xff -ets | Out-Null
        if ($LASTEXITCODE -eq 0) {
            Write-Host ("        + {0}" -f $p.Desc)
        } else {
            # Not fatal: a missing provider costs us that one layer,
            # and the WDI provider is the one that matters.
            Write-Warning ("could not add {0} ({1})" -f $p.Name, $p.Desc)
        }
    }

    Write-Host ''
    Write-Host '[2/4] Reproduce now: try to connect to the network.'
    Write-Host '      Let it fail, give it a couple of seconds, then press Enter.'
    Read-Host  '      Press Enter when done'

    Write-Host '[3/4] Stopping session'
    & logman stop $session -ets | Out-Null

    Write-Host '[4/4] Done'
    if (Test-Path $EtlPath) {
        $kb = [int]((Get-Item $EtlPath).Length / 1KB)
        Write-Host "      $EtlPath ($kb KB)"
        Write-Host ''
        Write-Host '      Decode it with:'
        Write-Host "        .\trace-wdi.ps1 -Decode -EtlPath $EtlPath -SysPath .\wdiwifi.sys"
        Write-Host '      (copy C:\Windows\System32\drivers\wdiwifi.sys alongside it)'
    } else {
        Write-Warning "no $EtlPath was produced"
    }
}

function Find-WdkTool([string] $Name) {
    $cmd = Get-Command $Name -ErrorAction SilentlyContinue
    if ($cmd) { return $cmd.Source }

    $roots = @(
        "${env:ProgramFiles(x86)}\Windows Kits\10\bin",
        "${env:ProgramFiles}\Windows Kits\10\bin"
    ) | Where-Object { Test-Path $_ }

    foreach ($r in $roots) {
        $hit = Get-ChildItem -Path $r -Filter $Name -Recurse -ErrorAction SilentlyContinue |
               Sort-Object FullName -Descending | Select-Object -First 1
        if ($hit) { return $hit.FullName }
    }
    return $null
}

function Invoke-Decode {
    if (-not (Test-Path $EtlPath)) { throw "no such file: $EtlPath" }

    $tracepdb = Find-WdkTool 'tracepdb.exe'
    $tracefmt = Find-WdkTool 'tracefmt.exe'
    if (-not $tracepdb -or -not $tracefmt) {
        throw 'tracepdb.exe / tracefmt.exe not found. These ship with the WDK; run -Decode on the build machine.'
    }

    $work = Join-Path ([IO.Path]::GetTempPath()) 'vwifi-wdi-tmf'
    $tmf  = Join-Path $work 'tmf'
    $sym  = Join-Path $work 'sym'
    New-Item -ItemType Directory -Force -Path $tmf, $sym | Out-Null

    if ($SysPath) {
        if (-not (Test-Path $SysPath)) { throw "no such file: $SysPath" }

        # symchk pulls the PDB that matches this exact binary. Matching
        # matters more than it looks: WPP message ids are assigned per
        # build, so a PDB from a different patch level decodes to
        # plausible, wrong text.
        $symchk = Find-WdkTool 'symchk.exe'
        if ($symchk) {
            Write-Host "[1/3] Fetching symbols for $SysPath"
            & $symchk /r $SysPath /s "srv*$sym*https://msdl.microsoft.com/download/symbols" 2>&1 |
                Select-Object -Last 3 | ForEach-Object { Write-Host "      $_" }
        } else {
            Write-Warning 'symchk.exe not found; relying on any PDB already in the symbol path'
        }

        Write-Host '[2/3] Building TMFs from PDBs'
        Get-ChildItem -Path $sym -Filter '*.pdb' -Recurse -ErrorAction SilentlyContinue |
            ForEach-Object { & $tracepdb -f $_.FullName -p $tmf 2>&1 | Out-Null }
    } else {
        Write-Warning 'no -SysPath given; WPP records will stay as raw ids'
    }

    $out = [IO.Path]::ChangeExtension($EtlPath, '.txt')
    Write-Host "[3/3] Decoding -> $out"
    & $tracefmt $EtlPath -p $tmf -o $out -nosummary 2>&1 |
        Select-Object -Last 5 | ForEach-Object { Write-Host "      $_" }

    if (Test-Path $out) {
        $n = (Get-Content $out -ErrorAction SilentlyContinue | Measure-Object -Line).Lines
        Write-Host "      $out ($n lines)"
    }
}

if ($Decode) { Invoke-Decode } else { Invoke-Capture }
