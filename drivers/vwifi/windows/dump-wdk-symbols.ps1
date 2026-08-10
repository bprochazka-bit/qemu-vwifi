<#
.SYNOPSIS
    Dump the WDI/NDIS declarations this driver depends on, from the WDK
    headers actually installed on this machine.

.DESCRIPTION
    The driver was written against Microsoft's WDI documentation rather
    than against dot11wdi.h, so several type and constant names are
    guesses. Rather than guess again, this dumps the real declarations
    into wdk-symbols.txt so they can be corrected in one pass.

    Run it from the same EWDK build shell you use for build.cmd (it
    picks up $env:WindowsSdkDir), or pass -IncludeRoot explicitly:

        powershell -ExecutionPolicy Bypass -File .\dump-wdk-symbols.ps1
        powershell ... -IncludeRoot "C:\Program Files (x86)\Windows Kits\10\Include\10.0.22621.0"

    Output is declarations only -- no source, no license text, just the
    names and signatures needed to fix the call sites.

.PARAMETER IncludeRoot
    The kit's Include\<version> directory. Auto-detected if omitted.

.PARAMETER OutFile
    Where to write the dump. Defaults to .\wdk-symbols.txt
#>
[CmdletBinding()]
param(
    [string] $IncludeRoot,
    [string] $OutFile = (Join-Path $PSScriptRoot 'wdk-symbols.txt')
)

$ErrorActionPreference = 'Stop'

# --- locate the kit --------------------------------------------------
function Find-IncludeRoot {
    if ($env:WindowsSdkDir) {
        $inc = Join-Path $env:WindowsSdkDir 'Include'
        if (Test-Path $inc) {
            $v = Get-ChildItem $inc -Directory |
                 Where-Object { $_.Name -match '^10\.' } |
                 Sort-Object Name -Descending | Select-Object -First 1
            if ($v) { return $v.FullName }
        }
    }
    foreach ($base in @("${env:ProgramFiles(x86)}\Windows Kits\10\Include",
                        "$env:ProgramFiles\Windows Kits\10\Include")) {
        if (Test-Path $base) {
            $v = Get-ChildItem $base -Directory |
                 Where-Object { $_.Name -match '^10\.' } |
                 Sort-Object Name -Descending | Select-Object -First 1
            if ($v) { return $v.FullName }
        }
    }
    # EWDK mounts: scan fixed drives for the kit layout.
    foreach ($d in (Get-PSDrive -PSProvider FileSystem)) {
        $p = Join-Path $d.Root 'Program Files\Windows Kits\10\Include'
        if (Test-Path $p) {
            $v = Get-ChildItem $p -Directory |
                 Where-Object { $_.Name -match '^10\.' } |
                 Sort-Object Name -Descending | Select-Object -First 1
            if ($v) { return $v.FullName }
        }
    }
    return $null
}

if (-not $IncludeRoot) { $IncludeRoot = Find-IncludeRoot }
if (-not $IncludeRoot -or -not (Test-Path $IncludeRoot)) {
    throw "Could not find the WDK Include\<version> directory. Pass -IncludeRoot explicitly."
}

Write-Host "Kit include root: $IncludeRoot"

# --- find the headers ------------------------------------------------
$headers = @{}
foreach ($name in @('dot11wdi.h','ndis.h','wditypes.hpp','TlvGeneratorParser.hpp','windot11.h')) {
    $f = Get-ChildItem -Path $IncludeRoot -Filter $name -Recurse -File -ErrorAction SilentlyContinue |
         Select-Object -First 1
    if ($f) { $headers[$name] = $f.FullName }
}

$out = New-Object System.Collections.Generic.List[string]
$out.Add("WDK symbol dump")
$out.Add("Include root : $IncludeRoot")
$out.Add("Generated for: qemu-vwifi drivers/vwifi/windows")
$out.Add('')
$out.Add('Headers found:')
foreach ($k in $headers.Keys | Sort-Object) { $out.Add("  $k -> $($headers[$k])") }
foreach ($k in @('dot11wdi.h','ndis.h','wditypes.hpp','TlvGeneratorParser.hpp')) {
    if (-not $headers.ContainsKey($k)) { $out.Add("  MISSING: $k") }
}
$out.Add('')

# --- helpers ---------------------------------------------------------

# Print a declaration block: from the first line matching $Anchor,
# forward until braces balance and a ';' is seen, capped at $Max lines.
function Add-Block {
    param([string]$File, [string]$Anchor, [int]$Max = 90, [int]$Back = 6)

    if (-not $File -or -not (Test-Path $File)) { return }
    $lines = Get-Content -LiteralPath $File
    $hits = 0
    for ($i = 0; $i -lt $lines.Count; $i++) {
        if ($lines[$i] -notmatch $Anchor) { continue }
        $hits++
        # walk back over attributes/typedef lead-in
        $start = $i
        for ($b = 1; $b -le $Back -and ($i - $b) -ge 0; $b++) {
            $prev = $lines[$i - $b].Trim()
            if ($prev -eq '' -or $prev -match '^\}' -or $prev -match ';\s*$') { break }
            $start = $i - $b
        }
        $depth = 0; $seen = $false
        for ($j = $start; $j -lt [Math]::Min($lines.Count, $start + $Max); $j++) {
            $out.Add($lines[$j])
            $depth += ([regex]::Matches($lines[$j], '\{')).Count
            $depth -= ([regex]::Matches($lines[$j], '\}')).Count
            if ($depth -gt 0) { $seen = $true }
            if ($lines[$j] -match ';\s*$' -and $depth -le 0 -and ($seen -or $j -gt $start)) { break }
        }
        $out.Add('')
        if ($hits -ge 3) { break }
    }
    if ($hits -eq 0) { $out.Add("  (no match for /$Anchor/ in $(Split-Path $File -Leaf))"); $out.Add('') }
}

# Print every line matching $Pattern, deduplicated. For flat #defines.
function Add-Grep {
    param([string]$File, [string]$Pattern, [int]$Limit = 200)
    if (-not $File -or -not (Test-Path $File)) { return }
    $m = Select-String -LiteralPath $File -Pattern $Pattern -AllMatches |
         ForEach-Object { $_.Line.TrimEnd() } | Select-Object -Unique -First $Limit
    if ($m) { $m | ForEach-Object { $out.Add($_) } }
    else    { $out.Add("  (no match for /$Pattern/ in $(Split-Path $File -Leaf))") }
    $out.Add('')
}

function Add-Header { param([string]$Text)
    $out.Add(''); $out.Add('=' * 70); $out.Add($Text); $out.Add('=' * 70); $out.Add('')
}

$dot11wdi = $headers['dot11wdi.h']
$ndis     = $headers['ndis.h']
$wditypes = $headers['wditypes.hpp']
$tlvgen   = $headers['TlvGeneratorParser.hpp']

# --- 1. the WDI handler role types -----------------------------------
Add-Header '1. MINIPORT_WDI_* handler typedefs (dot11wdi.h)'
Add-Grep $dot11wdi '^\s*(typedef\s+)?.*\bMINIPORT_WDI_[A-Z_0-9]+\b'

# --- 2. the characteristics struct -----------------------------------
Add-Header '2. NDIS_MINIPORT_DRIVER_WDI_CHARACTERISTICS (dot11wdi.h)'
Add-Block $dot11wdi 'NDIS_MINIPORT_DRIVER_WDI_CHARACTERISTICS' 140
Add-Grep  $dot11wdi 'NDIS_(SIZEOF_)?MINIPORT_WDI_CHARACTERISTICS_REVISION'
Add-Grep  $dot11wdi 'NDIS_OBJECT_TYPE_MINIPORT_WDI_CHARACTERISTICS'

# --- 3. WDI_MESSAGE_HEADER -------------------------------------------
Add-Header '3. WDI_MESSAGE_HEADER (field names for wdi_common.c)'
Add-Block $dot11wdi 'WDI_MESSAGE_HEADER' 60
if ($wditypes) { Add-Block $wditypes 'WDI_MESSAGE_HEADER' 60 }

# --- 4. handler parameter structs ------------------------------------
Add-Header '4. Handler parameter types (wdi_ops.c signatures)'
foreach ($t in @('WDI_ADAPTER_ATTRIBUTES',
                 'WDI_OPEN_ADAPTER_PARAMETERS',
                 'WDI_HANG_DIAGNOSTICS_BUFFER',
                 'NDIS_WDI_IDLE_NOTIFICATION_INFO',
                 'NDIS_WDI_INIT_PARAMETERS')) {
    $out.Add("--- $t ---")
    Add-Grep $dot11wdi "\b_?$t\b"
}

# --- 5. status indication codes --------------------------------------
Add-Header '5. NDIS_STATUS_WDI_INDICATION_* codes'
Add-Grep $dot11wdi '#define\s+NDIS_STATUS_WDI_INDICATION_\w+'

# --- 6. WDI version constants ----------------------------------------
Add-Header '6. WDI version constants'
Add-Grep $dot11wdi '#define\s+WDI_VERSION\w*'

# --- 7. ndis.h unload role type --------------------------------------
Add-Header '7. ndis.h: MINIPORT_UNLOAD / MINIPORT_DRIVER_UNLOAD'
Add-Grep $ndis '\bMINIPORT_(DRIVER_)?UNLOAD\b'

# --- 8. TLV parse/generate entry points ------------------------------
Add-Header '8. TlvGeneratorParser.hpp: Parse*/Generate*/Cleanup* entry points'
Add-Grep $tlvgen '^\s*\w[\w\s\*]*\b(Parse|Generate|CleanupParsed)Wdi\w+\s*\(' 400

Set-Content -LiteralPath $OutFile -Value $out -Encoding UTF8
Write-Host ''
Write-Host "Wrote $OutFile ($($out.Count) lines)."
Write-Host 'Send that file back.'
