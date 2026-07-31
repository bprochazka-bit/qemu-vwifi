<#
.SYNOPSIS
    Locate a file in the installed/mounted Windows kit.

.DESCRIPTION
    Used by build.cmd to find the two WDI TLV pieces that are not on the
    kit's default include/library paths: TlvGeneratorParser.hpp and
    TLVGeneratorParser.lib.

    Prints the single best match to stdout and nothing else, so the
    caller can capture it. Every candidate considered, and why it was
    accepted or rejected, goes to stderr — so a wrong or empty result is
    never silent.

    This lives in its own script rather than inline in build.cmd because
    the search needs ${env:ProgramFiles(x86)} and a pipeline full of
    parentheses, and embedding that in a cmd `for /f` inside an
    `if (...)` block is how you get "was unexpected at this time".

.PARAMETER Name
    File name to find, e.g. TlvGeneratorParser.hpp

.PARAMETER Subtree
    Restrict the search to this folder under the kit root (Include or
    Lib). Keeps the recursion to a few thousand files.

.PARAMETER WantDirectory
    Print the containing directory instead of the full file path.

.PARAMETER MustContain
    Semicolon-separated substrings. A candidate must contain ALL of
    them. This is load-bearing: a kit ships parallel copies of the WDI
    TLV files, and picking the wrong one produces a wall of errors
    inside the Microsoft header rather than anything pointing back here.
    This driver needs the kernel-mode copy — under \km — matching the
    dot11wdi.h it is written against.

.PARAMETER MustNotContain
    Semicolon-separated substrings. A candidate containing any of them
    is rejected. \um is the user-mode WDI 2.0 copy: it references types
    like MLO_LINK_INFO that do not exist in a kernel translation unit,
    and its C_ASSERTs collide with wdm.h's.

.PARAMETER Prefer
    If several candidates survive, prefer one whose path contains this.

.NOTES
    Parameters are single quoted tokens, split by this script rather
    than by PowerShell. `powershell -File` passes every argument as a
    literal string and does not evaluate `a,b` as array syntax the way
    an inline -Command would, so a [string[]] parameter binds the whole
    thing as one element and matches nothing.
#>
[CmdletBinding()]
param(
    [Parameter(Mandatory = $true)] [string] $Name,
    [string] $Subtree,
    [switch] $WantDirectory,
    [string] $MustContain,
    [string] $MustNotContain,
    [string] $Prefer
)

$ErrorActionPreference = 'SilentlyContinue'

function Split-List([string] $s) {
    if (-not $s) { return @() }
    return @($s -split ';' | Where-Object { $_ })
}

$must    = Split-List $MustContain
$mustNot = Split-List $MustNotContain

# Diagnostics to stderr: they reach the console without polluting the
# single line of stdout the caller captures.
function Note([string] $m) { [Console]::Error.WriteLine("    $m") }

# The answer goes straight to the console stream, NOT through
# Write-Output. Write-Output runs through PowerShell's formatter, which
# pads and wraps to the host width, so the caller captures a path with
# trailing spaces or split across lines. MSBuild trims property values,
# so a padded path still produced a correct /I and this went unnoticed;
# a plain `if exist "%VAR%\file"` in cmd does not trim, and fails on a
# path that looks perfect when echoed.
function Emit([string] $s) { [Console]::Out.WriteLine($s) }

# Candidate kit roots, best first. WindowsSdkDir is set by
# LaunchBuildEnv.cmd and by the VS developer prompt, and points at the
# kit actually in use — not necessarily the one on C:.
$roots = New-Object System.Collections.Generic.List[string]
foreach ($r in @($env:WindowsSdkDir,
                 $env:WDKContentRoot,
                 (Join-Path ${env:ProgramFiles(x86)} 'Windows Kits\10'),
                 (Join-Path $env:ProgramFiles 'Windows Kits\10'))) {
    if ($r -and (Test-Path $r)) { $roots.Add((Resolve-Path $r).Path) }
}
foreach ($d in (Get-PSDrive -PSProvider FileSystem)) {
    foreach ($suffix in @('Program Files\Windows Kits\10',
                          'Program Files (x86)\Windows Kits\10')) {
        $p = Join-Path $d.Root $suffix
        if (Test-Path $p) { $roots.Add($p) }
    }
}

$seen = @()
foreach ($root in ($roots | Select-Object -Unique)) {
    $scope = if ($Subtree) { Join-Path $root $Subtree } else { $root }
    if (-not (Test-Path $scope)) { continue }

    foreach ($f in (Get-ChildItem -Path $scope -Filter $Name -Recurse -File)) {
        if ($seen -notcontains $f.FullName) { $seen += $f.FullName }
    }
}

if ($seen.Count -eq 0) {
    Note "no $Name anywhere under the kit roots:"
    foreach ($r in ($roots | Select-Object -Unique)) { Note "  $r" }
    exit 1
}

$kept = @()
foreach ($p in $seen) {
    $reject = $null
    foreach ($m in $must) {
        if ($p -notlike "*$m*") { $reject = "missing '$m'"; break }
    }
    if (-not $reject) {
        foreach ($m in $mustNot) {
            if ($p -like "*$m*") { $reject = "contains '$m'"; break }
        }
    }
    if ($reject) { Note "reject  $p  ($reject)" }
    else         { Note "accept  $p"; $kept += $p }
}

if ($kept.Count -eq 0) {
    Note "every candidate was rejected"
    exit 1
}

$pick = @($kept | Sort-Object)[0]
if ($Prefer) {
    $p2 = @($kept | Where-Object { $_ -like "*$Prefer*" } | Sort-Object)
    if ($p2.Count -gt 0) { $pick = $p2[0] }
}
if ($kept.Count -gt 1) { Note "chose   $pick" }

if ($WantDirectory) { Emit (Split-Path $pick -Parent) }
else                { Emit $pick }
exit 0
