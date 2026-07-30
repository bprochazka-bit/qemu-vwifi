<#
.SYNOPSIS
    Locate a file in the installed/mounted Windows kit.

.DESCRIPTION
    Used by build.cmd to find the two WDI TLV pieces that are not on the
    kit's default include/library paths: TlvGeneratorParser.hpp and
    TLVGeneratorParser.lib.

    This lives in its own script rather than inline in build.cmd on
    purpose. The search needs ${env:ProgramFiles(x86)} and a pipeline
    full of parentheses, and embedding that in a cmd `for /f` inside an
    `if (...)` block is how you get "was unexpected at this time" — cmd
    parses the whole parenthesised block in one pass and the parens in
    the command terminate it early. A .ps1 file has no such hazard.

    Prints the single best match to stdout and nothing else, so the
    caller can capture it with `for /f`. Prints nothing if not found.

.PARAMETER Name
    File name to find, e.g. TlvGeneratorParser.hpp

.PARAMETER Subtree
    Restrict the search to this folder under the kit root (Include or
    Lib). Keeps the recursion to a few thousand files.

.PARAMETER WantDirectory
    Print the containing directory instead of the full file path.

.PARAMETER RequirePathMatch
    Only accept matches whose full path contains this substring — used
    to pick the x64 library out of the per-architecture copies.
#>
[CmdletBinding()]
param(
    [Parameter(Mandatory = $true)] [string] $Name,
    [string] $Subtree,
    [switch] $WantDirectory,
    [string] $RequirePathMatch
)

$ErrorActionPreference = 'SilentlyContinue'

# Candidate kit roots, best first. WindowsSdkDir is set by
# LaunchBuildEnv.cmd and by the VS developer prompt, and points at the
# kit actually in use — which is not necessarily the one on C:.
$roots = New-Object System.Collections.Generic.List[string]
foreach ($r in @($env:WindowsSdkDir,
                 $env:WDKContentRoot,
                 (Join-Path ${env:ProgramFiles(x86)} 'Windows Kits\10'),
                 (Join-Path $env:ProgramFiles 'Windows Kits\10'))) {
    if ($r -and (Test-Path $r)) { $roots.Add((Resolve-Path $r).Path) }
}

# Then any fixed drive with a kit on it — covers a kit installed to D:.
foreach ($d in (Get-PSDrive -PSProvider FileSystem)) {
    foreach ($suffix in @('Program Files\Windows Kits\10',
                          'Program Files (x86)\Windows Kits\10')) {
        $p = Join-Path $d.Root $suffix
        if (Test-Path $p) { $roots.Add($p) }
    }
}

foreach ($root in ($roots | Select-Object -Unique)) {
    $scope = if ($Subtree) { Join-Path $root $Subtree } else { $root }
    if (-not (Test-Path $scope)) { continue }

    $hits = Get-ChildItem -Path $scope -Filter $Name -Recurse -File
    if ($RequirePathMatch) {
        $hits = $hits | Where-Object { $_.FullName -like "*$RequirePathMatch*" }
    }
    # Highest kit version wins when several are installed side by side.
    $hit = $hits | Sort-Object FullName -Descending | Select-Object -First 1
    if ($hit) {
        if ($WantDirectory) { Write-Output $hit.DirectoryName }
        else                { Write-Output $hit.FullName }
        exit 0
    }
}

exit 1
