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
    Only accept matches whose full path contains ALL of these
    substrings. This is load-bearing, not a convenience: a kit carries
    several parallel copies of the WDI TLV files and picking the wrong
    one produces a wall of errors inside the Microsoft header rather
    than anything pointing back here.

    The pair this driver needs is km\wlan\1.0 — kernel-mode, WDI 1.x —
    matching the dot11wdi.h it is written against (WDI_VERSION_LATEST
    there is 1.1.13). The um\wlan\2.0 copy is user-mode WDI 2.0: it
    references types like MLO_LINK_INFO that do not exist in a
    kernel-mode translation unit, and its C_ASSERTs collide with
    wdm.h's.
#>
[CmdletBinding()]
param(
    [Parameter(Mandatory = $true)] [string] $Name,
    [string] $Subtree,
    [switch] $WantDirectory,
    [string[]] $RequirePathMatch
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
    foreach ($m in $RequirePathMatch) {
        $hits = $hits | Where-Object { $_.FullName -like "*$m*" }
    }
    # Deterministic pick among what survives. Sorting descending on the
    # whole path was a mistake: it sorts 'um' above 'km' and '2.0' above
    # '1.0', so it chose exactly the copy that cannot be compiled in
    # kernel mode. With the km\wlan\1.0 requirement above there is
    # normally one candidate; sort only to stay deterministic if a
    # machine has several kit versions installed.
    $hit = $hits | Sort-Object FullName | Select-Object -First 1
    if ($hit) {
        if ($WantDirectory) { Write-Output $hit.DirectoryName }
        else                { Write-Output $hit.FullName }
        exit 0
    }
}

exit 1
