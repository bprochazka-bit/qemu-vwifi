<#
.SYNOPSIS
    Shared symbol-server helpers: locate WDK tools, read a PE's CodeView
    record, turn it into a symbol-server URL, fetch the PDB.

.DESCRIPTION
    Dot-source this; it defines functions and does nothing on its own.

    A symbol server request is fully determined by three things stored
    in the image itself -- the PDB's base name, its GUID and its age,
    all in the PE CodeView (RSDS) debug record. Reading that record is
    the whole of what symchk does before it talks to the server, and
    doing it here means the EWDK's lack of Debugging Tools stops
    mattering.

    The key identifies one specific build, which matters beyond
    convenience: !ndiskd walks ndis.sys structures by symbol, and WPP
    message ids are assigned per build, so a near-miss PDB gives
    confidently wrong answers rather than obvious failure.

    The PE offsets are fiddly -- the debug data directory sits at a
    different place in PE32 and PE32+, and the RVA needs mapping through
    the section table -- so they were checked against a synthetic image
    with a known GUID, age and PDB name before being trusted. So was the
    byte order: .NET's Guid.ToByteArray gives the little-endian on-disk
    layout, not the display order, and searching for the display order
    would find nothing while looking like a clean negative.
#>

function Find-WdkTool {
    param([Parameter(Mandatory)][string] $Name)

    $cmd = Get-Command $Name -ErrorAction SilentlyContinue
    if ($cmd) { return $cmd.Source }

    # tracefmt/tracepdb live under bin\<ver>\<arch>; kd, windbg and
    # symchk under Debuggers\<arch>. WindowsSdkDir and
    # WindowsSdkVerBinPath are set by the EWDK's LaunchBuildEnv, which
    # carries the build tools but not the Debugging Tools -- so the
    # Debuggers directories are searched separately and are often absent.
    $roots = @(
        $env:WindowsSdkVerBinPath,
        $env:WindowsSdkDir,
        "${env:ProgramFiles(x86)}\Windows Kits\10\bin",
        "${env:ProgramFiles(x86)}\Windows Kits\10\Debuggers",
        "${env:ProgramFiles}\Windows Kits\10\bin",
        "${env:ProgramFiles}\Windows Kits\10\Debuggers",
        "$env:SystemDrive\Debuggers"
    ) | Where-Object { $_ -and (Test-Path $_) }

    foreach ($r in $roots) {
        $hit = Get-ChildItem -Path $r -Filter $Name -Recurse -ErrorAction SilentlyContinue |
               Sort-Object FullName -Descending | Select-Object -First 1
        if ($hit) { return $hit.FullName }
    }
    return $null
}

function Get-PeCodeViewInfo {
    param([Parameter(Mandatory)][string] $Path)

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
    $ddOff   = if ($magic -eq 0x20B) { $opt + 112 } else { $opt + 96 }
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
        if ([BitConverter]::ToUInt32($b, $e + 12) -ne 2) { continue }   # CODEVIEW
        $raw = [BitConverter]::ToUInt32($b, $e + 24)
        if ($raw -eq 0 -or ($raw + 24) -ge $b.Length) { continue }
        if ([Text.Encoding]::ASCII.GetString($b, $raw, 4) -ne 'RSDS') { continue }

        $guid = [guid]::new([byte[]]$b[($raw + 4)..($raw + 19)])
        $age  = [BitConverter]::ToUInt32($b, $raw + 20)

        $p = $raw + 24; $q = $p
        while ($q -lt $b.Length -and $b[$q] -ne 0) { $q++ }
        $name = [IO.Path]::GetFileName([Text.Encoding]::ASCII.GetString($b, $p, $q - $p))

        $key = '{0}{1:X}' -f $guid.ToString('N').ToUpper(), $age
        return [pscustomobject]@{
            Image   = $Path
            PdbName = $name
            Key     = $key
            Url     = 'https://msdl.microsoft.com/download/symbols/{0}/{1}/{0}' -f $name, $key
        }
    }
    return $null
}

function Get-SymbolFile {
    param(
        [Parameter(Mandatory)][string] $ImagePath,
        [Parameter(Mandatory)][string] $DestDir
    )

    $cv = Get-PeCodeViewInfo $ImagePath
    if (-not $cv) {
        Write-Warning "could not read a CodeView record from $ImagePath"
        return $null
    }

    $dest = Join-Path $DestDir $cv.PdbName
    if (Test-Path $dest) { return $dest }
    New-Item -ItemType Directory -Force -Path $DestDir | Out-Null

    Write-Host "      GET $($cv.Url)"
    try {
        # PowerShell 5.1 still defaults to TLS 1.0, which msdl refuses.
        [Net.ServicePointManager]::SecurityProtocol = [Net.SecurityProtocolType]::Tls12
        Invoke-WebRequest -Uri $cv.Url -OutFile $dest -UseBasicParsing `
            -UserAgent 'Microsoft-Symbol-Server/10.0.0.0' -TimeoutSec 120
    } catch {
        # A test machine whose only network adapter is the one being
        # debugged cannot reach the symbol server, so this failing is
        # expected rather than exceptional. The URL above is everything
        # needed to fetch the file from anywhere else.
        Write-Warning "symbol download failed: $($_.Exception.Message)"
        Remove-Item $dest -ErrorAction SilentlyContinue
        return $null
    }

    if (Test-Path $dest) { return $dest }
    return $null
}
