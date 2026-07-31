<#
.SYNOPSIS
    Assemble, catalog and test-sign the vwifi driver package.

.DESCRIPTION
    The project builds unsigned on purpose (SignMode=Off), so that a
    missing certificate is a signing failure rather than a build
    failure. This does the rest:

      1. assemble x64\<cfg>\vwifi\ from the flat build output
      2. create a self-signed test certificate the first time
      3. Inf2Cat over the package folder
      4. signtool the .cat and the .sys

    Each step verifies its own output rather than trusting an exit
    code. Inf2Cat in particular prints "Signability test failed." and
    still returns 0, so a catalog that was never written would
    otherwise sail through to a confusing signtool error.

.PARAMETER Configuration
    Debug (default) or Release.
#>
[CmdletBinding()]
param(
    [string] $Configuration = 'Debug'
)

$ErrorActionPreference = 'Stop'

$out      = Join-Path $PSScriptRoot "x64\$Configuration"
$pkg      = Join-Path $out 'vwifi'
$certName = 'vwifi-test-cert'
$certFile = Join-Path $PSScriptRoot "$certName.cer"

function Fail([string] $Message) {
    Write-Host ''
    Write-Host "ERROR: $Message" -ForegroundColor Red
    exit 1
}

# --- 1. assemble the package folder ----------------------------------
# MSBuild only creates x64\<cfg>\vwifi\ as part of its own signing step,
# which the project turns off, so the build output is flat in
# x64\<cfg>. Inf2Cat catalogs a *directory*, so gather it here.
Write-Host "Assembling $pkg ..."

if (-not (Test-Path $out)) {
    Fail "$out does not exist. Run build.cmd $Configuration first."
}

$need = @('vwifi.sys', 'vwifi.inf')
$missing = $need | Where-Object { -not (Test-Path (Join-Path $out $_)) }
if ($missing) {
    Fail ("$out is missing: {0}`n       Run build.cmd $Configuration -- a failed link leaves the .sys behind,`n       and a skipped StampInf leaves the .inf behind." -f ($missing -join ', '))
}

New-Item -ItemType Directory -Path $pkg -Force | Out-Null

# The .pdb is not catalogued, but it travels with the .sys: WinDbg
# wants them side by side.
foreach ($f in @('vwifi.sys', 'vwifi.inf', 'vwifi.pdb')) {
    $src = Join-Path $out $f
    if (Test-Path $src) {
        Copy-Item -LiteralPath $src -Destination $pkg -Force
    }
}

# Verify rather than trust: a copy that silently did nothing is exactly
# what produced "Could not find file ...\vwifi\vwifi.inf" from Inf2Cat.
foreach ($f in $need) {
    if (-not (Test-Path (Join-Path $pkg $f))) {
        Fail "failed to copy $f into $pkg"
    }
}
Write-Host ("  " + ((Get-ChildItem $pkg -File | ForEach-Object Name) -join ', '))

# --- 2. certificate: create once, reuse thereafter -------------------
$cert = Get-ChildItem Cert:\CurrentUser\My |
        Where-Object { $_.Subject -eq "CN=$certName" } |
        Select-Object -First 1
if (-not $cert) {
    Write-Host "Creating test code-signing certificate CN=$certName ..."
    $cert = New-SelfSignedCertificate -Type CodeSigningCert `
                -Subject "CN=$certName" `
                -KeyUsage DigitalSignature `
                -CertStoreLocation 'Cert:\CurrentUser\My' `
                -HashAlgorithm sha256 `
                -NotAfter (Get-Date).AddYears(5)
} else {
    Write-Host "Reusing existing certificate CN=$certName"
}
Export-Certificate -Cert $cert -FilePath $certFile -Force | Out-Null

# --- 3. catalog ------------------------------------------------------
Write-Host "Cataloging $pkg ..."
$cat = Join-Path $pkg 'vwifi.cat'
Remove-Item -LiteralPath $cat -Force -ErrorAction SilentlyContinue

$inf2catOutput = & Inf2Cat /driver:"$pkg" /os:10_x64,Server10_x64 /uselocaltime 2>&1
$inf2catOutput | ForEach-Object { Write-Host "  $_" }

# Inf2Cat reports "Signability test failed." and still exits 0. The
# only reliable signal is whether the catalog exists.
if (-not (Test-Path $cat)) {
    Fail @"
Inf2Cat produced no catalog. Its own output is above; the usual causes:
  * a directive in the INF that is not signable for the target OS
  * the INF referencing a file that is not in the package folder
%windir%\inf\setupapi.dev.log is not involved yet -- this is purely the
INF's own contents.
"@
}

# --- 4. sign ---------------------------------------------------------
# No /t timestamp URL: test certificates expire on their own schedule
# and the build machine may have no outbound network.
Write-Host 'Signing ...'
foreach ($f in @('vwifi.cat', 'vwifi.sys')) {
    $target = Join-Path $pkg $f
    & signtool sign /v /fd sha256 /s My /n $certName $target
    if ($LASTEXITCODE -ne 0) { Fail "signtool failed on $f" }
}

Write-Host ''
Write-Host 'Signed. Copy this folder to the guest:'
Write-Host "  $pkg"
Write-Host 'plus the certificate, for the one-time guest setup:'
Write-Host "  $certFile"
