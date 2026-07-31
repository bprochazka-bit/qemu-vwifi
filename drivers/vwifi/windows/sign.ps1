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

# --- 3. catalog, on a local drive -------------------------------------
# Inf2Cat cannot read a driver directory on a mapped network drive: it
# reports "Could not find file <path>" for a file that is demonstrably
# there, then "Signability test failed.", then exits 0. Working from a
# UNC path fails the same way. So catalog and sign in a local staging
# folder and copy the results back, which costs three file copies and
# removes the question entirely.
$stage = Join-Path $env:TEMP 'vwifi-sign-stage'
Remove-Item -LiteralPath $stage -Recurse -Force -ErrorAction SilentlyContinue
New-Item -ItemType Directory -Path $stage -Force | Out-Null

foreach ($f in @('vwifi.sys', 'vwifi.inf')) {
    Copy-Item -LiteralPath (Join-Path $pkg $f) -Destination $stage -Force
}

Write-Host "Cataloging (staged in $stage) ..."
$stagedCat = Join-Path $stage 'vwifi.cat'

$inf2catOutput = & Inf2Cat /driver:"$stage" /os:10_x64,Server10_x64 /uselocaltime 2>&1
$inf2catOutput | ForEach-Object { Write-Host "  $_" }

# Inf2Cat reports "Signability test failed." and still exits 0. The only
# reliable signal is whether the catalog exists.
if (-not (Test-Path $stagedCat)) {
    Fail @"
Inf2Cat produced no catalog. Its own output is above; the usual causes:
  * a directive in the INF that is not signable for the target OS
  * the INF referencing a file that is not in the staging folder
The staging folder is on a local drive, so this is no longer the
mapped-drive problem -- it is the INF's own contents.
"@
}

# --- 4. sign ----------------------------------------------------------
# No /t timestamp URL: test certificates expire on their own schedule
# and the build machine may have no outbound network.
Write-Host 'Signing ...'
foreach ($f in @('vwifi.cat', 'vwifi.sys')) {
    $target = Join-Path $stage $f
    & signtool sign /v /fd sha256 /s My /n $certName $target
    if ($LASTEXITCODE -ne 0) { Fail "signtool failed on $f" }
}

# --- 5. bring the signed package back ---------------------------------
foreach ($f in @('vwifi.sys', 'vwifi.inf', 'vwifi.cat')) {
    Copy-Item -LiteralPath (Join-Path $stage $f) -Destination $pkg -Force
}
Remove-Item -LiteralPath $stage -Recurse -Force -ErrorAction SilentlyContinue

foreach ($f in @('vwifi.sys', 'vwifi.inf', 'vwifi.cat')) {
    if (-not (Test-Path (Join-Path $pkg $f))) {
        Fail "signed $f did not make it back into $pkg"
    }
}

# --- 6. make the package self-sufficient ------------------------------
# The guest needs the install scripts, the one-time setup script and the
# certificate as well as the driver. Putting them in the package folder
# means one folder gets copied across and nothing is assembled by hand
# -- and it stops install.cmd being run from the source tree, where
# there is no driver to install.
foreach ($f in @('install.cmd', 'install.ps1', 'guest-debug-setup.ps1')) {
    $src = Join-Path $PSScriptRoot $f
    if (Test-Path $src) { Copy-Item -LiteralPath $src -Destination $pkg -Force }
}
if (Test-Path $certFile) {
    Copy-Item -LiteralPath $certFile -Destination $pkg -Force
}

Write-Host ''
Write-Host 'Signed. Copy this one folder to the guest -- it now holds the'
Write-Host 'driver, the install scripts, the guest setup script and the'
Write-Host 'certificate:'
Write-Host "  $pkg"
Write-Host ''
Write-Host 'In the guest, from that folder, elevated:'
Write-Host '  .\guest-debug-setup.ps1 -CertPath .\vwifi-test-cert.cer   (once, then reboot)'
Write-Host '  install.cmd                                               (after every build)'
