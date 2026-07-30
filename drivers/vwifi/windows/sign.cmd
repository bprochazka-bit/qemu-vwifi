@echo off
rem ---------------------------------------------------------------
rem  vwifi — test-sign the built driver package.
rem
rem  The project deliberately builds unsigned (SignMode=Off) so that a
rem  missing certificate is a signing failure, not a build failure.
rem  This script does the rest: create a self-signed test certificate
rem  if there isn't one, catalog the INF, and sign the .sys and .cat.
rem
rem      sign.cmd               rem signs x64\Debug\vwifi
rem      sign.cmd Release
rem
rem  The exported certificate (vwifi-test-cert.cer) must be imported
rem  into the *guest's* Root and TrustedPublisher stores — see
rem  guest-debug-setup.ps1.
rem ---------------------------------------------------------------
setlocal

set CFG=%~1
if "%CFG%"=="" set CFG=Debug

set OUT=%~dp0x64\%CFG%
set PKG=%OUT%\vwifi
set CERTNAME=vwifi-test-cert
set CERTFILE=%~dp0%CERTNAME%.cer

if not exist "%OUT%\vwifi.sys" (
    echo ERROR: %OUT%\vwifi.sys not found. Run build.cmd %CFG% first.
    exit /b 1
)

rem --- assemble the driver package ------------------------------------
rem  MSBuild only creates the x64\<cfg>\vwifi\ package folder as part of
rem  its own signing step, which the project turns off (SignMode=Off) so
rem  that a missing certificate cannot fail a build. The build output is
rem  therefore flat in x64\<cfg>. Inf2Cat catalogs a *directory*, so
rem  gather the three files that belong in the package here.
echo Assembling %PKG% ...
if not exist "%PKG%" mkdir "%PKG%"
copy /y "%OUT%\vwifi.sys" "%PKG%\" >nul
if errorlevel 1 exit /b 1
copy /y "%OUT%\vwifi.inf" "%PKG%\" >nul
if errorlevel 1 (
    echo ERROR: %OUT%\vwifi.inf not found — did StampInf run?
    exit /b 1
)
rem  The .pdb is not part of the catalog, but it travels with the .sys:
rem  WinDbg wants them side by side.
if exist "%OUT%\vwifi.pdb" copy /y "%OUT%\vwifi.pdb" "%PKG%\" >nul

rem --- certificate: create once, reuse thereafter -----------------
powershell -NoProfile -ExecutionPolicy Bypass -Command ^
  "$c = Get-ChildItem Cert:\CurrentUser\My | Where-Object { $_.Subject -eq 'CN=%CERTNAME%' } | Select-Object -First 1;" ^
  "if (-not $c) {" ^
  "  Write-Host 'Creating test code-signing certificate CN=%CERTNAME% ...';" ^
  "  $c = New-SelfSignedCertificate -Type CodeSigningCert -Subject 'CN=%CERTNAME%' -KeyUsage DigitalSignature -CertStoreLocation 'Cert:\CurrentUser\My' -HashAlgorithm sha256 -NotAfter (Get-Date).AddYears(5);" ^
  "} else { Write-Host 'Reusing existing certificate CN=%CERTNAME%'; }" ^
  "Export-Certificate -Cert $c -FilePath '%CERTFILE%' -Force | Out-Null;"
if errorlevel 1 exit /b 1

rem --- catalog ----------------------------------------------------
echo Cataloging %PKG% ...
Inf2Cat /driver:"%PKG%" /os:10_x64,Server10_x64 /uselocaltime
if errorlevel 1 (
    echo ERROR: Inf2Cat failed. Check the INF for signability warnings.
    exit /b 1
)

rem --- sign -------------------------------------------------------
rem  No /t timestamp URL: test certificates expire on their own
rem  schedule and the build machine may have no outbound network.
echo Signing ...
signtool sign /v /fd sha256 /s My /n "%CERTNAME%" "%PKG%\vwifi.cat"
if errorlevel 1 exit /b 1
signtool sign /v /fd sha256 /s My /n "%CERTNAME%" "%PKG%\vwifi.sys"
if errorlevel 1 exit /b 1

echo.
echo Signed. Copy these to the guest:
echo   %PKG%\vwifi.sys
echo   %PKG%\vwifi.inf
echo   %PKG%\vwifi.cat
echo   %PKG%\vwifi.pdb      (for WinDbg — keep it with the .sys)
echo   %CERTFILE%
endlocal
