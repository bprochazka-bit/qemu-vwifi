@echo off
rem ---------------------------------------------------------------
rem  vwifi — assemble, catalog and test-sign the driver package.
rem
rem      sign.cmd               rem signs x64\Debug\vwifi
rem      sign.cmd Release
rem
rem  Run it from the same EWDK build shell as build.cmd — Inf2Cat and
rem  signtool come from that environment.
rem
rem  This is a wrapper; the work is in sign.ps1. Each step there
rem  verifies its own output rather than trusting an exit code, because
rem  Inf2Cat prints "Signability test failed." and still exits 0.
rem
rem  The exported certificate (vwifi-test-cert.cer) must be imported
rem  into the *guest's* Root and TrustedPublisher stores — see
rem  guest-debug-setup.ps1.
rem ---------------------------------------------------------------
set "CFG=%~1"
if "%CFG%"=="" set "CFG=Debug"
powershell -NoProfile -ExecutionPolicy Bypass -File "%~dp0sign.ps1" -Configuration %CFG%
exit /b %errorlevel%
