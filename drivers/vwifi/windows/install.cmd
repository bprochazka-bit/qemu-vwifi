@echo off
rem ---------------------------------------------------------------
rem  vwifi — install or replace the driver in the guest.
rem
rem  Run from an ELEVATED command prompt in the Windows VM, from the
rem  folder holding vwifi.sys / vwifi.inf / vwifi.cat:
rem
rem      install.cmd
rem
rem  Replacing a driver is not the same as installing one. pnputil
rem  /add-driver adds a package but PnP keeps using the one already
rem  bound to the device unless the new package looks *better* --
rem  which, with an equal DriverVer, it does not. So this removes the
rem  old package outright rather than hoping the new one wins.
rem ---------------------------------------------------------------
setlocal enabledelayedexpansion

net session >nul 2>&1
if errorlevel 1 (
    echo ERROR: run this from an elevated command prompt.
    exit /b 1
)

if not exist "%~dp0vwifi.inf" (
    echo ERROR: vwifi.inf not found next to this script.
    echo   Copy vwifi.sys, vwifi.inf and vwifi.cat here first.
    exit /b 1
)
if not exist "%~dp0vwifi.cat" (
    echo WARNING: no vwifi.cat — the package is unsigned and will be
    echo          rejected unless the .sys carries an embedded signature.
)

rem --- 1. remove every previously installed vwifi package -------------
echo.
echo [1/4] Removing previously installed vwifi packages ...
set FOUND=
for /f "usebackq tokens=*" %%P in (`powershell -NoProfile -Command ^
    "(pnputil /enum-drivers) -join \"`n\" -split \"`n`n\" | Where-Object { $_ -match 'vwifi\.inf' } | ForEach-Object { if ($_ -match 'Published Name:\s*(oem\d+\.inf)') { $Matches[1] } }"`) do (
    set FOUND=1
    echo   removing %%P
    pnputil /delete-driver %%P /uninstall /force >nul 2>&1
    if errorlevel 1 echo     ^(delete reported an error; continuing^)
)
if not defined FOUND echo   none installed.

rem --- 2. add and install the new package -----------------------------
echo.
echo [2/4] Installing "%~dp0vwifi.inf" ...
pnputil /add-driver "%~dp0vwifi.inf" /install
if errorlevel 1 (
    echo.
    echo ERROR: pnputil failed. The usual causes:
    echo   * test-signing off, or Secure Boot / memory integrity on
    echo   * the test certificate not in Root AND TrustedPublisher
    echo   * vwifi.cat stale — re-run sign.cmd after every build
    echo   %%windir%%\inf\setupapi.dev.log records what PnP actually did.
    exit /b 1
)

rem --- 3. make PnP re-evaluate the device -----------------------------
echo.
echo [3/4] Rescanning for devices ...
pnputil /scan-devices >nul 2>&1

rem --- 4. report where the device ended up ----------------------------
echo.
echo [4/4] Device status:
powershell -NoProfile -Command ^
  "$d = Get-PnpDevice -InstanceId 'PCI\VEN_1AF4&DEV_0E00*' -ErrorAction SilentlyContinue;" ^
  "if (-not $d) { Write-Host '  no vwifi-virt device present — is -device vwifi-virt on the QEMU command line?'; exit }" ^
  "foreach ($x in $d) {" ^
  "  Write-Host ('  {0}' -f $x.FriendlyName);" ^
  "  Write-Host ('  Status: {0}   Problem: {1} {2}' -f $x.Status, $x.ProblemCode, $x.ProblemDescription) }"

echo.
echo If it did not start, capture the driver's own account of why:
echo   DebugView as admin, Capture Kernel + Enable Verbose Kernel Output,
echo   then disable and re-enable the device to re-run init with the
echo   capture already attached.
endlocal
