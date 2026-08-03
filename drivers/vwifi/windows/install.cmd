@echo off
rem ---------------------------------------------------------------
rem  vwifi -- install or replace the driver in the guest.
rem
rem  Run from an ELEVATED command prompt in the Windows VM, from the
rem  folder holding vwifi.sys / vwifi.inf / vwifi.cat:
rem
rem      install.cmd
rem
rem  Boot the guest with the vwifi-virt device OFF the QEMU command
rem  line first. install.ps1 removes the old package, and with no
rem  device present there is nothing for the removal to stop.
rem
rem  This is a wrapper. The work is in install.ps1, deliberately: the
rem  device enumeration needs a pipeline full of parentheses, and
rem  embedding that in a cmd `for /f` is how you get "was unexpected at
rem  this time" -- cmd parses a parenthesised block in one pass and the
rem  parens in the command terminate it early.
rem ---------------------------------------------------------------
rem  Print something before doing anything, so this can never return in
rem  silence. If PowerShell fails to launch, or the .ps1 is missing, the
rem  banner is still proof the batch file ran at all.
echo vwifi install from %~dp0

if not exist "%~dp0install.ps1" goto :no_script
if not exist "%~dp0vwifi.inf" echo   note: no vwifi.inf here, install.ps1 will look under x64\Debug\vwifi

rem  "%~dp0." rather than "%~dp0": the latter ends in a backslash, and a
rem  backslash immediately before a closing quote is read as an escaped
rem  quote. The trailing dot keeps it a valid path and side-steps that.
powershell -NoProfile -ExecutionPolicy Bypass -File "%~dp0install.ps1" -Path "%~dp0." %*
set "RC=%errorlevel%"
echo.
if not "%RC%"=="0" echo install.cmd: FAILED, exit code %RC%
if "%RC%"=="0" echo install.cmd: installed. Shut down, put -device vwifi-virt back, boot.
exit /b %RC%

:no_script
echo.
echo ERROR: install.ps1 is not next to install.cmd.
echo   Copy the whole x64\Debug\vwifi folder from the build machine --
echo   sign.cmd puts the driver and the scripts in it together.
echo.
exit /b 1
