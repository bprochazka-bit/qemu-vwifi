@echo off
rem ---------------------------------------------------------------
rem  vwifi — debug build of the Windows WDI miniport.
rem
rem  Run from an EWDK build shell (mount the ISO, run
rem  LaunchBuildEnv.cmd) or from a "Developer Command Prompt for
rem  VS 2022" with the WDK installed:
rem
rem      cd drivers\vwifi\windows
rem      build.cmd              rem Debug|x64, the bring-up default
rem      build.cmd Release
rem
rem  Leaves three logs next to this script. build-<cfg>.err is the one
rem  to paste when reporting a failure — it is errors only, in order.
rem ---------------------------------------------------------------
setlocal

set CFG=%~1
if "%CFG%"=="" set CFG=Debug

where msbuild >nul 2>&1
if errorlevel 1 (
    echo.
    echo ERROR: msbuild is not on PATH.
    echo.
    echo   EWDK:           mount the ISO and run LaunchBuildEnv.cmd, then
    echo                   re-run this script from that shell.
    echo   Visual Studio:  use "Developer Command Prompt for VS 2022".
    echo.
    exit /b 1
)

echo Building vwifi %CFG%^|x64 ...
msbuild "%~dp0vwifi.sln" ^
    /t:Build ^
    /p:Configuration=%CFG% ^
    /p:Platform=x64 ^
    /m ^
    /v:minimal ^
    /nologo ^
    /flp:logfile="%~dp0build-%CFG%.log";verbosity=normal ^
    /flp1:logfile="%~dp0build-%CFG%.err";errorsonly ^
    /flp2:logfile="%~dp0build-%CFG%.wrn";warningsonly

if errorlevel 1 (
    echo.
    echo BUILD FAILED. Errors:
    echo ---------------------------------------------------------------
    type "%~dp0build-%CFG%.err"
    echo ---------------------------------------------------------------
    echo Full log: %~dp0build-%CFG%.log
    exit /b 1
)

echo.
echo Build OK. Driver package:
echo   %~dp0x64\%CFG%\vwifi\
dir /b "%~dp0x64\%CFG%\vwifi" 2>nul
echo.
echo Warnings (review these^): %~dp0build-%CFG%.wrn
echo Next: sign.cmd %CFG%
endlocal
