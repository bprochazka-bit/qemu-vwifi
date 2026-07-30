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

rem --- locate TlvGeneratorParser.hpp ------------------------------
rem  Unlike dot11wdi.h and wditypes.hpp, this one is not on the kit's
rem  default include path. Find it once and pass its directory in.
rem  Set WdiTlvIncludeDir yourself to skip the search.
if not defined WdiTlvIncludeDir (
    echo Locating TlvGeneratorParser.hpp ...
    for /f "usebackq delims=" %%D in (`powershell -NoProfile -ExecutionPolicy Bypass -Command ^
        "$roots = @($env:WindowsSdkDir, ${env:ProgramFiles(x86)} + '\Windows Kits\10', $env:WDKContentRoot) | Where-Object { $_ -and (Test-Path $_) };" ^
        "foreach ($r in $roots) {" ^
        "  $h = Get-ChildItem -Path $r -Filter 'TlvGeneratorParser.hpp' -Recurse -File -ErrorAction SilentlyContinue | Select-Object -First 1;" ^
        "  if ($h) { $h.DirectoryName; break } }"`) do set "WdiTlvIncludeDir=%%D"
)

if not defined WdiTlvIncludeDir (
    echo.
    echo WARNING: TlvGeneratorParser.hpp not found in the mounted kit.
    echo   tlv_shim.cpp will fail with C1083. Find it with:
    echo     dir /s /b "%%ProgramFiles(x86)%%\Windows Kits\10\TlvGeneratorParser.hpp"
    echo   then re-run as:  set WdiTlvIncludeDir=^<its folder^> ^&^& build.cmd
    echo.
) else (
    echo   TlvGeneratorParser.hpp: %WdiTlvIncludeDir%
)

echo Building vwifi %CFG%^|x64 ...
msbuild "%~dp0vwifi.sln" ^
    /t:Build ^
    /p:Configuration=%CFG% ^
    /p:Platform=x64 ^
    /p:WdiTlvIncludeDir="%WdiTlvIncludeDir%" ^
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
