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
rem
rem  NOTE ON STYLE: the discovery steps are subroutines, not inline
rem  `if not defined X ( for /f ... )` blocks. cmd parses a
rem  parenthesised block in one pass, and a `for /f` with a caret
rem  continuation inside one breaks with "was unexpected at this time".
rem  Subroutines are parsed a line at a time and do not have the
rem  problem. Keep it that way.
rem ---------------------------------------------------------------
setlocal

set CFG=%~1
if "%CFG%"=="" set CFG=Debug

where msbuild >nul 2>&1
if errorlevel 1 goto :no_msbuild

call :find_tlv_include
call :find_tlv_lib
call :make_version

echo Building vwifi %CFG%^|x64 ...
msbuild "%~dp0vwifi.sln" ^
    /t:Build ^
    /p:Configuration=%CFG% ^
    /p:Platform=x64 ^
    /p:WdiTlvIncludeDir="%WdiTlvIncludeDir%" ^
    /p:WdiTlvLib="%WdiTlvLib%" ^
    /p:VwifiInfVersion="%VwifiInfVersion%" ^
    /m ^
    /v:minimal ^
    /nologo ^
    /flp:logfile="%~dp0build-%CFG%.log";verbosity=normal ^
    /flp1:logfile="%~dp0build-%CFG%.err";errorsonly ^
    /flp2:logfile="%~dp0build-%CFG%.wrn";warningsonly

if errorlevel 1 goto :build_failed

echo.
echo Build OK. Output:
echo   %~dp0x64\%CFG%\
dir /b "%~dp0x64\%CFG%\vwifi.sys" "%~dp0x64\%CFG%\vwifi.inf" 2>nul
echo.
echo   The output is flat here, not in a vwifi\ subfolder: MSBuild only
echo   creates the package folder as part of its own signing step, which
echo   this project turns off. sign.cmd assembles it.
echo.
echo Warnings (review these^): %~dp0build-%CFG%.wrn
echo Next: sign.cmd %CFG%
endlocal
exit /b 0


rem ===============================================================
rem  Subroutines
rem ===============================================================

:no_msbuild
echo.
echo ERROR: msbuild is not on PATH.
echo.
echo   EWDK:           mount the ISO and run LaunchBuildEnv.cmd, then
echo                   re-run this script from that shell.
echo   Visual Studio:  use "Developer Command Prompt for VS 2022".
echo.
endlocal
exit /b 1

:build_failed
echo.
echo BUILD FAILED. Errors:
echo ---------------------------------------------------------------
type "%~dp0build-%CFG%.err"
echo ---------------------------------------------------------------
echo Full log: %~dp0build-%CFG%.log
endlocal
exit /b 1


rem --- TlvGeneratorParser.hpp ------------------------------------
rem  Not on the kit's default include path, unlike dot11wdi.h and
rem  wditypes.hpp. Set WdiTlvIncludeDir yourself to skip the search.
:find_tlv_include
if defined WdiTlvIncludeDir goto :show_include
echo Locating TlvGeneratorParser.hpp ...
set "PF86=%ProgramFiles(x86)%"
for /f "usebackq delims=" %%D in (`powershell -NoProfile -ExecutionPolicy Bypass -File "%~dp0find-wdk-file.ps1" -Name TlvGeneratorParser.hpp -Subtree Include -WantDirectory`) do set "WdiTlvIncludeDir=%%D"
if not defined WdiTlvIncludeDir goto :no_include
:show_include
echo   TlvGeneratorParser.hpp: %WdiTlvIncludeDir%
exit /b 0

:no_include
echo.
echo WARNING: TlvGeneratorParser.hpp not found in the mounted kit.
echo   tlv_shim.cpp will fail with C1083. Find it with:
echo     dir /s /b "%%ProgramFiles(x86)%%\Windows Kits\10\TlvGeneratorParser.hpp"
echo   then re-run as:  set WdiTlvIncludeDir=^<its folder^> ^&^& build.cmd
echo.
exit /b 0


rem --- the WDI TLV static library --------------------------------
rem  TlvGenerated_.hpp only declares ParseWdi*/GenerateWdi*; the code
rem  is in a .lib in the kit's Lib tree.
:find_tlv_lib
if defined WdiTlvLib goto :show_lib
echo Locating the WDI TLV library ...
set "PF86=%ProgramFiles(x86)%"
for /f "usebackq delims=" %%L in (`powershell -NoProfile -ExecutionPolicy Bypass -File "%~dp0find-wdk-file.ps1" -Name TLVGeneratorParser.lib -Subtree Lib -RequirePathMatch x64`) do set "WdiTlvLib=%%L"
if not defined WdiTlvLib goto :no_lib
:show_lib
echo   WDI TLV library:        %WdiTlvLib%
exit /b 0

:no_lib
echo.
echo WARNING: no WDI TLV library found. The link will fail with
echo   LNK2019 on ParseWdiTaskScanToIhv and friends. Find it with:
echo     dir /s /b "%%ProgramFiles(x86)%%\Windows Kits\10\Lib\*.lib" ^| findstr /i tlv
echo   then re-run as:  set WdiTlvLib=^<full path to the .lib^> ^&^& build.cmd
echo.
exit /b 0


rem --- a strictly increasing INF version -------------------------
rem  stampinf writes DriverVer as <date>,<version>. Two builds on the
rem  same day with the same version are indistinguishable to Windows,
rem  which then keeps running the driver already installed. Derive the
rem  version from the build time so every build supersedes the last.
rem  MMdd and HHmm both fit the 0..65535 a version field allows.
:make_version
if defined VwifiInfVersion goto :show_version
for /f "usebackq delims=" %%V in (`powershell -NoProfile -Command "Get-Date -Format MMdd"`) do set "V_MMDD=%%V"
for /f "usebackq delims=" %%V in (`powershell -NoProfile -Command "Get-Date -Format HHmm"`) do set "V_HHMM=%%V"
if not defined V_MMDD set "V_MMDD=0"
if not defined V_HHMM set "V_HHMM=0"
set "VwifiInfVersion=1.0.%V_MMDD%.%V_HHMM%"
:show_version
echo   INF DriverVer version:  %VwifiInfVersion%
exit /b 0
