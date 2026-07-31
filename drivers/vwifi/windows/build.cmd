@echo off
rem ---------------------------------------------------------------
rem  vwifi -- debug build of the Windows WDI miniport.
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
rem  to paste when reporting a failure -- it is errors only, in order.
rem
rem  To skip the search and point at the WDI TLV pieces yourself:
rem      set "WdiTlvIncludeDir=<folder holding TlvGeneratorParser.hpp>"
rem      set "WdiTlvLib=<full path to TLVGeneratorParser.lib>"
rem  Either quoting style works -- the value has quotes stripped. If what
rem  you set does not exist, the script says so and searches anyway
rem  rather than failing.
rem
rem  STYLE, learned the hard way in this file: no parenthesised blocks
rem  around `for /f`, no caret continuations inside blocks, and no
rem  argument ending in a backslash right before a closing quote. Each
rem  of those has broken this script at least once.
rem ---------------------------------------------------------------
setlocal enabledelayedexpansion

set CFG=%~1
if "%CFG%"=="" set CFG=Debug

where msbuild >nul 2>&1
if errorlevel 1 goto :no_msbuild

call :find_tlv_include
if errorlevel 1 goto :abort
call :find_tlv_lib
if errorlevel 1 goto :abort
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

:abort
endlocal
exit /b 1

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
rem  wditypes.hpp. A kit carries parallel copies; this driver needs the
rem  kernel-mode one (under \km\), matching the dot11wdi.h it is written
rem  against. The \um\ copy is user-mode WDI 2.0 and references types
rem  that do not exist in a kernel translation unit.
:find_tlv_include
if not defined WdiTlvIncludeDir goto :search_include
call :clean_var WdiTlvIncludeDir
if exist "!WdiTlvIncludeDir!\TlvGeneratorParser.hpp" goto :show_include
echo   WdiTlvIncludeDir is set but holds no TlvGeneratorParser.hpp:
echo     !WdiTlvIncludeDir!
echo   Ignoring it and searching instead.
set "WdiTlvIncludeDir="

:search_include
echo Locating TlvGeneratorParser.hpp ...
powershell -NoProfile -ExecutionPolicy Bypass -File "%~dp0find-wdk-file.ps1" -Name TlvGeneratorParser.hpp -Subtree Include -WantDirectory -MustContain "\km" -MustNotContain "\um" -Prefer "\wlan\1.0" >"%TEMP%\vwifi_inc.txt"
set "WdiTlvIncludeDir="
set /p WdiTlvIncludeDir=<"%TEMP%\vwifi_inc.txt"
del "%TEMP%\vwifi_inc.txt" >nul 2>&1
if not defined WdiTlvIncludeDir goto :no_include
call :clean_var WdiTlvIncludeDir
if not exist "!WdiTlvIncludeDir!\TlvGeneratorParser.hpp" goto :no_include

:show_include
echo   TlvGeneratorParser.hpp: !WdiTlvIncludeDir!
exit /b 0

:no_include
echo.
echo ERROR: could not locate TlvGeneratorParser.hpp. Every copy the
echo   search looked at is listed above, with why it was rejected.
echo   Point at it directly with:
echo     set "WdiTlvIncludeDir=the folder holding it"
echo.
exit /b 1


rem --- the WDI TLV static library --------------------------------
rem  TlvGenerated_.hpp only declares ParseWdi*/GenerateWdi*; the code
rem  is in a .lib in the kit's Lib tree.
:find_tlv_lib
if not defined WdiTlvLib goto :search_lib
call :clean_var WdiTlvLib
if exist "!WdiTlvLib!" goto :show_lib
echo   WdiTlvLib is set but that file does not exist:
echo     !WdiTlvLib!
echo   Ignoring it and searching instead.
set "WdiTlvLib="

:search_lib
echo Locating the WDI TLV library ...
powershell -NoProfile -ExecutionPolicy Bypass -File "%~dp0find-wdk-file.ps1" -Name TLVGeneratorParser.lib -Subtree Lib -MustContain "\km;\x64" -MustNotContain "\um" -Prefer "\wlan\1.0" >"%TEMP%\vwifi_lib.txt"
set "WdiTlvLib="
set /p WdiTlvLib=<"%TEMP%\vwifi_lib.txt"
del "%TEMP%\vwifi_lib.txt" >nul 2>&1
if not defined WdiTlvLib goto :no_lib
call :clean_var WdiTlvLib
if not exist "!WdiTlvLib!" goto :no_lib

:show_lib
echo   WDI TLV library:        !WdiTlvLib!
exit /b 0

:no_lib
echo.
echo ERROR: could not locate the WDI TLV library. Every copy the search
echo   looked at is listed above, with why it was rejected. Point at it
echo   directly with:
echo     set "WdiTlvLib=full path to TLVGeneratorParser.lib"
echo.
exit /b 1


rem --- normalise a path variable ---------------------------------
rem  Strips surrounding quotes and any trailing spaces.
rem
rem  Quotes: `set VAR="C:\path with spaces"` puts them INSIDE the value,
rem  which makes every "%VAR%\file" test malformed. That is the natural
rem  way to type a path with spaces, not a user error.
rem
rem  Trailing spaces: they arrive from anything that formats output for
rem  a console. MSBuild trims property values, so a padded path still
rem  produced a correct /I and went unnoticed; `if exist` does not trim
rem  and fails on a path that looks perfect when echoed.
:clean_var
set "_cv=!%~1!"
set "_cv=!_cv:"=!"
:clean_var_loop
rem  No `if cond A & B` here: in cmd the `&` separates statements at the
rem  top level, so B would run whether or not the condition held -- the
rem  goto would fire every time and spin forever. Labels instead.
if "!_cv:~-1!"==" " goto :clean_var_trim
goto :clean_var_done
:clean_var_trim
set "_cv=!_cv:~0,-1!"
goto :clean_var_loop
:clean_var_done
set "%~1=!_cv!"
set "_cv="
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
