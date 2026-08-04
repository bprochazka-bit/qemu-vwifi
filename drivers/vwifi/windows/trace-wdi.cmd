@echo off
rem ---------------------------------------------------------------
rem  Capture and decode a WDI port driver trace.
rem
rem  Wrapper for trace-wdi.ps1. Exists because the tools it needs
rem  (tracepdb, tracefmt) only appear once an EWDK/WDK build
rem  environment is set up, and LaunchBuildEnv drops you in cmd --
rem  from which calling a .ps1 directly means remembering both -File
rem  and an execution-policy override every time.
rem
rem  Run elevated, from inside the build environment:
rem
rem      trace-wdi.cmd -Ssid vwifi-open
rem
rem  Any arguments are passed through to the script.
rem ---------------------------------------------------------------
powershell.exe -NoProfile -ExecutionPolicy Bypass -File "%~dp0trace-wdi.ps1" %*
exit /b %ERRORLEVEL%
