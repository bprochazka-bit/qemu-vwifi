@echo off
rem ---------------------------------------------------------------
rem  Dump the WDI port driver's view of our adapter, via !ndiskd.
rem
rem  Wrapper for dump-wdi-state.ps1, so it can be run from the cmd
rem  shell the EWDK's LaunchBuildEnv leaves you in without repeating
rem  -File and an execution-policy override each time.
rem
rem  Run elevated. Typical order:
rem
rem      dump-wdi-state.cmd -ShowSymbolKeys     (no internet? get URLs)
rem      dump-wdi-state.cmd -Enable             (then reboot)
rem      dump-wdi-state.cmd -SymbolPath C:\symbols
rem
rem  Any arguments are passed through to the script.
rem ---------------------------------------------------------------
powershell.exe -NoProfile -ExecutionPolicy Bypass -File "%~dp0dump-wdi-state.ps1" %*
exit /b %ERRORLEVEL%
