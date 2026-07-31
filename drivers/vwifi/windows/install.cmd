@echo off
rem ---------------------------------------------------------------
rem  vwifi -- install or replace the driver in the guest.
rem
rem  Run from an ELEVATED command prompt in the Windows VM, from the
rem  folder holding vwifi.sys / vwifi.inf / vwifi.cat:
rem
rem      install.cmd
rem
rem  This is a wrapper. The work is in install.ps1, deliberately: the
rem  device enumeration needs a pipeline full of parentheses, and
rem  embedding that in a cmd `for /f` is how you get "was unexpected at
rem  this time" -- cmd parses a parenthesised block in one pass and the
rem  parens in the command terminate it early.
rem ---------------------------------------------------------------
rem  "%~dp0." rather than "%~dp0": the latter ends in a backslash, and a
rem  backslash immediately before a closing quote is read as an escaped
rem  quote. The trailing dot keeps it a valid path and side-steps that.
powershell -NoProfile -ExecutionPolicy Bypass -File "%~dp0install.ps1" -Path "%~dp0." %*
exit /b %errorlevel%
