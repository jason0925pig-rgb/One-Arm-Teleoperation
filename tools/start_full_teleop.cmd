@echo off
setlocal
powershell.exe -NoProfile -ExecutionPolicy Bypass -File "%~dp0start_full_teleop.ps1" %*
exit /b %ERRORLEVEL%
