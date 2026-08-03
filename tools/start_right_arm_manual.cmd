@echo off
setlocal
powershell.exe -NoProfile -ExecutionPolicy Bypass -File "%~dp0start_right_arm_manual.ps1" %*
exit /b %ERRORLEVEL%
