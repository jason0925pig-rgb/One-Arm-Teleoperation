@echo off
setlocal
set "SCRIPT_DIR=%~dp0"
powershell.exe -NoProfile -ExecutionPolicy Bypass -File "%SCRIPT_DIR%my_run_teleop_screwdriver.ps1" %*
exit /b %ERRORLEVEL%
