@echo off
setlocal
set "SCRIPT_DIR=%~dp0"
powershell.exe -NoProfile -ExecutionPolicy Bypass -File "%SCRIPT_DIR%start_lingbot_va_teleop_collection.ps1" %*
exit /b %ERRORLEVEL%
