@echo off
setlocal
set "PIO_PYTHON=%USERPROFILE%\.platformio\penv\Scripts\python.exe"
set "RECORDER=%~dp0zlink2_leader_recorder.py"

if not exist "%PIO_PYTHON%" (
    echo ERROR: PlatformIO Python was not found:
    echo   %PIO_PYTHON%
    echo Install or repair PlatformIO, then try again.
    exit /b 2
)

"%PIO_PYTHON%" -B "%RECORDER%" %*
exit /b %ERRORLEVEL%
