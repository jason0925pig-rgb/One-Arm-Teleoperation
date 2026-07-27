@echo off
setlocal
set "PIO_PYTHON=%USERPROFILE%\.platformio\penv\Scripts\python.exe"
set "CALCULATOR=%~dp0calibration_calculator.py"

if not exist "%PIO_PYTHON%" (
    echo ERROR: PlatformIO Python was not found:
    echo   %PIO_PYTHON%
    exit /b 2
)

"%PIO_PYTHON%" -B "%CALCULATOR%" %*
exit /b %ERRORLEVEL%
