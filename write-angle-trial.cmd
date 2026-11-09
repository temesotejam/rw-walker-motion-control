@echo off
setlocal
cd /d "%~dp0"
set "PIO=%USERPROFILE%\.platformio\penv\Scripts\platformio.exe"
set "PLATFORMIO_CORE_DIR=%USERPROFILE%\.platformio"
set "PLATFORMIO_SETTING_ENABLE_TELEMETRY=No"
set "PYTHONDONTWRITEBYTECODE=1"
if not exist "%PIO%" (
  echo PlatformIO not found: %PIO%
  pause
  exit /b 1
)
echo Building and uploading the angle-early trial firmware...
"%PIO%" run -e atoms3cam-angle-early -t upload
if errorlevel 1 (
  echo Build or upload failed. Check the messages above.
  pause
  exit /b 1
)
echo Upload completed successfully.
pause
