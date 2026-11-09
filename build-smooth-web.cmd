@echo off
setlocal
cd /d "%~dp0"
set "PLATFORMIO_CORE_DIR=%USERPROFILE%\.platformio"
set "PIO=%USERPROFILE%\.platformio\penv\Scripts\platformio.exe"
if not exist "%PIO%" (echo PlatformIO not found.& pause & exit /b 1)
"%PIO%" run -e atoms3cam-angle-smooth
if errorlevel 1 (echo Build failed. No new web firmware was prepared.& pause & exit /b 1)
python tools\prepare_smooth_flasher.py
if errorlevel 1 (echo Preparing web firmware failed.& pause & exit /b 1)
echo Open http://127.0.0.1:8767/smooth/ with the existing local server running.
pause
