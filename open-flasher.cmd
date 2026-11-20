@echo off
setlocal
cd /d "%~dp0"
python tools\prepare_foot_periodic40_flasher.py --serve
if not errorlevel 1 exit /b 0
echo The installer could not be opened. Check the error above.
echo To package the current build, run python tools\prepare_foot_periodic40_flasher.py
pause
exit /b 1
