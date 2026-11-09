@echo off
setlocal
cd /d "%~dp0flasher"
echo Starting the local flasher at http://127.0.0.1:8767/
echo Keep this window open while writing the firmware.
start "" "http://127.0.0.1:8767/"
python -m http.server 8767 --bind 127.0.0.1
echo The server stopped. If the port is in use, close the other local server first.
pause
