@echo off
setlocal
python -B -u "%~dp0tools\download_rwlog.py" %*
set "result=%errorlevel%"
if "%~1"=="" pause
exit /b %result%
