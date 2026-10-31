@echo off
powershell.exe -NoProfile -ExecutionPolicy Bypass -File "%~dp0arduino-cli.ps1" %*
exit /b %ERRORLEVEL%
