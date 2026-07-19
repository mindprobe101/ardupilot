@echo off
curl -s -X GET http://192.168.144.9/api/v1/settings/aiding/magnetometer -o "%~dp0settings_check.json"
type "%~dp0settings_check.json"
echo.
powershell -NoProfile -ExecutionPolicy Bypass -File "%~dp0verify_settings.ps1"
pause
