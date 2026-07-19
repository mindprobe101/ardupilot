@echo off
echo Computing calibration from acquired data...
curl -s -X GET http://192.168.144.9/api/v1/magnetometer/calibration -o "%~dp0calibration.json"
type "%~dp0calibration.json"
echo.
powershell -NoProfile -ExecutionPolicy Bypass -File "%~dp0check_calibration.ps1"
pause
