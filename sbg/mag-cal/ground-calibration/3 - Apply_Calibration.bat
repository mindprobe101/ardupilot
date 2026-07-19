@echo off
powershell -NoProfile -ExecutionPolicy Bypass -File "%~dp0make_apply_json.ps1" || goto :eof
echo Applying corrections to settings/aiding/magnetometer...
curl -X POST http://192.168.144.9/api/v1/settings/aiding/magnetometer -d @"%~dp0apply.json"
echo.
echo If the response says needReboot true, power cycle after step 5.
pause
