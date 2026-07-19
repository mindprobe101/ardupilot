@echo off
echo Enabling magnetometer fusion (this was the missing step)...
curl -X POST http://192.168.144.9/api/v1/settings/aiding/magnetometer -d "{\"useMagData\":\"auto\"}"
echo.
echo Ensuring magnetic heading is not rejected...
curl -X POST http://192.168.144.9/api/v1/magnetometer/rejections/heading -d "\"auto\""
echo.
pause
