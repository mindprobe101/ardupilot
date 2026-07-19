@echo off
curl -X POST http://192.168.144.9/api/v1/settings/save
echo.
echo Settings saved. Power cycle the unit if any earlier response said needReboot true.
pause
