@echo off
echo ============================================================
echo  OPTIONAL STEP - ONLY needed when calibrating INDOORS
echo  (no GNSS fix). SBG 3D calibration requires position and
echo  date. Skip this step if you are outdoors with a GNSS fix.
echo ============================================================
echo.
echo Setting local position and today's date on the unit...
for /f %%i in ('powershell -NoProfile -Command "Get-Date -Format yyyy-MM-dd"') do set TODAY=%%i
curl -X POST http://192.168.144.9/api/v1/settings/localParam -d "{\"latitude\":13.03533,\"longitude\":77.80116,\"height\":880.0,\"date\":\"%TODAY%\",\"useOnBoardWMM\":true}"
echo.
echo Verifying...
curl http://192.168.144.9/api/v1/settings/localParam
echo.
echo If latitude/longitude/date above are set, continue with step 1.
pause
