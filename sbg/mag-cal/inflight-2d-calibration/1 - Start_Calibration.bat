@echo off
echo Starting 2D magnetic calibration acquisition (clears previous data)...
curl -X POST http://192.168.144.9/api/v1/magnetometer/calibration/start -d "{\"mode\":\"2d\"}"
echo.
echo Acquisition started. NOW:
echo   - Get airborne first, normal flight, gentle bank angles (2D mode expects
echo     roll and pitch within about +/-5 degrees).
echo   - Fly flat figure eights or two full circles in BOTH directions so all
echo     headings are covered. 2 to 3 minutes of pattern is enough.
echo Then run step 2 to compute and check the result.
pause
