@echo off
echo Starting 3D magnetic calibration acquisition (clears previous data)...
curl -X POST http://192.168.144.9/api/v1/magnetometer/calibration/start -d "{\"mode\":\"3d\"}"
echo.
echo Acquisition started. NOW:
echo   - Keep the flight controller POWERED ON. Pick a spot away from metal structures and vehicles.
echo   - Hand rotate the whole drone slowly
pause
