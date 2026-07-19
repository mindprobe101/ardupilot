Ground magnetic calibration (3D mode) for the SBG unit at 192.168.144.9
=========================================================================

Requirements
- Flash FC firmware arducopter-CubeOrangePlus-4.6.3-sbg-9d3760e or newer
  first. Older builds send the magnetic field in milligauss (values near
  400); the SBG expects gauss scale (earth field 0.2 to 0.7, calibrated
  norm 1.0) and the calibration cannot use the oversized values. On the
  new build the mag stream shows |B| around 0.4.
- The unit MUST have position and date: either do the calibration
  OUTDOORS with a GNSS fix (satellites used > 0), or run step 0 first
  to set them manually. The failed sessions 0131/0132 had lat/lon 0 and
  date 1970-01-01, and SBG 3D calibration requires position + date for
  its magnetic field reference. This alone produces the 422 error.
- The CubeOrangePlus must be powered and running ArduPilot the whole time:
  the SBG receives its magnetometer data from the FC over COM B at 10 Hz.
  Rotating the SBG alone acquires ZERO points.
- Do NOT power cycle the SBG between step 1 (start) and step 2 (compute):
  the acquisition buffer does not survive a reboot.
- Props OFF for safety. Do the rotations away from steel structures.
- 3D mode needs at least +/-30 degrees of BOTH roll and pitch plus good
  yaw coverage. The check in step 2 tells you exactly what is missing.

Expected results with the 9d3760e firmware (predicted from the recorded
session_0131 rotation data): hardIronCorrection around [0.011, -0.018,
0.001] and softIronCorrection with a diagonal around 2.5. The large
diagonal is CORRECT - it scales the 0.41 gauss earth field to the SBG
norm target of 1.0. Do not treat it as an error.

Run the numbered bat files in order (step 0 only if indoors):
1 - Start_Calibration    starts a fresh 3D acquisition (clears old data)
2 - Compute_And_Check    compute + quality gate; repeat until it says GOOD
                         (computing does not stop the acquisition)
3 - Apply_Calibration    extracts hardIronCorrection / softIronCorrection
                         only and writes them to settings
4 - Enable_Mag_Heading   sets useMagData=auto and heading rejection=auto
                         (without this the calibration changes nothing)
5 - Save_Settings        persists to flash; power cycle if needReboot=true
6 - Verify_Settings      reads back and confirms cal stored + fusion on

After flight, confirm in the logs that EKF_NAV status bit 9 (MAG_REF_USED)
is set. Known limitation: a ground calibration cannot capture motor current
interference. If heading gets noisy at high throttle, run the in flight 2D
calibration folder afterwards as a refinement.
