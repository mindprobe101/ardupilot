In flight magnetic calibration (2D mode) for the SBG unit at 192.168.144.9
============================================================================

Use this after (or instead of) the ground calibration when you want the
calibration to include the real flight magnetic environment (motor current,
power wiring). 2D mode is designed for level flight: roll and pitch within
about +/-5 degrees, so normal gentle patterns qualify.

Requires FC firmware arducopter-CubeOrangePlus-4.6.3-sbg-9d3760e or newer
(older builds send milligauss instead of gauss and the calibration cannot
use the data). In flight the unit has a GNSS fix, which also satisfies the
SBG position + date requirement for calibration.

Procedure (GCS must reach 192.168.144.9 over the datalink):
1 - Start_Calibration    start a fresh 2D acquisition WHILE FLYING level
2 - Compute_And_Check    fly flat figure eights covering all headings,
                         then compute; repeat until it says GOOD
3 - Apply_Calibration    can be done in a stable hover or after landing
4 - Enable_Mag_Heading   sets useMagData=auto and heading rejection=auto
5 - Save_Settings        prefer running this after landing; power cycle
                         on the ground if any response said needReboot=true
6 - Verify_Settings      reads back and confirms cal stored + fusion on

Note: a 2D calibration constrains fewer parameters than 3D. Best result on
this airframe: ground 3D calibration first, then this 2D refinement flight.
After the next flight check EKF_NAV status bit 9 (MAG_REF_USED) = 1 in the
session logs to confirm the EKF is actually using magnetic heading.
