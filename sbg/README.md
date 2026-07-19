# SBG Quanta Micro integration

This branch targets a Quanta-USG / Quanta Micro running firmware
`6.2.682-stable` with sbgECom 5.6, connected to a CubeOrangePlus at 921600
baud. `EAHRS_OPTIONS=2` publishes the SBG EKF navigation solution as the
ArduPilot ExternalAHRS GPS source. Other SBG modes retain their raw-GNSS
behavior.

## Quanta COM-B profile

Configure the Quanta through its web or REST interface. ArduPilot does not
change the device output configuration.

| Output | Period/mode |
| --- | --- |
| Baud | 921600 |
| `EKF_NAV` | 5 ms |
| `STATUS` | 100 ms |
| `UTC_TIME` | 1000 ms |
| `GPS1_POS` | On change |
| `GPS1_VEL` | On change |
| `GPS1_SAT` | Off |

NMEA GSA is not required. The driver derives the INS-compatible HDOP and VDOP
values from the EKF horizontal and vertical standard deviations documented by
SBG.

## ArduPilot parameters

Load [`params/cube-orange-plus-sbg.param`](params/cube-orange-plus-sbg.param),
or set these values manually:

```
EAHRS_TYPE=8
EAHRS_OPTIONS=2
EAHRS_SENSORS=1
EAHRS_RATE=200
GPS1_TYPE=21
GPS1_RATE_MS=50
SERIAL2_PROTOCOL=36
SERIAL2_BAUD=921
```

`GPS1_DELAY_MS` remains the operator calibration override. Leave it at zero to
use the ExternalAHRS default delay, or retain the value established by vehicle
latency calibration.

## CubeOrangePlus build

The CubeOrangePlus hardware definition enables SBG and disables the default
VectorNav, Inertial Labs, MicroStrain 5 and MicroStrain 7 ExternalAHRS
backends. PPP remains enabled. Build with:

```
./waf configure --board CubeOrangePlus
./waf copter
```

The navigation output requires fresh `EKF_NAV` and valid SBG position and
velocity flags. Loss of GNSS aiding, critical interference, probable or
confirmed spoofing, and OSNMA spoof detection cap the valid SBG solution at a
virtual 3D fix; they never advertise DGPS or RTK. Expired or invalid SBG
navigation publishes no fix. ZUPT, alignment and aiding state are recorded in
the `SBGS` DataFlash message and reported through rate-limited transition
messages.

`SBGS` keeps the full diagnostic set within DataFlash's compact-format limits.
`GS` is the raw GNSS status/type word; the `Sec` nibbles are IFM, spoofing and
OSNMA; the low/high `SV` bytes are satellites used/tracked; and `Flg` bits
0-3 are ZUPT, alignment-valid, GPS1-position-used and GPS1-velocity-used.
