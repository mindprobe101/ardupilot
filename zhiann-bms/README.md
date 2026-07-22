# Zhiann BMS integration assets

Reviewed ArduCopter 4.7.0 build for CubeOrangePlus with the
AP_BattMonitor_ZhiannBMS driver (BATT_MONITOR=33, CAN_Dx_PROTOCOL=15,
pack select via BATTn_SERIAL_NUM or -1 auto-bind).

## MIGRATION from the 4.6.3 branch — BATT_MONITOR value changed

On `Copter-4.6.3-zhiann-bms` the driver was `BATT_MONITOR=30`. Upstream
4.7 assigned 30/31/32 to new backends (INA3221, Analog Current Only,
TIBQ76952), so on this branch the driver is **`BATT_MONITOR=33`**.
Operators upgrading from the 4.6.3 firmware MUST set every
`BATTn_MONITOR` that was 30 to 33 (see the updated `params-*.param`
templates) and reboot. A stored value of 30 on 4.7 selects the INA3221
I2C backend, which is not compiled into this CubeOrangePlus build: the
instance allocates no backend at all, pre-arm fails with
"Battery N unhealthy", and that pack has no telemetry and no battery
failsafes until the parameter is corrected. There is deliberately no
automatic conversion in the firmware. `CAN_Dx_PROTOCOL=15` and
`BATTn_SERIAL_NUM` semantics are unchanged.

- `arducopter-CubeOrangePlus.apj` — flash with
  `Tools/scripts/uploader.py zhiann-bms/arducopter-CubeOrangePlus.apj`
- `FIRMWARE-MANIFEST.md` — exact source, toolchain, artifact hash and flash use
- `params-4pack.param` / `params-2pack.param` — mapping templates, not complete
  flight-safety profiles
- `docs/SUMMARY.md` — project summary: what exists, how it was built
- `docs/VALIDATION.md` — software/bench evidence and remaining experiments
- `docs/PROTOCOL.md` — the reverse-engineered CAN protocol reference
- `docs/LOGGING.md` — meaning of every logged entry (ZBMS/ZBC1/ZBC2,
  GCS messages, sniffer log formats)
- `docs/LEARNINGS.md` — consolidated bench-test learnings and operating
  procedures (read before operating multi-pack fleets)

Before flight, verify the APJ SHA-256 against the manifest, configure protocol
15 on exactly one CAN driver, and reboot after changing any
`BATTn_SERIAL_NUM`. Set and bench-test per-instance `ARM`, `LOW` and `CRT`
voltage or mAh thresholds plus `BATTn_FS_LOW_ACT`/`BATTn_FS_CRT_ACT`; the
provided parameter files intentionally do not invent airframe-specific safety
limits. A lost/unhealthy monitor blocks arming, but this ArduPilot base assigns
no automatic in-flight action to the `Unhealthy` state itself.

The reviewed 4.7.0 firmware fits CubeOrangePlus with 247,028 bytes free
(the 4.6.3 build was nearly full; 4.7 upstream is substantially smaller on
this board). The ExternalAHRS trims in the extra-hwdef are kept for parity
with the operator's SBG configuration. Do not modify or rebuild the
artifact without rechecking flash use and provenance in
`FIRMWARE-MANIFEST.md`.

## Building firmware for this integration

Configure with the extra hwdef (disables the unused non-SBG ExternalAHRS
backends to fit flash; Lua scripting stays enabled):

    ./waf configure --board CubeOrangePlus --extra-hwdef=zhiann-bms/extra-hwdef.dat
    ./waf copter
