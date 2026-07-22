# Zhiann BMS integration assets

Reviewed ArduCopter 4.6.3 build for CubeOrangePlus with the
AP_BattMonitor_ZhiannBMS driver (BATT_MONITOR=30, CAN_Dx_PROTOCOL=15,
pack select via BATTn_SERIAL_NUM or -1 auto-bind).

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

The reviewed firmware fits CubeOrangePlus with only **304 bytes free**. Do not
modify or rebuild it without rechecking flash use and artifact provenance.
