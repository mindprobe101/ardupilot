# Zhiann BMS integration assets

Prebuilt ArduCopter 4.6.3 for CubeOrangePlus with the
AP_BattMonitor_ZhiannBMS driver (BATT_MONITOR=30, CAN_Dx_PROTOCOL=15,
pack select via BATTn_SERIAL_NUM or -1 auto-bind).

- `arducopter-CubeOrangePlus.apj` — flash with
  `Tools/scripts/uploader.py zhiann-bms/arducopter-CubeOrangePlus.apj`
- `params-4pack.param` / `params-2pack.param` — mission profiles
- `docs/SUMMARY.md` — project summary: what exists, how it was built
- `docs/PROTOCOL.md` — the reverse-engineered CAN protocol reference
- `docs/LOGGING.md` — meaning of every logged entry (ZBMS/ZBC1/ZBC2,
  GCS messages, sniffer log formats)
- `docs/LEARNINGS.md` — consolidated bench-test learnings and operating
  procedures (read before operating multi-pack fleets)
