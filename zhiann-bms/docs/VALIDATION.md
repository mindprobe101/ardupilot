# Zhiann BMS driver validation report

Validation date: 2026-07-22. Reviewed base: ArduCopter 4.6.3 branch commit
`900cfe4445`; exact firmware provenance is in `../FIRMWARE-MANIFEST.md`.

## Software-verified

- Three independent protocol, lifecycle, CAN-isolation, logging, build and
  artifact reviews completed after reading all project documentation.
- 21 focused decoder/state-helper tests pass, covering ID/DLC classification,
  flagged coarse SOC, signed-current boundaries, plausibility, atomic ordered
  cell assembly, cell count, wrap/outage handling, consumption reconciliation,
  collision cadence and coherence hysteresis.
- A 2,000,000-iteration ASan/UBSan randomized stress run completed with no
  finding.
- Full SITL ArduCopter, parameter metadata and logger metadata generation pass.
- CubeOrangePlus builds twice byte-identically; see the manifest for hash and
  the critical 304-byte flash margin.
- Capture corpus evidence: all 437,558 recognized frames used canonical DLC;
  35,613 complete cell bursts spanned 22-55 ms; the 1 V cell-sum and SOC-mirror
  collision thresholds cleanly separate the reviewed clean/collision logs.

## Bench-verified before this review

- Nominal voltage, SOC, 24-cell mapping and dual-temperature broadcasts.
- Discharge-current sign and nominal 2 mA/LSB scale against an analog power
  module.
- One-to-five-pack traffic, standby/silence behavior and real duplicate-node
  collision captures.
- Continuous master heartbeat is harmful; the driver remains strictly RX-only.

## Still requires bench validation

1. Confirm the Cube emits no PF `0x43` or other CAN traffic with an independent
   RX-only sniffer on the full five-pack fleet.
2. Trigger a safe PF `0x24` fault on one pack to determine source address,
   repetition and clear behavior; verify single- and multi-pack latch policy.
3. Inject wrong-DLC, FD, RTR, invalid-value and missing/reordered cell frames;
   confirm no binding/liveness refresh, no snapshot flicker and correct expiry.
4. Unplug/replug a persistent duplicate node and verify it cannot qualify
   healthy during reconnect.
5. Test fastest real load steps against the 1 V SOC-mirror threshold and check
   CAN drops/errors with five packs plus `LOG_DISARMED=1`.
6. Verify temperature below 0 C, the coarse-SOC status bits, and current scale
   with a calibrated clamp meter.
7. Disconnect CAN while armed and confirm the airframe-level response. This
   ArduPilot base reports battery `Unhealthy` with action 0, so independent
   voltage/capacity failsafes must be configured and bench-tested.

## Operational limitations

- Configure protocol 15 on exactly one CAN driver; additional matching ports
  are ignored by `CANSensor`.
- `BATTn_SERIAL_NUM` accepts only `-1` or `0..15` and is snapshotted at boot;
  reboot after changes.
- Invalid/duplicate mappings fail safe as unhealthy but diagnostics are generic.
- The current scale is nominal, alarms remain unbenchmarked, and 304 bytes of
  flash headroom is not suitable for unreviewed feature growth.

## Copter-4.7.0 port validation (2026-07-22)

Everything above describes the 4.6.3-branch review and remains the bench
record. This branch is the tag `Copter-4.7.0` plus the same driver commits;
the notes below supersede the 4.6.3-specific figures.

- Port fidelity: the driver, decode header and decode tests are byte-identical
  to the 4.6.3 branch (empty diff). Integration wiring uses `BATT_MONITOR=33`
  (upstream 4.7 assigned 30/31/32 to INA3221, Analog Current Only and
  TIBQ76952) and the unchanged `CAN_Dx_PROTOCOL=15`, which upstream still
  leaves uncontended.
- Independent rerun on this branch: 29/29 decode tests, SITL build, and a
  byte-identical CubeOrangePlus rebuild matching `../FIRMWARE-MANIFEST.md`
  with 247,028 bytes of flash free — the 304-byte margin noted above applied
  only to the 4.6.3 build.
- A function-level diff of every front-end contract between the two tags found
  no behavioral change: 10 Hz `battery.read()` scheduling; the `voltage > 0`
  guard on low/critical voltage failsafes; `has_current()` gating of capacity
  failsafes; the 5 s unhealthy timeout; arming-check flow and texts;
  BATTERY_STATUS cell redistribution and the all-instances round-robin;
  `CANSensor` system-initialized gating and driver-slot allocation; every
  stored `BATTn_*` parameter index, including `CURR_MULT` at backend index 30
  which still aliases DroneCAN's identical entry; and the 64-character logger
  label limit that the ZBMS message uses exactly.
- The one migration hazard is the stored-parameter value change described in
  the README: `BATTn_MONITOR=30` left over from 4.6.3 allocates no backend on
  this build (INA3221 is not compiled in), so the instance fails pre-arm as
  "Battery N unhealthy" with no telemetry and no battery failsafes until the
  parameter is set to 33. Loud but misleading; apply the params template
  before first flight.
- Watch items for future upstream merges: reject any change that repurposes
  battery backend parameter index 30; and if the MAVLink logging backend
  (`LOG_BACKEND_TYPE` bit 2) is ever enabled, set `LOG_MAV_RATEMAX=0`,
  because its default became 10 Hz in 4.7 and the five instances share each
  ZBMS/ZBC1/ZBC2 message id.
- 4.7 replaced `ARMING_CHECK` with the inverted skip-mask `ARMING_SKIPCHK`
  (one-time faithful conversion at first boot); update any GCS preset files
  or scripts that still write the old parameter.
