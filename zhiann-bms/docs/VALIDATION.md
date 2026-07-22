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
