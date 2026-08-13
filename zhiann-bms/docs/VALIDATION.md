# Zhiann BMS driver validation report

Validation date: 2026-07-22. Reviewed base: ArduCopter 4.6.3 branch commit
`900cfe4445`. Build with the extra-hwdef as described in `../README.md`.

## Software-verified

- Three independent protocol, lifecycle, CAN-isolation, logging, build and
  artifact reviews completed after reading all project documentation.
- 39 focused decoder/state-helper tests pass, covering ID/DLC classification,
  flagged coarse SOC, signed-current boundaries, plausibility, atomic ordered
  cell assembly, cell count, wrap/outage handling, consumption reconciliation,
  collision cadence and coherence hysteresis.
- Duplicate detection replayed frame-by-frame against the full 2026-08-13/14
  capture corpus (8 captures, 8801 s, 149,255 SOC-coarse frames, 25
  node-instances). Ground truth was established from payload content —
  interleaved pack-voltage-mirror clusters — independently of any timing.
  Result: all 4 genuine collisions detected, 99.6 % of duplicate time covered,
  median onset 0.71 s, no flapping, and **zero false positives across 3.74 h of
  single-pack node time**. The 70,922 sub-20 ms retransmit/queue-drain gaps
  contribute no score; removing that floor produces 433 s of false activity.
- A 2,000,000-iteration ASan/UBSan randomized stress run completed with no
  finding.
- Full SITL ArduCopter, parameter metadata and logger metadata generation pass.
- CubeOrangePlus builds twice byte-identically.
- Capture corpus evidence: all 437,558 recognized frames used canonical DLC;
  35,613 complete cell bursts spanned 22-55 ms; the 1 V cell-sum and SOC-mirror
  collision thresholds cleanly separate the reviewed clean/collision logs.

## Bench-verified before this review

- Nominal voltage, SOC, 24-cell mapping and dual-temperature broadcasts.
- Discharge-current sign against an analog power
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
- Configure exactly one instance with `BATT_MONITOR=30`; a second instance of
  this type would publish a duplicate of the same battery and is held
  unhealthy instead.
- Pack count is discovered at runtime, so the set has no fixed mAh capacity.
  Leave `BATT_CAPACITY` at 0 and use voltage-based failsafes.
- Alarms remain unbenchmarked, and flash headroom is tight enough that it must
  be rechecked on every rebuild rather than assumed.
- The 1 mA/LSB current scale has not been confirmed under load on the aircraft.
  Bench runs on 2026-08-14 had all four packs idle, where the BMS reports
  exactly 0 A, so the scale is still supported only by the pack's own rated
  capacity and coulomb counting. Confirm it against a load before relying on
  consumed-mAh figures.
