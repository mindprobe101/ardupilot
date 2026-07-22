# Zhiann BMS ↔ ArduPilot integration — project summary

24S smart battery packs (Shenzhen Zhian New Energy, ZB3CN7 series,
~100 V, 44 Ah, up to 5 packs on one CAN bus) integrated with a
CubeOrange+ running ArduCopter 4.7.0, on branch `Copter-4.7.0-zhiann-bms`
(ported from `Copter-4.6.3-zhiann-bms`).

**MIGRATION note:** on the 4.6.3 branch the driver was `BATT_MONITOR=30`;
upstream 4.7 took values 30-32 for new backends, so here it is
`BATT_MONITOR=33`. Operators coming from the 4.6.3 firmware must update
every `BATTn_MONITOR` from 30 to 33 (30 now selects INA3221).
`CAN_Dx_PROTOCOL=15` is unchanged.

## What exists now

- **`AP_BattMonitor_ZhiannBMS`** (`BATT_MONITOR=33`,
  `CAN_Dx_PROTOCOL=15`): a receive-only CAN battery backend providing
  voltage, calibrated current with conservative BMS-SOC/current reconciled
  consumed mAh/Wh, all 24 cell voltages
  (14 live via MAVLink, 24 in dataflash), dual temperatures, healthy-state
  BMS SOC, alarm-frame fault mapping, and operator
  diagnostics: qualified-session duplicate-node detection, atomic cell
  snapshots, cell-sum/SOC-mirror coherence, standby detection,
  fail-safe unmapped-node handling, per-node fleet inventory. Multi-pack via `BATTn_SERIAL_NUM`
  (node number, or -1 auto-bind); nodes 0-15, up to 9 instances.
- **Pure decode core** (`AP_BattMonitor_ZhiannBMS_decode.h`) shared with
  a gtest replay suite (`tests/test_zhiann_decode.cpp`, 21 cases combining
  real captured frames with explicit synthetic boundary cases) -
  `./waf configure --board sitl` then build
  target `tests/test_zhiann_decode`.
- **Ops assets** (this folder): hash-attested firmware and build manifest,
  2-pack/4-pack mapping templates, PROTOCOL.md / LOGGING.md / LEARNINGS.md,
  and a software-versus-bench VALIDATION.md report.
- **Bench tooling** (developer machine, `~/can_bms`): NUCLEO-H753ZI
  sniffer firmware (auto-bitrate RX + command-driven TX), live web
  dashboard with simultaneous logging, capture/decode/fleet-query/
  load-test/correlation scripts.

## How it was built (methodology)

1. Protocol reverse-engineered from live traffic with the Nucleo sniffer
   (cross-validated by internal consistency: cell sums vs pack voltage,
   dual SOC fields, cell count field).
2. Current field identified and calibrated by time-aligning a BMS capture
   with an ArduPilot dataflash log from a motor load test (alignment on
   the shared pack-voltage waveform; plateau regression gave 2 mA/LSB).
3. Vendor spec V1.9 obtained later confirmed units and the polled command
   set; the broadcast profile itself is a customer profile absent from
   the spec (Appendix B stub) - our map remains the reference.
4. Driver hardened via multi-angle adversarial review: strict classic-CAN/
   DLC validation before binding, corrected flagged coarse SOC, bus isolation,
   atomic complete/fresh telemetry gating, monotonic SOC/current capacity
   reconciliation, physical plausibility gates, timer/integer-boundary fixes,
   and three complementary collision defenses.
5. Nominal telemetry, current calibration, standby, and real multi-pack
   collisions were bench-verified. Alarm behavior and the newest hardening
   paths remain explicit bench-validation items rather than assumed facts.

## Key facts discovered (details in LEARNINGS.md)

- Packs have three states: ON (full telemetry) / STANDBY on power-off
  (SOC frames only) / SILENT after 19m54s of standby (spec's 20-min rule;
  ON packs broadcast indefinitely). Packs keep each other awake on a
  shared bus; only physical disconnection leads to true sleep.
- Node claims are assigned at boot, can collide (boot race), persist in
  pack NVM, and CANNOT be rewritten over the bus with fw 2.01.0003 -
  every documented and undocumented mechanism was tried. Runtime
  re-arbitration exists but is RAM-only and dies with deep sleep.
  Duplicate claims are therefore an operational hazard to check before
  every flight (dashboard or the driver's GCS warning; the system fails
  safe - arming is blocked while a collision or missing pack exists).
- The spec master heartbeat (PF 0x43) is HARMFUL to these packs (caused
  shutdowns and the original claim scramble). The driver is deliberately
  receive-only.
- Polled queries work (unique ID, model, serial, cycle count, per-cell
  temperatures) but responses are anonymous with multiple packs.

## Current state / open items

- Fleet at last check: five packs on nodes 0-4, no collisions (a lucky
  boot draw - can regress after deep sleep).
- Vendor engagement pending: node re-assignment procedure (ZhianLink
  tool), supplementary broadcast-protocol doc, firmware fix for the
  boot-race duplicate claims. Contacts in LEARNINGS.md §7.
- Nice-to-haves parked: live GCS re-verification of the three newest
  messages, alarm-bit mapping check against a real fault, <1% current
  scale with a clamp meter, upstream ArduPilot PR.
