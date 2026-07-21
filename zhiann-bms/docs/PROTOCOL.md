# Zhiann BMS CAN protocol (reverse engineered)

> 2026-07-20: vendor spec obtained — `zhiann-can-protocol-v1.9.pdf` (智安BMS
> CAN通讯协议 V1.9, Chinese). It confirms the bus basics (CAN 2.0B ext,
> little-endian, 1 Mbit/s since V1.4) and unit conventions (cell mV, pack
> 0.01 V, current 0.01 A in std frames, SOC %, temps offset −40 in polled
> frames), and defines a J1939-style polled command set + master heartbeat.
> HOWEVER the periodic broadcast frames we sniffed (0x2E09xx / 0x401A10x /
> 0x402A100) are NOT the spec's standard data frames (PF 0x22/0x24/0x26 —
> never seen on our bus): they are the customer/drone profile, whose
> definition (Appendix B "standard extended info 1") is an empty stub in
> this copy. Our reverse-engineered map below remains the source of truth
> for the broadcast profile. Notable spec facts: BMS stops transmitting
> after 20 min without a master heartbeat (PF 0x43, dest 0xFF, 2 s) and
> addresses have a 15 s lease — but our packs broadcast for hours with no
> heartbeat on the bus, so the drone profile does not enforce this (the
> hub presumably owns addressing; node = hub port). Polled queries exist
> for rated capacity (0.01 Ah), serial, SW version, cell count, chemistry,
> cycle count, per-cell temps, SOP — untested whether the drone firmware
> answers them. Spec alarm/warning bit fields (PF 0x24) are documented but
> that frame is absent from the broadcast profile.

Captured 2026-07-18 from a live 24S pack using the NUCLEO-H753ZI sniffer
(`can_bms/can_sniffer/`, raw log in `capture.log`). Not vendor-documented —
values inferred from internal consistency (cell sum matched pack voltage,
two independent SOC fields agreed, cell count field matched frame layout).

## Bus

- 1 Mbit/s classic CAN, all IDs 29-bit extended
- All multi-byte values little-endian u16

## Frames

| ID | Period | Layout |
|---|---|---|
| 0x2E0941 | 500 ms | cells 19-22 (mV) |
| 0x2E0942 | 500 ms | temp1 (0.1 C), temp2 (0.1 C), SOC (0.1 %), 0 |
| 0x2E0943 | 500 ms | u16 unknown, cell count, cells 1-2 (mV) |
| 0x2E0944 | 500 ms | cells 3-6 (mV) |
| 0x2E0945 | 500 ms | cells 7-10 (mV) |
| 0x2E0946 | 500 ms | cells 11-14 (mV) |
| 0x2E0947 | 500 ms | cells 15-18 (mV) |
| 0x2E094A | 500 ms | cells 23-24 (mV), only 4 bytes |
| 0x2E0951 | 500 ms | u16 counter/crc, pack voltage (10 mV), **current: s32 LE, 2 mA/LSB, negative = discharge** |
| 0x401A100 | 200 ms | SOC (%), pack voltage mirror (u16, 1/320 V), u16 slow-rising with load (temperature? 17→19 during test), 0 |
| 0x402A100 | 2 s | u16 zeros, current coarse copy (s16 LE, 0.2 A/LSB), FF FF, const u16 (serial/crc?) |

Frame order within the 500 ms burst: 51, 42, 43, 44, 45, 46, 47, 41, 4A.

## Current calibration (2026-07-18 motor load test)

Identified from `bms_load_20260718_201602.log` + ArduPilot log 00000058.BIN
(different drone, analog power module on BAT instance 0, aligned via
arm/disarm events, r2=0.937 continuous / plateau ratios 2.05±0.05 mA/LSB):

- amps = -(s32 at 0x2E0951[4..7]) * 0.002  (positive = discharge)
- BMS reports exactly 0 below a few amps (deadband); updates ~2 Hz, filtered
- 0x402A100 s16[2..3] is the same current at 0.2 A/LSB (ratio exactly 100)
- 0x401A100[2..3] turned out to be voltage*320, NOT current
- 0x401A100[4..5] crept 17→19 during load and stayed after: temperature-like

## Multi-pack node addressing (confirmed 2026-07-18, 4 packs on one bus)

Each pack n (0..3) transmits its own ID block; the single-pack layout above
is node 0:

- cell/temp/SOC-fine frames: `0x2E0941 + 0x20*n` .. (types +0..+6, +9)
- pack voltage + current frame: `0x2E0951 + 0x20*n`
- SOC coarse frame: `0x401A100 + n`
- `0x402A100` appears only once on the bus (one transmitter, data format
  differed from the single-pack capture - not used by any consumer)

Node numbering appears positional (hub port / chain order): the original
test pack showed up as node 1 in the 4-pack setup. Bus at 1 Mbit/s carries
all four packs (~65 frames/s) without collisions or gaps.

## Polled command set (verified live 2026-07-21, 4 packs via sniffer TX)

The drone-profile firmware DOES answer the spec's polled commands. Send to
PS=0x00 ("any address"; packs hold no assigned addresses, PS=0x01..0x03 get
no reply), our SA=0xF0, priority 6 → id = 0x18000000|(PF<<16)|(PS<<8)|0xF0:

- PF 0x03 identify → 0x04: 6-byte unique ID per pack ✓
- PF 0x80 settings → 0x81: idx 1 model ('ZB3CN7E1VZ0084' etc.), idx 3
  serial ('ZP3CN7B3SZ0071' etc.), idx 7 CAN proto version (reports 1.14) —
  all multi-frame (Table 3 transport, ~36 bytes). Capacity (15), cell
  count (8), rated V (14), current limits (16/17): op=0x01 "no such
  setting" — NOT implemented in this firmware.
- PF 0x86 cycle count → 0x87: u16 cycles, per pack (fleet read 14/11/13/13)
- PF 0x82 cell temps → 0x83: 8 sensors/pack, 1 C, offset −40 (25-26 C)
- PF 0x84 cell volts → 0x85: 24 cells multi-frame (redundant w/ broadcast)
- PF 0x43 heartbeat (PS=0xFF): **HARMFUL — do not send.** A short 5-frame
  test looked harmless, but CONTINUOUS heartbeats (2026-07-21) made the
  packs power themselves off, and a registered-bitmap that omitted a
  momentarily-silent pack caused it to release its node and re-join on an
  already-taken node → two packs colliding on the same CAN IDs until the
  packs are power-cycled. The drone profile needs no heartbeat.

CAVEAT: all responses carry SA=0x00, so with multiple packs on one bus
responses are anonymous (attribution by arrival order only). Per-pack
targeting would require the spec's address-assignment flow (PF 0x07 with
unique ID → addr 1-31, 15 s lease refreshed by heartbeat).

Tool: `can_bms/pack_info.py` (uses sniffer TX passthrough `T,<id>,<hex>`).

## Node-collision investigation (2026-07-21/22)

After the heartbeat incident scrambled node claims, two packs persistently
claim node 2 (fleet: 100%→n0, 96%→n1, 44%+42%→n2, n3 empty). Findings:

- Claims survive full simultaneous power cycles (linked power-on means
  packs always boot together; staggered boot impossible).
- Claim arbitration reshuffles when bus membership changes (pack
  replacement moved 44% from n1 to n2) but keeps producing the duplicate —
  a pack-firmware arbitration race, not hub-positional assignment.
- PF 0x07 address assignment: pack pauses broadcasting ~7-15 s (lease
  hold), then rejoins on its OLD node even when actively occupied — the
  lease decays without heartbeats and does not touch the broadcast claim.
  No PF 0x08 response seen. Tested on two packs (35C2=100%, BCDE=42%).
- Settings scan 1..200: only 1,3 (multi-frame model/serial), 4 (HW v2.00),
  5 (SW v2.01.0003), 6 (SW date 2025-07-10), 7 (proto v1.14) respond.
  No CAN_IDX / node-index setting exposed, so no settings-write fix.
- CONCLUSIVE (2026-07-22): fw 2.01.0003 NEVER re-arbitrates persisted node claims: synchronized boot, staggered off-bus boot, hot-rejoin, cold-boot-on-live-bus-with-node-occupied, PF 0x07 with/without heartbeats, 2-pack membership boots ALL fail. The ONLY event that ever rewrote persisted claims was a pack BOOTING while master heartbeats were active (the incident: packs rebooted under bitmap 0x07 and adopted registered nodes 0,1,2 persistently - bitmap appears to act as the allowed-node set at boot). Untested final option: single-pack surgical rewrite = only the misclaimed pack on the bus + Nucleo heartbeating bitmap 0x08 (only node 3 registered) + power-cycle the pack, expect it to adopt node 3 persistently.
- Safe conclusion otherwise: needs the VENDOR (config tool, supplementary protocol
  doc for the drone profile, or firmware fix). System is failsafe
  meanwhile: the contested node feeds one ArduPilot instance interleaved
  data and the empty node keeps one instance unhealthy -> arming blocked.

## Vendor + node-arbitration algorithm (research, 2026-07-22)

Vendor: 深圳市智安新能源科技有限公司 (Shenzhen Zhian New Energy Technology),
brand 智安新能源, site energy-z.com. Drone pack line includes 24S70Ah/24S44Ah
(ours), companion charger ZACC090-01, "智能并联一键开机" = the linked
power-on. No public config tool or protocol docs; no public decoders for
these frame IDs anywhere (this integration appears to be the first).

Patent CN118101624B (their parallel-networking method) documents the node
claim: packs broadcast addresses; on detecting a DUPLICATE of its own
address a pack broadcasts an "address change instruction" -> ALL packs
reset and re-derive node = rank of their unique hardware ID among packs
present (deterministic). Supports hot-swap re-networking. Our stuck
duplicate exists because linked power-on boots the two packs bit-
synchronized: identical announcement frames merge on the CAN bus, so
neither pack ever *hears* the duplicate. Desynchronized joins (hot-join
to a live bus) DO trigger detection -> global reset -> re-rank, which is
why the replacement pack's join reshuffled nodes. Fix procedure: remove
one duplicate pack from the bus, let the fleet run, hot-rejoin it.

## Standby behaviour (2026-07-22, 15-min capture standby_timeout.log)

At power-on: ~18s of PF 0x0D frames (SA=0xFE, dest 0xFF, 4/s, fixed 8-byte
payload — power-on announcement / bootup broadcast), then the 500ms detail
cell blocks appear for a burst, then STOP. Standby is entered within ~30s.

In standby (verified stable across the full 15 min, no timeout):
- detail cell blocks (0x2E09xx): STOPPED
- SOC frames (0x401A10n): CONTINUE at full ~4.5/s per node (22/s for 5)
- 2s status (0x402A10n): CONTINUE
- so packs on a live bus DO NOT deep-sleep within 15 min — they hold in a
  reduced-telemetry standby indefinitely, kept awake by bus activity. The
  spec's 20-min no-heartbeat transmit-stop was not reached in this window
  (would need >20 min; and there were no heartbeats, so it may apply).
- PF 0x0C frames (SA=0xFE, fixed payload) appeared ~25s in — a second
  announcement/keepalive variant.

MEASURED (2026-07-23, single pack, ACKing listener attached): power-off ->
standby SOC broadcast for 19min54s -> complete silence. Matches spec 2.4's
20-minute no-heartbeat transmit stop; the rule governs standby only
(powered-ON packs broadcast indefinitely). Log: can_bms/silence_test.log.

Driver implication: health keyed on the PACK_VOLT (detail) frame correctly
reports a standby pack as unhealthy while its SOC frame still flows — a
standby pack cannot pass arming. Basis for improvement #1 (standby GCS msg).

## Unknowns / TODO

- Exact meaning of 0x401A100[4..5] (temperature-like) and 0x2E0943[0..1].
- First u16 of 0x2E0951 (counter/crc), 0x402A100 tail bytes.
- Scale is within power-brick tolerance of exactly 2 mA/LSB; a clamp-meter
  spot check would pin it further.
- ArduPilot driver reads node 0 only; multi-instance support (MultiCAN +
  node select via BATTn_SERIAL_NUM) not yet implemented.

## Consumers

- ArduPilot driver: `AP_BattMonitor_ZhiannBMS` on branch
  `Copter-4.6.3-zhiann-bms` of the fork (BATT_MONITOR=30, CAN_Dx_PROTOCOL=15).
- Host decoder: `can_bms/can_sniffer/scripts/decode.py`.
