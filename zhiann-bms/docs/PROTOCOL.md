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
> heartbeat on the bus, so the drone profile does not enforce this. Later
> testing proved the node is a pack-firmware claim, not a hub port. Polled queries exist
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
| 0x2E0951 | 500 ms | u16 counter/crc, pack voltage (10 mV), **current: s32 LE, 1 mA/LSB, negative = discharge** |
| 0x401A100 | 200 ms | SOC = byte 0 low 7 bits (%); byte 0 bit 7 + byte 1 are dynamic status; pack voltage mirror (u16, 1/320 V); u16 slow-rising with load (temperature? 17→19); 0 |
| 0x402A10n | 2 s | u16 zeros, current coarse copy (s16 LE), then a **stable per-pack identity in bytes 4..7** (wire order, e.g. `FFFF5B8C`) |

Frame order within the 500 ms burst: 51, 42, 43, 44, 45, 46, 47, 41, 4A.

## Current calibration (2026-07-18 motor load test)

Identified from `bms_load_20260718_201602.log` + ArduPilot log 00000058.BIN
(different drone, analog power module on BAT instance 0, aligned via
arm/disarm events). That correlation gave 2 mA/LSB, but the power module's
own calibration was the weak link; the scale was corrected to **1 mA/LSB**
in 2026-08 from the pack's broadcast 44.0 Ah rated capacity and from coulomb
counting flights against the packs' own SOC:

- amps = -(s32 at 0x2E0951[4..7]) * 0.001  (positive = discharge)
- BMS reports exactly 0 below a few amps (deadband); updates ~2 Hz, filtered
- 0x402A100 s16[2..3] is the same current at 0.1 A/LSB (ratio exactly 100)
- 0x401A100[2..3] turned out to be voltage*320, NOT current
- 0x401A100[4..5] crept 17→19 during load and stayed after: temperature-like

## Multi-pack node addressing (confirmed 2026-07-18, 4 packs on one bus)

Each pack n (0..3) transmits its own ID block; the single-pack layout above
is node 0:

- cell/temp/SOC-fine frames: `0x2E0941 + 0x20*n` .. (types +0..+6, +9)
- pack voltage + current frame: `0x2E0951 + 0x20*n`
- SOC coarse frame: `0x401A100 + n`
- 2 s identity frame: `0x402A100 + n`. **Not every occupied node transmits
  it**, and the rule is not known: node 0 emitted it in all seven captures,
  other nodes emitted it inconsistently, and in one capture three nodes emitted
  concurrently while an occupied fourth stayed silent. Two of the three real
  collisions in the corpus produced none at all, so it can name a colliding
  pack but cannot be relied on to detect a collision.

Node numbering initially appeared positional because the original pack moved
to node 1 in the four-pack setup. Later collision/rejoin testing disproved
that: nodes are persisted firmware claims that can migrate or collide. Bus at
1 Mbit/s carried a clean four-pack draw (~65 frames/s) without gaps.

## Broadcast validation findings (2026-07-22 audit)

- All 437,558 recognized frames in the available capture corpus used one
  canonical payload length: 0x2E09xA is 4 bytes; every other accepted custom
  frame is 8 bytes. The driver therefore rejects RTR, error, CAN-FD, and
  wrong-length frames before binding or refreshing state.
- The coarse SOC is **not** a little-endian u16. Load/standby examples
  `DEE9...`, `2CFF...`, and `ACFF...` decode as 94%, 44%, and 44% using
  `byte0 & 0x7F`; treating bytes 0-1 as u16 incorrectly reports 100%.
- 35,613 complete canonical cell bursts spanned 22-55 ms (p50 24 ms, p99
  39 ms), with none over 250 ms. The driver assembles them privately and
  commits only complete, ordered generations with a full u16 cell count of 24.
- Every full-cell-sum mismatch over 1 V in the reviewed corpus occurred in
  known node-collision logs. Across 9,871 clean matched SOC/PACK samples the
  voltage-mirror difference was at most 0.41 V; a collision log had 3,306
  samples over 1 V (p99 11.54 V, max 11.68 V). Both checks complement cadence
  detection and require a clean run before the coherence fault clears.
- Valid vendor alarm IDs use priority 6, PF 0x24, controller destination
  0xF0-0xFE and BMS source 0x00-0xEF (including random 0x80-0xEF). No alarm
  was observed on the live fleet, so source-to-broadcast-node mapping and
  clear semantics remain unverified.

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

## Master traffic and the polled profile (2026-08-13, V10Pro FC on the bus)

Captured with a listen-only sniffer alongside a vendor V10Pro flight
controller. Full analysis and raw logs:
`can_bms/doc_source/2026-08-13-v10pro-master-bus-findings.md` and
`can_bms/doc_source/v10pro-captures-20260813/`.

Every ID below is absent from all our FC-free captures and appears only with
the V10Pro present, i.e. it is master-originated:

| ID | Rate | Role |
|---|---|---|
| `10015514` (SA 0x14) | 1 Hz forever | keepalive; bytes 0-1 rolling counter, byte 7 = byte 0 − 1, **no membership bitmap** |
| `1803E814` (SA 0x14) | 2 Hz, first 14 s | startup enumeration (PF 0x03 identify) |
| `1E0`, `559` | 5 Hz / 2 Hz, first 30 s / 10 s | startup handshake, **11-bit standard IDs** (packs only ever use 29-bit) |
| `2E5039` / `2E5839` | 2 Hz | per-node poll, node 0 / node 1 |
| `3E5039`, `4E5039`, `5E5039` (+ `58xx`) | ~0.1 Hz | polls for the identity / nameplate / SOH blocks |

The pack answers a poll with a block that never appears on our RX-only bus:

- `3E09xx` — ASCII identity: model `ZB3CN7…`, part/fw `ZAB2444-02-ZA01.02`,
  and a **per-pack serial** (`B1VZ0019` etc). The only unique pack identifier
  available without TX is the `402A10n`/`403A10n` tail (4-byte hardware id).
- `4E0958` — u16 LE `[440, 8, 9000, 24]` = **rated capacity 44.0 Ah**, 8
  temperature sensors, 90.00 V nominal, 24 cells.
- `5E09xx` — SOH %, cycle count, capacity repeated.

Membership is expressed by *which node slots the FC polls*, fixed at
enumeration: after a cold boot with only node 1 present it polls only `…5839`;
if a pack is hot-unplugged it keeps polling the empty node until the next boot.

**The FC is powered from the packs**, so its first frame always arrives
1.6-17 s after the packs start transmitting. It cannot be present when a pack
chooses its node — it can only react afterwards.

Bus behaviour worth knowing: a pack transmitting with nothing else on the bus
retransmits each frame ~13x at ~4 ms spacing until something ACKs (seen at
every power-on until the FC boots). `180DFFFE` (SA 0xFE, the J1939 null
address, payload `F6C66C3B093993C4`) precedes each pack coming online and is
identical for every pack, so it is a fixed announce, not a unique id.

## Unknowns / TODO

- Exact meaning of 0x401A100[4..5] (temperature-like) and 0x2E0943[0..1].
- First u16 of 0x2E0951 (counter/crc), 0x402A100 tail bytes.
- Current scale RESOLVED: 1 mA/LSB, decoded directly by the driver
  (`BATTn_CURR_MULT` stays at 1.0). The pack broadcasts its own 44.0 Ah rated
  capacity in `4E0958`, and coulomb counting against the packs' own SOC
  agrees. A clamp meter would still pin it to <1 %.

## Consumers

- ArduPilot driver: `AP_BattMonitor_ZhiannBMS` on branch
  `Copter-4.6.3-zhiann-bms` of the fork (BATT_MONITOR=30,
  CAN_Dx_PROTOCOL=15), nodes 0-15 via BATTn_SERIAL_NUM or -1 auto-bind.
- Host decoder: `can_bms/can_sniffer/scripts/decode.py`.
