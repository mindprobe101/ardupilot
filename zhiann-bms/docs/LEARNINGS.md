# Zhiann BMS — consolidated bench-test learnings

Everything learned from the bench campaign 2026-07-18 → 2026-07-23, with
4-5 × 24S packs (ZB3CN7 series, fw 2.01.0003), CAN hub(s), NUCLEO-H753ZI
sniffer and CubeOrange+ running the custom `AP_BattMonitor_ZhiannBMS`
driver. Protocol details live in `2026-07-18-zhiann-bms-can-protocol.md`;
this file is the operational knowledge.

## 1. Power states and wake behaviour

- Three transmission states per pack:
  - **ON**: full telemetry — detail block (cells/temps/SOC-fine/voltage/
    current) every 500 ms (~18 f/s) + SOC frame every 200 ms + 2 s frame.
  - **STANDBY** (entered immediately on button-off): detail blocks stop,
    SOC frame (and 2 s frame) continue at full rate.
  - **SILENT**: standby ends after **19 min 54 s measured** — matches the
    spec's 20-minute no-heartbeat transmit stop. ON packs are exempt
    (observed broadcasting for hours). "Deep sleep" = this state.
- **Connecting a powered-off pack to a silent bus does NOT wake it**
  (verified: zero frames while connecting five off packs).
- **Bus activity wakes connected packs** ("linked power-on"): press one
  pack's button and every connected pack boots. Removing the CSS_EN pin
  from the connectors did NOT disable this — wake also rides on CAN
  activity. CSS_EN measured ~21.4 V fluctuating (modulated signal line,
  cannot decode without a scope; vendor holds a power-line-comms patent).
- **Mutual keep-awake**: packs on a shared live bus never sleep — each
  pack's standby SOC frames are wake activity for the others. Parasitic
  drain implication: for storage, physically disconnect packs from the
  bus; button-off alone never leads to silence while any neighbour talks.
- The 0x402A10x 2 s frame's source address follows the surviving pack
  (single pack on node 1 → 0x402A101), so it is per-pack, with one
  designated transmitter when several packs are present.
- Boot announcements: ~18 s of PF 0x0D frames (SA=0xFE, 4/s, fixed
  payload) at power-on; PF 0x0C variant seen ~25 s in. Usable in future
  as a "pack is (re)booting" signal.

## 2. Node claims — the central problem

- Each pack broadcasts on a node index (0..15 possible; 0-4 seen) that
  prefixes all its IDs. ArduPilot maps `BATTn_SERIAL_NUM` to this node.
- **Each pack has a preferred node.** It takes that node when free and falls
  back to another when it is occupied. Fallbacks are remembered, so the same
  pack set tends to reproduce the same allocation on every boot.
- **Two packs can end up on the same node**, and then stay there for as long
  as that pack set keeps being used. Two and even three packs have been seen
  broadcasting as node 2 simultaneously. Change the pack set and it can
  resolve; nothing on the bus resolves it in place.
- **Nothing sendable over the bus rewrites a claim.** Exhaustively tried and
  failed: synchronized boots, staggered off-bus boots, hot-rejoin while
  running, cold-boot onto a live bus with the node audibly occupied, PF 0x07
  address assignment with and without sustained heartbeats, and the
  settings-write path (no CAN_IDX setting exists in fw 2.01.0003; a settings
  scan of 1..200 answered only model/serial/versions).
- **A vendor flight controller does not help.** With a V10Pro running on the
  bus throughout, collisions still formed and were never repaired; it simply
  reported one pack where two existed. It is powered *from* the packs, so its
  first frame always arrives 1.6-17 s after they start transmitting — it
  cannot be present when a claim is made.
- **Pre-waking every pack improves the odds but is not a fix.** A short press
  wakes a pack's controller (SOC LEDs light) without powering it; a long press
  then powers it on. Short-pressing every pack before powering one on produced
  a clean four-node allocation three times out of five. It is worth doing, but
  it is not reliable and must not be treated as a guarantee.
- **Unexplained:** on 2026-08-14 a pack claimed node 2 a full **26 s** after
  another pack was established and broadcasting there. A simultaneous-claim
  race does not account for that, and no mechanism is yet known.
- **Always verify the node map before arming.** This is the only dependable
  step: confirm the expected number of distinct battery instances appears. If
  one is missing or flagged duplicate, power fully down and repeat.
- The permanent fix is distinct stored nodes per pack, reachable only through
  the ZhianLink USB-C tool, not over CAN. This is the outstanding vendor ask,
  along with whether firmware after 2.01.0003 implements the UID-rank
  arbitration described in patent CN118101624B (this one demonstrably does
  not).
- **NEVER transmit the master heartbeat (PF 0x43)**: continuous heartbeats
  made packs power themselves off, and a registered-bitmap omitting a
  momentarily-silent pack caused it to release its node and re-join on an
  occupied one.

### Detecting a duplicate

Measured signatures, all independent of each other:

| signal | one pack | two packs |
|---|---|---|
| SOC-coarse frame `0x401A10n` | 4.5/s (201 ms) | 9/s (70-104 ms) |
| detail frame `0x2E09(0x51+0x20n)` | 2/s (508 ms) | 4/s |
| temp/SOC frame payloads | one cluster | two clusters, ~50/50 |

The SOC-coarse cadence is the primary signal: every pack emits it on its own
node, and it doubles while the second pack is still in **standby**, roughly
150 s before the detail frames double.

Two traps that cost real effort and will catch anyone repeating this analysis:

- **A pack alone on the bus storms retransmits at ~4 ms**, because nothing
  ACKs it. Those gaps carry no cadence information and must be discarded, or
  every single pack reads as a duplicate. Worse, when something *starts*
  ACKing, the storm breaks into bursts whose **exit gaps** (53–135 ms measured)
  land squarely in the doubled band. That is the normal power-on order for the
  aircraft, and it is the only false positive the corpus contains.
- **A single pack dithers temperature by exactly 1.0 °C at ~1 Hz**, which reads
  as two payload clusters if you use temperature as ground truth. Use the
  pack-voltage mirror instead; it caught every real episode.

Cadence alone is also structurally blind to roughly 20 % of possible phase
alignments between two packs (an offset under ~20 ms merges into the retransmit
band), and the observed offsets were **locked**, not drifting — two of the four
pack-pairs sat at 36 ms, only 16 ms from that blind zone. The identity signal
is a necessary complement, not a formality.

The 2 s frame `0x402A10n` carries a stable per-pack identity in bytes 4..7,
useful for naming which packs collided — but **not every occupied node emits
it**, and the rule governing which do is not known. Node 0 emitted it in all
seven captures; other nodes emitted it inconsistently, and in one capture three
nodes emitted concurrently while an occupied fourth stayed silent. Two of the
three real collisions in the corpus produced no identity frames at all, so it
cannot be relied on to detect a collision — only to name one already found.

## 3. Operating procedures that work

- **Before every session/flight: check for node overlap.** Options:
  - live dashboard: `bms_dashboard.py` → http://localhost:8787 (red row
    = multiple packs on one node), logs simultaneously;
  - the Cube itself: GCS warning "ZhiannBMS: duplicate pack on node N"
    plus that instance held unhealthy (arming blocked while a collision
    or a missing pack exists — the system fails safe).
- Mission profiles: `zhiann-bms/params-4pack.param` / `params-2pack.param`
  (auto-bind serials -1; 2-pack profile disables BATT3/4). With auto-bind
  the BATTn↔pack mapping follows first-heard order and may swap between
  boots; pin serials only when per-pack identity matters.
- Cell display: MAVLink carries only 14 of 24 cells; the shown per-cell
  voltages are redistributed so their sum equals pack voltage (~7.19 V
  each at 100.7 V) — relative differences remain meaningful. True cells
  are in the Cube dataflash (`ZBC1`/`ZBC2` messages, all 24, plus `ZBMS`
  status) — enabled by `LOG_DISARMED=1` without arming.
- Never parallel the power leads of packs at different SOC on the bench
  (fleet spans ~88-103 V; equalization currents would be destructive).
  Match SOC before paralleling in the airframe.

## 4. Current, SOC and sensors (calibrated facts)

- Current: s32 LE in the pack-voltage frame, discharge negative on the
  wire, zero-deadband below a few amps, ~2 Hz filtered.
  **The scale is 1 mA/LSB**, which the driver now decodes directly, so
  `BATTn_CURR_MULT` stays at its 1.0 default. The original 2 mA/LSB came from
  correlating against an analog power module whose own calibration was the
  weak link. Three independent lines agree: coulomb-counting 14 pack-flights
  against the packs' own SOC; the vendor flight controller's decode of the
  same field; and the pack broadcasting its **own rated capacity of 44.0 Ah**
  in the `4E0958` frame. At 1 mA/LSB the packs measure ~41 Ah, i.e. ~94 % of
  rated, normal for used cells.
  Beware when re-measuring: flights confined to high SOC (100->75 %)
  under-estimate capacity badly, because the BMS SOC falls faster than real
  coulombs near full charge. Use sweeps of >=30 % SOC only.
- SOC: 0.1 % resolution in the temp/SOC frame, 1 % copy in byte 0's low
  seven bits at 5 Hz (bit 7 and byte 1 are dynamic status; the frame also
  carries a voltage×320 mirror). New/replaced packs may
  need minutes for their SOC estimate to settle (observed 90→96 %).
- Temperatures: two sensors in the broadcast (0.1 °C, assumed signed);
  eight per-cell sensors available via polled PF 0x82 (1 °C, offset −40).
- Cycle counts, model ('ZB3CN7…'), serial ('ZP3CN7…'), unique 6-byte ID
  are pollable (see protocol doc §polled) — but responses are anonymous
  (SA=0) with multiple packs on the bus. Bench tool: `pack_info.py`.

## 5. Tooling inventory (all in ~/can_bms)

| Tool | Purpose |
|---|---|
| `can_sniffer/` firmware | Nucleo sniffer. Currently a **listen-only** build: locked to 1 Mbit/s, FDCAN bus-monitoring mode, so it cannot ACK or transmit and cannot perturb an observed system. Restore the probe + `FDCAN_MODE_NORMAL` in `main()` if the TX passthrough (`T,<id>,<hex>` over VCP) is needed |
| `capture_v10pro.py` | wall-clock timestamped raw capture to file |
| `can_sniffer/scripts/capture.py` | raw capture to file |
| `can_sniffer/scripts/decode.py` | protocol decoder / live summary |
| `bms_dashboard.py` | live web view (localhost:8787) + simultaneous logging |
| `silence_test.py` | wall-clock-tagged logging with per-minute status |
| `pack_info.py` | fleet query: IDs, serials, models, cycles, temps |
| `log_bms_load.py` / `analyze_bms_load.py` / `correlate_ap_bms.py` | load-test capture, marker analysis, ArduPilot-log correlation (current calibration) |
| `zhiann-bms/` (in ardupilot repo) | protocol/ops documentation + parameter mapping templates |
| `libraries/AP_BattMonitor/tests/test_zhiann_decode.cpp` | replay/boundary regression tests (21 cases) |

## 6. ArduPilot driver capabilities (branch Copter-4.6.3-zhiann-bms)

Voltage, current, consumed mAh/Wh initialized from BMS SOC and then kept at
the conservative maximum of the SOC-derived floor and current integration,
24 cells (14 live via MAVLink, all 24 in dataflash), both temps (max
reported), and fresh BMS SOC. The safety path requires five clean PACK
intervals plus a fresh atomic 24-cell snapshot; it checks PACK against both
the cell sum and SOC voltage mirror, rejects implausible values, maps alarm
faults, detects duplicate nodes/standby/unmapped packs, and is RX-only.
Multi-pack via
BATTn_SERIAL_NUM = node or -1 auto-bind, up to node 15 / 9 instances.
`BATTn_SERIAL_NUM` is snapshotted at initialization; reboot after changing it.

## 7. Vendor situation

Shenzhen Zhian New Energy (energy-z.com / zhiann.com). After-sales:
Li Zehua +86 185 7557 4674 lizh@energy-z.com; Chen Hong +86 155 0256 0774
chenh@energy-z.com. Tools: ZhianLink PC tool (USB-C, has parameter tab —
request from vendor), ZHIANN Android APK / iOS app / WeChat mini-program
(BLE), OTA firmware updates. Open asks: node re-assignment procedure /
CAN_IDX, supplementary broadcast-protocol doc (补充协议), firmware fix
for the boot-race duplicate claims (with the RAM-vs-NVM detail).

## 8. Open questions

- Whether newer pack firmware fixes boot-claim arbitration (vendor).
- Alarm frame (PF 0x24) real-world bit/source/clear behaviour — never
  observed live. Until verified, alarms are fanned out conservatively; on a
  multi-pack bus severe faults remain latched until FC reboot.
- 0x401A100 byte0 bit7/byte1 status, word at offset 4 (temperature-like),
  0x2E0943 word 0,
  0x2E0951 word 0 (counter/CRC-like).
- Broadcast temp signedness below 0 °C (assumed s16).
- Current scale is 1 mA/LSB (§4); a clamp meter would still pin it to <1 %.
- Node assignment RESOLVED 2026-08-13 (§2): stored in the pack and claimed
  unconditionally; same-node packs always collide and nothing on the bus can
  fix it. Remaining ask is the ZhianLink parameter that writes it.
- Semantics of the per-pack value in `402A10n[4..7]` (serial hash? checksum?),
  and whether it is unique fleet-wide — needed before leaning on it hard.
- Whether a displaced pack that goes silent is genuinely still powered — if
  so, an unmonitored live pack is a real hazard needing a driver response.
- Whether replaying the FC's 1 Hz keepalive alone unlocks the `3E`/`4E`/`5E`
  identity/nameplate/SOH blocks for our driver. Requires TX; **not to be
  attempted without an explicit decision** given the July heartbeat incident.
  Note the FC's keepalive carries no membership bitmap, which is what made
  the July attempt harmful.
- Whether the FC can repair an *existing* duplicate-node condition rather than
  merely never provoking one. Needs the known-colliding pack set on that bus.
