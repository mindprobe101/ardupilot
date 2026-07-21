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

- Each pack claims a node index (0..15 possible; 0-4 seen) that prefixes
  all its broadcast IDs. ArduPilot maps `BATTn_SERIAL_NUM` to this node.
- **Claim arbitration runs only at boot.** Membership changes (a pack
  joining) can force a runtime re-arbitration — but that result lives in
  RAM ONLY and is lost when packs deep-sleep. The NVM (persisted) claims
  are what boots re-load, and those can collide: we observed two and even
  THREE packs simultaneously broadcasting as node 2.
- **Nothing sendable over the bus rewrites the NVM claim.** Exhaustively
  tried and failed: synchronized boots, staggered off-bus boots,
  hot-rejoin while running, cold-boot onto a live bus with the node
  audibly occupied, PF 0x07 address assignment with and without
  sustained heartbeats (lease decays; pack pauses ~7-15 s then rejoins
  its old node; no 0x08 ack), settings-write path (no CAN_IDX setting
  exists in fw 2.01.0003; settings scan 1..200 answered only
  model/serial/versions).
- Boot draws are **non-deterministic**: the same fleet gave
  {0,1,2,2,-} one morning and a clean {0,1,2,3,4} the same evening.
  A clean draw is luck, not a fix, until the vendor provides a tool.
- The vendor's patent (CN118101624B) describes duplicate-detection +
  UID-rank re-arbitration; this firmware demonstrably does not implement
  it at boot (bit-synchronized identical announcements may be why packs
  never "hear" each other in the race).
- **NEVER transmit the master heartbeat (PF 0x43)**: continuous
  heartbeats made packs power themselves off, and a registered-bitmap
  omitting a momentarily-silent pack caused it to release its node and
  persistently re-join on an occupied one — this incident created the
  whole collision mess. (Bitmap appears to act as the allowed-node set
  for packs booting under a master.)

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

- Current: s32 LE in the pack-voltage frame, **2 mA/LSB** (calibrated
  against an analog power module: plateaus 2.05±0.05 mA/LSB), discharge
  negative on the wire, zero-deadband below a few amps, ~2 Hz filtered.
  `BATTn_CURR_MULT` trims residual scale error.
- SOC: 0.1 % resolution in the temp/SOC frame, 1 % copy at 5 Hz in the
  SOC frame (also carries voltage×320 mirror). New/replaced packs may
  need minutes for their SOC estimate to settle (observed 90→96 %).
- Temperatures: two sensors in the broadcast (0.1 °C, assumed signed);
  eight per-cell sensors available via polled PF 0x82 (1 °C, offset −40).
- Cycle counts, model ('ZB3CN7…'), serial ('ZP3CN7…'), unique 6-byte ID
  are pollable (see protocol doc §polled) — but responses are anonymous
  (SA=0) with multiple packs on the bus. Bench tool: `pack_info.py`.

## 5. Tooling inventory (all in ~/can_bms)

| Tool | Purpose |
|---|---|
| `can_sniffer/` firmware | Nucleo bitrate-autodetect sniffer + TX passthrough (`T,<id>,<hex>` over VCP) |
| `can_sniffer/scripts/capture.py` | raw capture to file |
| `can_sniffer/scripts/decode.py` | protocol decoder / live summary |
| `bms_dashboard.py` | live web view (localhost:8787) + simultaneous logging |
| `silence_test.py` | wall-clock-tagged logging with per-minute status |
| `pack_info.py` | fleet query: IDs, serials, models, cycles, temps |
| `log_bms_load.py` / `analyze_bms_load.py` / `correlate_ap_bms.py` | load-test capture, marker analysis, ArduPilot-log correlation (current calibration) |
| `zhiann-bms/` (in ardupilot repo) | prebuilt firmware + param presets |
| `libraries/AP_BattMonitor/tests/test_zhiann_decode.cpp` | replay regression tests (11 cases, real frames) |

## 6. ArduPilot driver capabilities (branch Copter-4.6.3-zhiann-bms)

Voltage, current (+consumed mAh/Wh), 24 cells (14 live via MAVLink, all
24 in dataflash), both temps (max reported), BMS SOC with coulomb-count
fallback, alarm-frame fault mapping (severe alarms block arming),
duplicate-node detection (unhealthy + GCS warning), standby detection
("press the pack button" hint), unmapped-node warning (claim migration),
per-node fleet inventory at boot, RX-only by policy. Multi-pack via
BATTn_SERIAL_NUM = node or -1 auto-bind, up to node 15 / 9 instances.

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
- Alarm frame (PF 0x24) real-world bit behaviour — never observed live.
- 0x401A100 word at offset 4 (temperature-like), 0x2E0943 word 0,
  0x2E0951 word 0 (counter/CRC-like), 0x402A100 tail bytes.
- Broadcast temp signedness below 0 °C (assumed s16).
- Exact current scale to <1 % (needs a clamp-meter reference).
