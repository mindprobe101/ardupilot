# Zhiann BMS — meaning of every logged entry

Reference for all data recorded by this integration: the ArduPilot
dataflash messages written by `AP_BattMonitor_ZhiannBMS`, the GCS text
messages it emits, and the host-side sniffer log formats.

## 1. ArduPilot dataflash messages (SD card .BIN logs)

Written at 2 Hz. With `LOG_DISARMED=1` they record without arming.

### ZBMS — the pack set, as published to ArduPilot

| Field | Unit | Meaning |
|---|---|---|
| TimeUS | us | boot-relative timestamp |
| Np | - | packs delivering complete, fresh data |
| Dup | - | 1 = some node appears to carry two packs |
| Volt | V | mean pack voltage — what the voltage failsafes see |
| Curr | A | total current, summed across the packs, discharge positive |
| SOC | % | mean state of charge |
| Temp | degC | highest pack temperature |
| DV | V | pack voltage **spread** (highest minus lowest) — this is what the imbalance warning uses |
| DA | A | pack current spread |
| DS | % | pack state of charge spread |
| SDV | V | standard deviation of pack voltage across the set |
| SDA | A | standard deviation of pack current |
| SDS | % | standard deviation of pack state of charge |
| Alm | bitmask | alarm word from BMS PF 0x24; the alarm frame is anonymous, so this is the unexpired union across packs |
| Warn | bitmask | warning word from the same frame, likewise unioned and aged |

The `D*` columns are the ones to look at first after any battery complaint:
the aggregate can look perfectly healthy while one pack is failing, and the
spread is what shows it. Spread rather than standard deviation drives the
warning because standard deviation shrinks as packs are added — the same
physical fault reads 0.50x its size on two packs and 0.40x on five — so a
fixed threshold would mean different things on different flights. The standard
deviations are logged alongside for reference.

### ZBND — the per-node readings behind the aggregate

One row per live node per cycle, so a single misbehaving pack can be identified
after the fact.

| Field | Unit | Meaning |
|---|---|---|
| TimeUS | us | boot-relative timestamp |
| Node | - | proprietary broadcast node (0-15) |
| Ctb | - | 1 = this node was included in the published aggregate. A node can be present (SOC frames arriving) but not contributing, e.g. a pack in standby |
| Volt | V | this pack's voltage |
| Curr | A | this pack's current, discharge positive |
| SOC | % | this pack's state of charge |
| Temp | degC | higher of this pack's two sensors |
| Dup | - | 1 = this node's frame cadence says two packs share it |
| ID | raw | per-pack identity from the 2s frame, e.g. `FFFFBDBD`; identifies WHICH physical pack is on this node. 0 when the pack has not sent one |

Alarm/Warning word bits (vendor spec table 19): B15 discharge
overcurrent, B14 battery damaged, B13 AFE fault, B12 low temperature,
B11 high temperature, B10 cell undervoltage, B9 cell overvoltage,
B8 temperature sensor fault, B0 charge overcurrent. Alarms are messages
only: they are logged, mapped to the MAVLink fault bitmask and repeated
to the GCS every 10 s while active, but they never gate the health flag.
The alarm frame is anonymous - its source address does not reliably identify
the pack - so alarms are unioned across the set and aged out together.

### ZBC1 / ZBC2 — full cell voltages, per pack

One row each per live pack per cycle: `Node`, then ZBC1 carries cells 1-12 and
ZBC2 cells 13-24, in mV. Committed atomically only after all seven slices
arrive in canonical order within 250 ms and the pack's cell-count field equals
24; a pack that never satisfies that logs no rows.

MAVLink can carry only 14 cells, and the published battery averages the packs
together, so this is the only record of an individual cell in an individual
pack. It is what a post-flight investigation into one bad pack starts from.

### Standard messages affected by this driver

- `BAT`: Volt is the mean pack voltage, Curr the summed pack current. Consumed
  mAh/Wh is integrated from that sum. Temp is the hottest pack; RemPct is the
  mean BMS state of charge while the battery is healthy.
- `BCL`: cells 1-12 (+13/14 in BCL2), from the averaged cell set. Per-pack
  cells are in ZBC1/ZBC2.
- `MSG`: the GCS texts below are also recorded here.

## 2. GCS text messages (Mission Planner messages tab)

| Message | Meaning | Operator action |
|---|---|---|
| `ZhiannBMS: pack on node N` | a pack was heard on node N (once per node per boot) | none; the inventory line below is the one to check |
| `ZhiannBMS: N packs delivering` | once per boot, after the bus stops growing: how many packs are supplying complete data | **check against the packs you loaded** — this is the preflight step |
| `ZhiannBMS: a node carries 2 packs, count reads low` | two packs share a node, so the count above is one short of the packs present. Their data is interleaved and averaged in together | fine to fly; fix the collision when convenient by power-cycling the packs, or with the vendor ZhianLink tool |
| `ZhiannBMS: N of M packs, lost K` | a pack that was delivering has stopped; the rest now carry its share | land and investigate; the aircraft keeps flying on the remaining packs |
| `ZhiannBMS: packs differ by 1.6 V` | the spread across packs (voltage, current or state of charge) exceeded its threshold — one pack is not behaving like the others | land and investigate; check ZBND to see which node |
| `ZhiannBMS: set BATTx_MONITOR=0, one instance only` | more than one battery monitor is set to this type; only the first does anything and the rest are held unhealthy | set the extra instances to 0 and reboot |
| `ZhiannBMS: BMS alarm: <names>` | the BMS alarm word has active bits; repeated every 10 s while active | act on the named alarms before flight |
| `ZhiannBMS: protocol on multiple CAN ports; only first is used` | CAN_Dn_PROTOCOL selects ZhiannBMS on more than one driver slot | configure the protocol on exactly one port |

MAVLink BATTERY_STATUS notes: only 14 of 24 cells fit; ArduPilot spreads
the remaining voltage across the shown cells so their sum equals pack
voltage (~7.19 V per cell at 100.7 V is NORMAL; relative differences
remain meaningful). SOC shows -1 while the battery is unhealthy.

The battery is healthy while at least one pack is delivering complete, fresh
data; pack count and collisions are information only and never block arming.
Unhealthy data does block arming. In this ArduPilot base an in-flight
`Unhealthy` state has hardcoded action 0: it reports the failure but does not
run `BATT_FS_LOW_ACT`/`BATT_FS_CRT_ACT`. Configure and bench-test voltage
thresholds and actions for the airframe.

## 3. Host-side sniffer logs (Nucleo -> PC)

Raw line format from the sniffer firmware:

    F,<board_ms>,<hex id>,<E|S>,<len>,<hex data>

- `board_ms`: sniffer uptime in ms (monotonic, resets with the Nucleo)
- `hex id`: 29-bit extended CAN id (`E`) - see PROTOCOL.md for the map
- `len`: payload byte count; `hex data`: payload bytes in bus order
- Lines starting `#` are sniffer status (bitrate probe results, `# alive`
  heartbeats every 5 s with total frame count, TX acks `# TX <id> ok`).

Wall-clock-tagged variants (dashboard / silence tests) prefix each line:

    HH:MM:SS.mmm F,...              <- host system time, ms resolution
    # STATUS HH:MM:SS.mmm <text>    <- per-minute rate summaries etc.

The sniffer transmits nothing on its own; `T,<hex id>,<hex data>` typed
into its serial port transmits one frame (bench tooling only).
