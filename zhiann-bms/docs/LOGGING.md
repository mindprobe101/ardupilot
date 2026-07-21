# Zhiann BMS — meaning of every logged entry

Reference for all data recorded by this integration: the ArduPilot
dataflash messages written by `AP_BattMonitor_ZhiannBMS`, the GCS text
messages it emits, and the host-side sniffer log formats.

## 1. ArduPilot dataflash messages (SD card .BIN logs)

Written at 2 Hz per battery instance whenever the pack has ever been
heard. With `LOG_DISARMED=1` they record without arming.

### ZBMS — per-pack status

| Field | Unit | Meaning |
|---|---|---|
| TimeUS | us | boot-relative timestamp |
| Inst | - | battery instance (0-based; BATT1 = 0) |
| Node | - | pack node this instance is bound to; -1 if not yet bound |
| Flag | bitmask | see below |
| Volt | V | pack voltage from the detail frame |
| Curr | A | pack current, discharge positive (BMS 2 mA/LSB x CURR_MULT) |
| SOC | % | BMS-reported state of charge |
| T1, T2 | degC | both temperature sensors (MAVLink/BAT gets only the max) |
| Alrm | bitmask | raw Alarm word from BMS alarm frame (PF 0x24); 0 if never received |
| Warn | bitmask | raw Warning word from the same frame |
| Vmir | raw | voltage mirror from the SOC frame; volts = Vmir / 320 |
| NCel | - | cell count reported by the pack (24 for these packs) |
| Age | ms | time since the last detail frame (large = standby or lost) |

Flag bits: bit0 = healthy, bit1 = standby (SOC frames flowing, detail
frames stopped), bit2 = duplicate-node collision active, bit3 = SOC came
from the fine 0.1% frame (else the coarse 1% frame), bit4 = current has
been seen from this pack.

Alarm/Warning word bits (vendor spec table 19): B15 discharge
overcurrent, B14 battery damaged, B13 AFE fault, B12 low temperature,
B11 high temperature, B10 cell undervoltage, B9 cell overvoltage,
B8 temperature sensor fault, B0 charge overcurrent. B14|B13 also force
the instance unhealthy (arming blocked).

### ZBC1 / ZBC2 — full cell voltages

ZBC1 carries cells 1-12, ZBC2 cells 13-24, in mV. 65535 means the cell
has not been received yet. These exist because MAVLink can only carry 14
cells live; the dataflash has the true 24.

### Standard messages affected by this driver

- `BAT`: Volt/Curr/consumed from the BMS; Temp is the HOTTER of the two
  sensors; RemPct is the BMS SOC (or coulomb-count fallback).
- `BCL`: cells 1-12 (+13/14 in BCL2) - subset of ZBC1/2.
- `MSG`: the GCS texts below are also recorded here.

## 2. GCS text messages (Mission Planner messages tab)

| Message | Meaning | Operator action |
|---|---|---|
| `ZhiannBMS: pack on node N` | fleet inventory: a pack was heard on node N (once per node per boot) | verify the set matches the packs you installed |
| `ZhiannBMS: duplicate pack on node N` | two or more packs share node N; that instance's data is a mixture and is held unhealthy | do not fly; fix node claims (see LEARNINGS) |
| `ZhiannBMS: pack on node N not mapped to any battery` | a pack is broadcasting on a node no BATTn_SERIAL_NUM points at (claim migrated, or more packs than configured) | adjust BATTn_SERIAL_NUM / instance count |
| `ZhiannBMS: pack on node N in standby` | pack present (SOC frames flowing) but not enabled (detail frames stopped) | press the pack's power button |

MAVLink BATTERY_STATUS notes: only 14 of 24 cells fit; ArduPilot spreads
the remaining voltage across the shown cells so their sum equals pack
voltage (~7.19 V per cell at 100.7 V is NORMAL; relative differences
remain meaningful). SOC shows -1 while an instance is unhealthy.

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
