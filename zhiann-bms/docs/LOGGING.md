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
| Alm | bitmask | Alarm word from BMS PF 0x24; on multi-pack buses this is the unexpired conservative union of anonymous reports |
| Warn | bitmask | Warning word from the same frame; likewise unioned and aged on multi-pack buses |
| Vmir | raw | voltage mirror from the SOC frame; volts = Vmir / 320 |
| NCel | - | cell count reported by the pack (24 for these packs) |
| Age | ms | time since the last detail frame (large = standby or lost) |

Flag bits: bit0 = healthy, bit1 = standby (SOC frames flowing, detail
frames stopped), bit2 = duplicate-node collision active, bit3 = SOC came
from the fine 0.1% frame (else the coarse 1% frame), bit4 = current has
ever been seen from the pack, bit5 = an unmapped physical pack is
live, bit6 = a held coherence fault (PACK differs by more than 1 V from
the atomic 24-cell sum or from a recent SOC-frame voltage mirror, on two
consecutive checks), bit7 = the last PACK current reading was implausible
(voltage is still accepted; has_current is held false while it persists).

Alarm/Warning word bits (vendor spec table 19): B15 discharge
overcurrent, B14 battery damaged, B13 AFE fault, B12 low temperature,
B11 high temperature, B10 cell undervoltage, B9 cell overvoltage,
B8 temperature sensor fault, B0 charge overcurrent. Alarms are messages
only: they are logged, mapped to the MAVLink fault bitmask and repeated
to the GCS every 10 s while active, but they never gate the health flag.
Because source-address to pack-node mapping is unverified, every alarm is
delivered to every configured instance as an unexpired union.

### ZBC1 / ZBC2 — full cell voltages

ZBC1 carries cells 1-12, ZBC2 cells 13-24, in mV. The driver commits these
atomically only after all seven slices arrive in canonical order within
250 ms and the full LE cell-count field equals 24. 65535 means no recent
coherent snapshot. MAVLink can carry only 14 cells; dataflash retains all 24.

### Standard messages affected by this driver

- `BAT`: Volt/Curr from the BMS. Consumed mAh/Wh is seeded from the first
  valid SOC in each connection session, integrated from current thereafter,
  and raised when a lower BMS SOC implies more use; SOC noise/increases never
  erase integrated consumption. Temp is the HOTTER sensor; RemPct is fresh
  BMS SOC while the complete monitor state is healthy. The reconciled
  consumed-mAh capacity floor continues independently for failsafe checks.
- `BCL`: cells 1-12 (+13/14 in BCL2) - subset of ZBC1/2.
- `MSG`: the GCS texts below are also recorded here.

## 2. GCS text messages (Mission Planner messages tab)

| Message | Meaning | Operator action |
|---|---|---|
| `ZhiannBMS: pack on node N` | fleet inventory: a pack was heard on node N (once per node per boot) | verify the set matches the packs you installed |
| `ZhiannBMS: duplicate pack on node N` | two or more packs share node N; that instance's data is a mixture and is held unhealthy | do not fly; fix node claims (see LEARNINGS) |
| `ZhiannBMS: pack on node N not mapped to any battery` | a pack is broadcasting on a node no BATTn_SERIAL_NUM points at; all configured instances are held unhealthy while it remains live | do not fly; adjust BATTn_SERIAL_NUM / instance count |
| `ZhiannBMS: pack on node N in standby` | pack present (SOC frames flowing) but not enabled (detail frames stopped) | press the pack's power button |
| `ZhiannBMS: incoherent data on node N` | PACK differs by more than 1 V from an atomic 24-cell sum or a recent SOC voltage mirror on two consecutive checks; the fault is held until a clean run | do not fly; inspect node claims and raw log |
| `ZhiannBMS: BMS alarm: <names>` | the BMS alarm word has active bits; repeated every 10 s while active | act on the named alarms before flight |
| `ZhiannBMS: implausible current from pack on node N` | PACK current failed plausibility; voltage is kept, current/consumption are frozen and has_current is false while it persists | inspect the pack; current-based failsafes are degraded |
| `ZhiannBMS: temperature sensor fault on node N` | both temperature sensors read implausibly; no temperature update | inspect the pack temperature sensors |
| `ZhiannBMS: pack on node N reports M cells, expected 24` | the pack's cell-count word is not 24, so cell voltages never publish | verify pack model/firmware |
| `ZhiannBMS: BATTx_SERIAL_NUM invalid` | serial outside -1..15 (once per boot) | fix the parameter |
| `ZhiannBMS: BATTx_SERIAL_NUM duplicates BATTy` | two instances claim the same node (once per boot) | fix the parameters |
| `ZhiannBMS: protocol on multiple CAN ports; only first is used` | CAN_Dn_PROTOCOL selects ZhiannBMS on more than one driver slot (once per boot) | configure the protocol on exactly one port |

MAVLink BATTERY_STATUS notes: only 14 of 24 cells fit; ArduPilot spreads
the remaining voltage across the shown cells so their sum equals pack
voltage (~7.19 V per cell at 100.7 V is NORMAL; relative differences
remain meaningful). SOC shows -1 while an instance is unhealthy.

Unhealthy data blocks arming. In this ArduPilot base, an in-flight battery
`Unhealthy` state itself has hardcoded action 0; it reports the failure but
does not run `BATT_FS_LOW_ACT`/`BATT_FS_CRT_ACT`. Configure and bench-test
independent voltage/capacity thresholds and actions for every instance.

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
