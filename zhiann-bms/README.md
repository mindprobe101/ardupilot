# Zhiann BMS integration assets

Reviewed ArduCopter 4.6.3 build for CubeOrangePlus with the
AP_BattMonitor_ZhiannBMS driver (`BATT_MONITOR=30`, `CAN_Dx_PROTOCOL=15`).

- `params.param` — configuration template, not a complete flight-safety profile
- `docs/SUMMARY.md` — project summary: what exists, how it was built
- `docs/VALIDATION.md` — software/bench evidence and remaining experiments
- `docs/PROTOCOL.md` — the reverse-engineered CAN protocol reference
- `docs/LOGGING.md` — meaning of every logged entry (ZBMS/ZBND, GCS messages,
  sniffer log formats)
- `docs/LEARNINGS.md` — consolidated bench-test learnings and operating
  procedures

## The pack set is one battery

The packs are wired in parallel, and each broadcasts on its own bus "node" — a
number the pack firmware picks for itself, which can migrate between boots and
which two packs can end up claiming at once.

This driver does not map nodes to battery instances. It consumes **every** node
on the bus and publishes the set as a **single** battery:

| Published | Reduction | Why |
|---|---|---|
| Voltage | mean | parallel packs sit at the same voltage |
| Current | **sum** | the set's draw is the sum of its packs |
| State of charge | mean | |
| Temperature | max | the hottest pack is the one that matters |
| Cell voltages | mean per cell index | |

Configure exactly **one** battery monitor with type 30. A second one would
publish a duplicate of the same battery and is deliberately held unhealthy.

This is what makes node collisions a non-event. Two packs sharing a node just
means one fewer node carrying both packs' frames; the averages absorb it and
nothing needs reconfiguring. The same parameter file flies two packs or five.

Because the pack count is discovered at runtime, the set has **no fixed mAh
capacity**. Leave `BATT_CAPACITY` at 0 and use voltage-based failsafes.

## What the operator sees

Information, on the GCS. None of it blocks arming — checking the count against
the packs actually loaded is a preflight step:

```
ZhiannBMS: pack on node 0            (once per node, as each is found)
ZhiannBMS: 4 packs delivering        (once, after the bus stops growing)
ZhiannBMS: a node carries 2 packs, count reads low
ZhiannBMS: 3 of 4 packs, lost 1      (a pack stopped during the flight)
ZhiannBMS: packs differ by 1.6 V     (the packs stopped agreeing)
```

The last one is the check that averaging would otherwise hide: a mean says
nothing about whether one pack has collapsed. The driver measures the **spread**
(highest minus lowest) of pack voltage, current and state of charge and warns
when it stops looking normal, after the condition has held for 3 s.

Thresholds are calibrated against a 785 s flight on 2026-08-14, four packs,
354 A peak:

| | measured healthy | threshold |
|---|---|---|
| voltage spread | median 0.13 V, max 0.56 V | **1.0 V** |
| current spread | median 1.7 % of total, p95 4.7 % | **25 % of total** |
| SOC spread | max 2.1 % | **15 %** |

Current is judged as a fraction of the total rather than an absolute, and only
above 20 A. At peak load the four packs shared 88.6/87.5/88.8/89.5 A — an
absolute 30 A threshold fired four times in that flight, every one a throttle
transient where the BMS deadband parks a pack at exactly 0 A while its
neighbours ramp. The 3 s hold removes those.

Cell voltages are **not** sent over MAVLink. ArduPilot can carry at most 14 of
the 24 and rescales them so their sum matches pack voltage, which displays
~7.3 V per cell on a 4.2 V chemistry — a number that cannot be read at face
value. All 24 real cells per pack are in the `ZBC1`/`ZBC2` dataflash messages.

Health is simple: the battery is healthy while **at least one pack** is
delivering complete, fresh data.

## Before flying

Confirm `BATT_MONITOR=30` on exactly one instance and `CAN_Dx_PROTOCOL=15` on
exactly one CAN driver. Set and bench-test `BATT_LOW_VOLT`, `BATT_CRT_VOLT` and
`BATT_FS_LOW_ACT`/`BATT_FS_CRT_ACT` for the airframe; the parameter file
deliberately does not invent safety limits. Keep `BATT_FS_VOLTSRC` at 0 (Raw).

Flash is tight on CubeOrangePlus even with the extra hwdef. Recheck free flash
on every rebuild rather than assuming headroom.

## Building firmware for this integration

Configure with the extra hwdef (disables the unused non-SBG ExternalAHRS
backends to fit flash; Lua scripting stays enabled):

    ./waf configure --board CubeOrangePlus --extra-hwdef=zhiann-bms/extra-hwdef.dat
    ./waf copter
