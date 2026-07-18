/*
  battery monitor for the Zhiann CAN BMS

  protocol reverse engineered from live packs (2026-07): 1 Mbit/s classic
  CAN, all IDs 29-bit extended, values little-endian u16 unless noted.

  Up to 4 packs share the bus. Pack n (0..3) transmits the ID block
  0x2E0941 + 0x20*n plus an SOC frame at 0x401A100 + n. Node numbering
  follows hub/chain position. Frame types within a pack block (offset
  from 0x...41):

    +0x00  cell voltages 19..22 (mV)
    +0x01  temperature1 (0.1 C), temperature2 (0.1 C), SOC (0.1 %)
    +0x02  unknown, cell count, cell voltages 1..2 (mV)
    +0x03  cell voltages 3..6 (mV)
    +0x04  cell voltages 7..10 (mV)
    +0x05  cell voltages 11..14 (mV)
    +0x06  cell voltages 15..18 (mV)
    +0x09  cell voltages 23..24 (mV)
    +0x10  counter/crc, pack voltage (10 mV), current (s32, 2 mA,
           negative = discharge, reads 0 below a few amps)

  0x401A100+n: SOC (%), pack voltage mirror (1/320 V), temperature?
  0x402A100: single transmitter on the bus, unused here.

  The pack block repeats every ~500 ms. The current scale was calibrated
  against an ArduPilot analog power module (steady-load plateaus 13..48 A
  gave 2.05 +/- 0.05 mA/LSB -> 2 mA nominal).

  Instance selection: set BATTn_SERIAL_NUM to the pack node (0..3), or
  leave -1 to bind to the first pack not consumed by another instance.
 */
#include "AP_BattMonitor_ZhiannBMS.h"

#if AP_BATTERY_ZHIANNBMS_ENABLED

#include <AP_BoardConfig/AP_BoardConfig.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/utility/sparse-endian.h>

#include <string.h>

// pack n block: 0x2E0941 + 0x20*n, frame type = low 5 bits of (id - 0x41)
#define ZHIANN_BLOCK_BASE      0x2E0941UL
#define ZHIANN_BLOCK_LAST      0x2E09B1UL
#define ZHIANN_SOC_BASE        0x401A100UL

// frame types within a pack block
#define ZHIANN_FRAME_CELL_19_22   0x00
#define ZHIANN_FRAME_TEMP_SOC     0x01
#define ZHIANN_FRAME_CELL_1_2     0x02
#define ZHIANN_FRAME_CELL_3_6     0x03
#define ZHIANN_FRAME_CELL_7_10    0x04
#define ZHIANN_FRAME_CELL_11_14   0x05
#define ZHIANN_FRAME_CELL_15_18   0x06
#define ZHIANN_FRAME_CELL_23_24   0x09
#define ZHIANN_FRAME_PACK_VOLT    0x10
#define ZHIANN_FRAME_SOC_COARSE   0x1F   // internal alias for 0x401A100+n

// consider the BMS gone after 5s without a pack voltage frame
#define ZHIANN_TIMEOUT_US      5000000UL

// current scale, amps per LSB of the s32 in the pack voltage frame
// (discharge negative)
#define ZHIANN_CURRENT_SCALE   0.002f

AP_BattMonitor_ZhiannBMS::AP_BattMonitor_ZhiannBMS(AP_BattMonitor &mon,
        AP_BattMonitor::BattMonitor_State &mon_state,
        AP_BattMonitor_Params &params)
    : AP_BattMonitor_Backend(mon, mon_state, params)
{
    _multican = NEW_NOTHROW MultiCAN{
        FUNCTOR_BIND_MEMBER(&AP_BattMonitor_ZhiannBMS::handle_frame,
                            bool, AP_HAL::CANFrame &),
        AP_CAN::Protocol::ZhiannBMS, "ZhiannBMS"};
    if (_multican == nullptr) {
        AP_BoardConfig::allocation_error("Failed to create ZhiannBMS multican");
    }

    // cells not yet received report as not-present
    memset(_interim.cells_mv, 0xFF, sizeof(_interim.cells_mv));

    // starts with not healthy
    _state.healthy = false;
}

void AP_BattMonitor_ZhiannBMS::store_cells(uint8_t first_cell, const uint8_t *data, uint8_t ncells)
{
    for (uint8_t i = 0; i < ncells; i++) {
        const uint8_t idx = first_cell + i;
        if (idx < ARRAY_SIZE(_interim.cells_mv)) {
            _interim.cells_mv[idx] = le16toh_ptr(&data[2 * i]);
        }
    }
    _interim.cells_seen = true;
}

bool AP_BattMonitor_ZhiannBMS::handle_frame(AP_HAL::CANFrame &frame)
{
    if (!frame.isExtended() || frame.isRemoteTransmissionRequest() || frame.isErrorFrame()) {
        return false;
    }
    const uint32_t id = frame.id & AP_HAL::CANFrame::MaskExtID;

    uint8_t node;
    uint8_t frame_type;
    if (id >= ZHIANN_BLOCK_BASE && id <= ZHIANN_BLOCK_LAST) {
        node = (id - ZHIANN_BLOCK_BASE) >> 5;
        frame_type = (id - ZHIANN_BLOCK_BASE) & 0x1F;
    } else if ((id & ~0xFUL) == ZHIANN_SOC_BASE && (id & 0xF) <= 3) {
        node = id & 0xF;
        frame_type = ZHIANN_FRAME_SOC_COARSE;
    } else {
        return false;
    }

    // bind this instance to a pack: explicit BATTn_SERIAL_NUM, or first
    // pack heard that no earlier-called instance consumed
    const int32_t serial = _params._serial_number.get();
    if (serial >= 0) {
        if (node != (uint8_t)serial) {
            return false;
        }
    } else if (_node < 0) {
        _node = (int8_t)node;
    } else if (node != (uint8_t)_node) {
        return false;
    }

    WITH_SEMAPHORE(_sem);

    switch (frame_type) {
    case ZHIANN_FRAME_PACK_VOLT:
        if (frame.dlc >= 4) {
            const uint32_t now_us = AP_HAL::micros();
            _interim.voltage = le16toh_ptr(&frame.data[2]) * 0.01f;
            if (frame.dlc >= 8) {
                // BMS sign convention is discharge-negative; ArduPilot's is
                // discharge-positive
                const int32_t raw = (int32_t)le32toh_ptr(&frame.data[4]);
                _interim.current_amps = -raw * ZHIANN_CURRENT_SCALE;
                if (_interim.have_current && _interim.last_frame_us != 0) {
                    const uint32_t dt_us = now_us - _interim.last_frame_us;
                    _interim.consumed_mah +=
                        calculate_mah(_interim.current_amps, dt_us);
                    _interim.consumed_wh += _interim.current_amps *
                        _interim.voltage * dt_us * 1.0e-6f / 3600.0f;
                }
                _interim.have_current = true;
            }
            _interim.last_frame_us = now_us;
        }
        break;

    case ZHIANN_FRAME_TEMP_SOC:
        if (frame.dlc >= 6) {
            // two sensors; report the hotter one
            const float t1 = le16toh_ptr(&frame.data[0]) * 0.1f;
            const float t2 = le16toh_ptr(&frame.data[2]) * 0.1f;
            _interim.temperature = MAX(t1, t2);
            _interim.temperature_time_ms = AP_HAL::millis();
            _interim.soc_pct = MIN(le16toh_ptr(&frame.data[4]) / 10, 100);
            _interim.soc_valid = true;
            _interim.soc_is_fine = true;
        }
        break;

    case ZHIANN_FRAME_SOC_COARSE:
        // coarse SOC, only used if the 0.1% frame is not being sent
        if (frame.dlc >= 2 && !_interim.soc_is_fine) {
            _interim.soc_pct = MIN(le16toh_ptr(&frame.data[0]), 100);
            _interim.soc_valid = true;
        }
        break;

    case ZHIANN_FRAME_CELL_1_2:
        if (frame.dlc >= 8) {
            store_cells(0, &frame.data[4], 2);
        }
        break;
    case ZHIANN_FRAME_CELL_3_6:
        if (frame.dlc >= 8) {
            store_cells(2, frame.data, 4);
        }
        break;
    case ZHIANN_FRAME_CELL_7_10:
        if (frame.dlc >= 8) {
            store_cells(6, frame.data, 4);
        }
        break;
    case ZHIANN_FRAME_CELL_11_14:
        if (frame.dlc >= 8) {
            store_cells(10, frame.data, 4);
        }
        break;
    case ZHIANN_FRAME_CELL_15_18:
        if (frame.dlc >= 8) {
            store_cells(14, frame.data, 4);
        }
        break;
    case ZHIANN_FRAME_CELL_19_22:
        if (frame.dlc >= 8) {
            store_cells(18, frame.data, 4);
        }
        break;
    case ZHIANN_FRAME_CELL_23_24:
        if (frame.dlc >= 4) {
            store_cells(22, frame.data, 2);
        }
        break;

    default:
        // unknown frame type from our pack: consumed but ignored
        break;
    }
    return true;
}

void AP_BattMonitor_ZhiannBMS::read()
{
    WITH_SEMAPHORE(_sem);

    const uint32_t tnow_us = AP_HAL::micros();
    if (_interim.last_frame_us == 0 ||
        (tnow_us - _interim.last_frame_us) > ZHIANN_TIMEOUT_US) {
        // BMS gone: report zero rather than freezing the last reading.
        // 0V is ArduPilot's "no reading" convention and cannot trigger
        // the voltage failsafes, which are guarded by voltage > 0
        _state.healthy = false;
        _state.voltage = 0;
        _state.current_amps = 0;
        _has_cell_voltages = false;
        _has_temperature = false;
        return;
    }

    _state.healthy = true;
    _state.voltage = _interim.voltage;
    _state.current_amps = _interim.current_amps;
    _state.consumed_mah = _interim.consumed_mah;
    _state.consumed_wh = _interim.consumed_wh;
    _state.last_time_micros = _interim.last_frame_us;
    _state.temperature = _interim.temperature;
    _state.temperature_time = _interim.temperature_time_ms;
    memcpy(_state.cell_voltages.cells, _interim.cells_mv,
           sizeof(_state.cell_voltages.cells));

    _have_current = _interim.have_current;
    _has_cell_voltages = _interim.cells_seen;
    _has_temperature = (_interim.temperature_time_ms != 0) &&
        ((AP_HAL::millis() - _interim.temperature_time_ms) <= AP_BATT_MONITOR_TIMEOUT);
}

bool AP_BattMonitor_ZhiannBMS::capacity_remaining_pct(uint8_t &percentage) const
{
    if (!_state.healthy || !_interim.soc_valid) {
        return false;
    }
    percentage = _interim.soc_pct;
    return true;
}

#endif  // AP_BATTERY_ZHIANNBMS_ENABLED
