/*
  battery monitor for the Zhiann CAN BMS

  protocol reverse engineered from a live 24S pack (2026-07): 1 Mbit/s
  classic CAN, all IDs 29-bit extended, all values little-endian u16
  unless noted:

    0x2E0941   cell voltages 19..22 (mV)
    0x2E0942   temperature1 (0.1 C), temperature2 (0.1 C), SOC (0.1 %)
    0x2E0943   unknown, cell count, cell voltages 1..2 (mV)
    0x2E0944   cell voltages 3..6 (mV)
    0x2E0945   cell voltages 7..10 (mV)
    0x2E0946   cell voltages 11..14 (mV)
    0x2E0947   cell voltages 15..18 (mV)
    0x2E094A   cell voltages 23..24 (mV)
    0x2E0951   unknown (counter/crc), pack voltage (10 mV)
    0x401A100  SOC (%), unknown, unknown        (every 200 ms)
    0x402A100  status + rolling counter, unused (every 2 s)

  the 0x2E09xx group repeats every ~500 ms. The pack current has not
  been identified yet (suspected in 0x401A100), so this backend does
  not provide current.
 */
#include "AP_BattMonitor_ZhiannBMS.h"

#if AP_BATTERY_ZHIANNBMS_ENABLED

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/utility/sparse-endian.h>

#include <string.h>

// frame IDs (29-bit extended)
#define ZHIANN_ID_CELL_19_22   0x2E0941UL
#define ZHIANN_ID_TEMP_SOC     0x2E0942UL
#define ZHIANN_ID_CELL_1_2     0x2E0943UL
#define ZHIANN_ID_CELL_3_6     0x2E0944UL
#define ZHIANN_ID_CELL_7_10    0x2E0945UL
#define ZHIANN_ID_CELL_11_14   0x2E0946UL
#define ZHIANN_ID_CELL_15_18   0x2E0947UL
#define ZHIANN_ID_CELL_23_24   0x2E094AUL
#define ZHIANN_ID_PACK_VOLT    0x2E0951UL
#define ZHIANN_ID_STATUS_FAST  0x401A100UL

// consider the BMS gone after 5s without a pack voltage frame
#define ZHIANN_TIMEOUT_US      5000000UL

AP_BattMonitor_ZhiannBMS::AP_BattMonitor_ZhiannBMS(AP_BattMonitor &mon,
        AP_BattMonitor::BattMonitor_State &mon_state,
        AP_BattMonitor_Params &params)
    : CANSensor("ZhiannBMS"),
      AP_BattMonitor_Backend(mon, mon_state, params)
{
    register_driver(AP_CAN::Protocol::ZhiannBMS);

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

void AP_BattMonitor_ZhiannBMS::handle_frame(AP_HAL::CANFrame &frame)
{
    if (!frame.isExtended() || frame.isRemoteTransmissionRequest() || frame.isErrorFrame()) {
        return;
    }
    const uint32_t id = frame.id & AP_HAL::CANFrame::MaskExtID;

    WITH_SEMAPHORE(_sem);

    switch (id) {
    case ZHIANN_ID_PACK_VOLT:
        if (frame.dlc >= 4) {
            _interim.voltage = le16toh_ptr(&frame.data[2]) * 0.01f;
            _interim.last_frame_us = AP_HAL::micros();
        }
        break;

    case ZHIANN_ID_TEMP_SOC:
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

    case ZHIANN_ID_STATUS_FAST:
        // coarse SOC, only used if the 0.1% frame is not being sent
        if (frame.dlc >= 2 && !_interim.soc_is_fine) {
            _interim.soc_pct = MIN(le16toh_ptr(&frame.data[0]), 100);
            _interim.soc_valid = true;
        }
        break;

    case ZHIANN_ID_CELL_1_2:
        if (frame.dlc >= 8) {
            store_cells(0, &frame.data[4], 2);
        }
        break;
    case ZHIANN_ID_CELL_3_6:
        if (frame.dlc >= 8) {
            store_cells(2, frame.data, 4);
        }
        break;
    case ZHIANN_ID_CELL_7_10:
        if (frame.dlc >= 8) {
            store_cells(6, frame.data, 4);
        }
        break;
    case ZHIANN_ID_CELL_11_14:
        if (frame.dlc >= 8) {
            store_cells(10, frame.data, 4);
        }
        break;
    case ZHIANN_ID_CELL_15_18:
        if (frame.dlc >= 8) {
            store_cells(14, frame.data, 4);
        }
        break;
    case ZHIANN_ID_CELL_19_22:
        if (frame.dlc >= 8) {
            store_cells(18, frame.data, 4);
        }
        break;
    case ZHIANN_ID_CELL_23_24:
        if (frame.dlc >= 4) {
            store_cells(22, frame.data, 2);
        }
        break;

    default:
        break;
    }
}

void AP_BattMonitor_ZhiannBMS::read()
{
    WITH_SEMAPHORE(_sem);

    const uint32_t tnow_us = AP_HAL::micros();
    if (_interim.last_frame_us == 0 ||
        (tnow_us - _interim.last_frame_us) > ZHIANN_TIMEOUT_US) {
        _state.healthy = false;
        return;
    }

    _state.healthy = true;
    _state.voltage = _interim.voltage;
    _state.last_time_micros = _interim.last_frame_us;
    _state.temperature = _interim.temperature;
    _state.temperature_time = _interim.temperature_time_ms;
    memcpy(_state.cell_voltages.cells, _interim.cells_mv,
           sizeof(_state.cell_voltages.cells));

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
