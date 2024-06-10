/* The MIT License (MIT)
 * 
 * Copyright (c) 2014-2015 Pavel Kirienko
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#pragma once
#include <stdbool.h>
#include <stdint.h>
#include <canard.h>
#include <uavcan.Timestamp.h>

#ifdef __cplusplus
extern "C"
{
#endif

#define ARDUPILOT_EQUIPMENT_POWER_BATTERYINFOAUX_MAX_SIZE 528
#define ARDUPILOT_EQUIPMENT_POWER_BATTERYINFOAUX_SIGNATURE (0x7D7F49FC75484882ULL)
#define ARDUPILOT_EQUIPMENT_POWER_BATTERYINFOAUX_ID 20004

struct ardupilot_equipment_power_BatteryInfoAux {
    struct uavcan_Timestamp timestamp;
    struct { uint8_t len; float data[255]; }voltage_cell;
    uint16_t cycle_count;
    uint16_t over_discharge_count;
    float max_current;
    float nominal_voltage;
    bool is_powering_off;
    uint8_t battery_id;
};

uint32_t ardupilot_equipment_power_BatteryInfoAux_encode(struct ardupilot_equipment_power_BatteryInfoAux* msg, uint8_t* buffer
#if CANARD_ENABLE_TAO_OPTION
    , bool tao
#endif
);
bool ardupilot_equipment_power_BatteryInfoAux_decode(const CanardRxTransfer* transfer, struct ardupilot_equipment_power_BatteryInfoAux* msg);

#if defined(CANARD_DSDLC_INTERNAL)
static inline void _ardupilot_equipment_power_BatteryInfoAux_encode(uint8_t* buffer, uint32_t* bit_ofs, struct ardupilot_equipment_power_BatteryInfoAux* msg, bool tao);
static inline void _ardupilot_equipment_power_BatteryInfoAux_decode(const CanardRxTransfer* transfer, uint32_t* bit_ofs, struct ardupilot_equipment_power_BatteryInfoAux* msg, bool tao);
void _ardupilot_equipment_power_BatteryInfoAux_encode(uint8_t* buffer, uint32_t* bit_ofs, struct ardupilot_equipment_power_BatteryInfoAux* msg, bool tao) {
    (void)buffer;
    (void)bit_ofs;
    (void)msg;
    (void)tao;

    _uavcan_Timestamp_encode(buffer, bit_ofs, &msg->timestamp, false);
    canardEncodeScalar(buffer, *bit_ofs, 8, &msg->voltage_cell.len);
    *bit_ofs += 8;
    for (size_t i=0; i < msg->voltage_cell.len; i++) {
        {
            uint16_t float16_val = canardConvertNativeFloatToFloat16(msg->voltage_cell.data[i]);
            canardEncodeScalar(buffer, *bit_ofs, 16, &float16_val);
        }
        *bit_ofs += 16;
    }
    canardEncodeScalar(buffer, *bit_ofs, 16, &msg->cycle_count);
    *bit_ofs += 16;
    canardEncodeScalar(buffer, *bit_ofs, 16, &msg->over_discharge_count);
    *bit_ofs += 16;
    {
        uint16_t float16_val = canardConvertNativeFloatToFloat16(msg->max_current);
        canardEncodeScalar(buffer, *bit_ofs, 16, &float16_val);
    }
    *bit_ofs += 16;
    {
        uint16_t float16_val = canardConvertNativeFloatToFloat16(msg->nominal_voltage);
        canardEncodeScalar(buffer, *bit_ofs, 16, &float16_val);
    }
    *bit_ofs += 16;
    canardEncodeScalar(buffer, *bit_ofs, 1, &msg->is_powering_off);
    *bit_ofs += 1;
    canardEncodeScalar(buffer, *bit_ofs, 8, &msg->battery_id);
    *bit_ofs += 8;
}

void _ardupilot_equipment_power_BatteryInfoAux_decode(const CanardRxTransfer* transfer, uint32_t* bit_ofs, struct ardupilot_equipment_power_BatteryInfoAux* msg, bool tao) {
    (void)transfer;
    (void)bit_ofs;
    (void)msg;
    (void)tao;

    _uavcan_Timestamp_decode(transfer, bit_ofs, &msg->timestamp, false);

    canardDecodeScalar(transfer, *bit_ofs, 8, false, &msg->voltage_cell.len);
    *bit_ofs += 8;
    for (size_t i=0; i < msg->voltage_cell.len; i++) {
        {
            uint16_t float16_val;
            canardDecodeScalar(transfer, *bit_ofs, 16, true, &float16_val);
            msg->voltage_cell.data[i] = canardConvertFloat16ToNativeFloat(float16_val);
        }
        *bit_ofs += 16;
    }

    canardDecodeScalar(transfer, *bit_ofs, 16, false, &msg->cycle_count);
    *bit_ofs += 16;

    canardDecodeScalar(transfer, *bit_ofs, 16, false, &msg->over_discharge_count);
    *bit_ofs += 16;

    {
        uint16_t float16_val;
        canardDecodeScalar(transfer, *bit_ofs, 16, true, &float16_val);
        msg->max_current = canardConvertFloat16ToNativeFloat(float16_val);
    }
    *bit_ofs += 16;

    {
        uint16_t float16_val;
        canardDecodeScalar(transfer, *bit_ofs, 16, true, &float16_val);
        msg->nominal_voltage = canardConvertFloat16ToNativeFloat(float16_val);
    }
    *bit_ofs += 16;

    canardDecodeScalar(transfer, *bit_ofs, 1, false, &msg->is_powering_off);
    *bit_ofs += 1;

    canardDecodeScalar(transfer, *bit_ofs, 8, false, &msg->battery_id);
    *bit_ofs += 8;

}
#endif
#ifdef CANARD_DSDLC_TEST_BUILD
struct ardupilot_equipment_power_BatteryInfoAux sample_ardupilot_equipment_power_BatteryInfoAux_msg(void);
#endif
#ifdef __cplusplus
} // extern "C"
#endif
