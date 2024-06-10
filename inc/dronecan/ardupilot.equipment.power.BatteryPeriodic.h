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

#ifdef __cplusplus
extern "C"
{
#endif

#define ARDUPILOT_EQUIPMENT_POWER_BATTERYPERIODIC_MAX_SIZE 125
#define ARDUPILOT_EQUIPMENT_POWER_BATTERYPERIODIC_SIGNATURE (0xF012494E97358D2ULL)
#define ARDUPILOT_EQUIPMENT_POWER_BATTERYPERIODIC_ID 20011

struct ardupilot_equipment_power_BatteryPeriodic {
    struct { uint8_t len; uint8_t data[50]; }name;
    struct { uint8_t len; uint8_t data[32]; }serial_number;
    struct { uint8_t len; uint8_t data[9]; }manufacture_date;
    float design_capacity;
    uint8_t cells_in_series;
    float nominal_voltage;
    float discharge_minimum_voltage;
    float charging_minimum_voltage;
    float charging_maximum_voltage;
    float charging_maximum_current;
    float discharge_maximum_current;
    float discharge_maximum_burst_current;
    float full_charge_capacity;
    uint16_t cycle_count;
    uint8_t state_of_health;
};

uint32_t ardupilot_equipment_power_BatteryPeriodic_encode(struct ardupilot_equipment_power_BatteryPeriodic* msg, uint8_t* buffer
#if CANARD_ENABLE_TAO_OPTION
    , bool tao
#endif
);
bool ardupilot_equipment_power_BatteryPeriodic_decode(const CanardRxTransfer* transfer, struct ardupilot_equipment_power_BatteryPeriodic* msg);

#if defined(CANARD_DSDLC_INTERNAL)
static inline void _ardupilot_equipment_power_BatteryPeriodic_encode(uint8_t* buffer, uint32_t* bit_ofs, struct ardupilot_equipment_power_BatteryPeriodic* msg, bool tao);
static inline void _ardupilot_equipment_power_BatteryPeriodic_decode(const CanardRxTransfer* transfer, uint32_t* bit_ofs, struct ardupilot_equipment_power_BatteryPeriodic* msg, bool tao);
void _ardupilot_equipment_power_BatteryPeriodic_encode(uint8_t* buffer, uint32_t* bit_ofs, struct ardupilot_equipment_power_BatteryPeriodic* msg, bool tao) {
    (void)buffer;
    (void)bit_ofs;
    (void)msg;
    (void)tao;

    canardEncodeScalar(buffer, *bit_ofs, 6, &msg->name.len);
    *bit_ofs += 6;
    for (size_t i=0; i < msg->name.len; i++) {
        canardEncodeScalar(buffer, *bit_ofs, 8, &msg->name.data[i]);
        *bit_ofs += 8;
    }
    canardEncodeScalar(buffer, *bit_ofs, 6, &msg->serial_number.len);
    *bit_ofs += 6;
    for (size_t i=0; i < msg->serial_number.len; i++) {
        canardEncodeScalar(buffer, *bit_ofs, 8, &msg->serial_number.data[i]);
        *bit_ofs += 8;
    }
    canardEncodeScalar(buffer, *bit_ofs, 4, &msg->manufacture_date.len);
    *bit_ofs += 4;
    for (size_t i=0; i < msg->manufacture_date.len; i++) {
        canardEncodeScalar(buffer, *bit_ofs, 8, &msg->manufacture_date.data[i]);
        *bit_ofs += 8;
    }
    canardEncodeScalar(buffer, *bit_ofs, 32, &msg->design_capacity);
    *bit_ofs += 32;
    canardEncodeScalar(buffer, *bit_ofs, 8, &msg->cells_in_series);
    *bit_ofs += 8;
    {
        uint16_t float16_val = canardConvertNativeFloatToFloat16(msg->nominal_voltage);
        canardEncodeScalar(buffer, *bit_ofs, 16, &float16_val);
    }
    *bit_ofs += 16;
    {
        uint16_t float16_val = canardConvertNativeFloatToFloat16(msg->discharge_minimum_voltage);
        canardEncodeScalar(buffer, *bit_ofs, 16, &float16_val);
    }
    *bit_ofs += 16;
    {
        uint16_t float16_val = canardConvertNativeFloatToFloat16(msg->charging_minimum_voltage);
        canardEncodeScalar(buffer, *bit_ofs, 16, &float16_val);
    }
    *bit_ofs += 16;
    {
        uint16_t float16_val = canardConvertNativeFloatToFloat16(msg->charging_maximum_voltage);
        canardEncodeScalar(buffer, *bit_ofs, 16, &float16_val);
    }
    *bit_ofs += 16;
    canardEncodeScalar(buffer, *bit_ofs, 32, &msg->charging_maximum_current);
    *bit_ofs += 32;
    canardEncodeScalar(buffer, *bit_ofs, 32, &msg->discharge_maximum_current);
    *bit_ofs += 32;
    canardEncodeScalar(buffer, *bit_ofs, 32, &msg->discharge_maximum_burst_current);
    *bit_ofs += 32;
    canardEncodeScalar(buffer, *bit_ofs, 32, &msg->full_charge_capacity);
    *bit_ofs += 32;
    canardEncodeScalar(buffer, *bit_ofs, 16, &msg->cycle_count);
    *bit_ofs += 16;
    canardEncodeScalar(buffer, *bit_ofs, 8, &msg->state_of_health);
    *bit_ofs += 8;
}

void _ardupilot_equipment_power_BatteryPeriodic_decode(const CanardRxTransfer* transfer, uint32_t* bit_ofs, struct ardupilot_equipment_power_BatteryPeriodic* msg, bool tao) {
    (void)transfer;
    (void)bit_ofs;
    (void)msg;
    (void)tao;

    canardDecodeScalar(transfer, *bit_ofs, 6, false, &msg->name.len);
    *bit_ofs += 6;
    for (size_t i=0; i < msg->name.len; i++) {
        canardDecodeScalar(transfer, *bit_ofs, 8, false, &msg->name.data[i]);
        *bit_ofs += 8;
    }

    canardDecodeScalar(transfer, *bit_ofs, 6, false, &msg->serial_number.len);
    *bit_ofs += 6;
    for (size_t i=0; i < msg->serial_number.len; i++) {
        canardDecodeScalar(transfer, *bit_ofs, 8, false, &msg->serial_number.data[i]);
        *bit_ofs += 8;
    }

    canardDecodeScalar(transfer, *bit_ofs, 4, false, &msg->manufacture_date.len);
    *bit_ofs += 4;
    for (size_t i=0; i < msg->manufacture_date.len; i++) {
        canardDecodeScalar(transfer, *bit_ofs, 8, false, &msg->manufacture_date.data[i]);
        *bit_ofs += 8;
    }

    canardDecodeScalar(transfer, *bit_ofs, 32, true, &msg->design_capacity);
    *bit_ofs += 32;

    canardDecodeScalar(transfer, *bit_ofs, 8, false, &msg->cells_in_series);
    *bit_ofs += 8;

    {
        uint16_t float16_val;
        canardDecodeScalar(transfer, *bit_ofs, 16, true, &float16_val);
        msg->nominal_voltage = canardConvertFloat16ToNativeFloat(float16_val);
    }
    *bit_ofs += 16;

    {
        uint16_t float16_val;
        canardDecodeScalar(transfer, *bit_ofs, 16, true, &float16_val);
        msg->discharge_minimum_voltage = canardConvertFloat16ToNativeFloat(float16_val);
    }
    *bit_ofs += 16;

    {
        uint16_t float16_val;
        canardDecodeScalar(transfer, *bit_ofs, 16, true, &float16_val);
        msg->charging_minimum_voltage = canardConvertFloat16ToNativeFloat(float16_val);
    }
    *bit_ofs += 16;

    {
        uint16_t float16_val;
        canardDecodeScalar(transfer, *bit_ofs, 16, true, &float16_val);
        msg->charging_maximum_voltage = canardConvertFloat16ToNativeFloat(float16_val);
    }
    *bit_ofs += 16;

    canardDecodeScalar(transfer, *bit_ofs, 32, true, &msg->charging_maximum_current);
    *bit_ofs += 32;

    canardDecodeScalar(transfer, *bit_ofs, 32, true, &msg->discharge_maximum_current);
    *bit_ofs += 32;

    canardDecodeScalar(transfer, *bit_ofs, 32, true, &msg->discharge_maximum_burst_current);
    *bit_ofs += 32;

    canardDecodeScalar(transfer, *bit_ofs, 32, true, &msg->full_charge_capacity);
    *bit_ofs += 32;

    canardDecodeScalar(transfer, *bit_ofs, 16, false, &msg->cycle_count);
    *bit_ofs += 16;

    canardDecodeScalar(transfer, *bit_ofs, 8, false, &msg->state_of_health);
    *bit_ofs += 8;

}
#endif
#ifdef CANARD_DSDLC_TEST_BUILD
struct ardupilot_equipment_power_BatteryPeriodic sample_ardupilot_equipment_power_BatteryPeriodic_msg(void);
#endif
#ifdef __cplusplus
} // extern "C"
#endif
