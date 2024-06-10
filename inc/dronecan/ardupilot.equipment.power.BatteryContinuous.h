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

#define ARDUPILOT_EQUIPMENT_POWER_BATTERYCONTINUOUS_MAX_SIZE 25
#define ARDUPILOT_EQUIPMENT_POWER_BATTERYCONTINUOUS_SIGNATURE (0x756B561340D5E4AEULL)
#define ARDUPILOT_EQUIPMENT_POWER_BATTERYCONTINUOUS_ID 20010

#define ARDUPILOT_EQUIPMENT_POWER_BATTERYCONTINUOUS_STATUS_FLAG_READY_TO_USE 1
#define ARDUPILOT_EQUIPMENT_POWER_BATTERYCONTINUOUS_STATUS_FLAG_CHARGING 2
#define ARDUPILOT_EQUIPMENT_POWER_BATTERYCONTINUOUS_STATUS_FLAG_CELL_BALANCING 4
#define ARDUPILOT_EQUIPMENT_POWER_BATTERYCONTINUOUS_STATUS_FLAG_FAULT_CELL_IMBALANCE 8
#define ARDUPILOT_EQUIPMENT_POWER_BATTERYCONTINUOUS_STATUS_FLAG_AUTO_DISCHARGING 16
#define ARDUPILOT_EQUIPMENT_POWER_BATTERYCONTINUOUS_STATUS_FLAG_REQUIRES_SERVICE 32
#define ARDUPILOT_EQUIPMENT_POWER_BATTERYCONTINUOUS_STATUS_FLAG_BAD_BATTERY 64
#define ARDUPILOT_EQUIPMENT_POWER_BATTERYCONTINUOUS_STATUS_FLAG_PROTECTIONS_ENABLED 128
#define ARDUPILOT_EQUIPMENT_POWER_BATTERYCONTINUOUS_STATUS_FLAG_FAULT_PROTECTION_SYSTEM 256
#define ARDUPILOT_EQUIPMENT_POWER_BATTERYCONTINUOUS_STATUS_FLAG_FAULT_OVER_VOLT 512
#define ARDUPILOT_EQUIPMENT_POWER_BATTERYCONTINUOUS_STATUS_FLAG_FAULT_UNDER_VOLT 1024
#define ARDUPILOT_EQUIPMENT_POWER_BATTERYCONTINUOUS_STATUS_FLAG_FAULT_OVER_TEMP 2048
#define ARDUPILOT_EQUIPMENT_POWER_BATTERYCONTINUOUS_STATUS_FLAG_FAULT_UNDER_TEMP 4096
#define ARDUPILOT_EQUIPMENT_POWER_BATTERYCONTINUOUS_STATUS_FLAG_FAULT_OVER_CURRENT 8192
#define ARDUPILOT_EQUIPMENT_POWER_BATTERYCONTINUOUS_STATUS_FLAG_FAULT_SHORT_CIRCUIT 16384
#define ARDUPILOT_EQUIPMENT_POWER_BATTERYCONTINUOUS_STATUS_FLAG_FAULT_INCOMPATIBLE_VOLTAGE 32768
#define ARDUPILOT_EQUIPMENT_POWER_BATTERYCONTINUOUS_STATUS_FLAG_FAULT_INCOMPATIBLE_FIRMWARE 65536
#define ARDUPILOT_EQUIPMENT_POWER_BATTERYCONTINUOUS_STATUS_FLAG_FAULT_INCOMPATIBLE_CELLS_CONFIGURATION 131072
#define ARDUPILOT_EQUIPMENT_POWER_BATTERYCONTINUOUS_STATUS_FLAG_CAPACITY_RELATIVE_TO_FULL 262144

struct ardupilot_equipment_power_BatteryContinuous {
    float temperature_cells;
    float temperature_pcb;
    float temperature_other;
    float current;
    float voltage;
    float state_of_charge;
    uint8_t slot_id;
    float capacity_consumed;
    uint32_t status_flags;
};

uint32_t ardupilot_equipment_power_BatteryContinuous_encode(struct ardupilot_equipment_power_BatteryContinuous* msg, uint8_t* buffer
#if CANARD_ENABLE_TAO_OPTION
    , bool tao
#endif
);
bool ardupilot_equipment_power_BatteryContinuous_decode(const CanardRxTransfer* transfer, struct ardupilot_equipment_power_BatteryContinuous* msg);

#if defined(CANARD_DSDLC_INTERNAL)
static inline void _ardupilot_equipment_power_BatteryContinuous_encode(uint8_t* buffer, uint32_t* bit_ofs, struct ardupilot_equipment_power_BatteryContinuous* msg, bool tao);
static inline void _ardupilot_equipment_power_BatteryContinuous_decode(const CanardRxTransfer* transfer, uint32_t* bit_ofs, struct ardupilot_equipment_power_BatteryContinuous* msg, bool tao);
void _ardupilot_equipment_power_BatteryContinuous_encode(uint8_t* buffer, uint32_t* bit_ofs, struct ardupilot_equipment_power_BatteryContinuous* msg, bool tao) {
    (void)buffer;
    (void)bit_ofs;
    (void)msg;
    (void)tao;

    {
        uint16_t float16_val = canardConvertNativeFloatToFloat16(msg->temperature_cells);
        canardEncodeScalar(buffer, *bit_ofs, 16, &float16_val);
    }
    *bit_ofs += 16;
    {
        uint16_t float16_val = canardConvertNativeFloatToFloat16(msg->temperature_pcb);
        canardEncodeScalar(buffer, *bit_ofs, 16, &float16_val);
    }
    *bit_ofs += 16;
    {
        uint16_t float16_val = canardConvertNativeFloatToFloat16(msg->temperature_other);
        canardEncodeScalar(buffer, *bit_ofs, 16, &float16_val);
    }
    *bit_ofs += 16;
    canardEncodeScalar(buffer, *bit_ofs, 32, &msg->current);
    *bit_ofs += 32;
    canardEncodeScalar(buffer, *bit_ofs, 32, &msg->voltage);
    *bit_ofs += 32;
    {
        uint16_t float16_val = canardConvertNativeFloatToFloat16(msg->state_of_charge);
        canardEncodeScalar(buffer, *bit_ofs, 16, &float16_val);
    }
    *bit_ofs += 16;
    canardEncodeScalar(buffer, *bit_ofs, 8, &msg->slot_id);
    *bit_ofs += 8;
    canardEncodeScalar(buffer, *bit_ofs, 32, &msg->capacity_consumed);
    *bit_ofs += 32;
    canardEncodeScalar(buffer, *bit_ofs, 32, &msg->status_flags);
    *bit_ofs += 32;
}

void _ardupilot_equipment_power_BatteryContinuous_decode(const CanardRxTransfer* transfer, uint32_t* bit_ofs, struct ardupilot_equipment_power_BatteryContinuous* msg, bool tao) {
    (void)transfer;
    (void)bit_ofs;
    (void)msg;
    (void)tao;

    {
        uint16_t float16_val;
        canardDecodeScalar(transfer, *bit_ofs, 16, true, &float16_val);
        msg->temperature_cells = canardConvertFloat16ToNativeFloat(float16_val);
    }
    *bit_ofs += 16;

    {
        uint16_t float16_val;
        canardDecodeScalar(transfer, *bit_ofs, 16, true, &float16_val);
        msg->temperature_pcb = canardConvertFloat16ToNativeFloat(float16_val);
    }
    *bit_ofs += 16;

    {
        uint16_t float16_val;
        canardDecodeScalar(transfer, *bit_ofs, 16, true, &float16_val);
        msg->temperature_other = canardConvertFloat16ToNativeFloat(float16_val);
    }
    *bit_ofs += 16;

    canardDecodeScalar(transfer, *bit_ofs, 32, true, &msg->current);
    *bit_ofs += 32;

    canardDecodeScalar(transfer, *bit_ofs, 32, true, &msg->voltage);
    *bit_ofs += 32;

    {
        uint16_t float16_val;
        canardDecodeScalar(transfer, *bit_ofs, 16, true, &float16_val);
        msg->state_of_charge = canardConvertFloat16ToNativeFloat(float16_val);
    }
    *bit_ofs += 16;

    canardDecodeScalar(transfer, *bit_ofs, 8, false, &msg->slot_id);
    *bit_ofs += 8;

    canardDecodeScalar(transfer, *bit_ofs, 32, true, &msg->capacity_consumed);
    *bit_ofs += 32;

    canardDecodeScalar(transfer, *bit_ofs, 32, false, &msg->status_flags);
    *bit_ofs += 32;

}
#endif
#ifdef CANARD_DSDLC_TEST_BUILD
struct ardupilot_equipment_power_BatteryContinuous sample_ardupilot_equipment_power_BatteryContinuous_msg(void);
#endif
#ifdef __cplusplus
} // extern "C"
#endif
