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

#define UAVCAN_EQUIPMENT_POWER_BATTERYINFO_MAX_SIZE 55
#define UAVCAN_EQUIPMENT_POWER_BATTERYINFO_SIGNATURE (0x249C26548A711966ULL)
#define UAVCAN_EQUIPMENT_POWER_BATTERYINFO_ID 1092

#define UAVCAN_EQUIPMENT_POWER_BATTERYINFO_STATUS_FLAG_IN_USE 1
#define UAVCAN_EQUIPMENT_POWER_BATTERYINFO_STATUS_FLAG_CHARGING 2
#define UAVCAN_EQUIPMENT_POWER_BATTERYINFO_STATUS_FLAG_CHARGED 4
#define UAVCAN_EQUIPMENT_POWER_BATTERYINFO_STATUS_FLAG_TEMP_HOT 8
#define UAVCAN_EQUIPMENT_POWER_BATTERYINFO_STATUS_FLAG_TEMP_COLD 16
#define UAVCAN_EQUIPMENT_POWER_BATTERYINFO_STATUS_FLAG_OVERLOAD 32
#define UAVCAN_EQUIPMENT_POWER_BATTERYINFO_STATUS_FLAG_BAD_BATTERY 64
#define UAVCAN_EQUIPMENT_POWER_BATTERYINFO_STATUS_FLAG_NEED_SERVICE 128
#define UAVCAN_EQUIPMENT_POWER_BATTERYINFO_STATUS_FLAG_BMS_ERROR 256
#define UAVCAN_EQUIPMENT_POWER_BATTERYINFO_STATUS_FLAG_RESERVED_A 512
#define UAVCAN_EQUIPMENT_POWER_BATTERYINFO_STATUS_FLAG_RESERVED_B 1024
#define UAVCAN_EQUIPMENT_POWER_BATTERYINFO_STATE_OF_HEALTH_UNKNOWN 127

struct uavcan_equipment_power_BatteryInfo {
    float temperature;
    float voltage;
    float current;
    float average_power_10sec;
    float remaining_capacity_wh;
    float full_charge_capacity_wh;
    float hours_to_full_charge;
    uint16_t status_flags;
    uint8_t state_of_health_pct;
    uint8_t state_of_charge_pct;
    uint8_t state_of_charge_pct_stdev;
    uint8_t battery_id;
    uint32_t model_instance_id;
    struct { uint8_t len; uint8_t data[31]; }model_name;
};

uint32_t uavcan_equipment_power_BatteryInfo_encode(struct uavcan_equipment_power_BatteryInfo* msg, uint8_t* buffer
#if CANARD_ENABLE_TAO_OPTION
    , bool tao
#endif
);
bool uavcan_equipment_power_BatteryInfo_decode(const CanardRxTransfer* transfer, struct uavcan_equipment_power_BatteryInfo* msg);

#if defined(CANARD_DSDLC_INTERNAL)
static inline void _uavcan_equipment_power_BatteryInfo_encode(uint8_t* buffer, uint32_t* bit_ofs, struct uavcan_equipment_power_BatteryInfo* msg, bool tao);
static inline void _uavcan_equipment_power_BatteryInfo_decode(const CanardRxTransfer* transfer, uint32_t* bit_ofs, struct uavcan_equipment_power_BatteryInfo* msg, bool tao);
void _uavcan_equipment_power_BatteryInfo_encode(uint8_t* buffer, uint32_t* bit_ofs, struct uavcan_equipment_power_BatteryInfo* msg, bool tao) {
    (void)buffer;
    (void)bit_ofs;
    (void)msg;
    (void)tao;

    {
        uint16_t float16_val = canardConvertNativeFloatToFloat16(msg->temperature);
        canardEncodeScalar(buffer, *bit_ofs, 16, &float16_val);
    }
    *bit_ofs += 16;
    {
        uint16_t float16_val = canardConvertNativeFloatToFloat16(msg->voltage);
        canardEncodeScalar(buffer, *bit_ofs, 16, &float16_val);
    }
    *bit_ofs += 16;
    {
        uint16_t float16_val = canardConvertNativeFloatToFloat16(msg->current);
        canardEncodeScalar(buffer, *bit_ofs, 16, &float16_val);
    }
    *bit_ofs += 16;
    {
        uint16_t float16_val = canardConvertNativeFloatToFloat16(msg->average_power_10sec);
        canardEncodeScalar(buffer, *bit_ofs, 16, &float16_val);
    }
    *bit_ofs += 16;
    {
        uint16_t float16_val = canardConvertNativeFloatToFloat16(msg->remaining_capacity_wh);
        canardEncodeScalar(buffer, *bit_ofs, 16, &float16_val);
    }
    *bit_ofs += 16;
    {
        uint16_t float16_val = canardConvertNativeFloatToFloat16(msg->full_charge_capacity_wh);
        canardEncodeScalar(buffer, *bit_ofs, 16, &float16_val);
    }
    *bit_ofs += 16;
    {
        uint16_t float16_val = canardConvertNativeFloatToFloat16(msg->hours_to_full_charge);
        canardEncodeScalar(buffer, *bit_ofs, 16, &float16_val);
    }
    *bit_ofs += 16;
    canardEncodeScalar(buffer, *bit_ofs, 11, &msg->status_flags);
    *bit_ofs += 11;
    canardEncodeScalar(buffer, *bit_ofs, 7, &msg->state_of_health_pct);
    *bit_ofs += 7;
    canardEncodeScalar(buffer, *bit_ofs, 7, &msg->state_of_charge_pct);
    *bit_ofs += 7;
    canardEncodeScalar(buffer, *bit_ofs, 7, &msg->state_of_charge_pct_stdev);
    *bit_ofs += 7;
    canardEncodeScalar(buffer, *bit_ofs, 8, &msg->battery_id);
    *bit_ofs += 8;
    canardEncodeScalar(buffer, *bit_ofs, 32, &msg->model_instance_id);
    *bit_ofs += 32;
    if (!tao) {
        canardEncodeScalar(buffer, *bit_ofs, 5, &msg->model_name.len);
        *bit_ofs += 5;
    }
    for (size_t i=0; i < msg->model_name.len; i++) {
        canardEncodeScalar(buffer, *bit_ofs, 8, &msg->model_name.data[i]);
        *bit_ofs += 8;
    }
}

void _uavcan_equipment_power_BatteryInfo_decode(const CanardRxTransfer* transfer, uint32_t* bit_ofs, struct uavcan_equipment_power_BatteryInfo* msg, bool tao) {
    (void)transfer;
    (void)bit_ofs;
    (void)msg;
    (void)tao;

    {
        uint16_t float16_val;
        canardDecodeScalar(transfer, *bit_ofs, 16, true, &float16_val);
        msg->temperature = canardConvertFloat16ToNativeFloat(float16_val);
    }
    *bit_ofs += 16;

    {
        uint16_t float16_val;
        canardDecodeScalar(transfer, *bit_ofs, 16, true, &float16_val);
        msg->voltage = canardConvertFloat16ToNativeFloat(float16_val);
    }
    *bit_ofs += 16;

    {
        uint16_t float16_val;
        canardDecodeScalar(transfer, *bit_ofs, 16, true, &float16_val);
        msg->current = canardConvertFloat16ToNativeFloat(float16_val);
    }
    *bit_ofs += 16;

    {
        uint16_t float16_val;
        canardDecodeScalar(transfer, *bit_ofs, 16, true, &float16_val);
        msg->average_power_10sec = canardConvertFloat16ToNativeFloat(float16_val);
    }
    *bit_ofs += 16;

    {
        uint16_t float16_val;
        canardDecodeScalar(transfer, *bit_ofs, 16, true, &float16_val);
        msg->remaining_capacity_wh = canardConvertFloat16ToNativeFloat(float16_val);
    }
    *bit_ofs += 16;

    {
        uint16_t float16_val;
        canardDecodeScalar(transfer, *bit_ofs, 16, true, &float16_val);
        msg->full_charge_capacity_wh = canardConvertFloat16ToNativeFloat(float16_val);
    }
    *bit_ofs += 16;

    {
        uint16_t float16_val;
        canardDecodeScalar(transfer, *bit_ofs, 16, true, &float16_val);
        msg->hours_to_full_charge = canardConvertFloat16ToNativeFloat(float16_val);
    }
    *bit_ofs += 16;

    canardDecodeScalar(transfer, *bit_ofs, 11, false, &msg->status_flags);
    *bit_ofs += 11;

    canardDecodeScalar(transfer, *bit_ofs, 7, false, &msg->state_of_health_pct);
    *bit_ofs += 7;

    canardDecodeScalar(transfer, *bit_ofs, 7, false, &msg->state_of_charge_pct);
    *bit_ofs += 7;

    canardDecodeScalar(transfer, *bit_ofs, 7, false, &msg->state_of_charge_pct_stdev);
    *bit_ofs += 7;

    canardDecodeScalar(transfer, *bit_ofs, 8, false, &msg->battery_id);
    *bit_ofs += 8;

    canardDecodeScalar(transfer, *bit_ofs, 32, false, &msg->model_instance_id);
    *bit_ofs += 32;

    if (!tao) {
        canardDecodeScalar(transfer, *bit_ofs, 5, false, &msg->model_name.len);
        *bit_ofs += 5;
    } else {
        msg->model_name.len = ((transfer->payload_len*8)-*bit_ofs)/8;
    }

    for (size_t i=0; i < msg->model_name.len; i++) {
        canardDecodeScalar(transfer, *bit_ofs, 8, false, &msg->model_name.data[i]);
        *bit_ofs += 8;
    }

}
#endif
#ifdef CANARD_DSDLC_TEST_BUILD
struct uavcan_equipment_power_BatteryInfo sample_uavcan_equipment_power_BatteryInfo_msg(void);
#endif
#ifdef __cplusplus
} // extern "C"
#endif
