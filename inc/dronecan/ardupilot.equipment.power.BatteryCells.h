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

#define ARDUPILOT_EQUIPMENT_POWER_BATTERYCELLS_MAX_SIZE 51
#define ARDUPILOT_EQUIPMENT_POWER_BATTERYCELLS_SIGNATURE (0x5C8B1ABD15890EA4ULL)
#define ARDUPILOT_EQUIPMENT_POWER_BATTERYCELLS_ID 20012

struct ardupilot_equipment_power_BatteryCells {
    struct { uint8_t len; float data[24]; }voltages;
    uint16_t index;
};

uint32_t ardupilot_equipment_power_BatteryCells_encode(struct ardupilot_equipment_power_BatteryCells* msg, uint8_t* buffer
#if CANARD_ENABLE_TAO_OPTION
    , bool tao
#endif
);
bool ardupilot_equipment_power_BatteryCells_decode(const CanardRxTransfer* transfer, struct ardupilot_equipment_power_BatteryCells* msg);

#if defined(CANARD_DSDLC_INTERNAL)
static inline void _ardupilot_equipment_power_BatteryCells_encode(uint8_t* buffer, uint32_t* bit_ofs, struct ardupilot_equipment_power_BatteryCells* msg, bool tao);
static inline void _ardupilot_equipment_power_BatteryCells_decode(const CanardRxTransfer* transfer, uint32_t* bit_ofs, struct ardupilot_equipment_power_BatteryCells* msg, bool tao);
void _ardupilot_equipment_power_BatteryCells_encode(uint8_t* buffer, uint32_t* bit_ofs, struct ardupilot_equipment_power_BatteryCells* msg, bool tao) {
    (void)buffer;
    (void)bit_ofs;
    (void)msg;
    (void)tao;

    canardEncodeScalar(buffer, *bit_ofs, 5, &msg->voltages.len);
    *bit_ofs += 5;
    for (size_t i=0; i < msg->voltages.len; i++) {
        {
            uint16_t float16_val = canardConvertNativeFloatToFloat16(msg->voltages.data[i]);
            canardEncodeScalar(buffer, *bit_ofs, 16, &float16_val);
        }
        *bit_ofs += 16;
    }
    canardEncodeScalar(buffer, *bit_ofs, 16, &msg->index);
    *bit_ofs += 16;
}

void _ardupilot_equipment_power_BatteryCells_decode(const CanardRxTransfer* transfer, uint32_t* bit_ofs, struct ardupilot_equipment_power_BatteryCells* msg, bool tao) {
    (void)transfer;
    (void)bit_ofs;
    (void)msg;
    (void)tao;

    canardDecodeScalar(transfer, *bit_ofs, 5, false, &msg->voltages.len);
    *bit_ofs += 5;
    for (size_t i=0; i < msg->voltages.len; i++) {
        {
            uint16_t float16_val;
            canardDecodeScalar(transfer, *bit_ofs, 16, true, &float16_val);
            msg->voltages.data[i] = canardConvertFloat16ToNativeFloat(float16_val);
        }
        *bit_ofs += 16;
    }

    canardDecodeScalar(transfer, *bit_ofs, 16, false, &msg->index);
    *bit_ofs += 16;

}
#endif
#ifdef CANARD_DSDLC_TEST_BUILD
struct ardupilot_equipment_power_BatteryCells sample_ardupilot_equipment_power_BatteryCells_msg(void);
#endif
#ifdef __cplusplus
} // extern "C"
#endif
