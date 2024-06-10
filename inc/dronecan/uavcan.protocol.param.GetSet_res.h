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
#include <uavcan.protocol.param.NumericValue.h>
#include <uavcan.protocol.param.Value.h>

#ifdef __cplusplus
extern "C"
{
#endif

#define UAVCAN_PROTOCOL_PARAM_GETSET_RESPONSE_MAX_SIZE 371
#define UAVCAN_PROTOCOL_PARAM_GETSET_RESPONSE_SIGNATURE (0xA7B622F939D1A4D5ULL)
#define UAVCAN_PROTOCOL_PARAM_GETSET_RESPONSE_ID 11

struct uavcan_protocol_param_GetSetResponse {
    struct uavcan_protocol_param_Value value;
    struct uavcan_protocol_param_Value default_value;
    struct uavcan_protocol_param_NumericValue max_value;
    struct uavcan_protocol_param_NumericValue min_value;
    struct { uint8_t len; uint8_t data[92]; }name;
};

uint32_t uavcan_protocol_param_GetSetResponse_encode(struct uavcan_protocol_param_GetSetResponse* msg, uint8_t* buffer
#if CANARD_ENABLE_TAO_OPTION
    , bool tao
#endif
);
bool uavcan_protocol_param_GetSetResponse_decode(const CanardRxTransfer* transfer, struct uavcan_protocol_param_GetSetResponse* msg);

#if defined(CANARD_DSDLC_INTERNAL)
static inline void _uavcan_protocol_param_GetSetResponse_encode(uint8_t* buffer, uint32_t* bit_ofs, struct uavcan_protocol_param_GetSetResponse* msg, bool tao);
static inline void _uavcan_protocol_param_GetSetResponse_decode(const CanardRxTransfer* transfer, uint32_t* bit_ofs, struct uavcan_protocol_param_GetSetResponse* msg, bool tao);
void _uavcan_protocol_param_GetSetResponse_encode(uint8_t* buffer, uint32_t* bit_ofs, struct uavcan_protocol_param_GetSetResponse* msg, bool tao) {
    (void)buffer;
    (void)bit_ofs;
    (void)msg;
    (void)tao;

    *bit_ofs += 5;
    _uavcan_protocol_param_Value_encode(buffer, bit_ofs, &msg->value, false);
    *bit_ofs += 5;
    _uavcan_protocol_param_Value_encode(buffer, bit_ofs, &msg->default_value, false);
    *bit_ofs += 6;
    _uavcan_protocol_param_NumericValue_encode(buffer, bit_ofs, &msg->max_value, false);
    *bit_ofs += 6;
    _uavcan_protocol_param_NumericValue_encode(buffer, bit_ofs, &msg->min_value, false);
    if (!tao) {
        canardEncodeScalar(buffer, *bit_ofs, 7, &msg->name.len);
        *bit_ofs += 7;
    }
    for (size_t i=0; i < msg->name.len; i++) {
        canardEncodeScalar(buffer, *bit_ofs, 8, &msg->name.data[i]);
        *bit_ofs += 8;
    }
}

void _uavcan_protocol_param_GetSetResponse_decode(const CanardRxTransfer* transfer, uint32_t* bit_ofs, struct uavcan_protocol_param_GetSetResponse* msg, bool tao) {
    (void)transfer;
    (void)bit_ofs;
    (void)msg;
    (void)tao;

    *bit_ofs += 5;

    _uavcan_protocol_param_Value_decode(transfer, bit_ofs, &msg->value, false);

    *bit_ofs += 5;

    _uavcan_protocol_param_Value_decode(transfer, bit_ofs, &msg->default_value, false);

    *bit_ofs += 6;

    _uavcan_protocol_param_NumericValue_decode(transfer, bit_ofs, &msg->max_value, false);

    *bit_ofs += 6;

    _uavcan_protocol_param_NumericValue_decode(transfer, bit_ofs, &msg->min_value, false);

    if (!tao) {
        canardDecodeScalar(transfer, *bit_ofs, 7, false, &msg->name.len);
        *bit_ofs += 7;
    } else {
        msg->name.len = ((transfer->payload_len*8)-*bit_ofs)/8;
    }

    for (size_t i=0; i < msg->name.len; i++) {
        canardDecodeScalar(transfer, *bit_ofs, 8, false, &msg->name.data[i]);
        *bit_ofs += 8;
    }

}
#endif
#ifdef CANARD_DSDLC_TEST_BUILD
struct uavcan_protocol_param_GetSetResponse sample_uavcan_protocol_param_GetSetResponse_msg(void);
#endif
#ifdef __cplusplus
} // extern "C"
#endif
