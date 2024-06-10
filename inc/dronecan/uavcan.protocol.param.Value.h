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
#include <uavcan.protocol.param.Empty.h>

#ifdef __cplusplus
extern "C"
{
#endif

#define UAVCAN_PROTOCOL_PARAM_VALUE_MAX_SIZE 130
#define UAVCAN_PROTOCOL_PARAM_VALUE_SIGNATURE (0x29F14BF484727267ULL)

enum uavcan_protocol_param_Value_type_t {
    UAVCAN_PROTOCOL_PARAM_VALUE_EMPTY,
    UAVCAN_PROTOCOL_PARAM_VALUE_INTEGER_VALUE,
    UAVCAN_PROTOCOL_PARAM_VALUE_REAL_VALUE,
    UAVCAN_PROTOCOL_PARAM_VALUE_BOOLEAN_VALUE,
    UAVCAN_PROTOCOL_PARAM_VALUE_STRING_VALUE,
};

struct uavcan_protocol_param_Value {
    enum uavcan_protocol_param_Value_type_t union_tag;
    union {
        struct uavcan_protocol_param_Empty empty;
        int64_t integer_value;
        float real_value;
        uint8_t boolean_value;
        struct { uint8_t len; uint8_t data[128]; }string_value;
    };
};

uint32_t uavcan_protocol_param_Value_encode(struct uavcan_protocol_param_Value* msg, uint8_t* buffer
#if CANARD_ENABLE_TAO_OPTION
    , bool tao
#endif
);
bool uavcan_protocol_param_Value_decode(const CanardRxTransfer* transfer, struct uavcan_protocol_param_Value* msg);

#if defined(CANARD_DSDLC_INTERNAL)
static inline void _uavcan_protocol_param_Value_encode(uint8_t* buffer, uint32_t* bit_ofs, struct uavcan_protocol_param_Value* msg, bool tao);
static inline void _uavcan_protocol_param_Value_decode(const CanardRxTransfer* transfer, uint32_t* bit_ofs, struct uavcan_protocol_param_Value* msg, bool tao);
void _uavcan_protocol_param_Value_encode(uint8_t* buffer, uint32_t* bit_ofs, struct uavcan_protocol_param_Value* msg, bool tao) {
    (void)buffer;
    (void)bit_ofs;
    (void)msg;
    (void)tao;

    uint8_t union_tag = msg->union_tag;
    canardEncodeScalar(buffer, *bit_ofs, 3, &union_tag);
    *bit_ofs += 3;

    switch(msg->union_tag) {
        case UAVCAN_PROTOCOL_PARAM_VALUE_EMPTY: {
            _uavcan_protocol_param_Empty_encode(buffer, bit_ofs, &msg->empty, tao);
            break;
        }
        case UAVCAN_PROTOCOL_PARAM_VALUE_INTEGER_VALUE: {
            canardEncodeScalar(buffer, *bit_ofs, 64, &msg->integer_value);
            *bit_ofs += 64;
            break;
        }
        case UAVCAN_PROTOCOL_PARAM_VALUE_REAL_VALUE: {
            canardEncodeScalar(buffer, *bit_ofs, 32, &msg->real_value);
            *bit_ofs += 32;
            break;
        }
        case UAVCAN_PROTOCOL_PARAM_VALUE_BOOLEAN_VALUE: {
            canardEncodeScalar(buffer, *bit_ofs, 8, &msg->boolean_value);
            *bit_ofs += 8;
            break;
        }
        case UAVCAN_PROTOCOL_PARAM_VALUE_STRING_VALUE: {
            if (!tao) {
                canardEncodeScalar(buffer, *bit_ofs, 8, &msg->string_value.len);
                *bit_ofs += 8;
            }
            for (size_t i=0; i < msg->string_value.len; i++) {
                canardEncodeScalar(buffer, *bit_ofs, 8, &msg->string_value.data[i]);
                *bit_ofs += 8;
            }
            break;
        }
    }
}

void _uavcan_protocol_param_Value_decode(const CanardRxTransfer* transfer, uint32_t* bit_ofs, struct uavcan_protocol_param_Value* msg, bool tao) {
    (void)transfer;
    (void)bit_ofs;
    (void)msg;
    (void)tao;

    uint8_t union_tag;
    canardDecodeScalar(transfer, *bit_ofs, 3, false, &union_tag);
    msg->union_tag = union_tag;
    *bit_ofs += 3;

    switch(msg->union_tag) {
        case UAVCAN_PROTOCOL_PARAM_VALUE_EMPTY: {
            _uavcan_protocol_param_Empty_decode(transfer, bit_ofs, &msg->empty, tao);
            break;
        }

        case UAVCAN_PROTOCOL_PARAM_VALUE_INTEGER_VALUE: {
            canardDecodeScalar(transfer, *bit_ofs, 64, true, &msg->integer_value);
            *bit_ofs += 64;
            break;
        }

        case UAVCAN_PROTOCOL_PARAM_VALUE_REAL_VALUE: {
            canardDecodeScalar(transfer, *bit_ofs, 32, true, &msg->real_value);
            *bit_ofs += 32;
            break;
        }

        case UAVCAN_PROTOCOL_PARAM_VALUE_BOOLEAN_VALUE: {
            canardDecodeScalar(transfer, *bit_ofs, 8, false, &msg->boolean_value);
            *bit_ofs += 8;
            break;
        }

        case UAVCAN_PROTOCOL_PARAM_VALUE_STRING_VALUE: {
            if (!tao) {
                canardDecodeScalar(transfer, *bit_ofs, 8, false, &msg->string_value.len);
                *bit_ofs += 8;
            } else {
                msg->string_value.len = ((transfer->payload_len*8)-*bit_ofs)/8;
            }

            for (size_t i=0; i < msg->string_value.len; i++) {
                canardDecodeScalar(transfer, *bit_ofs, 8, false, &msg->string_value.data[i]);
                *bit_ofs += 8;
            }
            break;
        }

    }
}
#endif
#ifdef CANARD_DSDLC_TEST_BUILD
struct uavcan_protocol_param_Value sample_uavcan_protocol_param_Value_msg(void);
#endif
#ifdef __cplusplus
} // extern "C"
#endif
