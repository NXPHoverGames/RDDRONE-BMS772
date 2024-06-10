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

#define UAVCAN_PROTOCOL_PARAM_EMPTY_MAX_SIZE 0
#define UAVCAN_PROTOCOL_PARAM_EMPTY_SIGNATURE (0x6C4D0E8EF37361DFULL)

struct uavcan_protocol_param_Empty {
};

uint32_t uavcan_protocol_param_Empty_encode(struct uavcan_protocol_param_Empty* msg, uint8_t* buffer
#if CANARD_ENABLE_TAO_OPTION
    , bool tao
#endif
);
bool uavcan_protocol_param_Empty_decode(const CanardRxTransfer* transfer, struct uavcan_protocol_param_Empty* msg);

#if defined(CANARD_DSDLC_INTERNAL)
static inline void _uavcan_protocol_param_Empty_encode(uint8_t* buffer, uint32_t* bit_ofs, struct uavcan_protocol_param_Empty* msg, bool tao);
static inline void _uavcan_protocol_param_Empty_decode(const CanardRxTransfer* transfer, uint32_t* bit_ofs, struct uavcan_protocol_param_Empty* msg, bool tao);
void _uavcan_protocol_param_Empty_encode(uint8_t* buffer, uint32_t* bit_ofs, struct uavcan_protocol_param_Empty* msg, bool tao) {
    (void)buffer;
    (void)bit_ofs;
    (void)msg;
    (void)tao;

}

void _uavcan_protocol_param_Empty_decode(const CanardRxTransfer* transfer, uint32_t* bit_ofs, struct uavcan_protocol_param_Empty* msg, bool tao) {
    (void)transfer;
    (void)bit_ofs;
    (void)msg;
    (void)tao;

}
#endif
#ifdef CANARD_DSDLC_TEST_BUILD
struct uavcan_protocol_param_Empty sample_uavcan_protocol_param_Empty_msg(void);
#endif
#ifdef __cplusplus
} // extern "C"
#endif
