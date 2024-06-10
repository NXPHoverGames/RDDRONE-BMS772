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

#define UAVCAN_TIMESTAMP_MAX_SIZE 7
#define UAVCAN_TIMESTAMP_SIGNATURE (0x5BD0B5C81087E0DULL)

#define UAVCAN_TIMESTAMP_UNKNOWN 0

struct uavcan_Timestamp {
    uint64_t usec;
};

uint32_t uavcan_Timestamp_encode(struct uavcan_Timestamp* msg, uint8_t* buffer
#if CANARD_ENABLE_TAO_OPTION
    , bool tao
#endif
);
bool uavcan_Timestamp_decode(const CanardRxTransfer* transfer, struct uavcan_Timestamp* msg);

#if defined(CANARD_DSDLC_INTERNAL)
static inline void _uavcan_Timestamp_encode(uint8_t* buffer, uint32_t* bit_ofs, struct uavcan_Timestamp* msg, bool tao);
static inline void _uavcan_Timestamp_decode(const CanardRxTransfer* transfer, uint32_t* bit_ofs, struct uavcan_Timestamp* msg, bool tao);
void _uavcan_Timestamp_encode(uint8_t* buffer, uint32_t* bit_ofs, struct uavcan_Timestamp* msg, bool tao) {
    (void)buffer;
    (void)bit_ofs;
    (void)msg;
    (void)tao;

    canardEncodeScalar(buffer, *bit_ofs, 56, &msg->usec);
    *bit_ofs += 56;
}

void _uavcan_Timestamp_decode(const CanardRxTransfer* transfer, uint32_t* bit_ofs, struct uavcan_Timestamp* msg, bool tao) {
    (void)transfer;
    (void)bit_ofs;
    (void)msg;
    (void)tao;

    canardDecodeScalar(transfer, *bit_ofs, 56, false, &msg->usec);
    *bit_ofs += 56;

}
#endif
#ifdef CANARD_DSDLC_TEST_BUILD
struct uavcan_Timestamp sample_uavcan_Timestamp_msg(void);
#endif
#ifdef __cplusplus
} // extern "C"
#endif
