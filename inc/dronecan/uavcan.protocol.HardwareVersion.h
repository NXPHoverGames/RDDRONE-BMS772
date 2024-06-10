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

#define UAVCAN_PROTOCOL_HARDWAREVERSION_MAX_SIZE 274
#define UAVCAN_PROTOCOL_HARDWAREVERSION_SIGNATURE (0xAD5C4C933F4A0C4ULL)

struct uavcan_protocol_HardwareVersion {
    uint8_t major;
    uint8_t minor;
    uint8_t unique_id[16];
    struct { uint8_t len; uint8_t data[255]; }certificate_of_authenticity;
};

uint32_t uavcan_protocol_HardwareVersion_encode(struct uavcan_protocol_HardwareVersion* msg, uint8_t* buffer
#if CANARD_ENABLE_TAO_OPTION
    , bool tao
#endif
);
bool uavcan_protocol_HardwareVersion_decode(const CanardRxTransfer* transfer, struct uavcan_protocol_HardwareVersion* msg);

#if defined(CANARD_DSDLC_INTERNAL)
static inline void _uavcan_protocol_HardwareVersion_encode(uint8_t* buffer, uint32_t* bit_ofs, struct uavcan_protocol_HardwareVersion* msg, bool tao);
static inline void _uavcan_protocol_HardwareVersion_decode(const CanardRxTransfer* transfer, uint32_t* bit_ofs, struct uavcan_protocol_HardwareVersion* msg, bool tao);
void _uavcan_protocol_HardwareVersion_encode(uint8_t* buffer, uint32_t* bit_ofs, struct uavcan_protocol_HardwareVersion* msg, bool tao) {
    (void)buffer;
    (void)bit_ofs;
    (void)msg;
    (void)tao;

    canardEncodeScalar(buffer, *bit_ofs, 8, &msg->major);
    *bit_ofs += 8;
    canardEncodeScalar(buffer, *bit_ofs, 8, &msg->minor);
    *bit_ofs += 8;
    for (size_t i=0; i < 16; i++) {
        canardEncodeScalar(buffer, *bit_ofs, 8, &msg->unique_id[i]);
        *bit_ofs += 8;
    }
    if (!tao) {
        canardEncodeScalar(buffer, *bit_ofs, 8, &msg->certificate_of_authenticity.len);
        *bit_ofs += 8;
    }
    for (size_t i=0; i < msg->certificate_of_authenticity.len; i++) {
        canardEncodeScalar(buffer, *bit_ofs, 8, &msg->certificate_of_authenticity.data[i]);
        *bit_ofs += 8;
    }
}

void _uavcan_protocol_HardwareVersion_decode(const CanardRxTransfer* transfer, uint32_t* bit_ofs, struct uavcan_protocol_HardwareVersion* msg, bool tao) {
    (void)transfer;
    (void)bit_ofs;
    (void)msg;
    (void)tao;

    canardDecodeScalar(transfer, *bit_ofs, 8, false, &msg->major);
    *bit_ofs += 8;

    canardDecodeScalar(transfer, *bit_ofs, 8, false, &msg->minor);
    *bit_ofs += 8;

    for (size_t i=0; i < 16; i++) {
        canardDecodeScalar(transfer, *bit_ofs, 8, false, &msg->unique_id[i]);
        *bit_ofs += 8;
    }

    if (!tao) {
        canardDecodeScalar(transfer, *bit_ofs, 8, false, &msg->certificate_of_authenticity.len);
        *bit_ofs += 8;
    } else {
        msg->certificate_of_authenticity.len = ((transfer->payload_len*8)-*bit_ofs)/8;
    }

    for (size_t i=0; i < msg->certificate_of_authenticity.len; i++) {
        canardDecodeScalar(transfer, *bit_ofs, 8, false, &msg->certificate_of_authenticity.data[i]);
        *bit_ofs += 8;
    }

}
#endif
#ifdef CANARD_DSDLC_TEST_BUILD
struct uavcan_protocol_HardwareVersion sample_uavcan_protocol_HardwareVersion_msg(void);
#endif
#ifdef __cplusplus
} // extern "C"
#endif
