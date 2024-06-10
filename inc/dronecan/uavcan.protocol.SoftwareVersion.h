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

#define UAVCAN_PROTOCOL_SOFTWAREVERSION_MAX_SIZE 15
#define UAVCAN_PROTOCOL_SOFTWAREVERSION_SIGNATURE (0xDD46FD376527FEA1ULL)

#define UAVCAN_PROTOCOL_SOFTWAREVERSION_OPTIONAL_FIELD_FLAG_VCS_COMMIT 1
#define UAVCAN_PROTOCOL_SOFTWAREVERSION_OPTIONAL_FIELD_FLAG_IMAGE_CRC 2

struct uavcan_protocol_SoftwareVersion {
    uint8_t major;
    uint8_t minor;
    uint8_t optional_field_flags;
    uint32_t vcs_commit;
    uint64_t image_crc;
};

uint32_t uavcan_protocol_SoftwareVersion_encode(struct uavcan_protocol_SoftwareVersion* msg, uint8_t* buffer
#if CANARD_ENABLE_TAO_OPTION
    , bool tao
#endif
);
bool uavcan_protocol_SoftwareVersion_decode(const CanardRxTransfer* transfer, struct uavcan_protocol_SoftwareVersion* msg);

#if defined(CANARD_DSDLC_INTERNAL)
static inline void _uavcan_protocol_SoftwareVersion_encode(uint8_t* buffer, uint32_t* bit_ofs, struct uavcan_protocol_SoftwareVersion* msg, bool tao);
static inline void _uavcan_protocol_SoftwareVersion_decode(const CanardRxTransfer* transfer, uint32_t* bit_ofs, struct uavcan_protocol_SoftwareVersion* msg, bool tao);
void _uavcan_protocol_SoftwareVersion_encode(uint8_t* buffer, uint32_t* bit_ofs, struct uavcan_protocol_SoftwareVersion* msg, bool tao) {
    (void)buffer;
    (void)bit_ofs;
    (void)msg;
    (void)tao;

    canardEncodeScalar(buffer, *bit_ofs, 8, &msg->major);
    *bit_ofs += 8;
    canardEncodeScalar(buffer, *bit_ofs, 8, &msg->minor);
    *bit_ofs += 8;
    canardEncodeScalar(buffer, *bit_ofs, 8, &msg->optional_field_flags);
    *bit_ofs += 8;
    canardEncodeScalar(buffer, *bit_ofs, 32, &msg->vcs_commit);
    *bit_ofs += 32;
    canardEncodeScalar(buffer, *bit_ofs, 64, &msg->image_crc);
    *bit_ofs += 64;
}

void _uavcan_protocol_SoftwareVersion_decode(const CanardRxTransfer* transfer, uint32_t* bit_ofs, struct uavcan_protocol_SoftwareVersion* msg, bool tao) {
    (void)transfer;
    (void)bit_ofs;
    (void)msg;
    (void)tao;

    canardDecodeScalar(transfer, *bit_ofs, 8, false, &msg->major);
    *bit_ofs += 8;

    canardDecodeScalar(transfer, *bit_ofs, 8, false, &msg->minor);
    *bit_ofs += 8;

    canardDecodeScalar(transfer, *bit_ofs, 8, false, &msg->optional_field_flags);
    *bit_ofs += 8;

    canardDecodeScalar(transfer, *bit_ofs, 32, false, &msg->vcs_commit);
    *bit_ofs += 32;

    canardDecodeScalar(transfer, *bit_ofs, 64, false, &msg->image_crc);
    *bit_ofs += 64;

}
#endif
#ifdef CANARD_DSDLC_TEST_BUILD
struct uavcan_protocol_SoftwareVersion sample_uavcan_protocol_SoftwareVersion_msg(void);
#endif
#ifdef __cplusplus
} // extern "C"
#endif
