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
#include <uavcan.protocol.HardwareVersion.h>
#include <uavcan.protocol.NodeStatus.h>
#include <uavcan.protocol.SoftwareVersion.h>

#ifdef __cplusplus
extern "C"
{
#endif

#define UAVCAN_PROTOCOL_GETNODEINFO_RESPONSE_MAX_SIZE 377
#define UAVCAN_PROTOCOL_GETNODEINFO_RESPONSE_SIGNATURE (0xEE468A8121C46A9EULL)
#define UAVCAN_PROTOCOL_GETNODEINFO_RESPONSE_ID 1

struct uavcan_protocol_GetNodeInfoResponse {
    struct uavcan_protocol_NodeStatus status;
    struct uavcan_protocol_SoftwareVersion software_version;
    struct uavcan_protocol_HardwareVersion hardware_version;
    struct { uint8_t len; uint8_t data[80]; }name;
};

uint32_t uavcan_protocol_GetNodeInfoResponse_encode(struct uavcan_protocol_GetNodeInfoResponse* msg, uint8_t* buffer
#if CANARD_ENABLE_TAO_OPTION
    , bool tao
#endif
);
bool uavcan_protocol_GetNodeInfoResponse_decode(const CanardRxTransfer* transfer, struct uavcan_protocol_GetNodeInfoResponse* msg);

#if defined(CANARD_DSDLC_INTERNAL)
static inline void _uavcan_protocol_GetNodeInfoResponse_encode(uint8_t* buffer, uint32_t* bit_ofs, struct uavcan_protocol_GetNodeInfoResponse* msg, bool tao);
static inline void _uavcan_protocol_GetNodeInfoResponse_decode(const CanardRxTransfer* transfer, uint32_t* bit_ofs, struct uavcan_protocol_GetNodeInfoResponse* msg, bool tao);
void _uavcan_protocol_GetNodeInfoResponse_encode(uint8_t* buffer, uint32_t* bit_ofs, struct uavcan_protocol_GetNodeInfoResponse* msg, bool tao) {
    (void)buffer;
    (void)bit_ofs;
    (void)msg;
    (void)tao;

    _uavcan_protocol_NodeStatus_encode(buffer, bit_ofs, &msg->status, false);
    _uavcan_protocol_SoftwareVersion_encode(buffer, bit_ofs, &msg->software_version, false);
    _uavcan_protocol_HardwareVersion_encode(buffer, bit_ofs, &msg->hardware_version, false);
    if (!tao) {
        canardEncodeScalar(buffer, *bit_ofs, 7, &msg->name.len);
        *bit_ofs += 7;
    }
    for (size_t i=0; i < msg->name.len; i++) {
        canardEncodeScalar(buffer, *bit_ofs, 8, &msg->name.data[i]);
        *bit_ofs += 8;
    }
}

void _uavcan_protocol_GetNodeInfoResponse_decode(const CanardRxTransfer* transfer, uint32_t* bit_ofs, struct uavcan_protocol_GetNodeInfoResponse* msg, bool tao) {
    (void)transfer;
    (void)bit_ofs;
    (void)msg;
    (void)tao;

    _uavcan_protocol_NodeStatus_decode(transfer, bit_ofs, &msg->status, false);

    _uavcan_protocol_SoftwareVersion_decode(transfer, bit_ofs, &msg->software_version, false);

    _uavcan_protocol_HardwareVersion_decode(transfer, bit_ofs, &msg->hardware_version, false);

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
struct uavcan_protocol_GetNodeInfoResponse sample_uavcan_protocol_GetNodeInfoResponse_msg(void);
#endif
#ifdef __cplusplus
} // extern "C"
#endif
