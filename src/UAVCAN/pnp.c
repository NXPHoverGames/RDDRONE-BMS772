/****************************************************************************
 * nxp_bms/BMS_v1/src/UAVCAN/pnp.c
 *
 * BSD 3-Clause License
 * 
 * Copyright 2020 NXP
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 * 
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include "pnp.h"

// Use NuttX crc64 function TODO fallback header for other platforms
#include <crc64.h>


#ifdef PNP_CAN_FD
#include "uavcan/pnp/NodeIDAllocationData_2_0.h"
#error "Not yet supported"
#else
#include "uavcan/pnp/NodeIDAllocationData_1_0.h"
#endif

#include "uavcan/node/ID_1_0.h"

#define PNP_PORT_ID        UAVCAN_PNP_NODE_IDALLOCATION_DATA_PORT_ID
#define PNP_PAYLOAD_SIZE   UAVCAN_PNP_NODE_IDALLOCATION_DATA_MSG_SIZE
#define PNP_REQUEST_SIZE   PNP_PAYLOAD_SIZE - UAVCAN_NODE_ID_MSG_SIZE
#define PNP_RESPONSE_SIZE  PNP_PAYLOAD_SIZE

#define PNP_UNIQUE_ID_SIZE 16 // 128 bit unique id

/****************************************************************************
 * private data
 ****************************************************************************/ 

CanardRxSubscription allocSub;

// 128 bit unique id
uint8_t local_unique_id[PNP_UNIQUE_ID_SIZE];

// Unsupported see V2
CanardNodeID preffered_node_id;

CanardNodeID node_id = CANARD_NODE_ID_UNSET;

// NodeIDAllocationData message
uavcan_pnp_node_idallocation_data node_id_alloc_msg;
uint8_t node_id_alloc_payload_buffer[PNP_PAYLOAD_SIZE];

CanardTransferID node_id_alloc_transfer_id = 0;

/****************************************************************************
 * private Functions declerations 
 ****************************************************************************/

uint64_t makePseudoUniqueId(uint8_t* pUniqueID){
    // NuttX CRC64/WE implementation
    return crc64(pUniqueID, PNP_UNIQUE_ID_SIZE);
}


/****************************************************************************
 * public functions
 ****************************************************************************/

/* Rule A. On initialization:
 * 1. The allocatee subscribes to this message.
 * 2. The allocatee starts the Request Timer with a random interval [0, 1) sec of Trequest.
 */
uint32_t initPNPAllocatee(CanardInstance* ins, uint8_t* unique_id){
    // Store unique_id locally
    memcpy(&local_unique_id[0],&unique_id[0], sizeof(local_unique_id));
    
    // Create RX Subscriber so we can listen to NodeIDAllocationData msgs
    (void) canardRxSubscribe(ins,   // Subscribe to messages uavcan.node.Heartbeat.
                CanardTransferKindMessage,
                PNP_PORT_ID,  // The fixed Subject-ID of the Heartbeat message type (see DSDL definition).
                PNP_PAYLOAD_SIZE,      // The maximum payload size (max DSDL object size) from the DSDL definition.
                CANARD_DEFAULT_TRANSFER_ID_TIMEOUT_USEC,
                &allocSub);

    // Callee should start a timer with a random interval between 0 and 1 sec
}

/* Rule B. On expiration of the Request Timer (started as per rules A, B, or C):
 * 1. Request Timer restarts with a random interval [0, 1) sec of Trequest (chosen anew).
 * 2. The allocatee broadcasts an allocation request message, where the fields are populated as follows:
 *    unique_id_hash    - a 48-bit hash of the unique-ID of the allocatee.
 *    allocated_node_id - empty (not populated).
 */
int32_t PNPAllocRequest(CanardInstance* ins) {
    // Callee should restart timer

    // Allocation already done, nothing to do TODO maybe stop subscribing
    if(node_id != CANARD_NODE_ID_UNSET)
        return 1;

    node_id_alloc_msg.unique_id_hash = makePseudoUniqueId(local_unique_id);

    CanardMicrosecond transmission_deadline = getMonotonicTimestampUSec() + 1000 * 10;

    const CanardTransfer transfer = {
		.timestamp_usec = transmission_deadline,      // Zero if transmission deadline is not limited.
		.priority       = CanardPriorityNominal,
		.transfer_kind  = CanardTransferKindMessage,
		.port_id        = PNP_PORT_ID,                // This is the subject-ID.
		.remote_node_id = CANARD_NODE_ID_UNSET,       // Messages cannot be unicast, so use UNSET.
		.transfer_id    = node_id_alloc_transfer_id,
		.payload_size   = PNP_REQUEST_SIZE,
		.payload        = &node_id_alloc_payload_buffer,
	};

    uavcan_pnp_node_idallocation_data_serializeToBuffer(&node_id_alloc_msg, &node_id_alloc_payload_buffer,0);

    // set the data ready in the buffer and chop if needed 
	++node_id_alloc_transfer_id;  // The transfer-ID shall be incremented after every transmission on this subject.
	int32_t result = canardTxPush(ins, &transfer);

	if (result < 0) {
		// An error has occurred: either an argument is invalid or we've ran out of memory.
		// It is possible to statically prove that an out-of-memory will never occur for a given application if the
		// heap is sized correctly; for background, refer to the Robson's Proof and the documentation for O1Heap.
		return result;
	}
	return 0;
}

/* Rule C. On any allocation message, even if other rules also match:
 * 1. Request Timer restarts with a random interval of Trequest (chosen anew).
 *
 * Rule D. On an allocation message WHERE (source node-ID is non-anonymous, i.e., regular allocation response)
 *                                  AND   (the field unique_id_hash matches the allocatee's 48-bit unique-ID hash)
 *                                  AND   (the field allocated_node_id is populated):
 * 1. Request Timer stops.
 * 2. The allocatee initializes its node-ID with the received value.
 * 3. The allocatee terminates its subscription to allocation messages.
 * 4. Exit.
 */
int32_t PNPProcess(CanardInstance* ins, CanardTransfer* transfer){
    // Allocation already done, nothing to do
    if(node_id != CANARD_NODE_ID_UNSET)
        return 1;
    
    if (transfer->remote_node_id == CANARD_NODE_ID_UNSET)  // Another request, ignore.
        return 0;
    
    int32_t allocated = CANARD_NODE_ID_UNSET;
    uavcan_pnp_node_idallocation_data msg;
    
    uavcan_pnp_node_idallocation_data_deserializeFromBuffer(&msg, transfer->payload, transfer->payload_size ,0);
 
#ifdef PNP_CAN_FD
    //TODO V2 Stuff here
#else
    
    if(transfer->payload_size == PNP_RESPONSE_SIZE) {
        if(msg.unique_id_hash == (makePseudoUniqueId(local_unique_id) & 0xFFFFFFFFFFFF)) {
            allocated = msg.allocated_node_id.value;
        }
    }
#endif

    if (allocated == CANARD_NODE_ID_UNSET)
        return -1; // UID mismatch.

    if (allocated <= 0 || allocated >= CANARD_NODE_ID_MAX)
        // Allocated node-ID ignored because it exceeds max_node_id
        return -1;

    // Plug-and-play allocation done: got node-ID %s from server %s', allocated, meta.source_node_id)
    node_id = allocated;

    return 1; 
}


CanardNodeID PNPGetNodeID(){
    return node_id;
}


const CanardPortID PNPGetPortID(){
    return PNP_PORT_ID;
}
