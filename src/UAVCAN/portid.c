/****************************************************************************
 * nxp_bms/BMS_v1/src/UAVCAN/portid.c
 *
 * BSD 3-Clause License
 * 
 * Copyright 2020-2021 NXP
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

#include "portid.h"

#include "uavcan/_register/Access_1_0.h"
#include "uavcan/_register/List_1_0.h"
#include "uavcan/_register/Name_1_0.h"
#include "uavcan/_register/Value_1_0.h"

#include <stdio.h>
#include <string.h>

#include "util.h"
#include "cli.h"

#include <nuttx/board.h>

/****************************************************************************
 * private data
 ****************************************************************************/ 

uavcan_node_GetInfo_Response_1_0* node_info;
// CanardTransferID getinfo_response_transfer_id = 0;
CanardTransferID register_access_response_transfer_id = 0;
// CanardTransferID register_list_response_transfer_id = 0;
CanardTransferID execute_command_response_transfer_id = 0;

// CanardRxSubscription getinfo_subscription;
CanardRxSubscription register_access_subscription;
// CanardRxSubscription register_list_subscription;
CanardRxSubscription execute_command_subscription;

//TODO register list and data
uavcan_register_interface_entry register_list[UAVCAN_REGISTER_COUNT];
uint32_t register_list_size = 0;

static const char *reset_cli_args[] = {
  "bms",
  "reset"
};

static CanardTransfer response;

static char register_string[255];
static uint8_t access_response_payload_buffer[uavcan_register_Access_Response_1_0_SERIALIZATION_BUFFER_SIZE_BYTES_];

/****************************************************************************
 * public functions
 ****************************************************************************/

int32_t uavcan_register_interface_init(CanardInstance* ins, uavcan_node_GetInfo_Response_1_0* info){
    node_info = info;

	// int8_t ret = canardRxSubscribe(ins,
	// 			 CanardTransferKindRequest,
	// 			 uavcan_node_GetInfo_1_0_FIXED_PORT_ID_,
	// 			 uavcan_node_GetInfo_Request_1_0_SERIALIZATION_BUFFER_SIZE_BYTES_,
	// 			 CANARD_DEFAULT_TRANSFER_ID_TIMEOUT_USEC,
	// 			 &getinfo_subscription);

    int8_t ret = canardRxSubscribe(ins,
				 CanardTransferKindRequest,
				 uavcan_register_Access_1_0_FIXED_PORT_ID_,
				 uavcan_register_Access_Request_1_0_SERIALIZATION_BUFFER_SIZE_BYTES_,
				 CANARD_DEFAULT_TRANSFER_ID_TIMEOUT_USEC,
				 &register_access_subscription);

    // ret = canardRxSubscribe(ins,
	// 			 CanardTransferKindRequest,
	// 			 uavcan_register_List_1_0_FIXED_PORT_ID_,
	// 			 uavcan_register_List_Request_1_0_SERIALIZATION_BUFFER_SIZE_BYTES_,
	// 			 CANARD_DEFAULT_TRANSFER_ID_TIMEOUT_USEC,
	// 			 &register_list_subscription);

    // TODO(Charles): Probably want to handle the actual return codes from the subscribes()
    return (int32_t)ret;
}

int32_t uavcan_services_init(CanardInstance* ins)
{
    int8_t ret = canardRxSubscribe(ins,
				 CanardTransferKindRequest,
				 uavcan_node_ExecuteCommand_1_1_FIXED_PORT_ID_,
				 uavcan_node_ExecuteCommand_Request_1_1_SERIALIZATION_BUFFER_SIZE_BYTES_,
				 CANARD_DEFAULT_TRANSFER_ID_TIMEOUT_USEC,
				 &execute_command_subscription);

    return 0;
}



int32_t uavcan_register_interface_add_entry(const char* name, register_access_set_callback cb_set, register_access_get_callback cb_get){
    if(register_list_size < UAVCAN_REGISTER_COUNT){
        register_list[register_list_size].name = name;
        register_list[register_list_size].cb_set = cb_set;
        register_list[register_list_size].cb_get = cb_get;
        register_list_size++;
        return 0;
    } else {
        return -UAVCAN_REGISTER_ERROR_OUT_OF_MEMORY; // register list full
    }
    
}

// Handler for all PortID registration related messages
int32_t uavcan_register_interface_process(CanardInstance* ins, CanardTransfer* transfer) {
    // if(transfer->port_id == uavcan_node_GetInfo_1_0_FIXED_PORT_ID_) {
    //     return uavcan_register_interface_get_info_response(ins, transfer);
    // } else if(transfer->port_id == uavcan_register_Access_1_0_FIXED_PORT_ID_) {
    //     return uavcan_register_interface_access_response(ins, transfer);
    // } else if(transfer->port_id == uavcan_register_List_1_0_FIXED_PORT_ID_) {
    //     return uavcan_register_interface_list_response(ins, transfer);
    // }

    if(transfer->port_id == uavcan_register_Access_1_0_FIXED_PORT_ID_) {
        return uavcan_register_interface_access_response(ins, transfer);
    }

    return 0; // Nothing to do
}

int32_t uavcan_service_process(CanardInstance* ins, CanardTransfer* transfer) {
    if(transfer->port_id == uavcan_node_ExecuteCommand_1_1_FIXED_PORT_ID_) {
        return uavcan_service_handle_execute_command(ins, transfer);
    }

    return 0; // Nothing to do
}

// Handler for node.GetInfo which yields a response
// int32_t uavcan_register_interface_get_info_response(CanardInstance* ins, CanardTransfer* request){

//     // NOTE(Charles): The request has no payload, no need to deserialize here - it would fail

//     // Setup node.GetInfo response
//     uint8_t response_payload_buffer[uavcan_node_GetInfo_Response_1_0_SERIALIZATION_BUFFER_SIZE_BYTES_];
    
//     CanardMicrosecond transmission_deadline = getMonotonicTimestampUSec() + 1000 * 10;

//     response.timestamp_usec = transmission_deadline; // Zero if transmission deadline is not limited.
//     response.priority       = CanardPriorityNominal;
//     response.transfer_kind  = CanardTransferKindResponse;
//     response.port_id        = uavcan_node_GetInfo_1_0_FIXED_PORT_ID_; // This is the subject-ID.
//     response.remote_node_id = request->remote_node_id;       // Send back to request Node
//     response.transfer_id    = getinfo_response_transfer_id;
//     response.payload_size   = uavcan_node_GetInfo_Response_1_0_SERIALIZATION_BUFFER_SIZE_BYTES_;
//     response.payload        = &response_payload_buffer;

//     int32_t result = uavcan_node_GetInfo_Response_1_0_serialize_(node_info, &response_payload_buffer, &response.payload_size);
//     if(result == 0) {
//         // set the data ready in the buffer and chop if needed 
//         ++getinfo_response_transfer_id;  // The transfer-ID shall be incremented after every transmission on this subject.
//         result = canardTxPush(ins, &response);
//     }

// 	if (result < 0) {
// 		// An error has occurred: either an argument is invalid or we've ran out of memory.
// 		// It is possible to statically prove that an out-of-memory will never occur for a given application if the
// 		// heap is sized correctly; for background, refer to the Robson's Proof and the documentation for O1Heap.
// 		return -UAVCAN_REGISTER_ERROR_SERIALIZATION;
// 	}
// 	return 1;
    
// }

// Handler for register access interface
int32_t uavcan_register_interface_access_response(CanardInstance* ins, CanardTransfer* request){
    int index;
    {
        uavcan_register_Access_Request_1_0 msg;

        if(uavcan_register_Access_Request_1_0_deserialize_(&msg, request->payload, &request->payload_size) < 0){
            //Error deserialize failed
            return -UAVCAN_REGISTER_ERROR_SERIALIZATION;
        }

        {
            size_t reg_str_len;
            for(index = 0; index < register_list_size; index++) 
            {
                // Handle bms_fault_code register explicitly. It is just a fetch
                if( strcmp( register_list[index].name, "bms_fault_code" ) == 0 )
                {
                    reg_str_len = sprintf(register_string, "%s", register_list[index].name);
                    break;
                }

                // Handle Port ID assignment registers
                reg_str_len = sprintf(register_string, "uavcan.pub.%s.id", register_list[index].name); //TODO more option then pub (sub rate etc)
                if(strncmp(msg.name.name.elements, register_string, reg_str_len) == 0) {
                    if(msg.value._tag_ != 0) { // Value has been set thus we call set handler
                        if(register_list[index].cb_set(&msg.value) != 0) {
                            // TODO error ocurred check doc for correct response
                        }
                    }
                    break; // We're done exit loop
                }
            }
        }
    }

    uavcan_register_Access_Response_1_0 response_msg;
    uavcan_register_Access_Response_1_0_initialize_(&response_msg);

    // TODO(Charles): To be correct per Cyphal/UAVCAN spec, we would also need to fetch whether the register is mutable/permanent
    if(index < register_list_size)  { // Index is available
        response_msg.value = register_list[index].cb_get();
    } else {
        uavcan_register_Value_1_0_initialize_(&response_msg.value);
        uavcan_register_Value_1_0_select_empty_(&response_msg.value);
    }

    CanardMicrosecond transmission_deadline = getMonotonicTimestampUSec() + 1000 * 10;

    response.timestamp_usec = transmission_deadline; // Zero if transmission deadline is not limited.
    response.priority       = CanardPriorityNominal;
    response.transfer_kind  = CanardTransferKindResponse;
    response.port_id        = uavcan_register_Access_1_0_FIXED_PORT_ID_; // This is the subject-ID.
    response.remote_node_id = request->remote_node_id;       // Send back to request Node
    response.transfer_id    = register_access_response_transfer_id;
    response.payload_size   = uavcan_register_Access_Response_1_0_SERIALIZATION_BUFFER_SIZE_BYTES_;
    response.payload        = &access_response_payload_buffer;

    int32_t result = uavcan_register_Access_Response_1_0_serialize_(&response_msg, &access_response_payload_buffer, &response.payload_size);

    if(result == 0){
        // set the data ready in the buffer and chop if needed 
        ++register_access_response_transfer_id;  // The transfer-ID shall be incremented after every transmission on this subject.
        result = canardTxPush(ins, &response);
    }

    if (result < 0) {
        // An error has occurred: either an argument is invalid or we've ran out of memory.
        // It is possible to statically prove that an out-of-memory will never occur for a given application if the
        // heap is sized correctly; for background, refer to the Robson's Proof and the documentation for O1Heap.
        return -UAVCAN_REGISTER_ERROR_SERIALIZATION;
    }

    return 1;
	
}

// // Handler for register list interface
// int32_t uavcan_register_interface_list_response(CanardInstance* ins, CanardTransfer* request){
//     uavcan_register_List_Request_1_0 msg;
    
//     if(uavcan_register_List_Request_1_0_deserialize_(&msg, request->payload, &request->payload_size) < 0){
//         //Error deserialize failed
//         return -UAVCAN_REGISTER_ERROR_SERIALIZATION;
//     }
    
//     //Setup register response
    
//     uint8_t response_payload_buffer[uavcan_register_List_Response_1_0_SERIALIZATION_BUFFER_SIZE_BYTES_]; //TODO we know already how big our response is, don't overallocate.
    
//     CanardMicrosecond transmission_deadline = getMonotonicTimestampUSec() + 1000 * 10;
    
//     uavcan_register_List_Response_1_0 response_msg;
    
//     // Reponse magic start
    
//     if(msg.index <= register_list_size) {
//         response_msg.name.name.count = sprintf(response_msg.name.name.elements, 
//                                                 "uavcan.pub.%s.id", 
//                                                 register_list[msg.index].name); 
//     }
//     //TODO more option then pub (sub rate

//     // Response magic end
//     response.timestamp_usec = transmission_deadline; // Zero if transmission deadline is not limited.
//     response.priority       = CanardPriorityNominal;
//     response.transfer_kind  = CanardTransferKindResponse;
//     response.port_id        = uavcan_register_List_1_0_FIXED_PORT_ID_; // This is the subject-ID.
//     response.remote_node_id = request->remote_node_id;       // Send back to request Node
//     response.transfer_id    = register_list_response_transfer_id;
//     response.payload_size   = uavcan_register_List_Response_1_0_SERIALIZATION_BUFFER_SIZE_BYTES_; //See prev TODO
//     response.payload        = &response_payload_buffer;

//     int32_t result = uavcan_register_List_Response_1_0_serialize_(&response_msg, &response_payload_buffer, &response.payload_size);

//     if(result == 0) {
//         // set the data ready in the buffer and chop if needed 
//         ++register_list_response_transfer_id;  // The transfer-ID shall be incremented after every transmission on this subject.
//         result = canardTxPush(ins, &response);
//     }

// 	if (result < 0) {
// 		// An error has occurred: either an argument is invalid or we've ran out of memory.
// 		// It is possible to statically prove that an out-of-memory will never occur for a given application if the
// 		// heap is sized correctly; for background, refer to the Robson's Proof and the documentation for O1Heap.
// 		return -UAVCAN_REGISTER_ERROR_SERIALIZATION;
// 	}
// 	return 1;
// }

int32_t uavcan_service_handle_execute_command(CanardInstance* ins, CanardTransfer* request)
{
    uavcan_node_ExecuteCommand_Request_1_1 msg;
    
    if(uavcan_node_ExecuteCommand_Request_1_1_deserialize_(&msg, request->payload, &request->payload_size) < 0){
        //Error deserialize failed
        return -UAVCAN_REGISTER_ERROR_SERIALIZATION;
    }

    // Process command
    uint8_t cmd_status = uavcan_node_ExecuteCommand_Response_1_1_STATUS_BAD_COMMAND;
    switch( msg.command )
    {
        case EXECUTE_COMMAND_ID_RESET_FAULT:
        {
            cli_processCommands( 2, (char **)reset_cli_args );
            cmd_status = uavcan_node_ExecuteCommand_Response_1_1_STATUS_SUCCESS;
            break;
        }

        case EXECUTE_COMMAND_ID_REBOOT:
        {
            board_reset( 0 );
            cmd_status = uavcan_node_ExecuteCommand_Response_1_1_STATUS_SUCCESS;
            break;
        }
    }
    
    //Setup register response
    
    //TODO we know already how big our response is, don't overallocate.
    uint8_t response_payload_buffer[uavcan_node_ExecuteCommand_Response_1_1_SERIALIZATION_BUFFER_SIZE_BYTES_]; 
    CanardMicrosecond transmission_deadline = getMonotonicTimestampUSec() + 1000 * 10;
    uavcan_node_ExecuteCommand_Response_1_1 response_msg;

    response_msg.status = cmd_status;
    
    response.timestamp_usec = transmission_deadline; // Zero if transmission deadline is not limited.
    response.priority       = CanardPriorityNominal;
    response.transfer_kind  = CanardTransferKindResponse;
    response.port_id        = uavcan_node_ExecuteCommand_1_1_FIXED_PORT_ID_; // This is the subject-ID.
    response.remote_node_id = request->remote_node_id;       // Send back to request Node
    response.transfer_id    = execute_command_response_transfer_id;
    response.payload_size   = uavcan_node_ExecuteCommand_Response_1_1_SERIALIZATION_BUFFER_SIZE_BYTES_; //See prev TODO
    response.payload        = &response_payload_buffer;

    int32_t result = uavcan_node_ExecuteCommand_Response_1_1_serialize_(&response_msg, &response_payload_buffer, &response.payload_size);

    if(result == 0) {
        // set the data ready in the buffer and chop if needed 
        ++execute_command_response_transfer_id;  // The transfer-ID shall be incremented after every transmission on this subject.
        result = canardTxPush(ins, &response);
    }

	if (result < 0) {
		// An error has occurred: either an argument is invalid or we've ran out of memory.
		// It is possible to statically prove that an out-of-memory will never occur for a given application if the
		// heap is sized correctly; for background, refer to the Robson's Proof and the documentation for O1Heap.
		return -UAVCAN_REGISTER_ERROR_SERIALIZATION;
	}
	return 1;
}

