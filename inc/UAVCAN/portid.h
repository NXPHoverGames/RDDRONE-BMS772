/****************************************************************************
 * nxp_bms/BMS_v1/inc/UAVCAN/portid.h
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
 ** ###################################################################
 **     Filename    : portid.h
 **     Project     : SmartBattery_RDDRONE_BMS772
 **     Processor   : S32K144
 **     Version     : 1.00
 **     Date   		: 2021-01-19
 **     Abstract    :
 **        portid module.
 **
 ** ###################################################################*/
/*!
 ** @file portid.h
 **
 ** @version 01.00
 **
 ** @brief
 **        portid module. this module implements the UAVCAN port id 
 **
 */

#ifndef UAVCAN_REGISTER_INTERFACE_H_
#define UAVCAN_REGISTER_INTERFACE_H_

#include <canard.h>

#define NUNAVUT_ASSERT
#include "uavcan/node/GetInfo_1_0.h"
#include "uavcan/_register/Value_1_0.h"

//No of pre allocated register entries
#ifndef UAVCAN_REGISTER_COUNT
# define UAVCAN_REGISTER_COUNT 5
#endif

#define UAVCAN_REGISTER_ERROR_SERIALIZATION 1
#define UAVCAN_REGISTER_ERROR_OUT_OF_MEMORY 2

typedef int32_t (*register_access_set_callback)(uavcan_register_Value_1_0* value);
typedef uavcan_register_Value_1_0 (*register_access_get_callback)(void);

typedef struct
{
    /// uavcan.register.Name.1.0 name
    const char* name;
    register_access_set_callback cb_set;
    register_access_get_callback cb_get;
} uavcan_register_interface_entry;


int32_t uavcan_register_interface_init(CanardInstance* ins, uavcan_node_GetInfo_Response_1_0* info);

int32_t uavcan_register_interface_add_entry(const char* name, register_access_set_callback cb_set, register_access_get_callback cb_get);

// Handler for all PortID registration related messages
int32_t uavcan_register_interface_process(CanardInstance* ins, CanardTransfer* transfer);

// Handler for node.GetInfo which yields a response
int32_t uavcan_register_interface_get_info_response(CanardInstance* ins, CanardTransfer* request);

// Handler for register access interface
int32_t uavcan_register_interface_access_response(CanardInstance* ins, CanardTransfer* request);

// Handler for register list interface
int32_t uavcan_register_interface_list_response(CanardInstance* ins, CanardTransfer* request);

#endif //UAVCAN_REGISTER_INTERFACE_H_
