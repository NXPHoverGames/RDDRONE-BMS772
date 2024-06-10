/****************************************************************************
 * nxp_bms/BMS_v1/src/dronecan.c
 *
 * BSD 3-Clause License
 *
 * Copyright 2023-2024 NXP
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

/****************************************************************************
 * Included Files
 ****************************************************************************/
#include <sched.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <string.h>
#include <assert.h>
#include <errno.h>
#include <math.h>

#include <net/if.h>
#include <sys/time.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/eventfd.h>
#include <nuttx/random.h>

#include <sys/boardctl.h>

#include <poll.h>

#include <nuttx/can.h>
#include <netpacket/can.h>

#include <semaphore.h>

#include <nuttx/version.h>

#include "dronecan.h"
#include "cli.h"
#include "gpio.h"

#include "data.h"
#include "timestamp.h"

#ifdef CANARD_VERSION_MAJOR
#    undef CANARD_VERSION_MAJOR
#endif
#ifdef CANARD_VERSION_MINOR
#    undef CANARD_VERSION_MINOR
#endif

#include "socketcan.h"

#define CANARD_DSDLC_INTERNAL // Use header-only encode/decode
#include <uavcan.protocol.GetNodeInfo.h>
#include <uavcan.protocol.NodeStatus.h>
#include <uavcan.protocol.param.GetSet.h>
#include <uavcan.protocol.param.ExecuteOpcode.h>
#include <uavcan.protocol.RestartNode.h>

#include <ardupilot.equipment.power.BatteryContinuous.h>
#include <ardupilot.equipment.power.BatteryPeriodic.h>
#include <ardupilot.equipment.power.BatteryCells.h>
#include <ardupilot.equipment.power.BatteryInfoAux.h>
#include <uavcan.equipment.power.BatteryInfo.h>

/****************************************************************************
 * Defines
 ****************************************************************************/

//!< Use this define to enable some DroneCAN informational messages via the CLI
#define ENABLE_DRONECAN_INFO_MESSAGES_ON_CONSOLE
//#define ENABLE_DRONECAN_DEBUG_MESSAGES_ON_CONSOLE

#ifndef SEM_OCCUPIED_ERROR
#    define SEM_OCCUPIED_ERROR 255
#endif

#define ONE_SEC_IN_NS 1000000000
#define MS_TO_NS_MULT 1000000

#define APP_NODE_NAME "com.nxp.rddrone.bms772"

#define UAVCAN_NODE_ID_ALLOCATION_DATA_TYPE_ID              1
#define UAVCAN_NODE_ID_ALLOCATION_DATA_TYPE_SIGNATURE       0x0b2a812620a11d40
#define UAVCAN_NODE_ID_ALLOCATION_RANDOM_TIMEOUT_RANGE_USEC 400000UL
#define UAVCAN_NODE_ID_ALLOCATION_REQUEST_DELAY_OFFSET_USEC 600000UL

#define UNIQUE_ID_LENGTH_BYTES 16

#define DRONECAN_DAEMON_PRIORITY   110
#define DRONECAN_DAEMON_STACK_SIZE 3600
#define DRONECAN_TAO               1
#define CAN_DEVICE                 "can0"

#define BOOL_VAL   UAVCAN_PROTOCOL_PARAM_VALUE_BOOLEAN_VALUE
#define INT_VAL    UAVCAN_PROTOCOL_PARAM_VALUE_INTEGER_VALUE
#define STRING_VAL UAVCAN_PROTOCOL_PARAM_VALUE_STRING_VALUE
#define REAL_VAL   UAVCAN_PROTOCOL_PARAM_VALUE_REAL_VALUE

/* DroneCAN Specific parameters */
#define BMS_RESET_FAULT (NONE + 0)

#define CONSTANTS_ABSOLUTE_NULL_CELSIUS 273.15f

#define MEMORY_POOL_SIZE      3072
#define MEMORY_POOL_ALIGNMENT (sizeof(void *) * 4U)

// To avoid naming conflict with Cyphal
#define CanardInstance DroneCanardInstance
#define canardInit     dronecanardInit

/****************************************************************************
 * Types
 ****************************************************************************/

/****************************************************************************
 * private data
 ****************************************************************************/
struct file * gDroneCANEventfp;
static bool   gDronecanInitialized = false;
struct pollfd pDfds[2];

// Strings needed for the bms reset command
const char *bmsString     = "bms";
const char *resetString   = "reset";
const char *saveString    = "save";
const char *defaultString = "default";


/*
 * Variables used for dynamic node ID allocation.
 * RTFM at http://uavcan.org/Specification/6._Application_level_functions/#dynamic-node-id-allocation
 */
static uint64_t
               g_send_next_node_id_allocation_request_at; ///< When the next node ID allocation request should be sent
static uint8_t g_node_id_allocation_unique_id_offset; ///< Depends on the stage of the next request

static bool g_canard_daemon_started;

// this will hold the 16B long unique ID
static uint8_t gMy_unique_id[UNIQUE_ID_LENGTH_BYTES];

/****************************************************************************
 * private Functions declerations
 ****************************************************************************/

static int32_t dronecan_task_initialize(DroneCanardInstance *ins, CanardSocketInstance *sock_ins);

//! @brief the DRONECAN deamon task
static int DRONECANTask(int argc, char *argv[]);

static void pubPowerBatteryContinuous(DroneCanardInstance *ins, uint8_t *transfer_id);
static void pubPowerBatteryPeriodic(DroneCanardInstance *ins, uint8_t *transfer_id);
static void pubPowerBatteryCells(DroneCanardInstance *ins, uint8_t *transfer_id);
static void pubPowerBatteryInfo(DroneCanardInstance *ins, uint8_t *transfer_id);
static void pubPowerBatteryInfoAux(DroneCanardInstance *ins, uint8_t *transfer_id);

static bool processTxRxOnce(DroneCanardInstance *ins, CanardSocketInstance *sock_ins, int timeout_msec);

/****************************************************************************
 * public functions
 ****************************************************************************/
/*!
 * @brief   this function initializes the DRONECAN part
 *
 *          It will create the task to check and update the data
 *
 * @param   none
 *
 * @return  If successful, the function will return zero (OK). Otherwise, an error number will be returned to
 * indicate the error:
 *
 */
int dronecan_initialize(void)
{
    int ret = EXIT_SUCCESS;

    if(!gDronecanInitialized)
    {
        // check for errors
#ifdef ENABLE_DRONECAN_DEBUG_MESSAGES_ON_CONSOLE
        cli_printfWarning("dronecan_initialize WARNING: DroneCAN debug messages on!\n");
#endif

        // get the unique ID
        ret = data_getUniqueid((uintptr_t)&gMy_unique_id[0], sizeof(gMy_unique_id));
        if(ret)
        {
            // output to user
            cli_printfError("dronecan_initialize ERROR: Couldn't get unique id! %d\n", ret);
            return ret;
        }

        if(g_canard_daemon_started)
        {
            cli_printf("canard_main: receive and send task already running\n");
            return EXIT_SUCCESS;
        }

        ret =
            task_create("DRONECAN", DRONECAN_DAEMON_PRIORITY, DRONECAN_DAEMON_STACK_SIZE, DRONECANTask, NULL);


        if(ret < 0)
        {
            int errcode = errno;
            cli_printfError("dronecan_initialize ERROR: Failed to start DRONECAN: %d\n", errcode);
            return EXIT_FAILURE;
        }
        else
        {
            ret = 0;
        }

        // remember it is initialized
        gDronecanInitialized = true;
    }

    return ret;
}

/*!
 * @brief   this function will increase the semaphore so the DRONECAN task will send the BMS status using
 * DRONECAN
 *
 * @return  If successful, the function will return zero (OK). Otherwise, an error number will be returned to
 * indicate the error:
 *
 */
int dronecan_sendBMSStatus(void)
{
    int ret = 0;

    if(!gDronecanInitialized)
    {
        ret = EXIT_FAILURE;
    }
    else
    {
        // signal pol to stop blocking 
        // this will make sure it will send the data when it is time.
        if(dronecan_flushtx() > 0)
        {
            ret = 0;
        }
        else
        {
            ret = -1;
        }
    }

    // return to user
    return ret;
}

/****************************************************************************
 * private functions
 ****************************************************************************/

/**
 * Returns a pseudo random float in the range [0, 1].
 */
static float getRandomFloat(void)
{
    static bool initialized = false;
    if(!initialized) // This is not thread safe, but a race condition here is not harmful.
    {
        initialized = true;
        srand((uint32_t)time(NULL));
    }
    // coverity[dont_call]
    return (float)rand() / (float)RAND_MAX;
}

static void makeNodeStatusMessage(uint8_t buffer[UAVCAN_PROTOCOL_NODESTATUS_MAX_SIZE])
{
    struct uavcan_protocol_NodeStatus status;

    status.uptime_sec = (uint32_t)(getMonotonicTimestampUSec() / 1000000U);
    status.mode       = UAVCAN_PROTOCOL_NODESTATUS_MODE_OPERATIONAL;

    states_t state = data_getMainState();

    if(state == INIT)
    {
        status.mode = UAVCAN_PROTOCOL_NODESTATUS_MODE_INITIALIZATION;
    }

    if(state == FAULT_ON || state == FAULT_OFF)
    {
        status.health = UAVCAN_PROTOCOL_NODESTATUS_HEALTH_ERROR;
    }
    else
    {
        status.health = UAVCAN_PROTOCOL_NODESTATUS_HEALTH_OK;
    }

    uint32_t bit_ofs = 0;
    memset(buffer, 0, UAVCAN_PROTOCOL_NODESTATUS_MAX_SIZE);
    _uavcan_protocol_NodeStatus_encode(buffer, &bit_ofs, &status, DRONECAN_TAO);
}

static void dronecan_param_getvalue(int index, struct uavcan_protocol_param_GetSetResponse *resp)
{
    void *          dataReturn;
    variableTypes_u variable1;
    int             readOnlyVar = 0;
    resp->name.len              = strlen(gGetSetParameters[index]);
    strncpy((char *)resp->name.data, gGetSetParameters[index], sizeof(resp->name.data));

    // check if read only
    readOnlyVar = data_getParameterIfUserReadOnly(index);

    valueType_t paramType = data_getType(index);
    switch(paramType)
    {
        case FLOATVAL: // so somthing with floating points values
            dataReturn = (int32_t *)data_getParameter(index, &resp->value.real_value, NULL);

            if(dataReturn != NULL)
            {
                resp->value.union_tag = REAL_VAL;

                // if not read only
                if(!readOnlyVar)
                {
                    // get the min and max value
                    if(data_getParameterMinMax(
                           index, &resp->min_value.real_value, &resp->max_value.real_value))
                    {
                        // TODO peter, add error?
                        cli_printfError(
                            "dronecan_param_getvalue ERROR: Could not get min max of %d\n", index);
                    }
                    else
                    {
                        /* it went OK */
                        resp->min_value.union_tag = REAL_VAL;
                        resp->max_value.union_tag = REAL_VAL;
                    }

                    /* Set the default value */
                    if(data_getParameterDefault(index, &resp->default_value.real_value, NULL))
                    {
                        cli_printfError(
                            "dronecan_param_getvalue ERROR: Could not get default of %d\n", index);
                    }
                    else
                    {
                        /* It went OK */
                        resp->default_value.union_tag = REAL_VAL;
                    }
                }
            }
            break;

        case STRINGVAL: // do somthing with string values
        {
            // check if string buffer is large enough
            DEBUGASSERT(sizeof(resp->value.string_value.data) >= STRING_MAX_CHARS);

            dataReturn = (int32_t *)data_getParameter(
                index, &resp->value.string_value.data, (uint16_t *)&resp->value.string_value.len);

            if(dataReturn != NULL)
            {
                resp->value.union_tag = STRING_VAL;

                // if not read only
                if(!readOnlyVar)
                {
                    if(data_getParameterDefault(index, &resp->default_value.string_value.data,
                           (uint16_t *)&resp->default_value.string_value.len))
                    {
                        /* Error */
                        cli_printfError(
                            "dronecan_param_getvalue ERROR: Could not get default of %d\n", index);
                    }
                    else
                    {
                        resp->default_value.union_tag = STRING_VAL;
                    }
                }
            }
        }
        break;

        case UINT8VAL:
        {
            /* Get the variable */
            dataReturn = (int32_t *)data_getParameter(index, &variable1.uint8Var, NULL);

            // Check for errors
            if(dataReturn != NULL)
            {
                // set the variable with casting
                resp->value.integer_value = variable1.uint8Var;
                resp->value.union_tag     = INT_VAL;

                // if not read only
                if(!readOnlyVar)
                {
                    if(data_getParameterDefault(index, &variable1.uint8Var, NULL))
                    {
                        /* Error */
                        cli_printfError(
                            "dronecan_param_getvalue ERROR: Could not get default of %d\n", index);
                    }
                    else
                    {
                        /* set the default value */
                        resp->default_value.integer_value = variable1.uint8Var;
                        resp->default_value.union_tag     = INT_VAL;
                    }
                }
            }
        }
        break;
        case UINT16VAL:
        {
            /* Get the variable */
            dataReturn = (int32_t *)data_getParameter(index, &variable1.uint16Var, NULL);

            // Check for errors
            if(dataReturn != NULL)
            {
                // set the variable with casting
                resp->value.integer_value = variable1.uint16Var;
                resp->value.union_tag     = INT_VAL;

                // if not read only
                if(!readOnlyVar)
                {

                    if(data_getParameterDefault(index, &variable1.uint16Var, NULL))
                    {
                        /* Error */
                        cli_printfError(
                            "dronecan_param_getvalue ERROR: Could not get default of %d\n", index);
                    }
                    else
                    {
                        /* set the default value */
                        resp->default_value.integer_value = variable1.uint16Var;
                        resp->default_value.union_tag     = INT_VAL;
                    }
                }
            }
        }
        break;
        case INT32VAL:
        {
            /* Get the variable */
            dataReturn = (int32_t *)data_getParameter(index, &variable1.int32Var, NULL);

            // Check for errors
            if(dataReturn != NULL)
            {
                // set the variable with casting
                resp->value.integer_value = variable1.int32Var;
                resp->value.union_tag     = INT_VAL;

                // if not read only
                if(!readOnlyVar)
                {
                    if(data_getParameterDefault(index, &variable1.int32Var, NULL))
                    {
                        /* Error */
                        cli_printfError(
                            "dronecan_param_getvalue ERROR: Could not get default of %d\n", index);
                    }
                    else
                    {
                        /* set the default value */
                        resp->default_value.integer_value = variable1.int32Var;
                        resp->default_value.union_tag     = INT_VAL;
                    }
                }
            }
        }
        break;
        case UINT64VAL:
        {
            uint64_t uint64Val = 0;
            /* Get the variable */
            dataReturn = (int32_t *)data_getParameter(index, &uint64Val, NULL);

            // Check for errors
            if(dataReturn != NULL)
            {
                // set the variable with casting
                resp->value.integer_value = (int64_t)uint64Val;
                resp->value.union_tag     = INT_VAL;
            }

            // NOTE: Type does not support default
        }
        break;
    }

    // Check for integer
    if(resp->value.union_tag == INT_VAL && !readOnlyVar)
    {
        // get the min and max value
        if(data_getParameterMinMax(index, &resp->min_value.integer_value, &resp->max_value.integer_value))
        {
            cli_printfError("dronecan_param_getvalue ERROR: Could not get min max of %d\n", index);
            // TODO peter, add error?
        }
        else
        {
            // Check for a boolean
            if((paramType == UINT8VAL) && (resp->max_value.integer_value == 1))
            {
                // change to bool
                resp->value.union_tag = BOOL_VAL;
            }

            // set min max to int.
            resp->min_value.union_tag = INT_VAL;
            resp->max_value.union_tag = INT_VAL;
        }
    }
}

static void dronecan_process_param_getset(
    struct uavcan_protocol_param_GetSetRequest *req, struct uavcan_protocol_param_GetSetResponse *resp)
{
    int index = 0;

    if(req->name.len > 0)
    {
        for(index = 0; index < NONE; index++)
        {
            if(strncmp((char *)&req->name.data, gGetSetParameters[index], req->name.len) == 0)
            {
                break;
            }
        }
    }
    else
    {
        index = req->index;
    }

#ifdef ENABLE_DRONECAN_DEBUG_MESSAGES_ON_CONSOLE
        printf("DroneCAN: dronecan_process_param_getset 0x%04x\n", index);
        fflush(stdout); // Will now print everything in the stdout buffer
#endif
    // check if it may be written
    if(index < NONE)
    {
        // Check if dronecan may set the BMS parameter
        if(data_getParameterIfUserReadOnly(index) == 0)
        {
            if(req->value.union_tag == INT_VAL)
            {
                data_setParameter(index, &req->value.integer_value);
            }
            else if(req->value.union_tag == BOOL_VAL)
            {
                data_setParameter(index, &req->value.boolean_value);
            }
            else if(req->value.union_tag == REAL_VAL)
            {
                data_setParameter(index, &req->value.real_value);
            }
            else if(req->value.union_tag == STRING_VAL)
            {
                // Zero terminate string and copy it in data struct for STRING_MAX_CHARS max
                req->value.string_value.data[req->value.string_value.len] = 0;
                data_setParameter(index, &req->value.string_value.data);
            }
        }

        // fetch the parameter to send to dronecan
        dronecan_param_getvalue(index, resp);
    }
    else if(req->index == BMS_RESET_FAULT || index == BMS_RESET_FAULT)
    {
        /* Special DroneCAN parameters */
        const char *bms_reset_param = "BMS_RESET_FAULT";

        resp->value.integer_value         = 0;
        resp->value.union_tag             = INT_VAL;
        resp->default_value.integer_value = 0;
        resp->default_value.union_tag     = INT_VAL;
        resp->min_value.integer_value     = -1;
        resp->min_value.union_tag         = INT_VAL;
        resp->max_value.integer_value     = 1;
        resp->max_value.union_tag         = INT_VAL;

        if(req->name.len > 0)
        {
            if(strncmp((char *)&req->name.data, bms_reset_param, req->name.len) == 0)
            {
                if(req->value.union_tag == INT_VAL && req->value.integer_value == 1)
                {
                    // reset the fault, but and set the status.
                    // resp->value.integer_value = processCLICommand(CLI_RESET);

                    // Make the bms reset command string and call the public cli function to reset the bms
                    const char *resetCommand[] = { bmsString, resetString };
                    resp->value.integer_value  = cli_processCommands(2, (char **)resetCommand);
                }
            }
        }

        resp->name.len = strlen(bms_reset_param);
        strncpy((char *)resp->name.data, bms_reset_param, sizeof(resp->name.data));
    }
}


/**
 * This callback is invoked by the library when a new message or request or response is received.
 */
static void onTransferReceived(CanardInstance *ins, CanardRxTransfer *transfer)
{
    /*
     * Dynamic node ID allocation protocol.
     * Taking this branch only if we don't have a node ID, ignoring otherwise.
     */
    if((canardGetLocalNodeID(ins) == CANARD_BROADCAST_NODE_ID) &&
        (transfer->transfer_type == CanardTransferTypeBroadcast) &&
        (transfer->data_type_id == UAVCAN_NODE_ID_ALLOCATION_DATA_TYPE_ID))
    {
        // Rule C - updating the randomized time interval
        g_send_next_node_id_allocation_request_at = getMonotonicTimestampUSec() +
            UAVCAN_NODE_ID_ALLOCATION_REQUEST_DELAY_OFFSET_USEC +
            (uint64_t)(getRandomFloat() * UAVCAN_NODE_ID_ALLOCATION_RANDOM_TIMEOUT_RANGE_USEC);

        if(transfer->source_node_id == CANARD_BROADCAST_NODE_ID)
        {
            cli_printf("DroneCAN: Allocation request from another allocatee\n");
            g_node_id_allocation_unique_id_offset = 0;
            return;
        }

        // Copying the unique ID from the message
        static const uint8_t UniqueIDBitOffset = 8;
        uint8_t              received_unique_id[UNIQUE_ID_LENGTH_BYTES];
        uint8_t              received_unique_id_len = 0;
        for(; received_unique_id_len < (transfer->payload_len - (UniqueIDBitOffset / 8U));
            received_unique_id_len++)
        {
            assert(received_unique_id_len < UNIQUE_ID_LENGTH_BYTES);
            const uint8_t bit_offset = (uint8_t)(UniqueIDBitOffset + received_unique_id_len * 8U);
            (void)canardDecodeScalar(
                transfer, bit_offset, 8, false, &received_unique_id[received_unique_id_len]);
        }

        // Matching the received UID against the local one
        if(memcmp(received_unique_id, gMy_unique_id, received_unique_id_len) != 0)
        {

#ifdef ENABLE_DRONECAN_INFO_MESSAGES_ON_CONSOLE
            cli_printfError("DroneCAN: Mismatching allocation response from %d:", transfer->source_node_id);
            for(uint8_t i = 0; i < received_unique_id_len; i++)
            {
                cli_printf(" %02x/%02x", received_unique_id[i], gMy_unique_id[i]);
            }
            cli_printf("\n");
#endif
            g_node_id_allocation_unique_id_offset = 0;
            return; // No match, return
        }

        if(received_unique_id_len < UNIQUE_ID_LENGTH_BYTES)
        {
            // The allocator has confirmed part of unique ID, switching to the next stage and updating the
            // timeout.
            g_node_id_allocation_unique_id_offset = received_unique_id_len;
            g_send_next_node_id_allocation_request_at -= UAVCAN_NODE_ID_ALLOCATION_REQUEST_DELAY_OFFSET_USEC;

#ifdef ENABLE_DRONECAN_INFO_MESSAGES_ON_CONSOLE
            cli_printf("DroneCAN: Matching allocation response from %d offset %d\n", transfer->source_node_id,
                g_node_id_allocation_unique_id_offset);
#endif
        }
        else
        {
            // Allocation complete - copying the allocated node ID from the message
            uint8_t allocated_node_id = 0;
            (void)canardDecodeScalar(transfer, 0, 7, false, &allocated_node_id);
            assert(allocated_node_id <= 127);

            canardSetLocalNodeID(ins, allocated_node_id);
            cli_printf("DroneCAN: Node ID %d allocated by %d\n", allocated_node_id, transfer->source_node_id);
        }
    }

    if((transfer->transfer_type == CanardTransferTypeRequest) &&
        (transfer->data_type_id == UAVCAN_PROTOCOL_GETNODEINFO_REQUEST_ID))
    {

#ifdef ENABLE_DRONECAN_INFO_MESSAGES_ON_CONSOLE
        cli_printf("DroneCAN: GetNodeInfo request from %d\n", transfer->source_node_id);
#endif
        uint8_t buffer[UAVCAN_PROTOCOL_GETNODEINFO_RESPONSE_MAX_SIZE];
        memset(buffer, 0, UAVCAN_PROTOCOL_GETNODEINFO_RESPONSE_MAX_SIZE);

        // NodeStatus
        makeNodeStatusMessage(buffer);

        // SoftwareVersion
        buffer[7] = BMS_MAJOR_VERSION_NUMBER;
        buffer[8] = BMS_MINOR_VERSION_NUMBER;
        buffer[9] = 1; // Optional field flags, VCS commit is set

        {
            uint32_t u32 = 0;
            char     short_hash[9];

            // copy the first 4 bytes of the verion build string (5 bytes = 8 of the 10 chars)
            strncpy(short_hash, CONFIG_VERSION_BUILD, 8);

            // 0 terminate after the 8th byte
            short_hash[8] = 0x0;

            // get the 32-bit integer value
            u32 = atoi(short_hash);

            // encode in the buffer
            canardEncodeScalar(buffer, 80, 32, &u32);
        }

        // Image CRC skipped

        // HardwareVersion skipped
        // Major skipped
        // Minor skipped

        {
            int i;

            // loop throught the unique ID array
            for(i = 0; i < 16 && i < UNIQUE_ID_LENGTH_BYTES; i++)
            {
                // copy the unique ID in buffer, starting from byte 24 and for 16 bytes
                buffer[24 + i] = gMy_unique_id[i];
            }
        }

        // Certificate of authenticity skipped

        // Name
        const size_t name_len = strlen(APP_NODE_NAME);
        memcpy(&buffer[41], APP_NODE_NAME, name_len);

        const size_t total_size = 41 + name_len;

        /*
         * Transmitting; in this case we don't have to release the payload because it's empty anyway.
         */
        const int16_t resp_res = canardRequestOrRespond(ins, transfer->source_node_id,
            UAVCAN_PROTOCOL_GETNODEINFO_RESPONSE_SIGNATURE, UAVCAN_PROTOCOL_GETNODEINFO_RESPONSE_ID,
            &transfer->transfer_id, transfer->priority, CanardResponse, &buffer[0], (uint16_t)total_size);
        if(resp_res <= 0)
        {

#ifdef ENABLE_DRONECAN_INFO_MESSAGES_ON_CONSOLE
            cli_printfError("DroneCAN: Could not respond to GetNodeInfo; error %d\n", resp_res);
#endif
        }
#ifdef ENABLE_DRONECAN_DEBUG_MESSAGES_ON_CONSOLE
        printf("DroneCAN: GetNodeInfo request done\n");
        fflush(stdout); // Will now print everything in the stdout buffer
#endif
    }
    if((transfer->transfer_type == CanardTransferTypeRequest) &&
        (transfer->data_type_id == UAVCAN_PROTOCOL_PARAM_GETSET_REQUEST_ID))
    {

#ifdef ENABLE_DRONECAN_DEBUG_MESSAGES_ON_CONSOLE
        printf("DroneCAN: UAVCAN_PROTOCOL_PARAM_GETSET_REQUEST_ID request\n");
        fflush(stdout); // Will now print everything in the stdout buffer
#endif
        uint32_t                                    bit_ofs;
        struct uavcan_protocol_param_GetSetRequest  get_set_req;
        struct uavcan_protocol_param_GetSetResponse get_set_resp;
        uint8_t                                     buff_resp[UAVCAN_PROTOCOL_PARAM_GETSET_REQUEST_MAX_SIZE];

        memset(&get_set_resp, 0, sizeof(get_set_resp));

        bit_ofs = 0;
        _uavcan_protocol_param_GetSetRequest_decode(transfer, &bit_ofs, &get_set_req, DRONECAN_TAO);

        /* Process request */

        dronecan_process_param_getset(&get_set_req, &get_set_resp);

        bit_ofs = 0;
        _uavcan_protocol_param_GetSetResponse_encode(buff_resp, &bit_ofs, &get_set_resp, DRONECAN_TAO);

        /* Transmit response */
        const int16_t resp_res = canardRequestOrRespond(ins, transfer->source_node_id,
            UAVCAN_PROTOCOL_PARAM_GETSET_RESPONSE_SIGNATURE, UAVCAN_PROTOCOL_PARAM_GETSET_RESPONSE_ID,
            &transfer->transfer_id, transfer->priority, CanardResponse, &buff_resp[0],
            (uint16_t)((bit_ofs + 7) / 8));
        if(resp_res <= 0)
        {
#ifdef ENABLE_DRONECAN_INFO_MESSAGES_ON_CONSOLE
            cli_printfError("DroneCAN: Could not respond to GetNodeInfo; error %d\n", resp_res);
#endif
        }

#ifdef ENABLE_DRONECAN_DEBUG_MESSAGES_ON_CONSOLE
        printf("DroneCAN: UAVCAN_PROTOCOL_PARAM_GETSET_REQUEST_ID request done\n");
        fflush(stdout); // Will now print everything in the stdout buffer
#endif
    }
    if((transfer->transfer_type == CanardTransferTypeRequest) &&
        (transfer->data_type_id == UAVCAN_PROTOCOL_PARAM_EXECUTEOPCODE_ID))
    {

        uint32_t                                           bit_ofs;
        struct uavcan_protocol_param_ExecuteOpcodeRequest  opcode_req;
        struct uavcan_protocol_param_ExecuteOpcodeResponse opcode_resp;
        uint8_t buff_resp[UAVCAN_PROTOCOL_PARAM_EXECUTEOPCODE_RESPONSE_MAX_SIZE];
        int     ret = 0;

        memset(&opcode_resp, 0, sizeof(opcode_resp));

        bit_ofs = 0;
        _uavcan_protocol_param_ExecuteOpcodeRequest_decode(transfer, &bit_ofs, &opcode_req, DRONECAN_TAO);

        /* Process request */
        if(opcode_req.opcode == UAVCAN_PROTOCOL_PARAM_EXECUTEOPCODE_REQUEST_OPCODE_SAVE)
        {
            // Make the bms save command string and call the public cli function
            const char *saveCommand[] = { bmsString, saveString };
            ret                       = cli_processCommands(2, (char **)saveCommand);
        }
        else if(opcode_req.opcode == UAVCAN_PROTOCOL_PARAM_EXECUTEOPCODE_REQUEST_OPCODE_ERASE)
        {
            // Make the bms default command string and call the public cli function
            const char *defaultCommand[] = { bmsString, defaultString };
            ret                          = cli_processCommands(2, (char **)defaultCommand);

            // check if it went OK
            if(ret != 0)
            {
                // Make the bms save command string and call the public cli function
                const char *saveCommand[] = { bmsString, saveString };
                ret                       = cli_processCommands(2, (char **)saveCommand);
            }
        }

        // if OK (0, it will set ok to 1)
        opcode_resp.ok = (ret == 0);

        bit_ofs = 0;
        _uavcan_protocol_param_ExecuteOpcodeResponse_encode(buff_resp, &bit_ofs, &opcode_resp, DRONECAN_TAO);

        /* Transmit response */
        const int16_t resp_res = canardRequestOrRespond(ins, transfer->source_node_id,
            UAVCAN_PROTOCOL_PARAM_EXECUTEOPCODE_RESPONSE_SIGNATURE,
            UAVCAN_PROTOCOL_PARAM_EXECUTEOPCODE_RESPONSE_ID, &transfer->transfer_id, transfer->priority,
            CanardResponse, &buff_resp[0], (uint16_t)((bit_ofs + 7) / 8));
        if(resp_res <= 0)
        {
#ifdef ENABLE_DRONECAN_INFO_MESSAGES_ON_CONSOLE
            cli_printfError("DroneCAN: Could not respond to GetNodeInfo; error %d\n", resp_res);
#endif
        }
    }
    if((transfer->transfer_type == CanardTransferTypeRequest) &&
        (transfer->data_type_id == UAVCAN_PROTOCOL_RESTARTNODE_REQUEST_ID))
    {

        uint32_t                                  bit_ofs;
        struct uavcan_protocol_RestartNodeRequest restart_req;

        bit_ofs = 0;
        _uavcan_protocol_RestartNodeRequest_decode(transfer, &bit_ofs, &restart_req, DRONECAN_TAO);

        /* Process request */
        if(restart_req.magic_number == UAVCAN_PROTOCOL_RESTARTNODE_REQUEST_MAGIC_NUMBER)
        {
            boardctl(BOARDIOC_RESET, 0);
        }
    }
}


/**
 * This callback is invoked by the library when it detects beginning of a new transfer on the bus that can be
 * received by the local node. If the callback returns true, the library will receive the transfer. If the
 * callback returns false, the library will ignore the transfer. All transfers that are addressed to other
 * nodes are always ignored.
 */
static bool shouldAcceptTransfer(const CanardInstance *ins, uint64_t *out_data_type_signature,
    uint16_t data_type_id, CanardTransferType transfer_type, uint8_t source_node_id)
{
    (void)source_node_id;

    if(canardGetLocalNodeID(ins) == CANARD_BROADCAST_NODE_ID)
    {
        /*
         * If we're in the process of allocation of dynamic node ID, accept only relevant transfers.
         */
        if((transfer_type == CanardTransferTypeBroadcast) &&
            (data_type_id == UAVCAN_NODE_ID_ALLOCATION_DATA_TYPE_ID))
        {
            *out_data_type_signature = UAVCAN_NODE_ID_ALLOCATION_DATA_TYPE_SIGNATURE;
            return true;
        }
    }
    else
    {
        if((transfer_type == CanardTransferTypeRequest) &&
            (data_type_id == UAVCAN_PROTOCOL_GETNODEINFO_REQUEST_ID))
        {
            *out_data_type_signature = UAVCAN_PROTOCOL_GETNODEINFO_REQUEST_SIGNATURE;
            return true;
        }
        else if((transfer_type == CanardTransferTypeRequest) &&
            (data_type_id == UAVCAN_PROTOCOL_PARAM_GETSET_REQUEST_ID))
        {
            *out_data_type_signature = UAVCAN_PROTOCOL_PARAM_GETSET_REQUEST_SIGNATURE;
            return true;
        }
        else if((transfer_type == CanardTransferTypeRequest) &&
            (data_type_id == UAVCAN_PROTOCOL_PARAM_EXECUTEOPCODE_ID))
        {
            *out_data_type_signature = UAVCAN_PROTOCOL_PARAM_EXECUTEOPCODE_SIGNATURE;
            return true;
        }
        else if((transfer_type == CanardTransferTypeRequest) &&
            (data_type_id == UAVCAN_PROTOCOL_RESTARTNODE_ID))
        {
            *out_data_type_signature = UAVCAN_PROTOCOL_RESTARTNODE_SIGNATURE;
            return true;
        }
    }

    return false;
}

static int32_t dronecan_task_initialize(DroneCanardInstance *ins, CanardSocketInstance *sock_ins)
{
    uint8_t can_fd = 0;
    uint8_t nodeID;
    void *  dataReturn;
    void *  memoryPool;
    int32_t canBitrate, canFdBitrate;

    // get the node ID
    dataReturn = (int32_t *)data_getParameter(CAN_FD_MODE, &can_fd, NULL);

    // check for error
    if(dataReturn == NULL)
    {
        // set the default value
        can_fd = CAN_FD_MODE_DEFAULT;

        cli_printfError("DRONECANTask ERROR: couldn't get canfd mode! setting default\n");
    }

    // mask the variable to be sure
    can_fd &= 1;

    // TODO CAN FD Support hooks are in but not all hw supports thus testing is hard

    // get the node ID
    dataReturn = (int32_t *)data_getParameter(DRONECAN_NODE_STATIC_ID, &nodeID, NULL);

    // check for error
    if(dataReturn == NULL)
    {
        // set the default value
        nodeID = DRONECAN_NODE_STATIC_ID_DEFAULT;

        cli_printfError("DRONECANTask ERROR: couldn't get node id! setting default\n");
    }

    /* Open the CAN device for reading */
    socketcanOpen(sock_ins, CAN_DEVICE, can_fd);

    pDfds[0].fd     = sock_ins->s;
    pDfds[0].events = POLLIN;

    // get the bitrates
    dataReturn = (int32_t *)data_getParameter(CAN_BITRATE, &canBitrate, NULL);
    // check for error
    if(dataReturn == NULL)
    {
        // set the default value
        canBitrate = CAN_BITRATE_DEFAULT;

        cli_printfError("DRONECANTask ERROR: couldn't get canBitrate! setting default\n");
    }

    // get the CAN FD bitrate
    dataReturn = (int32_t *)data_getParameter(CAN_FD_BITRATE, &canFdBitrate, NULL);
    // check for error
    if(dataReturn == NULL)
    {
        // set the default value
        canFdBitrate = CAN_FD_BITRATE_DEFAULT;

        cli_printfError("DRONECANTask ERROR: couldn't get canFdBitrate! setting default\n");
    }

    // set the bitrates
    if(socketcanSetBitrate(sock_ins, CAN_DEVICE, canBitrate, canFdBitrate))
    {
        cli_printfError("DRONECANTask ERROR: couldn't set bitrates!\n");
        return -1;
    }

    if(sock_ins->s < 0)
    {
        cli_printfError("canard_daemon: ERROR: open %s failed: %d\n", CAN_DEVICE, errno);
        return -2;
    }

    memoryPool = memalign(MEMORY_POOL_ALIGNMENT, MEMORY_POOL_SIZE);

    if(memoryPool == NULL)
    {
        cli_printfError("canard_daemon: memory pool allocation size %i failed\n", MEMORY_POOL_SIZE);
        return -2;
    }

    /*
     * Initializing the Libcanard instance.
     */
    dronecanardInit(ins, memoryPool, MEMORY_POOL_SIZE, onTransferReceived, shouldAcceptTransfer, NULL);

    /*
     * Performing the dynamic node ID allocation procedure.
     */
    static const uint8_t PreferredNodeID =
        CANARD_BROADCAST_NODE_ID; ///< This can be made configurable, obviously

    g_node_id_allocation_unique_id_offset = 0;

    uint8_t node_id_allocation_transfer_id = 0;

    if(nodeID >= CANARD_MIN_NODE_ID && nodeID <= CANARD_MAX_NODE_ID)
    {
        canardSetLocalNodeID(ins, nodeID);
    }

    // TODO maybe add wait here until first dronecan_sendBMSStatus() to continue initialize

    // wait for the bms application to trigger, before continuing 
    while(1)
    {
        // the event was triggered to publish the messages
        if((poll(&pDfds[1], 1, -1) > 0) && pDfds[1].revents & POLLIN)
        {
            // break the while looop to continue
            break;
        }
    }

    cli_printf("DroneCAN: Waiting for dynamic node ID allocation\n");

    // Waiting for dynamic node ID allocation...
    while(canardGetLocalNodeID(ins) == CANARD_BROADCAST_NODE_ID)
    {
        g_send_next_node_id_allocation_request_at = getMonotonicTimestampUSec() +
            UAVCAN_NODE_ID_ALLOCATION_REQUEST_DELAY_OFFSET_USEC +
            (uint64_t)(getRandomFloat() * UAVCAN_NODE_ID_ALLOCATION_RANDOM_TIMEOUT_RANGE_USEC);

        while((getMonotonicTimestampUSec() < g_send_next_node_id_allocation_request_at) &&
            (canardGetLocalNodeID(ins) == CANARD_BROADCAST_NODE_ID))
        {
            // process Tx and wait for Rx and timeout after 1ms
            processTxRxOnce(ins, sock_ins, 1);
        }

        if(canardGetLocalNodeID(ins) != CANARD_BROADCAST_NODE_ID)
        {
            break;
        }

        // Structure of the request is documented in the DSDL definition
        // See http://uavcan.org/Specification/6._Application_level_functions/#dynamic-node-id-allocation
        uint8_t allocation_request[CANARD_CAN_FRAME_MAX_DATA_LEN - 1];
        allocation_request[0] = (uint8_t)(PreferredNodeID << 1U);

        if(g_node_id_allocation_unique_id_offset == 0)
        {
            allocation_request[0] |= 1; // First part of unique ID
        }

        static const uint8_t MaxLenOfUniqueIDInRequest = 6;
        uint8_t uid_size = (uint8_t)(UNIQUE_ID_LENGTH_BYTES - g_node_id_allocation_unique_id_offset);
        if(uid_size > MaxLenOfUniqueIDInRequest)
        {
            uid_size = MaxLenOfUniqueIDInRequest;
        }

        // Paranoia time
        assert(g_node_id_allocation_unique_id_offset < UNIQUE_ID_LENGTH_BYTES);
        assert(uid_size <= MaxLenOfUniqueIDInRequest);
        assert(uid_size > 0);
        assert((uid_size + g_node_id_allocation_unique_id_offset) <= UNIQUE_ID_LENGTH_BYTES);

        memmove(&allocation_request[1], &gMy_unique_id[g_node_id_allocation_unique_id_offset], uid_size);

        // Broadcasting the request
        const int16_t bcast_res = canardBroadcast(ins, UAVCAN_NODE_ID_ALLOCATION_DATA_TYPE_SIGNATURE,
            UAVCAN_NODE_ID_ALLOCATION_DATA_TYPE_ID, &node_id_allocation_transfer_id,
            CANARD_TRANSFER_PRIORITY_LOW, &allocation_request[0], (uint16_t)(uid_size + 1));
        if(bcast_res < 0)
        {
            cli_printfError(
                "DroneCAN ERROR: Could not broadcast dynamic node ID allocation request; error %d\n",
                bcast_res);
        }

        // Preparing for timeout; if response is received, this value will be updated from the callback.
        g_node_id_allocation_unique_id_offset = 0;

        // check if it will loop
        if(canardGetLocalNodeID(ins) == CANARD_BROADCAST_NODE_ID)
        {
            // sleep for a small time, just to give other lower priority tasks a chance
            usleep(100);
        }
    }

    // Set CAN HW ID filter to only get interrupts from CAN messages for this device
    socketcanSetHwCanFilterID(sock_ins, CAN_DEVICE, canardGetLocalNodeID(ins));

#ifdef ENABLE_DRONECAN_INFO_MESSAGES_ON_CONSOLE
    cli_printf("DroneCAN: Dynamic node ID allocation complete [%d]\n", canardGetLocalNodeID(ins));
#endif

    return 0;
}

static void transmitNodeStatus(DroneCanardInstance *ins)
{
    uint8_t buffer[UAVCAN_PROTOCOL_NODESTATUS_MAX_SIZE];
    makeNodeStatusMessage(buffer);

    static uint8_t transfer_id; // Note that the transfer ID variable MUST BE STATIC (or heap-allocated)!

    const int16_t bc_res =
        canardBroadcast(ins, UAVCAN_PROTOCOL_NODESTATUS_SIGNATURE, UAVCAN_PROTOCOL_NODESTATUS_ID,
            &transfer_id, CANARD_TRANSFER_PRIORITY_LOW, buffer, UAVCAN_PROTOCOL_NODESTATUS_MAX_SIZE);
    if(bc_res <= 0)
    {
        cli_printfError("DroneCAN ERROR: Could not broadcast node status; error %d\n", bc_res);
    }
}

static void pubPowerBatteryContinuous(DroneCanardInstance *ins, uint8_t *transfer_id)
{
    uint32_t bit_ofs = 0;
    uint16_t enable  = 0;
    uint8_t  buffer[ARDUPILOT_EQUIPMENT_POWER_BATTERYCONTINUOUS_MAX_SIZE];
    uint8_t  statusFlagBits = 0;

    // Get the variable if battery continuous
    void *dataReturn = (int32_t *)data_getParameter(DRONECAN_BAT_CONTINUOUS, &enable, NULL);

    if(dataReturn == NULL || enable == 0)
    {
        return;
    }

    memset(buffer, 0, ARDUPILOT_EQUIPMENT_POWER_BATTERYCONTINUOUS_MAX_SIZE);

    struct ardupilot_equipment_power_BatteryContinuous batContinuous;
    memset(&batContinuous, 0, sizeof(struct ardupilot_equipment_power_BatteryContinuous));

    {
        uint8_t batterySensorEnabled;

        // get the sensor enable variable
        dataReturn = (int32_t *)data_getParameter(SENSOR_ENABLE, &batterySensorEnabled, NULL);

        // check for error
        if(dataReturn == NULL)
        {
            // set status flag
            statusFlagBits |= STATUS_BMS_ERROR_BIT;

            // set the default value
            batterySensorEnabled = SENSOR_ENABLE_DEFAULT;

            // error output
            cli_printfError("DroneCAN ERROR: could not get sensor-enable!\n");
        }

        // if the sensor is enabled
        if(batterySensorEnabled)
        {
            // set the battery temperature
            dataReturn = (int32_t *)data_getParameter(C_BATT, &batContinuous.temperature_cells, NULL);

            // check for error
            if(dataReturn == NULL)
            {
                // set status flag
                statusFlagBits |= STATUS_BMS_ERROR_BIT;

                // set the default value
                batContinuous.temperature_cells = NAN;

                // error output
                cli_printfError("DroneCAN ERROR: could not get c-batt!\n");
            }
        }
        else
        {
            batContinuous.temperature_cells = NAN;
        }
    }

    // set the PCB temperature
    // Battery PCB temperature (likely output FET(s) or current sense resistor), NAN: field not provided
    // TODO: maybe check if sense resistor temp is more?
    dataReturn = (int32_t *)data_getParameter(C_T, &batContinuous.temperature_pcb, NULL);

    // check for error
    if(dataReturn == NULL)
    {
        // set status flag
        statusFlagBits |= STATUS_BMS_ERROR_BIT;

        // set the default value
        batContinuous.temperature_pcb = NAN;

        // error output
        cli_printfError("DroneCAN ERROR: could not get c-t!\n");
    }

    // Application dependent, NAN: field not provided
    // set the Analog front end temperature
    dataReturn = (int32_t *)data_getParameter(C_AFE, &batContinuous.temperature_other, NULL);

    // check for error
    if(dataReturn == NULL)
    {
        // set status flag
        statusFlagBits |= STATUS_BMS_ERROR_BIT;

        // set the default value
        batContinuous.temperature_other = NAN;

        // error output
        cli_printfError("DroneCAN ERROR: could not get c-afe!\n");
    }

    {
        float current;

        // Positive: defined as a discharge current. Negative: defined as a charging current, NAN: field not
        // provided set the current (average?)
        dataReturn = (int32_t *)data_getParameter(I_BATT_AVG, &current, NULL);

        // check for error
        if(dataReturn == NULL)
        {
            // set status flag
            statusFlagBits |= STATUS_BMS_ERROR_BIT;

            // set the default value
            batContinuous.current = NAN;

            // error output
            cli_printfError("DroneCAN ERROR: could not get i-batt-avg!\n");
        }
        else
        {
            // convert negative to positive current
            batContinuous.current = -1 * current;
        }
    }

    // set the battery voltage
    dataReturn = (int32_t *)data_getParameter(V_OUT, &batContinuous.voltage, NULL);

    // check for error
    if(dataReturn == NULL)
    {
        // set status flag
        statusFlagBits |= STATUS_BMS_ERROR_BIT;

        // set the default value
        batContinuous.voltage = NAN;

        // error output
        cli_printfError("DroneCAN ERROR: could not get v-out!\n");
    }

    {
        uint8_t soc;

        // set the state of charge
        dataReturn = (int32_t *)data_getParameter(S_CHARGE, &soc, NULL);

        // check for error
        if(dataReturn == NULL)
        {
            // set status flag
            statusFlagBits |= STATUS_BMS_ERROR_BIT;

            // set the wrong value
            batContinuous.state_of_charge = NAN; // S_CHARGE_DEFAULT;

            // error output
            cli_printfError("DroneCAN ERROR: could not get s-charge!\n");
        }
        else
        {
            batContinuous.state_of_charge = (float)soc;
        }
    }

    // set the slot id
    dataReturn = (int32_t *)data_getParameter(BATT_ID, &batContinuous.slot_id, NULL);

    // check for error
    if(dataReturn == NULL)
    {
        // set status flag
        statusFlagBits |= STATUS_BMS_ERROR_BIT;

        // set the default value
        batContinuous.slot_id = 0;

        // error output
        cli_printfError("DroneCAN ERROR: could not get batt-id!\n");
    }

    {
        float a_full;
        // get the full capacity (in Ah)
        dataReturn = (int32_t *)data_getParameter(A_FULL, &a_full, NULL);

        // check for error
        if(dataReturn == NULL)
        {
            // set status flag
            statusFlagBits |= STATUS_BMS_ERROR_BIT;

            // set the default value
            batContinuous.capacity_consumed = NAN;

            // error output
            cli_printfError("DroneCAN ERROR: could not get a-full!\n");
        }
        else
        {
            float a_rem;

            // get the rem capacity (in Ah)
            dataReturn = (int32_t *)data_getParameter(A_REM, &a_rem, NULL);

            if(dataReturn == NULL)
            {
                // set status flag
                statusFlagBits |= STATUS_BMS_ERROR_BIT;

                // set the default value
                batContinuous.capacity_consumed = NAN;

                // error output
                cli_printfError("DroneCAN ERROR: could not get a-rem!\n");
            }
            else
            {
                // calculate the capacity consumed in Ah
                batContinuous.capacity_consumed = a_full - a_rem;

                // set relative to full flag
                batContinuous.status_flags |=
                    ARDUPILOT_EQUIPMENT_POWER_BATTERYCONTINUOUS_STATUS_FLAG_CAPACITY_RELATIVE_TO_FULL;
            }
        }
    }

    {
        // get the BMS main state
        states_t mainState = data_getMainState();

        // check what the state is to set some flags
        switch(mainState)
        {
            case INIT:
            case SELF_TEST:
                break;
            case NORMAL:
            case SLEEP:
            case OCV:
                // BMS is ready to use
                batContinuous.status_flags |=
                    ARDUPILOT_EQUIPMENT_POWER_BATTERYCONTINUOUS_STATUS_FLAG_READY_TO_USE;
                break;
            case CHARGE:
                // set charging
                batContinuous.status_flags |=
                    ARDUPILOT_EQUIPMENT_POWER_BATTERYCONTINUOUS_STATUS_FLAG_CHARGING;

                // get the charge state
                charge_states_t chargeState = data_getChargeState();

                // check for balaning (could be that it is done)
                if((chargeState == CHARGE_CB) || (chargeState == RELAXATION))
                {
                    batContinuous.status_flags |=
                        ARDUPILOT_EQUIPMENT_POWER_BATTERYCONTINUOUS_STATUS_FLAG_CELL_BALANCING;
                }
                else if(chargeState == CHARGE_COMPLETE)
                {
                    batContinuous.status_flags |=
                        ARDUPILOT_EQUIPMENT_POWER_BATTERYCONTINUOUS_STATUS_FLAG_READY_TO_USE;
                }

                break;
            case FAULT_ON:
            case FAULT_OFF:
                // since button needs to be pressed, it needs service
                batContinuous.status_flags |=
                    ARDUPILOT_EQUIPMENT_POWER_BATTERYCONTINUOUS_STATUS_FLAG_REQUIRES_SERVICE;

                // get the bms fault
                {
                    int bmsFault = data_getBmsFault();

                    // check which fault is active and add the reason to the status
                    if(bmsFault & BMS_OV_FAULT)
                    {
                        batContinuous.status_flags |=
                            ARDUPILOT_EQUIPMENT_POWER_BATTERYCONTINUOUS_STATUS_FLAG_FAULT_OVER_VOLT;
                    }
                    if(bmsFault & BMS_UV_FAULT)
                    {
                        batContinuous.status_flags |=
                            ARDUPILOT_EQUIPMENT_POWER_BATTERYCONTINUOUS_STATUS_FLAG_FAULT_UNDER_VOLT;
                    }
                    if(bmsFault & BMS_OT_FAULT)
                    {
                        batContinuous.status_flags |=
                            ARDUPILOT_EQUIPMENT_POWER_BATTERYCONTINUOUS_STATUS_FLAG_FAULT_OVER_TEMP;
                    }
                    if(bmsFault & BMS_UT_FAULT)
                    {
                        batContinuous.status_flags |=
                            ARDUPILOT_EQUIPMENT_POWER_BATTERYCONTINUOUS_STATUS_FLAG_FAULT_UNDER_TEMP;
                    }
                    if(bmsFault & BMS_OC_FAULT)
                    {
                        batContinuous.status_flags |=
                            ARDUPILOT_EQUIPMENT_POWER_BATTERYCONTINUOUS_STATUS_FLAG_FAULT_OVER_CURRENT;
                    }
                }
                break;
            case SELF_DISCHARGE:
                batContinuous.status_flags |=
                    (ARDUPILOT_EQUIPMENT_POWER_BATTERYCONTINUOUS_STATUS_FLAG_AUTO_DISCHARGING |
                        ARDUPILOT_EQUIPMENT_POWER_BATTERYCONTINUOUS_STATUS_FLAG_CELL_BALANCING);
                break;
            case DEEP_SLEEP:
                break;
            default:
                cli_printfError("DroneCAN ERROR: Not supported main state: %d=%s, please add here\n",
                    mainState, gStatesArray[mainState]);
                break;
        }
    }
    {
        uint8_t stateOfHealth, batt_eol;

        // get the state of health
        dataReturn = (int32_t *)data_getParameter(S_HEALTH, &stateOfHealth, NULL);

        if(dataReturn == NULL)
        {
            stateOfHealth = 100;

            // set status flag
            statusFlagBits |= STATUS_BMS_ERROR_BIT;

            // error output
            cli_printfError("DroneCAN ERROR: could not get s-health!\n");
        }

        // get the battery eol percentage
        dataReturn = (int32_t *)data_getParameter(BATT_EOL, &batt_eol, NULL);

        if(dataReturn == NULL)
        {
            batt_eol = BATT_EOL_DEFAULT;

            // set status flag
            statusFlagBits |= STATUS_BMS_ERROR_BIT;

            // error output
            cli_printfError("DroneCAN ERROR: could not get batt-eol!\n");
        }


        // check for a bad battery
        if(stateOfHealth <= batt_eol)
        {
            // set the BAD battery flag
            batContinuous.status_flags |= ARDUPILOT_EQUIPMENT_POWER_BATTERYCONTINUOUS_STATUS_FLAG_BAD_BATTERY;
        }
    }
    {
        uint8_t inFlightStatus;

        // check for protections enabled
        // get the s_in_flight flag (this is set by the sw when protections are off, could be that it will be
        // off because of flight-mode == 1)
        dataReturn = (int32_t *)data_getParameter(S_IN_FLIGHT, &inFlightStatus, NULL);

        if(dataReturn == NULL)
        {
            // set to 1 just in case as it might be off
            inFlightStatus = 1;

            // set status flag
            statusFlagBits |= STATUS_BMS_ERROR_BIT;

            // error output
            cli_printfError("DroneCAN ERROR: could not get s-in-flight!\n");
        }

        // if not in flight, protections are enabled
        if(!inFlightStatus)
        {
            // set the flag
            batContinuous.status_flags |=
                ARDUPILOT_EQUIPMENT_POWER_BATTERYCONTINUOUS_STATUS_FLAG_PROTECTIONS_ENABLED;
        }
    }
    {
        // get the overcurrent GPIO state to check for short current
        if(gpio_readPin(OVERCURRENT))
        {
            batContinuous.status_flags |=
                ARDUPILOT_EQUIPMENT_POWER_BATTERYCONTINUOUS_STATUS_FLAG_FAULT_SHORT_CIRCUIT;
        }
    }

    _ardupilot_equipment_power_BatteryContinuous_encode(buffer, &bit_ofs, &batContinuous, DRONECAN_TAO);

    const int16_t bc_res = canardBroadcast(ins, ARDUPILOT_EQUIPMENT_POWER_BATTERYCONTINUOUS_SIGNATURE,
        ARDUPILOT_EQUIPMENT_POWER_BATTERYCONTINUOUS_ID, transfer_id, CANARD_TRANSFER_PRIORITY_LOW, buffer,
        (uint16_t)((bit_ofs + 7) / 8));

    if(bc_res <= 0)
    {
        cli_printfError("DroneCAN: Could not broadcast BatteryContinuous; error %d\n", bc_res);
    }
}

static void pubPowerBatteryPeriodic(DroneCanardInstance *ins, uint8_t *transfer_id)
{
    uint32_t bit_ofs    = 0;
    uint16_t enable     = 0;
    void *   dataReturn = (int32_t *)data_getParameter(DRONECAN_BAT_PERIODIC, &enable, NULL);
    uint8_t  buffer[ARDUPILOT_EQUIPMENT_POWER_BATTERYPERIODIC_MAX_SIZE];

    if(dataReturn == NULL || enable == 0)
    {
        return;
    }

    memset(buffer, 0, ARDUPILOT_EQUIPMENT_POWER_BATTERYPERIODIC_MAX_SIZE);

    struct ardupilot_equipment_power_BatteryPeriodic batPeriodic;
    memset(&batPeriodic, 0, sizeof(struct ardupilot_equipment_power_BatteryPeriodic));

    // set the name (manufacturer_product)
    sprintf((char *)batPeriodic.name.data, "NXP_%s", (char *)MODEL_NAME_DEFAULT);
    batPeriodic.name.len = strlen((char *)batPeriodic.name.data);

    {
        uint64_t uniqueId;
        int      i;

        // get the model id (uint64)
        dataReturn = (int32_t *)data_getParameter(MODEL_ID, &uniqueId, NULL);

        if(dataReturn == NULL)
        {
            // set to 1 just in case as it might be off
            uniqueId = 0;

            // error output
            cli_printfError("DroneCAN ERROR: could not get model-id!\n");
        }
        // check if the user did not set the unique ID
        else if(uniqueId == 0)
        {
            // copyt the first x bytes to uint64_t number
            for(i = 0; i < UNIQUE_ID_LENGTH_BYTES && i < sizeof(uniqueId); i++)
            {
                // add the unique id number to the variable
                uniqueId += (gMy_unique_id[i] << (8 * i));
            }
        }

        // convert to a string and get the length
        sprintf((char *)batPeriodic.serial_number.data, "%" PRIu64, uniqueId);
        batPeriodic.serial_number.len = strlen((char *)batPeriodic.serial_number.data);
    }

    // Not implemented: Manufacturer date

    // get the model id (uint64)
    dataReturn = (int32_t *)data_getParameter(A_FACTORY, &batPeriodic.design_capacity, NULL);

    if(dataReturn == NULL)
    {
        // set to 0 just in case
        batPeriodic.design_capacity = 0;

        // error output
        cli_printfError("DroneCAN ERROR: could not get a-factory!\n");
    }

    // get the number of cells
    dataReturn = (int32_t *)data_getParameter(N_CELLS, &batPeriodic.cells_in_series, NULL);

    if(dataReturn == NULL)
    {
        // set to 0 just in case
        batPeriodic.cells_in_series = 0;

        // error output
        cli_printfError("DroneCAN ERROR: could not get n-cells!\n");
    }

    // get the nominal voltage
    dataReturn = (int32_t *)data_getParameter(V_CELL_NOMINAL, &batPeriodic.nominal_voltage, NULL);

    if(dataReturn == NULL)
    {
        // set to 0 just in case
        batPeriodic.nominal_voltage = 0;

        // error output
        cli_printfError("DroneCAN ERROR: could not get v-cell-nominal!\n");
    }

    // get the value to fill in the next field
    dataReturn = (int32_t *)data_getParameter(V_CELL_UV, &batPeriodic.discharge_minimum_voltage, NULL);

    if(dataReturn == NULL)
    {
        // set to 0 just in case
        batPeriodic.discharge_minimum_voltage = 0;

        // error output
        cli_printfError("DroneCAN ERROR: could not get V-cell-uv!\n");
    }

    // get the value to fill in the next field
    dataReturn = (int32_t *)data_getParameter(V_CELL_UV, &batPeriodic.charging_minimum_voltage, NULL);

    if(dataReturn == NULL)
    {
        // set to 0 just in case
        batPeriodic.charging_minimum_voltage = 0;

        // error output
        cli_printfError("DroneCAN ERROR: could not get V-cell-uv!\n");
    }

    // get the value to fill in the next field
    dataReturn = (int32_t *)data_getParameter(V_CELL_OV, &batPeriodic.charging_maximum_voltage, NULL);

    if(dataReturn == NULL)
    {
        // set to 0 just in case
        batPeriodic.charging_maximum_voltage = 0;

        // error output
        cli_printfError("DroneCAN ERROR: could not get v-cell-ov!\n");
    }

    // get the value to fill in the next field
    dataReturn = (int32_t *)data_getParameter(I_CHARGE_MAX, &batPeriodic.charging_maximum_current, NULL);

    if(dataReturn == NULL)
    {
        // set to 0 just in case
        batPeriodic.charging_maximum_current = 0;

        // error output
        cli_printfError("DroneCAN ERROR: could not get i-charge-max!\n");
    }

    // get the value to fill in the next field
    dataReturn = (int32_t *)data_getParameter(I_OUT_MAX, &batPeriodic.discharge_maximum_current, NULL);

    if(dataReturn == NULL)
    {
        // set to 0 just in case
        batPeriodic.discharge_maximum_current = 0;

        // error output
        cli_printfError("DroneCAN ERROR: could not get i-out-max!\n");
    }

    // get the value to fill in the next field
    dataReturn = (int32_t *)data_getParameter(I_OUT_MAX, &batPeriodic.discharge_maximum_burst_current, NULL);

    if(dataReturn == NULL)
    {
        // set to 0 just in case
        batPeriodic.discharge_maximum_burst_current = 0;

        // error output
        cli_printfError("DroneCAN ERROR: could not get i-out-max!\n");
    }

    // get the value to fill in the next field
    dataReturn = (int32_t *)data_getParameter(A_FULL, &batPeriodic.full_charge_capacity, NULL);

    if(dataReturn == NULL)
    {
        // set to 0 just in case
        batPeriodic.full_charge_capacity = NAN;

        // error output
        cli_printfError("DroneCAN ERROR: could not get a-full!\n");
    }

    // get the value to fill in the next field
    dataReturn = (int32_t *)data_getParameter(N_CHARGES_FULL, &batPeriodic.cycle_count, NULL);

    if(dataReturn == NULL)
    {
        // set to 0 just in case
        batPeriodic.cycle_count = UINT16_MAX;

        // error output
        cli_printfError("DroneCAN ERROR: could not get n-charges!\n");
    }

    // get the value to fill in the next field
    dataReturn = (int32_t *)data_getParameter(S_HEALTH, &batPeriodic.state_of_health, NULL);

    if(dataReturn == NULL)
    {
        // set to 0 just in case
        batPeriodic.state_of_health = UINT8_MAX;

        // error output
        cli_printfError("DroneCAN ERROR: could not get s-health!\n");
    }

    _ardupilot_equipment_power_BatteryPeriodic_encode(buffer, &bit_ofs, &batPeriodic, DRONECAN_TAO);

    const int16_t bc_res = canardBroadcast(ins, ARDUPILOT_EQUIPMENT_POWER_BATTERYPERIODIC_SIGNATURE,
        ARDUPILOT_EQUIPMENT_POWER_BATTERYPERIODIC_ID, transfer_id, CANARD_TRANSFER_PRIORITY_LOW, buffer,
        (uint16_t)((bit_ofs + 7) / 8));

    if(bc_res <= 0)
    {
        cli_printfError("DroneCAN: Could not broadcast BatteryPeriodic; error %d\n", bc_res);
    }
}

static void pubPowerBatteryCells(DroneCanardInstance *ins, uint8_t *transfer_id)
{
    uint32_t bit_ofs    = 0;
    uint16_t enable     = 0;
    void *   dataReturn = (int32_t *)data_getParameter(DRONECAN_BAT_CELLS, &enable, NULL);
    uint8_t  buffer[ARDUPILOT_EQUIPMENT_POWER_BATTERYCELLS_MAX_SIZE];
    int      i;

    if(dataReturn == NULL || enable == 0)
    {
        return;
    }

    memset(buffer, 0, ARDUPILOT_EQUIPMENT_POWER_BATTERYCELLS_MAX_SIZE);

    struct ardupilot_equipment_power_BatteryCells batCells;
    memset(&batCells, 0, sizeof(struct ardupilot_equipment_power_BatteryCells));

    // get the number of cells
    dataReturn = (int32_t *)data_getParameter(N_CELLS, &batCells.voltages.len, NULL);

    if(dataReturn == NULL)
    {
        // set to 0 just in case
        batCells.voltages.len = 0;

        // error output
        cli_printfError("DroneCAN ERROR: could not get n-cells!\n");
    }

    /* Loop through all the cells */
    for(i = 0; i < batCells.voltages.len; i++)
    {
        // get the cell voltage
        dataReturn =
            (int32_t *)data_getParameter((parameterKind_t)(V_CELL1 + i), &batCells.voltages.data[i], NULL);

        // check for error
        if(dataReturn == NULL)
        {
            // set to 0 just in case
            batCells.voltages.data[i] = 0;

            // error output
            cli_printfError("DroneCAN ERROR: could not get v-cell%D!\n", i + 1);
        }
    }

    // set the index to 0
    batCells.index = 0;

    _ardupilot_equipment_power_BatteryCells_encode(buffer, &bit_ofs, &batCells, DRONECAN_TAO);

    const int16_t bc_res = canardBroadcast(ins, ARDUPILOT_EQUIPMENT_POWER_BATTERYCELLS_SIGNATURE,
        ARDUPILOT_EQUIPMENT_POWER_BATTERYCELLS_ID, transfer_id, CANARD_TRANSFER_PRIORITY_LOW, buffer,
        (uint16_t)((bit_ofs + 7) / 8));

    if(bc_res <= 0)
    {
        cli_printfError("DroneCAN: Could not broadcast BatteryCells; error %d\n", bc_res);
    }
}

static void pubPowerBatteryInfo(DroneCanardInstance *ins, uint8_t *transfer_id)
{
    uint32_t bit_ofs    = 0;
    uint16_t enable     = 0;
    void *   dataReturn = (int32_t *)data_getParameter(DRONECAN_BAT_INFO, &enable, NULL);
    uint8_t  buffer[UAVCAN_EQUIPMENT_POWER_BATTERYINFO_MAX_SIZE];
    float    floatVal, floatVal2, floatMinValue, floatMaxValue;
    uint8_t  uint8Val, statusFlagBits = 0, nCells, i;
    uint64_t modelId;
    uint16_t modelNameSize = 0;

    if(dataReturn == NULL || enable == 0)
    {
        return;
    }

    memset(buffer, 0, UAVCAN_EQUIPMENT_POWER_BATTERYINFO_MAX_SIZE);

    struct uavcan_equipment_power_BatteryInfo batteryInfo;
    memset(&batteryInfo, 0, sizeof(struct uavcan_equipment_power_BatteryInfo));

    {
        uint8_t batterySensorEnabled;

        // get the sensor enable variable
        dataReturn = (int32_t *)data_getParameter(SENSOR_ENABLE, &batterySensorEnabled, NULL);

        // check for error
        if(dataReturn == NULL)
        {
            // set status flag
            statusFlagBits |= STATUS_BMS_ERROR_BIT;

            // set the default value
            uint8Val = SENSOR_ENABLE_DEFAULT;

            // error output
            cli_printfError("DroneCAN ERROR: could not get sensor-enable!\n");
        }

        // if the sensor is enabled
        if(batterySensorEnabled)
        {
            // set the battery temperature
            dataReturn = (int32_t *)data_getParameter(C_BATT, &batteryInfo.temperature, NULL);

            // check for error
            if(dataReturn == NULL)
            {
                // set status flag
                statusFlagBits |= STATUS_BMS_ERROR_BIT;

                // set the default value
                batteryInfo.temperature = -CONSTANTS_ABSOLUTE_NULL_CELSIUS;

                // error output
                cli_printfError("DroneCAN ERROR: could not get c-batt!\n");
            }
            else
            {
                batteryInfo.temperature += CONSTANTS_ABSOLUTE_NULL_CELSIUS;
            }
        }
        else
        {
            batteryInfo.temperature = -CONSTANTS_ABSOLUTE_NULL_CELSIUS;
        }
    }

    // set the voltage
    dataReturn = (int32_t *)data_getParameter(V_OUT, &batteryInfo.voltage, NULL);

    // check for error
    if(dataReturn == NULL)
    {
        // set status flag
        statusFlagBits |= STATUS_BMS_ERROR_BIT;

        // set the default value
        batteryInfo.voltage = V_OUT_DEFAULT;

        // error output
        cli_printfError("DroneCAN ERROR: could not get v-out!\n");
    }

    // set the current (average?)
    dataReturn = (int32_t *)data_getParameter(I_BATT_AVG, &batteryInfo.current, NULL);

    // check for error
    if(dataReturn == NULL)
    {
        // set status flag
        statusFlagBits |= STATUS_BMS_ERROR_BIT;

        // set the default value
        batteryInfo.current = I_BATT_AVG_DEFAULT;

        // error output
        cli_printfError("DroneCAN ERROR: could not get i-batt-avg!\n");
    }

    // set the average power 10 sec
    dataReturn = (int32_t *)data_getParameter(P_AVG, &batteryInfo.average_power_10sec, NULL);

    // check for error
    if(dataReturn == NULL)
    {
        // set status flag
        statusFlagBits |= STATUS_BMS_ERROR_BIT;

        // set the default value
        batteryInfo.average_power_10sec = P_AVG_DEFAULT;

        // error output
        cli_printfError("DroneCAN ERROR: could not get p-avg!\n");
    }

    // get the remaining capacity (in Ah)
    dataReturn = (int32_t *)data_getParameter(A_REM, &floatVal, NULL);

    // check for error
    if(dataReturn == NULL)
    {
        // set status flag
        statusFlagBits |= STATUS_BMS_ERROR_BIT;

        // set the default value
        floatVal = A_REM_DEFAULT;

        // error output
        cli_printfError("DroneCAN ERROR: could not get a-rem!\n");
    }

    // get the cell nominal voltage
    dataReturn = (int32_t *)data_getParameter(V_CELL_NOMINAL, &floatVal2, NULL);

    // check for error
    if(dataReturn == NULL)
    {
        // set status flag
        statusFlagBits |= STATUS_BMS_ERROR_BIT;

        // set the default value
        floatVal2 = V_CELL_NOMINAL_DEFAULT;

        // error output
        cli_printfError("DroneCAN ERROR: could not get v-batt!\n");
    }

    // get the number of cells
    dataReturn = (int32_t *)data_getParameter(N_CELLS, &nCells, NULL);

    // check for error
    if(dataReturn == NULL)
    {
        // set status flag
        statusFlagBits |= STATUS_BMS_ERROR_BIT;

        // set the default value
        nCells = N_CELLS_DEFAULT;

        // error output
        cli_printfError("DroneCAN ERROR: could not get n-cells!\n");
    }

    // calculate the energy in Wh (Ah*Vcell*cells)
    floatVal = floatVal * floatVal2 * nCells;

    // set the energy in wh
    batteryInfo.remaining_capacity_wh = floatVal;

    // get the full charge capacity
    dataReturn = (int32_t *)data_getParameter(A_FULL, &floatVal, NULL);

    // check for error
    if(dataReturn == NULL)
    {
        // set status flag
        statusFlagBits |= STATUS_BMS_ERROR_BIT;

        // set the default value
        floatVal = A_FULL_DEFAULT;

        // error output
        cli_printfError("DroneCAN ERROR: could not get a-full!\n");
    }

    // calculate and set the full charge capacity in wh (A_FULL * V_CELL_NOMINAL * nCells)
    batteryInfo.full_charge_capacity_wh = floatVal * floatVal2 * nCells;

    // set the hours to full charge
    dataReturn = (int32_t *)data_getParameter(T_FULL, &batteryInfo.hours_to_full_charge, NULL);

    // check for error
    if(dataReturn == NULL)
    {
        // set status flag
        statusFlagBits |= STATUS_BMS_ERROR_BIT;

        // set the default value
        batteryInfo.hours_to_full_charge = T_FULL_DEFAULT;

        // error output
        cli_printfError("DroneCAN ERROR: could not get t-full!\n");
    }

    // set the state of health
    dataReturn = (int32_t *)data_getParameter(S_HEALTH, &batteryInfo.state_of_health_pct, NULL);

    // check for error
    if(dataReturn == NULL)
    {
        // set status flag
        statusFlagBits |= STATUS_BMS_ERROR_BIT;

        // set the default value
        batteryInfo.state_of_health_pct = S_HEALTH_DEFAULT;

        // error output
        cli_printfError("DroneCAN ERROR: could not get s-health!\n");
    }

    // set the state of charge
    dataReturn = (int32_t *)data_getParameter(S_CHARGE, &batteryInfo.state_of_charge_pct, NULL);

    // check for error
    if(dataReturn == NULL)
    {
        // set status flag
        statusFlagBits |= STATUS_BMS_ERROR_BIT;

        // set the default value
        batteryInfo.state_of_charge_pct = S_CHARGE_DEFAULT;

        // error output
        cli_printfError("DroneCAN ERROR: could not get s-charge!\n");
    }

    // set the state of charge stdev value
    batteryInfo.state_of_charge_pct_stdev = 0;

    // set the battery id
    dataReturn = (int32_t *)data_getParameter(BATT_ID, &batteryInfo.battery_id, NULL);

    // check for error
    if(dataReturn == NULL)
    {
        // set status flag
        statusFlagBits |= STATUS_BMS_ERROR_BIT;

        // set the default value
        batteryInfo.battery_id = BATT_ID_DEFAULT;

        // error output
        cli_printfError("DroneCAN ERROR: could not get batt-id!\n");
    }

    // get the model id
    dataReturn = (int32_t *)data_getParameter(MODEL_ID, &modelId, NULL);

    // check for error
    if(dataReturn == NULL)
    {
        // set status flag
        statusFlagBits |= STATUS_BMS_ERROR_BIT;

        // set the default value
        modelId = MODEL_ID_DEFAULT;

        // error output
        cli_printfError("DroneCAN ERROR: could not get model-id!\n");
    }

    // set the model_id
    batteryInfo.model_instance_id = (uint32_t)modelId;

    // set the model name and the size
    dataReturn = (int32_t *)data_getParameter(MODEL_NAME, &batteryInfo.model_name.data[0], &modelNameSize);

    // check for error
    if(dataReturn == NULL)
    {
        // set status flag
        statusFlagBits |= STATUS_BMS_ERROR_BIT;

        // set the default value
        strcpy((char *)batteryInfo.model_name.data, (char *)MODEL_NAME_DEFAULT);

        // set the size to 0
        modelNameSize = 0;

        // error output
        cli_printfError("DroneCAN ERROR: could not get model-name!\n");
    }

    // set the model name size
    batteryInfo.model_name.len = strlen((char *)batteryInfo.model_name.data);

    // reset the status flags but set the error if there is one
    batteryInfo.status_flags =
        ((uint16_t)statusFlagBits << 3) & UAVCAN_EQUIPMENT_POWER_BATTERYINFO_STATUS_FLAG_BMS_ERROR; // do last

    // get the battery status flags
    dataReturn = (int32_t *)data_getParameter(S_FLAGS, &statusFlagBits, NULL);

    // check if it is the unknown value
    if(statusFlagBits == S_FLAGS_UKNOWN)
    {
        // reset it to 0
        statusFlagBits = 0;
    }

    // check for error
    if(dataReturn == NULL)
    {
        // set the default value
        statusFlagBits = 0;

        // set status flag
        statusFlagBits |= STATUS_BMS_ERROR_BIT;

        // error output
        cli_printfError("DroneCAN ERROR: could not get s-flags!\n");
    }

    // set the correct bits for everything except in use, charing, charged, temp hot, temp cold
    batteryInfo.status_flags |= (((uint16_t)statusFlagBits & 0x00FC) << 3);

    // get the sleep current th
    dataReturn = (int32_t *)data_getParameter(I_SLEEP_OC, &uint8Val, NULL);

    // check for error
    if(dataReturn == NULL)
    {
        // set status flag
        statusFlagBits |= STATUS_BMS_ERROR_BIT;

        // set the default value
        uint8Val = I_SLEEP_OC_DEFAULT;

        // error output
        cli_printfError("DroneCAN ERROR: could not get i-sleep-oc!\n");
    }

    // check if in use
    if((batteryInfo.current > ((float)uint8Val / 1000.0)) ||
        (batteryInfo.current < ((float)uint8Val / -1000.0)))
    {
        // set the battery in use on
        batteryInfo.status_flags |= UAVCAN_EQUIPMENT_POWER_BATTERYINFO_STATUS_FLAG_IN_USE;
    }

    // check if charging
    if(data_getMainState() == CHARGE)
    {
        // set the charging bit high
        batteryInfo.status_flags |= UAVCAN_EQUIPMENT_POWER_BATTERYINFO_STATUS_FLAG_CHARGING;

        // check if done charging
        // check current charge state
        if(data_getChargeState() == CHARGE_COMPLETE)
        {
            // set the charging done bit
            batteryInfo.status_flags |= UAVCAN_EQUIPMENT_POWER_BATTERYINFO_STATUS_FLAG_CHARGED;
        }
    }

    // check for temperature error
    if(statusFlagBits & (1 << STATUS_TEMP_ERROR_BIT))
    {
        // get the highest and lowest temperature

        // get the sensor enable variable
        dataReturn = (int32_t *)data_getParameter(SENSOR_ENABLE, &uint8Val, NULL);

        // check for error
        if(dataReturn == NULL)
        {
            // set status flag
            statusFlagBits |= STATUS_BMS_ERROR_BIT;

            // set the default value
            uint8Val = SENSOR_ENABLE_DEFAULT;

            // error output
            cli_printfError("DroneCAN ERROR: could not get sensor-enable!\n");
        }

        // check if it is used
        if(uint8Val)
        {
            // get the battery temperature
            dataReturn = (int32_t *)data_getParameter(C_BATT, &floatVal, NULL);

            // check for error
            if(dataReturn == NULL)
            {
                // set status flag
                statusFlagBits |= STATUS_BMS_ERROR_BIT;

                // set the value that it will be overwritten
                floatMinValue = 100;
                floatMaxValue = -40;

                // error output
                cli_printfError("DroneCAN ERROR: could not get c-batt!\n");
            }
            else
            {
                // set the values with the battery temperature
                floatMinValue = floatVal;
                floatMaxValue = floatVal;
            }
        }
        else
        {
            // set the value that it will be overwritten
            floatMinValue = 100;
            floatMaxValue = -40;
        }

        // loop through the cells to find the min and max
        for(i = 0; i < 3; i++)
        {
            // get the cell voltage
            dataReturn = (int32_t *)data_getParameter((parameterKind_t)(C_AFE + i), &floatVal, NULL);

            // check for error
            if(dataReturn == NULL)
            {
                // set status flag
                statusFlagBits |= STATUS_BMS_ERROR_BIT;

                // set the default value
                floatVal = V_CELL1_DEFAULT;

                // error output
                cli_printfError("DroneCAN ERROR: could not get c-afe + %d!\n", i);
            }
            else
            {
                // check for min
                if(floatVal < floatMinValue)
                {
                    // set new min
                    floatMinValue = floatVal;
                }

                // check for max
                if(floatVal > floatMaxValue)
                {
                    // set new max
                    floatMaxValue = floatVal;
                }
            }
        }

        // get battery over and under temperature values
        dataReturn = (int32_t *)data_getParameter(C_CELL_OT, &floatVal, NULL);

        // check for error
        if(dataReturn == NULL)
        {
            // set status flag
            statusFlagBits |= STATUS_BMS_ERROR_BIT;

            // set the default value
            floatVal = C_CELL_OT_DEFAULT;

            // error output
            cli_printfError("DroneCAN ERROR: could not get c-cell-ot!\n");
        }

        // get the cell under temperature
        dataReturn = (int32_t *)data_getParameter(C_CELL_UT, &floatVal2, NULL);

        // check for error
        if(dataReturn == NULL)
        {
            // set status flag
            statusFlagBits |= STATUS_BMS_ERROR_BIT;

            // set the default value
            floatVal2 = C_CELL_UT_DEFAULT;

            // error output
            cli_printfError("DroneCAN ERROR: could not get c-cell-ut!\n");
        }

        // check for too hot or too cold error
        if((floatVal - floatMaxValue) < (floatMinValue - floatVal2))
        {
            // set the battery temp hot error on
            batteryInfo.status_flags |= UAVCAN_EQUIPMENT_POWER_BATTERYINFO_STATUS_FLAG_TEMP_HOT;
        }
        else
        {
            // set the battery temp cold error on
            batteryInfo.status_flags |= UAVCAN_EQUIPMENT_POWER_BATTERYINFO_STATUS_FLAG_TEMP_COLD;
        }
    }

    _uavcan_equipment_power_BatteryInfo_encode(buffer, &bit_ofs, &batteryInfo, DRONECAN_TAO);

    const int16_t bc_res = canardBroadcast(ins, UAVCAN_EQUIPMENT_POWER_BATTERYINFO_SIGNATURE,
        UAVCAN_EQUIPMENT_POWER_BATTERYINFO_ID, transfer_id, CANARD_TRANSFER_PRIORITY_LOW, buffer,
        (uint16_t)((bit_ofs + 7) / 8));

    if(bc_res <= 0)
    {
        cli_printfError("DroneCAN: Could not broadcast BatteryInfo; error %d\n", bc_res);
    }
}
static void pubPowerBatteryInfoAux(DroneCanardInstance *ins, uint8_t *transfer_id)
{
    uint32_t bit_ofs    = 0;
    uint16_t enable     = 0;
    void *   dataReturn = (int32_t *)data_getParameter(DRONECAN_BAT_INFO_AUX, &enable, NULL);
    uint8_t  buffer[ARDUPILOT_EQUIPMENT_POWER_BATTERYINFOAUX_MAX_SIZE];
    int      i;

    if(dataReturn == NULL || enable == 0)
    {
        return;
    }

    memset(buffer, 0, ARDUPILOT_EQUIPMENT_POWER_BATTERYINFOAUX_MAX_SIZE);

    struct ardupilot_equipment_power_BatteryInfoAux batInfoAux;
    memset(&batInfoAux, 0, sizeof(struct ardupilot_equipment_power_BatteryInfoAux));

    // not implemented: over_discharge_count, max_current, timestamp

    // get the number of cells
    dataReturn = (int32_t *)data_getParameter(N_CELLS, &batInfoAux.voltage_cell.len, NULL);

    if(dataReturn == NULL)
    {
        // set to 0 just in case
        batInfoAux.voltage_cell.len = 0;

        // error output
        cli_printfError("DroneCAN ERROR: could not get n-cells!\n");
    }

    /* Loop through all the cells */
    for(i = 0; i < batInfoAux.voltage_cell.len; i++)
    {
        // get the cell voltage
        dataReturn = (int32_t *)data_getParameter(
            (parameterKind_t)(V_CELL1 + i), &batInfoAux.voltage_cell.data[i], NULL);

        // check for error
        if(dataReturn == NULL)
        {
            // set to 0 just in case
            batInfoAux.voltage_cell.data[i] = 0;

            // error output
            cli_printfError("DroneCAN ERROR: could not get v-cell%D!\n", i + 1);
        }
    }

    // get the value to fill in the next field
    dataReturn = (int32_t *)data_getParameter(N_CHARGES_FULL, &batInfoAux.cycle_count, NULL);

    if(dataReturn == NULL)
    {
        // set to 0 just in case
        batInfoAux.cycle_count = UINT16_MAX;

        // error output
        cli_printfError("DroneCAN ERROR: could not get n-charges!\n");
    }

    // get the nominal voltage
    dataReturn = (int32_t *)data_getParameter(V_CELL_NOMINAL, &batInfoAux.nominal_voltage, NULL);

    if(dataReturn == NULL)
    {
        // set to 0 just in case
        batInfoAux.nominal_voltage = 0;

        // error output
        cli_printfError("DroneCAN ERROR: could not get v-cell-nominal!\n");
    }

    // this nominal_voltage is nominal battery voltage, so multiply batInfoAux.nominal_voltage with n-cells
    batInfoAux.nominal_voltage *= batInfoAux.voltage_cell.len;

    // check if in the fault on state (it will move to fault off)
    if(data_getMainState() == FAULT_ON)
    {
        // indicate it will power off
        batInfoAux.is_powering_off = true;
    }

    // set the slot id
    dataReturn = (int32_t *)data_getParameter(BATT_ID, &batInfoAux.battery_id, NULL);

    // check for error
    if(dataReturn == NULL)
    {
        // set the default value
        batInfoAux.battery_id = BATT_ID_DEFAULT;

        // error output
        cli_printfError("DroneCAN ERROR: could not get batt-id!\n");
    }

    _ardupilot_equipment_power_BatteryInfoAux_encode(buffer, &bit_ofs, &batInfoAux, DRONECAN_TAO);

    const int16_t bc_res = canardBroadcast(ins, ARDUPILOT_EQUIPMENT_POWER_BATTERYINFOAUX_SIGNATURE,
        ARDUPILOT_EQUIPMENT_POWER_BATTERYINFOAUX_ID, transfer_id, CANARD_TRANSFER_PRIORITY_LOW, buffer,
        (uint16_t)((bit_ofs + 7) / 8));

    if(bc_res <= 0)
    {
        cli_printfError("DroneCAN: Could not broadcast BatteryInfoAux; error %d\n", bc_res);
    }
}

/****************************************************************************
 * Name: DRONECANTask
 *
 * Description:
 *
 ****************************************************************************/

static int DRONECANTask(int argc, char *argv[])
{
    DroneCanardInstance  ins;
    CanardSocketInstance sock_ins;
    uint16_t             countBS = 10000, countBP = 10000;
    void *               dataReturn;
    uint16_t             t_meas;
    static int           efd;
    uint8_t              BatteryContinuousTransferId;
    uint8_t              BatteryPeriodicTransferId;
    uint8_t              BatteryCellsTransferId;
    uint8_t              BatteryInfoTransferId;
    uint8_t              BatteryInfoAuxTransferId;
    bool                 needPublish = false;


    // initialize eventfd to signal select while reading
    if((efd = eventfd(0, 0)) < 0)
    {
        // output to user
        cli_printfError("DRONECANTask ERROR: Couldn't initialize eventfd!\n");

        // return to the user
        return -1;
    }

    if(fs_getfilep(efd, &gDroneCANEventfp) < 0)
    {
        cli_printfError("DRONECANTask ERROR: Couldn't initialize gDroneCANEventfp!\n");
    }

    // Setup pollfd for receive
    pDfds[1].fd     = efd;
    pDfds[1].events = POLLIN;

    // initialize the task and check if ok
    if(dronecan_task_initialize(&ins, &sock_ins) == 0)
    {
        // if the deamon is started
        g_canard_daemon_started = true;

        // loop endlessly
        for(;;)
        {
            // check the BMS application wants to send the BMS data
            if(needPublish)
            {
                // get the measurment time
                dataReturn = (int32_t *)data_getParameter(T_MEAS, &t_meas, NULL);

                // check for error
                if(dataReturn == NULL)
                {
                    // set the default value
                    t_meas = T_MEAS_DEFAULT;

                    // error output
                    cli_printfError("DRONECAN ERROR: could not get t-meas!\n");
                }

                // check if at least 1 seconds is passed
                if(++countBS >= (1000 / t_meas))
                {
                    // transmit the node status
                    transmitNodeStatus(&ins);

                    // make the battery continuous message and add it to the buffer
                    pubPowerBatteryContinuous(&ins, &BatteryContinuousTransferId);

                    // make the battery info message and add it to the buffer
                    pubPowerBatteryInfo(&ins, &BatteryInfoTransferId);

                    // reset count
                    countBS = 0;
                }

                // check if the 5 seconds have passed
                if(++countBP >= (5000 / t_meas))
                {
                    // make the battery periodic message and add it to the buffer
                    pubPowerBatteryPeriodic(&ins, &BatteryPeriodicTransferId);

                    // make the battery cells message and add it to the buffer
                    pubPowerBatteryCells(&ins, &BatteryCellsTransferId);

                    // make the battery info auxilary message and add it to the buffer
                    pubPowerBatteryInfoAux(&ins, &BatteryInfoAuxTransferId);

                    // reset count
                    countBP = 0;
                }
            }

            // process the TX and RX buffer
            // And check if the BMS wants to publisch the BMS data
            needPublish = processTxRxOnce(&ins, &sock_ins, 4000);
        }
    }

    // terminate the deamon
    g_canard_daemon_started = false;
    cli_printfWarning("canard_daemon: Terminating!\n");
    fflush(stdout);
    return -1;
}


/*!
 * @brief   this function will flush the can TX
 *
 * @return  If successful, the function will return zero (OK). Otherwise, an error number will be returned to
 * indicate the error:
 *
 */
int dronecan_flushtx(void)
{
    eventfd_t value = 1ULL;

    return file_write(gDroneCANEventfp, &value, sizeof(value));
}


/****************************************************************************
 * Name: processTxRxOnce
 *
 * Description:
 *   Transmits all frames from the TX queue, receives up to one frame.
 *
 ****************************************************************************/

bool processTxRxOnce(DroneCanardInstance *ins, CanardSocketInstance *sock_ins, int timeout_msec)
{
    bool publish = false; 

    // Transmitting
    for(const CanardCANFrame *txf = NULL; (txf = canardPeekTxQueue(ins)) != NULL;)
    {
        uint64_t      transmission_deadline = getMonotonicTimestampUSec() + 1000 * 10;
        const int16_t tx_res                = socketcanDroneCANTransmit(sock_ins, txf, transmission_deadline);
        if(tx_res < 0) // Failure - drop the frame and report
        {
            canardPopTxQueue(ins);
            cli_printfError(
                "DroneCAN: Transmit error %d, frame dropped, errno '%s'\n", tx_res, strerror(errno));
        }
        else if(tx_res > 0) // Success - just drop the frame
        {
            canardPopTxQueue(ins);
        }
        else // Timeout - just exit and try again later
        {
            break;
        }
    }

    if(poll(pDfds, 2, timeout_msec) > 0)
    {
        // received messages
        if(pDfds[0].revents & POLLIN)
        {
            // Receiving
            CanardCANFrame rx_frame;
            uint64_t       timestamp;
            const int32_t  rx_res = socketcanDroneCANReceive(sock_ins, &rx_frame, &timestamp);
            if(rx_res < 0) // Failure - report
            {
                // Check if the fault is not EAGAIN (could be because of FD frame)
                if(rx_res != -EAGAIN)
                {
                    cli_printfError("DroneCAN: Receive error %d, errno '%s'\n", rx_res, strerror(errno));
                }
            }
            else if(rx_res > 0) // Success - process the frame
            {
                canardHandleRxFrame(ins, &rx_frame, timestamp);
            }
            else
            {
                ; // Timeout - nothing to do
            }
        }

        // the event was triggered to publish the messages
        if(pDfds[1].revents & POLLIN)
        {
            eventfd_t value;
            file_read(gDroneCANEventfp, &value, sizeof(value));
            publish = true;
        }
    }

    // return what to do
    return publish;
}
