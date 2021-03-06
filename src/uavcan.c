/****************************************************************************
 * nxp_bms/BMS_v1/src/uavcan.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <canard.h>
#include <canard_dsdl.h>

#include <sched.h>

#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <string.h>
#include <assert.h>
#include <errno.h>

#include <net/if.h>
#include <sys/time.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/random.h>

#include <poll.h>

#include <nuttx/can.h>
#include <netpacket/can.h>

#include <semaphore.h>

#include "uavcan.h"
#include "cli.h"

#include "socketcan.h"
#include "o1heap.h"

#include "data.h"

#include "pnp.h"
#include "portid.h"

#define NUNAVUT_ASSERT
#include "nunavut/support/serialization.h"
#include "reg/drone/srv/battery/Status_0_1.h"
#include "reg/drone/srv/battery/Parameters_0_1.h"
#include "uavcan/node/Heartbeat_1_0.h"
#include "uavcan/node/GetInfo_1_0.h"

/****************************************************************************
 * Defines
 ****************************************************************************/
#ifndef SEM_OCCUPIED_ERROR
#define SEM_OCCUPIED_ERROR 255
#endif

#define ONE_SEC_IN_NS 							1000000000
#define MS_TO_NS_MULT 							1000000

#define APP_NODE_NAME                            "org.nxp.bms" //"org.uavcan.libcanardv1.nuttx.demo" //CONFIG_EXAMPLES_LIBCANARDV1_APP_NODE_NAME

#define UAVCAN_NODE_HEALTH_OK                    0
#define UAVCAN_NODE_HEALTH_WARNING               1
#define UAVCAN_NODE_HEALTH_ERROR                 2
#define UAVCAN_NODE_HEALTH_CRITICAL              3

#define UAVCAN_NODE_MODE_OPERATIONAL             0
#define UAVCAN_NODE_MODE_INITIALIZATION          1

#define UAVCAN_GET_NODE_INFO_RESPONSE_MAX_SIZE   ((3015 + 7) / 8)
#define UAVCAN_GET_NODE_INFO_DATA_TYPE_SIGNATURE 0xee468a8121c46a9e
#define UAVCAN_GET_NODE_INFO_DATA_TYPE_ID        1

#define UNIQUE_ID_LENGTH_BYTES                   16

#define LIBCANARDV1_DAEMON_PRIORITY 			100
#define LIBCANARDV1_DAEMON_STACK_SIZE 			4000
#define CAN_DEVICE 								"can0"

#define CELSIUS_TO_KELVIN						272.15
#define AMPERE_HOURS_TO_COULOMB 				3600

/****************************************************************************
 * Types
 ****************************************************************************/

/****************************************************************************
 * private data
 ****************************************************************************/ 
pthread_t gCanTID;

static sem_t gUavcanSem;

static bool gUavcanInitialized = false; 


CanardRxSubscription heartbeat_subscription;
CanardRxSubscription my_subscription;

/* Arena for memory allocation, used by the library */

#define O1_HEAP_SIZE 4096 //CONFIG_EXAMPLES_LIBCANARDV1_NODE_MEM_POOL_SIZE

/* Temporary development UAVCAN topic service ID to publish/subscribe from */
#define PORT_ID                                  4421
#define TOPIC_SIZE                               512

O1HeapInstance *my_allocator;
static uint8_t uavcan_heap[O1_HEAP_SIZE]
__attribute__((aligned(O1HEAP_ALIGNMENT)));

static bool g_canard_daemon_started;

static uint8_t my_message_transfer_id;  // Must be static or heap-allocated to retain state between calls.

struct pollfd gFd;
/****************************************************************************
 * private Functions declerations 
 ****************************************************************************/
//! @brief the UAVCAN deamon task
static int UAVCANTask(int argc, char *argv[]);

static void *memAllocate(CanardInstance *const ins, const size_t amount);

static void memFree(CanardInstance *const ins, void *const pointer);

uint64_t getMonotonicTimestampUSec(void);

void BatteryStatusToTransmitBuffer(CanardInstance *ins);//, uint64_t timestamp_usec);

void BatteryParametersToTransmitBuffer(CanardInstance *ins);

static void processReceivedTransfer(CanardTransfer *receive);

void processTxRxOnce(CanardInstance *ins, CanardSocketInstance *sock_ins, int timeout_msec);


/****************************************************************************
 * public functions
 ****************************************************************************/
/*!
 * @brief 	this function initializes the UAVCAN part
 *
 * 			It will create the task to check and update the data		
 *
 * @return 	If successful, the function will return zero (OK). Otherwise, an error number will be returned to indicate the error:
 *
 */
int uavcan_initialize(void)
{
	int ret = EXIT_SUCCESS;

	if(!gUavcanInitialized)
	{
		// initialize semaphore
		ret = sem_init(&gUavcanSem, 0, 0);
		sem_setprotocol(&gUavcanSem, SEM_PRIO_NONE);

		// check for errors
		if(ret)
	    {
	    	// output to user
	    	cli_printfError("uavcan_initialize ERROR: Couldn't initialize semaphore!\n");

	       	// return to the user 
	        return ret;
	    }

		//cli_printf("canard_main: Starting canard_daemon\n");

		if (g_canard_daemon_started) {
			cli_printf("canard_main: receive and send task already running\n");
			return EXIT_SUCCESS;
		}

		ret = task_create("UAVCAN", LIBCANARDV1_DAEMON_PRIORITY,
				  LIBCANARDV1_DAEMON_STACK_SIZE, UAVCANTask,
				  NULL);

		gCanTID = ret;

		if (ret < 0) {
			int errcode = errno;
			cli_printfError("uavcan_initialize ERROR: Failed to start UAVCAN: %d\n",
			       errcode);
			return EXIT_FAILURE;
		}
		else
		{
			ret = 0;
		}

		// say it is initialized
		gUavcanInitialized = true;

		//cli_printf("canard_main: UAVCAN started\n");
	}

	// return 
	return ret;
}

/*!
 * @brief 	this function will increase the semaphore so the UAVCAN task will send the BMS status using UAVCAN	
 *
 * @return 	If successful, the function will return zero (OK). Otherwise, an error number will be returned to indicate the error:
 *
 */
int uavcan_sendBMSStatus(void)
{
	int ret = 0;
	int semValue;

	if(!gUavcanInitialized)
	{
		ret = EXIT_FAILURE;
	}
	else
	{
 		// check if semaphore needs to be posted
	    sem_getvalue(&gUavcanSem, &semValue);

	    //cli_printf("sem: %d\n", semValue);

	    // check if needed 
	    if(!semValue)
	    {
	        // increase the semaphore
	        ret = sem_post(&gUavcanSem);
	        //cli_printf("posting sem\n");

	        // check for errors
	        if(ret)
	        {
	        	cli_printfError("uavcan_sendBMSStatus ERROR: couldn't post sem! %d\n", ret);
	        }
	    }

	    // signal pol to stop blocking
	    pthread_kill(gCanTID, SIGUSR1);
	}

	// return to user
	return ret;
}

/****************************************************************************
 * private functions
 ****************************************************************************/


//TODO move this to a seperate file probably

int32_t set_battery_status_port_id(uavcan_register_Value_1_0* value){
    if(uavcan_register_Value_1_0_is_natural16_(value) && value->natural16.value.count == 1) { // Natural 16
        //TODO check validity
        void* dataReturn;
        dataReturn = (int32_t*)data_setParameter(UAVCAN_BS_SUB_ID, &value->natural16.value.elements[0]);
        if(dataReturn == NULL)
        {
            return -UAVCAN_REGISTER_ERROR_SERIALIZATION;
        }
        return 0;
    }
    return -UAVCAN_REGISTER_ERROR_SERIALIZATION;
}

uavcan_register_Value_1_0 get_battery_status_port_id() {
	void* dataReturn;
    uavcan_register_Value_1_0 value;
    
    dataReturn = (int32_t*)data_getParameter(UAVCAN_BS_SUB_ID, &value.natural16.value.elements[0], NULL);
    if(dataReturn == NULL)
    {
        value._tag_ = 0; // Empty
    }
    value.natural16.value.count = 1;
    value._tag_ = 10; // TODO does nunavut generate ENUM/defines for this??
    return value;
}


int32_t uavcan_task_initialize(CanardInstance* ins, CanardSocketInstance* sock_ins) {
    int errval = 0;
	uint8_t can_fd = 0;
	uint8_t nodeID;
	void* dataReturn;
	int32_t canBitrate, canFdBitrate;

	my_allocator = o1heapInit(&uavcan_heap, O1_HEAP_SIZE, NULL, NULL);

	if (my_allocator == NULL) {
		cli_printf("o1heapInit failed with size %d\n", O1_HEAP_SIZE);
		return -2;		
	}

	*ins = canardInit(&memAllocate, &memFree);

	// get the node ID
	dataReturn = (int32_t*)data_getParameter(UAVCAN_FD_MODE, &can_fd, NULL);

    // check for error 
    if(dataReturn == NULL)
    {
    	// set the default value
    	can_fd = UAVCAN_FD_MODE_DEFAULT;

    	cli_printfError("UAVCANTask ERROR: couldn't get canfd mode! setting default\n");
    }

    // mask the variable to be sure
	can_fd &= 1;

	// check if CAN FD is used 
	if (can_fd) {
		ins->mtu_bytes = CANARD_MTU_CAN_FD;

	} else {
		ins->mtu_bytes = CANARD_MTU_CAN_CLASSIC;
	}

	// get the node ID
	dataReturn = (int32_t*)data_getParameter(UAVCAN_NODE_STATIC_ID, &nodeID, NULL);

    // check for error 
    if(dataReturn == NULL)
    {
    	// set the default value
    	nodeID = UAVCAN_NODE_STATIC_ID_DEFAULT;

    	cli_printfError("UAVCANTask ERROR: couldn't get node id! setting default\n");
    }

	/* Open the CAN device for reading */
	socketcanOpen(sock_ins, CAN_DEVICE, can_fd);

	// get the bitrates
	dataReturn = (int32_t*)data_getParameter(UAVCAN_BITRATE, &canBitrate, NULL);
	// check for error 
    if(dataReturn == NULL)
    {
    	// set the default value
    	canBitrate = UAVCAN_BITRATE_DEFAULT;

    	cli_printfError("UAVCANTask ERROR: couldn't get canBitrate! setting default\n");
    }

    // get the CAN FD bitrate
	dataReturn = (int32_t*)data_getParameter(UAVCAN_FD_BITRATE, &canFdBitrate, NULL);
	// check for error 
    if(dataReturn == NULL)
    {
    	// set the default value
    	canFdBitrate = UAVCAN_FD_BITRATE_DEFAULT;

    	cli_printfError("UAVCANTask ERROR: couldn't get canFdBitrate! setting default\n");
    }

	// set the bitrates
	if(socketcanSetBitrate(sock_ins, CAN_DEVICE, canBitrate, canFdBitrate))
	{
		cli_printfError("UAVCANTask ERROR: couldn't set bitrates!\n");
	}

	/* setup poll fd */
	gFd.fd = sock_ins->s;
	gFd.events = POLLIN;

	if (sock_ins->s < 0) {
		cli_printfError("canard_daemon: ERROR: open %s failed: %d\n",
		       CAN_DEVICE, errno);
		return -2;
	}
	
	if(nodeID == CANARD_NODE_ID_UNSET) { // PNP is enabled
	
        //TODO get unique id from S32K HW
        uint8_t unique_id[16];
        unique_id[0] = 0x19;
        unique_id[1] = 0xaf;
        unique_id[3] = 0x58;
        unique_id[4] = 0xe4;
        unique_id[5] = 0xcb;
        unique_id[6] = 0x38;
        unique_id[7] = 0x11;
        unique_id[8] = 0xea;
        unique_id[9] = 0x87;
        unique_id[10] = 0xd0;
        unique_id[11] = 0x02;
        unique_id[12] = 0x42;
        unique_id[13] = 0xac;
        unique_id[14] = 0x13;
        unique_id[15] = 0x00;

        initPNPAllocatee(ins, unique_id);

        uint32_t random_no;
        random_no = ((float)rand() / RAND_MAX) * (1000000);   

        uint64_t next_alloc_req = getMonotonicTimestampUSec() + random_no;	
    
        while(ins->node_id == CANARD_NODE_ID_UNSET) {
            // process the TX and RX buffer
            processTxRxOnce(ins, sock_ins, 10); //10Ms
            
            const uint64_t ts = getMonotonicTimestampUSec();

            if (ts >= next_alloc_req)
                {
                    next_alloc_req += ((float)rand() / RAND_MAX) * (1000000);
                    int32_t result = PNPAllocRequest(ins);
                    if(result) {
                        ins->node_id = PNPGetNodeID();
                    }
                }
        }
    } else {
        ins->node_id = nodeID; // Static preconfigured nodeID
    }

	//cli_printf("canard_daemon: canard initialized\n");
	cli_printf("start node (ID: %d Name: %s MTU: %d)\n", ins->node_id,
	       APP_NODE_NAME, ins->mtu_bytes);
    
    // Init UAVCAN register interfaces
    uavcan_node_GetInfo_Response_1_0 node_information; // TODO ADD INFO
    uavcan_register_interface_init(ins, &node_information);
    
    //uavcan_register_interface_add_entry("uavcan.pub.energy_source_state",0,0);
    uavcan_register_interface_add_entry("battery_status",set_battery_status_port_id,get_battery_status_port_id);
    //uavcan_register_interface_add_entry("uavcan.pub.battery_parameters",0,0);

	(void) canardRxSubscribe(ins,   // Subscribe to messages uavcan.node.Heartbeat.
				 CanardTransferKindMessage,
				 32085,  // The fixed Subject-ID of the Heartbeat message type (see DSDL definition).
				 7,      // The maximum payload size (max DSDL object size) from the DSDL definition.
				 CANARD_DEFAULT_TRANSFER_ID_TIMEOUT_USEC,
				 &heartbeat_subscription);

	(void) canardRxSubscribe(ins,
				 CanardTransferKindMessage,
				 PORT_ID,                     // The Service-ID to subscribe to.
				 TOPIC_SIZE,                  // The maximum payload size (max DSDL object size).
				 CANARD_DEFAULT_TRANSFER_ID_TIMEOUT_USEC,
				 &my_subscription);
    
    return 0;
}

/****************************************************************************
 * Name: UAVCANTask
 *
 * Description:
 *
 ****************************************************************************/

static int UAVCANTask(int argc, char *argv[])
{
    CanardInstance ins;
    CanardSocketInstance sock_ins;
	int error;
    
    if(uavcan_task_initialize(&ins, &sock_ins) == 0) {

        g_canard_daemon_started = true;
        //uint64_t next_1hz_service_at = getMonotonicTimestampUSec();

        for (;;) {
            // try to decrease the semaphore
            error = sem_trywait(&gUavcanSem);

            //cli_printf(" exit\n");

            // check if decrease succeeded
            if(!error)
            {
                //cli_printf("sending over UAVCAN!\n");
                // set the message 
                BatteryStatusToTransmitBuffer(&ins);//, ts);
            }
            
            // process the TX and RX buffer
            processTxRxOnce(&ins, &sock_ins, 4000);//10);

        }
	}


	g_canard_daemon_started = false;
	cli_printf("canard_daemon: Terminating!\n");
	fflush(stdout);
	return -1;
}

/****************************************************************************
 * Name: memAllocate
 *
 * Description:
 *
 ****************************************************************************/
static void *memAllocate(CanardInstance *const ins, const size_t amount)
{
	(void) ins;
	return o1heapAllocate(my_allocator, amount);
}

/****************************************************************************
 * Name: memFree
 *
 * Description:
 *
 ****************************************************************************/

static void memFree(CanardInstance *const ins, void *const pointer)
{
	(void) ins;
	o1heapFree(my_allocator, pointer);
}

/****************************************************************************
 * Name: getMonotonicTimestampUSec
 *
 * Description:
 *
 ****************************************************************************/
uint64_t getMonotonicTimestampUSec(void)
{
	struct timespec ts;

	memset(&ts, 0, sizeof(ts));

	if (clock_gettime(CLOCK_MONOTONIC, &ts) != 0) {
		abort();
	}

	return ts.tv_sec * 1000000ULL + ts.tv_nsec / 1000ULL;
}

/****************************************************************************
 * Name: BatteryStatusToTransmitBuffer
 *
 * Description:
 *   This function is called at 1 Hz rate from the main loop.
 *
 ****************************************************************************/

void BatteryStatusToTransmitBuffer(CanardInstance *ins)
{
    void* dataReturn;
    uint8_t outputStatusU8, statusFlagBits = 0;
    uint16_t subjectID;
    uint8_t i, uint8Val;
    float floatMaxValue, floatMinValue, floatValue;
    
	//cli_printf("BatteryStatusToTransmitBuffer!\n");

	CanardMicrosecond transmission_deadline = getMonotonicTimestampUSec() + 1000 * 10;

	// get the subject id
	 // // get the output_status
    dataReturn = (int32_t*)data_getParameter(UAVCAN_BS_SUB_ID, &subjectID, NULL);

    // check for error 
    if(dataReturn == NULL)
    {
    	// set status flag
    	statusFlagBits |= STATUS_BMS_ERROR_BIT;

    	// set the default value
    	subjectID = UAVCAN_BS_SUB_ID_DEFAULT;
    }
    
    if(subjectID == 65535){
        return;
    }
    
    uint8_t gBatteryStatus_payload_buffer[reg_drone_srv_battery_Status_0_1_SERIALIZATION_BUFFER_SIZE_BYTES_];

	CanardTransfer transfer = {
		.timestamp_usec = transmission_deadline,      // Zero if transmission deadline is not limited.
		.priority       = CanardPriorityNominal,
		.transfer_kind  = CanardTransferKindMessage,
		.port_id        = subjectID/*1234*/,                  // This is the subject-ID.
		.remote_node_id = CANARD_NODE_ID_UNSET,       // Messages cannot be unicast, so use UNSET.
		.transfer_id    = my_message_transfer_id,
		.payload_size   = reg_drone_srv_battery_Status_0_1_SERIALIZATION_BUFFER_SIZE_BYTES_,
		.payload        = &gBatteryStatus_payload_buffer,
	};
    
    reg_drone_srv_battery_Status_0_1 gBatteryStatus;

	// set the hearbeat
	// set the readiness
	gBatteryStatus.heartbeat.readiness.value = 3;

	// set the health
	gBatteryStatus.heartbeat.health.value = 0;

	// get the min and max temperature
	// check if the external temperature sensor is used

	// get the sensor enable variable
	dataReturn = (int32_t*)data_getParameter(SENSOR_ENABLE, &uint8Val, NULL);

    // check for error 
    if(dataReturn == NULL)
    {
    	// set status flag
    	statusFlagBits |= STATUS_BMS_ERROR_BIT;

    	// set the default value
    	uint8Val = SENSOR_ENABLE_DEFAULT;
    }

    // check if it is used
    if(uint8Val)
    {
    	// get the battery temperature
    	dataReturn = (int32_t*)data_getParameter(C_BATT, &floatValue, NULL);

    	// check for error 
	    if(dataReturn == NULL)
	    {
	    	// set status flag
	    	statusFlagBits |= STATUS_BMS_ERROR_BIT;

	    	// set the value that it will be overwritten
    		floatMinValue = 100;
    		floatMaxValue = -40;
	    }
	    else
	    {
	    	// set the values with the battery temperature
	    	floatMinValue = floatValue;
	    	floatMaxValue = floatValue;
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
		dataReturn = (int32_t*)data_getParameter((parameterKind_t)(C_AFE + i), &floatValue, NULL);

	    // check for error 
	    if(dataReturn == NULL)
	    {
	    	// set status flag
	    	statusFlagBits |= STATUS_BMS_ERROR_BIT;

	    	// set the default value
	    	floatValue = V_CELL1_DEFAULT;
	    }
	    else
	    {
	    	// check for min
		    if(floatValue < floatMinValue)
		    {
		    	// set new min
		    	floatMinValue = floatValue;
		    }

		    // check for max
		    if(floatValue > floatMaxValue)
		    {
		    	// set new max
		    	floatMaxValue = floatValue;
		    }
	    }
    }

    // set the temperaturs
    gBatteryStatus.temperature_min_max[0].kelvin = floatMinValue + CELSIUS_TO_KELVIN;
    gBatteryStatus.temperature_min_max[1].kelvin = floatMaxValue + CELSIUS_TO_KELVIN;

    // get the min and max cell voltage
	// check if the external temperature sensor is used

	// get the cell count
	dataReturn = (int32_t*)data_getParameter(N_CELLS, &uint8Val, NULL);

    // check for error 
    if(dataReturn == NULL)
    {
    	// set status flag
    	statusFlagBits |= STATUS_BMS_ERROR_BIT;

    	// set the default value
    	uint8Val = N_CELLS_DEFAULT;
    }

    // set the min and max 
    floatMinValue = 100;
    floatMaxValue = 0;

 	// loop through the cells to find the min and max
    for(i = 0; i < uint8Val; i++)
    {
    	// get the cell voltage
		dataReturn = (int32_t*)data_getParameter((parameterKind_t)(V_CELL1 + i), &floatValue, NULL);

	    // check for error 
	    if(dataReturn == NULL)
	    {
	    	// set status flag
	    	statusFlagBits |= STATUS_BMS_ERROR_BIT;

	    	// set the default value
	    	floatValue = V_CELL1_DEFAULT;
	    }

	    // check for min
	    if(floatValue < floatMinValue)
	    {
	    	// set new min
	    	floatMinValue = floatValue;
	    }

	    // check for max
	    if(floatValue > floatMaxValue)
	    {
	    	// set new max
	    	floatMaxValue = floatValue;
	    }
    }

    // set the min and max in the struct
    gBatteryStatus.cell_voltage_min_max[0].volt = floatMinValue;
    gBatteryStatus.cell_voltage_min_max[1].volt = floatMaxValue;

    // get the available charge 
    // get the remaining capacity 

   	dataReturn = (int32_t*)data_getParameter(A_REM, &floatValue, NULL);
	// check for error 
    if(dataReturn == NULL)
    {
    	// set status flag
    	statusFlagBits |= STATUS_BMS_ERROR_BIT;

    	// set the default value
    	floatValue = A_REM_DEFAULT;
    }

    // set the amount of coulombs
    gBatteryStatus.available_charge.coulomb = floatValue * AMPERE_HOURS_TO_COULOMB;

    // set the error value
    gBatteryStatus._error.value = 0;
    
    //gBatteryStatus.voltage = 5.0f; //Dummy voltage
/*
    // get the state_of_charge
    dataReturn = (int32_t*)data_getParameter(S_CHARGE, &gBatteryStatus.available_charge.coulomb, NULL);

    // check for error 
    if(dataReturn == NULL)
    {
    	// set status flag
    	statusFlagBits |= STATUS_BMS_ERROR_BIT;

    	// set the default value
    	gBatteryStatus.available_charge.coulomb = S_CHARGE_DEFAULT;
    }

    // // get the output_status
    dataReturn = (int32_t*)data_getParameter(S_OUT, &outputStatusU8, NULL);
    gBatteryStatus.output_status = outputStatusU8 & 1;

    // check for error
    if(dataReturn == NULL)
    {
    	// set status flag
    	statusFlagBits |= STATUS_BMS_ERROR_BIT;

    	// set the default value
    	gBatteryStatus.output_status = S_OUT_DEFAULT;
    }

    // get the temperature
    dataReturn = (int32_t*)data_getParameter(C_BATT, &gBatteryStatus.temperature, NULL);

    // check for error 
    if(dataReturn == NULL)
    {
    	// set status flag
    	statusFlagBits |= STATUS_BMS_ERROR_BIT;

    	// set the default value
    	gBatteryStatus.temperature = C_BATT_DEFAULT;
    }
    
    // get the voltage
    dataReturn = (int32_t*)data_getParameter(V_BATT, &gBatteryStatus.voltage, NULL);

    // check for error 
    if(dataReturn == NULL)
    {
    	// set status flag
    	statusFlagBits |= STATUS_BMS_ERROR_BIT;

    	// set the default value
    	gBatteryStatus.voltage = V_BATT;
    }

    // get the current
    //dataReturn = (int32_t*)data_getParameter(I_BATT, &gBatteryStatus.current, NULL);
    dataReturn = (int32_t*)data_getParameter(I_BATT_AVG, &gBatteryStatus.current, NULL);

    // check for error 
    if(dataReturn == NULL)
    {
    	// set status flag
    	statusFlagBits |= STATUS_BMS_ERROR_BIT;

    	// set the default value
    	gBatteryStatus.current = I_BATT_AVG_DEFAULT;
    }

    // get the average_current_10sec
    /*dataReturn = (int32_t*)data_getParameter(P_AVG, &gBatteryStatus.average_current_10sec, NULL);

    // check for error 
    if(dataReturn == NULL)
    {
    	// set status flag
    	statusFlagBits |= STATUS_BMS_ERROR_BIT;

    	// set the default value
    	gBatteryStatus.average_current_10sec = P_AVG_DEFAULT;
    }*/
/*
    // get the energy_consumed
    dataReturn = (int32_t*)data_getParameter(E_USED, &gBatteryStatus.energy_consumed, NULL);

    // check for error 
    if(dataReturn == NULL)
    {
    	// set status flag
    	statusFlagBits |= STATUS_BMS_ERROR_BIT;

    	// set the default value
    	gBatteryStatus.energy_consumed = E_USED_DEFAULT;
    }

    // get the battery_id
    dataReturn = (int32_t*)data_getParameter(BATT_ID, &gBatteryStatus.battery_id, NULL);

    // check for error 
    if(dataReturn == NULL)
    {
    	// set status flag
    	statusFlagBits |= STATUS_BMS_ERROR_BIT;

    	// set the default value
    	gBatteryStatus.battery_id = BATT_ID_DEFAULT;
    }

    // get the status flag bits
    dataReturn = (int32_t*)data_getParameter(S_FLAGS, &statusFlagBits, NULL);
    
    // check for error 
    if(dataReturn == NULL)
    {
    	// set the default value
    	statusFlagBits = S_FLAGS_DEFAULT;
    }

    // set it in the status flags
    gBatteryStatus.status.status = statusFlagBits;
*/
    // convert byte to UAVCAN protocol
    
    reg_drone_srv_battery_Status_0_1_serialize_(&gBatteryStatus, gBatteryStatus_payload_buffer, &transfer.payload_size);

	// set the data ready in the buffer and chop if needed 
	++my_message_transfer_id;  // The transfer-ID shall be incremented after every transmission on this subject.
	int32_t result = canardTxPush(ins, &transfer);

	if (result < 0) {
		// An error has occurred: either an argument is invalid or we've ran out of memory.
		// It is possible to statically prove that an out-of-memory will never occur for a given application if the
		// heap is sized correctly; for background, refer to the Robson's Proof and the documentation for O1Heap.
		fprintf(stderr, "Transmit error %d\n", result);
	}
}

/****************************************************************************
 * Name: BatteryParametersToTransmitBuffer
 *
 * Description:
 *   This function is called at 1 Hz rate from the main loop.
 *
 ****************************************************************************/

void BatteryParametersToTransmitBuffer(CanardInstance *ins)
{
    void* dataReturn;
    uint8_t outputStatusU8, statusFlagBits = 0;
    uint16_t subjectID;
    uint8_t i, uint8Val;
    float floatValue;
    
	//cli_printf("BatteryParametersToTransmitBuffer!\n");

	CanardMicrosecond transmission_deadline = getMonotonicTimestampUSec() + 1000 * 10;

	// get the subject id
	 // // get the output_status
    dataReturn = (int32_t*)data_getParameter(UAVCAN_BP_SUB_ID, &subjectID, NULL);

    // check for error 
    if(dataReturn == NULL)
    {
    	// set status flag
    	statusFlagBits |= STATUS_BMS_ERROR_BIT;

    	// set the default value
    	subjectID = UAVCAN_BP_SUB_ID_DEFAULT;
    }

    uint8_t gBatteryParameters_payload_buffer[reg_drone_srv_battery_Parameters_0_1_SERIALIZATION_BUFFER_SIZE_BYTES_];

	CanardTransfer transfer = {
		.timestamp_usec = transmission_deadline,      // Zero if transmission deadline is not limited.
		.priority       = CanardPriorityNominal,
		.transfer_kind  = CanardTransferKindMessage,
		.port_id        = subjectID/*1234*/,                  // This is the subject-ID.
		.remote_node_id = CANARD_NODE_ID_UNSET,       // Messages cannot be unicast, so use UNSET.
		.transfer_id    = my_message_transfer_id,
		.payload_size   = reg_drone_srv_battery_Parameters_0_1_SERIALIZATION_BUFFER_SIZE_BYTES_,
		.payload        = &gBatteryParameters_payload_buffer,
	};

    reg_drone_srv_battery_Parameters_0_1 gBatteryParameters;

	// set the unique id
    dataReturn = (int32_t*)data_getParameter(M_MASS, &gBatteryParameters.unique_id, NULL);

    // check for error 
    if(dataReturn == NULL)
    {
    	// set status flag
    	statusFlagBits |= STATUS_BMS_ERROR_BIT;

    	// set the default value
    	gBatteryParameters.unique_id = M_MASS_DEFAULT;
    }

    // set the mass
    dataReturn = (int32_t*)data_getParameter(MODEL_ID, &gBatteryParameters.mass.kilogram, NULL);

    // check for error 
    if(dataReturn == NULL)
    {
    	// set status flag
    	statusFlagBits |= STATUS_BMS_ERROR_BIT;

    	// set the default value
    	gBatteryParameters.mass.kilogram = MODEL_ID_DEFAULT;
    }

    // set the design capacity
    // get the design capacity
    dataReturn = (int32_t*)data_getParameter(A_FACTORY, &floatValue, NULL);

    // check for error 
    if(dataReturn == NULL)
    {
    	// set status flag
    	statusFlagBits |= STATUS_BMS_ERROR_BIT;

    	// set the default value
    	floatValue = A_FACTORY_DEFAULT;
    }

    // set it
    gBatteryParameters.design_capacity.coulomb = floatValue * AMPERE_HOURS_TO_COULOMB;

    // set the design_cell_voltage_min_max
    // set the min voltage
    dataReturn = (int32_t*)data_getParameter(V_CELL_UV, &gBatteryParameters.design_cell_voltage_min_max[0].volt, NULL);

    // check for error 
    if(dataReturn == NULL)
    {
    	// set status flag
    	statusFlagBits |= STATUS_BMS_ERROR_BIT;

    	// set the default value
    	gBatteryParameters.design_cell_voltage_min_max[0].volt = V_CELL_UV_DEFAULT;
    }

    // set the max voltage
    dataReturn = (int32_t*)data_getParameter(V_CELL_OV, &gBatteryParameters.design_cell_voltage_min_max[1].volt, NULL);

    // check for error 
    if(dataReturn == NULL)
    {
    	// set status flag
    	statusFlagBits |= STATUS_BMS_ERROR_BIT;

    	// set the default value
    	gBatteryParameters.design_cell_voltage_min_max[1].volt = V_CELL_OV_DEFAULT;
    }

    // set the discharge_current
    dataReturn = (int32_t*)data_getParameter(I_OUT_NOMINAL, &gBatteryParameters.discharge_current.ampere, NULL);

    // check for error 
    if(dataReturn == NULL)
    {
    	// set status flag
    	statusFlagBits |= STATUS_BMS_ERROR_BIT;

    	// set the default value
    	gBatteryParameters.discharge_current.ampere = I_OUT_NOMINAL_DEFAULT;
    }
	
	// set the discharge_current_burst    
	dataReturn = (int32_t*)data_getParameter(I_OUT_MAX, &gBatteryParameters.discharge_current_burst.ampere, NULL);

    // check for error 
    if(dataReturn == NULL)
    {
    	// set status flag
    	statusFlagBits |= STATUS_BMS_ERROR_BIT;

    	// set the default value
    	gBatteryParameters.discharge_current_burst.ampere = I_OUT_MAX_DEFAULT;
    }  
	
	// set the charge_current
	dataReturn = (int32_t*)data_getParameter(I_CHARGE_NOMINAL, &gBatteryParameters.charge_current.ampere, NULL);

    // check for error 
    if(dataReturn == NULL)
    {
    	// set status flag
    	statusFlagBits |= STATUS_BMS_ERROR_BIT;

    	// set the default value
    	gBatteryParameters.charge_current.ampere = I_CHARGE_NOMINAL_DEFAULT;
    }
	
	// set the charge_current_fast
	dataReturn = (int32_t*)data_getParameter(I_CHARGE_MAX, &gBatteryParameters.charge_current_fast.ampere, NULL);

    // check for error 
    if(dataReturn == NULL)
    {
    	// set status flag
    	statusFlagBits |= STATUS_BMS_ERROR_BIT;

    	// set the default value
    	gBatteryParameters.charge_current_fast.ampere = I_CHARGE_MAX_DEFAULT;
    }
	
	// set the charge_termination_treshold
	dataReturn = (int32_t*)data_getParameter(I_CHARGE_FULL, &gBatteryParameters.charge_termination_treshold.ampere, NULL);

    // check for error 
    if(dataReturn == NULL)
    {
    	// set status flag
    	statusFlagBits |= STATUS_BMS_ERROR_BIT;

    	// set the default value
    	gBatteryParameters.charge_termination_treshold.ampere = I_CHARGE_FULL_DEFAULT;
    }
	
	// set the charge_voltage   
	dataReturn = (int32_t*)data_getParameter(V_CELL_OV, &gBatteryParameters.charge_voltage.volt, NULL);

    // check for error 
    if(dataReturn == NULL)
    {
    	// set status flag
    	statusFlagBits |= STATUS_BMS_ERROR_BIT;

    	// set the default value
    	gBatteryParameters.charge_voltage.volt = V_CELL_OV_DEFAULT;
    }            
	
	// set the cycle_count
	dataReturn = (int32_t*)data_getParameter(N_CHARGES, &gBatteryParameters.cycle_count, NULL);

    // check for error 
    if(dataReturn == NULL)
    {
    	// set status flag
    	statusFlagBits |= STATUS_BMS_ERROR_BIT;

    	// set the default value
    	gBatteryParameters.cycle_count = N_CHARGES_DEFAULT;
    }
	
	// set the series_cell_count
	dataReturn = (int32_t*)data_getParameter(N_CELLS, &gBatteryParameters.series_cell_count, NULL);

    // check for error 
    if(dataReturn == NULL)
    {
    	// set status flag
    	statusFlagBits |= STATUS_BMS_ERROR_BIT;

    	// set the default value
    	gBatteryParameters.series_cell_count = N_CELLS_DEFAULT;
    }
	
	// set the state_of_health_pct
	dataReturn = (int32_t*)data_getParameter(S_HEALTH, &gBatteryParameters.state_of_health_pct, NULL);

    // check for error 
    if(dataReturn == NULL)
    {
    	// set status flag
    	statusFlagBits |= STATUS_BMS_ERROR_BIT;

    	// set the default value
    	gBatteryParameters.state_of_health_pct = S_HEALTH_DEFAULT;
    }  
	
	// set the Technology
    gBatteryParameters.technology.value = 110;
    

    // convert byte to UAVCAN protocol    
    //reg_drone_srv_battery_Status_0_1_serialize_(&gBatteryParameters, gBatteryParameters_payload_buffer, &transfer.payload_size);

	// set the data ready in the buffer and chop if needed 
	// ++my_message_transfer_id;  // The transfer-ID shall be incremented after every transmission on this subject.
	// int32_t result = canardTxPush(ins, &transfer);

	// if (result < 0) {
	// 	// An error has occurred: either an argument is invalid or we've ran out of memory.
	// 	// It is possible to statically prove that an out-of-memory will never occur for a given application if the
	// 	// heap is sized correctly; for background, refer to the Robson's Proof and the documentation for O1Heap.
	// 	fprintf(stderr, "Transmit error %d\n", result);
	// }
}

static void processReceivedTransfer(CanardTransfer *receive)
{
	cli_printf("Received transfer remote_node_id %d transfer_id: %d payload size: %d\n",
	       receive->remote_node_id, receive->transfer_id, receive->payload_size);

}

/****************************************************************************
 * Name: processTxRxOnce
 *
 * Description:
 *   Transmits all frames from the TX queue, receives up to one frame.
 *
 ****************************************************************************/

void processTxRxOnce(CanardInstance *ins, CanardSocketInstance *sock_ins, int timeout_msec)
{
	int32_t result;
	int ret;
    
	/* Transmitting */
	for (const CanardFrame *txf = NULL; (txf = canardTxPeek(ins)) != NULL;) { // Look at the top of the TX queue.
		if (txf->timestamp_usec > getMonotonicTimestampUSec()) { // Check if the frame has timed out.
			if (socketcanTransmit(sock_ins, txf) == 0) {           // Send the frame. Redundant interfaces may be used here.
				break;                             // If the driver is busy, break and retry later.
			}
		}

		canardTxPop(ins);                         // Remove the frame from the queue after it's transmitted.
		ins->memory_free(ins, (CanardFrame *)txf); // Deallocate the dynamic memory afterwards.
	}

	// check for incomming data or timeout 
	ret = poll(&gFd, 1, timeout_msec);

	// if timeout by other source
	if(ret <= 0)
	{
		return;
	}

	/* Receiving */
	CanardFrame received_frame;

	socketcanReceive(sock_ins, &received_frame);

	CanardTransfer receive;
	result = canardRxAccept(ins,
				&received_frame, // The CAN frame received from the bus.
				0,               // If the transport is not redundant, use 0.
				&receive);

	if (result < 0) {
		// An error has occurred: either an argument is invalid or we've ran out of memory.
		// It is possible to statically prove that an out-of-memory will never occur for a given application if
		// the heap is sized correctly; for background, refer to the Robson's Proof and the documentation for O1Heap.
		// Reception of an invalid frame is NOT an error.
		fprintf(stderr, "Receive error %d\n", result);

	} else if (result == 1) {
		// A transfer has been received, process it. !!!!

        if(receive.port_id == PNPGetPortID(ins)) {
            PNPProcess(ins,&receive);
        } else {
            uavcan_register_interface_process(ins, &receive);
        }

		ins->memory_free(ins, (void *)receive.payload); // Deallocate the dynamic memory afterwards.

	} else {
		// cli_printf("RX canard %d\r\n", result);
		// Nothing to do.
		// The received frame is either invalid or it's a non-last frame of a multi-frame transfer.
		// Reception of an invalid frame is NOT reported as an error because it is not an error.
	}

}
