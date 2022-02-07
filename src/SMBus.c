/****************************************************************************
 * nxp_bms/BMS_v1/src/SMBus.c
 *
 * BSD 3-Clause License
 * 
 * Copyright 2021-2022 NXP
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

#include <nuttx/config.h>

#include <sys/ioctl.h>
#include <stdbool.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <errno.h>

#include "../../drivers/smart_battery/simple_sbs.h"
#include "SMBus.h"
#include "data.h"
#include "cli.h"

/****************************************************************************
 * Defines
 ****************************************************************************/

#define KELVIN_TO_CELCIUS                 273.15
#define MINUTES_PER_HOUR                  60

#define MAX_ERROR_VAL                     5

#define MANUFACT_YEAR                     2021
#define MANUFACT_MONTH                    05
#define MANUFACT_DAY                      12

#define MANUFACT_DATA_LENGHT              1

/****************************************************************************
 * Private Variables
 ****************************************************************************/
/*! @brief variable to indicate of it is initialized */
static bool gSMBusInitialized             = false;
static bool gDontDoSMBus                  = false;
const char SMBus_path_sbs[]               = "/dev/sbs0";  

/*! @brief  The manufactrure name*/
static const char manufacture_name[]      = "NXP";

/*! @brief  The device name*/
static const char device_name[]           = "RDDRONE-BMS772";

/*! @brief  The chemistry name*/
static const char *device_chemistry[]    = 
{
    "LiP", 
    "LFP",
    "LFY"
};

/*! @brief  The array that will be written in the manufacturer_data of the SBS data*/
static const uint8_t manufacturer_data[MANUFACT_DATA_LENGHT]  =
{
    0x0,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/
/*!
 * @brief   This function will initialze the SMBus (smart battery bus)
 *          it will open the device and start the SMBus task
 *    
 * @return  0 if ok, -1 if there is an error
 * @example 
 *          if(SMBus_initialize())
 *          {
 *            // do something with the error
 *          }
 */
int SMBus_initialize(void)
{
    int lvRetValue = 0;

    // check if not initialized
    if(!gSMBusInitialized)
    {
        gSMBusInitialized = true;
    }

    // return to the user
    return lvRetValue;
}

/*!
 * @brief   this function will increase the semaphore so the SMBus
 *          task will update the BMS status of the SBS driver (SMBus)  
 *
 * @param   resetCurrent If this value is true, the current will be set to 0
 *          This could be used if a low power state is used and the SMBus struct 
 *          will not be updated anymore
 *
 * @return  If successful, the function will return zero (OK). 
 *          Otherwise, an error number will be returned to indicate the error:
 */
int SMBus_updateInformation(bool resetCurrent)
{
    int lvRetValue = -1, fd;
    uint8_t uint8Val, i;
    float floatValue, floatValue2;
    void* dataReturn;
    int32_t intVal;

    // check if SMBus shouldn't be done
    if(gDontDoSMBus)
    {
        // just return without an error
        return 0;
    }

    // check if it is not initialized
    if(!gSMBusInitialized)
    {
        // inform the user
        cli_printfError("SMBus ERROR: Not initialized!\n");
        
        // set the error
        lvRetValue = EXIT_FAILURE;

        // don't do SMBus anymore
        gDontDoSMBus = true;
    }
    else
    {
        // get the battery type 
        dataReturn = (int32_t*)data_getParameter(BATTERY_TYPE, 
            &uint8Val, NULL);

        // check for error 
        if(dataReturn == NULL)
        {
            // set the default value
            uint8Val = BATTERY_TYPE_DEFAULT;

            // error output
            cli_printfError("SMBus ERROR: could not get battery-type!\n");
        }

        // make the struct with the default parameters
        /*! @brief  The structure that will be written in the SBS driver, this could be retreived with SMBUs*/
        struct sbs_data_s sbs_data =
        {
            .temperature              = 0,    /* 0.1  K */
            .voltage                  = 0,    /* 1.0 mV */
            .current                  = 0,    /* 1.0 mA */
            .average_current          = 0,    /* 1.0 mA */
            .max_error                = MAX_ERROR_VAL,    /* 1.0  % */
            .relative_state_of_charge = 0,    /* 1.0  % */
            .absolute_state_of_charge = 0,    /* 1.0  % */
            .remaining_capacity       = 0,    /* 1.0 mAh (or 10 mWh?) */
            .full_charge_capacity     = 0,    /* 1.0 mAh (or 10 mWh?) */
            .run_time_to_empty        = 0,    /* 1.0  min */
            .average_time_to_empty    = 0,    /* 1.0  min */

            .cycle_count              = 0,    /* 1.0  cycle */
            .design_capacity          = 0,    /* 1.0 mAh (or 10 mWh?) */
            .design_voltage           = 0,    /* 1.0 mV */

            .manufacture_date         = ((MANUFACT_YEAR - 1980) * 
                                        512 + MANUFACT_MONTH * 
                                        32 + MANUFACT_DAY),   /* (year - 1980) * 512 + month * 32 + day */
            .serial_number            = 0,
            .manufacturer_name        = manufacture_name,
            .device_name              = device_name,
            .device_chemistry         = device_chemistry[uint8Val],
            .manufacturer_data        = manufacturer_data,
            .manufacturer_data_length = MANUFACT_DATA_LENGHT,

            .cell1_voltage            = 0,    /* 1.0 mV */
            .cell2_voltage            = 0,    /* 1.0 mV */
            .cell3_voltage            = 0,    /* 1.0 mV */
            .cell4_voltage            = 0,    /* 1.0 mV */
            .cell5_voltage            = 0,    /* 1.0 mV */
            .cell6_voltage            = 0,    /* 1.0 mV */
        };

        // open the device 
        fd = open("/dev/sbs0", O_RDWR);

        if (fd == ERROR)
        {
            /* Something went wrong.  You could try to handle the error here, pass
             * it on to the calling function, or just ignore it.
             */

            cli_printfError("SMBus ERROR: could not open FD: %d\n", fd);
        
            // don't do SMBus anymore
            gDontDoSMBus = true;

            // error on exit
            lvRetValue = -1;
        }
        else
        {
            // update the struct 

            // check if the battery sensor is enabled 
            // get the sensor enable variable
            dataReturn = (int32_t*)data_getParameter(SENSOR_ENABLE, 
                &uint8Val, NULL);

            // check for error 
            if(dataReturn == NULL)
            {
                // set the default value
                uint8Val = SENSOR_ENABLE_DEFAULT;

                // error output
                cli_printfError("SMBus ERROR: could not get sensor-enable!\n");
            }

            // set the floatValue to 0.0 K 
            floatValue = 0.0;

            // check if it is used
            if(uint8Val)
            {
                // get the battery temperature
                dataReturn = (int32_t*)data_getParameter(C_BATT, 
                    &floatValue, NULL);

                // check for error 
                if(dataReturn == NULL)
                {
                    // error output
                    cli_printfError("SMBus ERROR: could not get c-batt!\n");
                }
                else
                {
                    // convert to 0.1 K
                    floatValue = (floatValue + KELVIN_TO_CELCIUS) * 10;
                }
            }

            // set the temperature in the struct
            sbs_data.temperature = (uint16_t)floatValue;

            // get the battery output voltage
            dataReturn = (int32_t*)data_getParameter(V_BATT, 
                &floatValue, NULL);

            // check for error 
            if(dataReturn == NULL)
            {
                // set the default value
                floatValue = V_BATT_DEFAULT;

                // error output
                cli_printfError("SMBus ERROR: could not get v-batt!\n");
            }

            // convert to mv and place in struct
            sbs_data.voltage = (uint16_t)(floatValue*1000);

            // get the remaining capacity
            dataReturn = (int32_t*)data_getParameter(A_REM, 
                &floatValue, NULL);

            // check for error 
            if(dataReturn == NULL)
            {
                // set the default value
                floatValue = A_REM_DEFAULT;

                // error output
                cli_printfError("SMBus ERROR: could not get a-rem!\n");
            }

            // convert to mAh and place in struct
            sbs_data.remaining_capacity = (uint16_t)(floatValue*1000);

            // get the current 
            dataReturn = (int32_t*)data_getParameter(I_BATT, 
                &floatValue2, NULL);

            // check for error 
            if(dataReturn == NULL)
            {
                // set the default value
                floatValue2 = I_BATT_DEFAULT;

                // error output
                cli_printfError("SMBus ERROR: could not get i-batt!\n");
            }

            // check if the current needs to be updated
            if(!resetCurrent)
            {
                // convert to mA and place in struct
                sbs_data.current = (uint16_t)((int32_t)(floatValue2*1000));
            }

            // calculate the time to empty ((Ah/A) / minutes per h)
            intVal = (floatValue/floatValue) / MINUTES_PER_HOUR;

            // limit the value
            if(intVal < 0)
            {
                // set to 0
                intVal = 0;
            }
            // check if it is larger than the max
            else if(intVal > UINT16_MAX)
            {
                // set to the max value
                intVal = UINT16_MAX;
            }

            // set the time to empty
            sbs_data.run_time_to_empty = (uint16_t)intVal;

            // get the current average
            dataReturn = (int32_t*)data_getParameter(I_BATT_AVG, 
                &floatValue2, NULL);

            // check for error 
            if(dataReturn == NULL)
            {
                // set the default value
                floatValue2 = I_BATT_AVG_DEFAULT;

                // error output
                cli_printfError("SMBus ERROR: could not get i-batt-avg!\n");
            }

            // check if the current needs to be updated
            if(!resetCurrent)
            {
                // convert to mA and place in struct
                sbs_data.average_current = (uint16_t)((int32_t)(floatValue2*1000));
            }

            // calculate the time to empty ((Ah/A) / minutes per h)
            intVal = (floatValue/floatValue) / MINUTES_PER_HOUR;

            // limit the value
            if(intVal < 0)
            {
                // set to 0
                intVal = 0;
            }
            // check if it is larger than the max
            else if(intVal > UINT16_MAX)
            {
                // set to the max value
                intVal = UINT16_MAX;
            }

            // set the time to empty
            sbs_data.average_time_to_empty = (uint16_t)intVal;

            // get the state of charge
            dataReturn = (int32_t*)data_getParameter(S_CHARGE, 
                &uint8Val, NULL);

            // check for error 
            if(dataReturn == NULL)
            {
                // set the default value
                uint8Val = S_CHARGE_DEFAULT;

                // error output
                cli_printfError("SMBus ERROR: could not get s-charge!\n");
            }

            // set both state of charges in %
            sbs_data.relative_state_of_charge = uint8Val;
            sbs_data.absolute_state_of_charge = uint8Val;

            // get the remaining capacity
            dataReturn = (int32_t*)data_getParameter(A_FULL, 
                &floatValue, NULL);

            // check for error 
            if(dataReturn == NULL)
            {
                // set the default value
                floatValue = A_FULL_DEFAULT;

                // error output
                cli_printfError("SMBus ERROR: could not get a-full!\n");
            }

            // convert to mAh and place in struct
            sbs_data.full_charge_capacity = (uint16_t)(floatValue*1000);

            // get the cycle count
             dataReturn = (int32_t*)data_getParameter(N_CHARGES, 
                &intVal, NULL);

            // check for error 
            if(dataReturn == NULL)
            {
                // set the default value
                intVal = N_CHARGES_DEFAULT;

                // error output
                cli_printfError("SMBus ERROR: could not get n-charges!\n");
            }

            // set the cycle count
            sbs_data.cycle_count = (uint16_t)(intVal & UINT16_MAX);

            // get the design capacity 
            dataReturn = (int32_t*)data_getParameter(A_FACTORY, 
                &floatValue, NULL);

            // check for error 
            if(dataReturn == NULL)
            {
                // set the default value
                floatValue = A_FACTORY_DEFAULT;

                // error output
                cli_printfError("SMBus ERROR: could not get a-factory!\n");
            }

            // convert to mAh and place in struct
            sbs_data.design_capacity = (uint16_t)(floatValue*1000);

            // get the cell-ov 
            dataReturn = (int32_t*)data_getParameter(V_CELL_OV, 
                &floatValue, NULL);

            // check for error 
            if(dataReturn == NULL)
            {
                // set the default value
                floatValue = V_CELL_OV_DEFAULT;

                // error output
                cli_printfError("SMBus ERROR: could not get v-cell-ov!\n");
            }

            // convert to mv and place in struct
            sbs_data.design_voltage = (uint16_t)(floatValue*1000);

            // get the battery ID 
            dataReturn = (int32_t*)data_getParameter(BATT_ID, 
                &uint8Val, NULL);

            // check for error 
            if(dataReturn == NULL)
            {
                // set the default value
                uint8Val = BATT_ID_DEFAULT;

                // error output
                cli_printfError("SMBus ERROR: could not get batt-id!\n");
            }

            // set the battery ID
            sbs_data.serial_number = (uint16_t)uint8Val;

            // get the cell count
            dataReturn = (int32_t*)data_getParameter(N_CELLS, &uint8Val, NULL);

            // check for error 
            if(dataReturn == NULL)
            {
                // set the default value
                uint8Val = N_CELLS_DEFAULT;

                // error output
                cli_printfError("SMBus ERROR: could not get n-cells!\n");
            }

            // loop through the cells to set them
            for(i = 0; i < uint8Val; i++)
            {
                // get the cell voltage
                dataReturn = (int32_t*)data_getParameter((parameterKind_t)(V_CELL1 + i), 
                    &floatValue, NULL);

                // check for error 
                if(dataReturn == NULL)
                {
                    // set the default value
                    floatValue = V_CELL1_DEFAULT;

                    // error output
                    cli_printfError("SMBus ERROR: could not get v-cell%d!\n", i);
                }

                // set the cell voltage in the struct
                switch(i)
                {
                    case 0: 
                        // set the cell voltage
                        sbs_data.cell1_voltage = (uint16_t)(floatValue*1000);

                        // set the next voltage to 0 (which can be 0)
                        sbs_data.cell4_voltage = 0;
                    break;
                    case 1: 
                        // set the cell voltage
                        sbs_data.cell2_voltage = (uint16_t)(floatValue*1000);

                        // set the next voltage to 0 (which can be 0)
                        sbs_data.cell5_voltage = 0;
                    break; 
                    case 2: 
                        // set the cell voltage
                        sbs_data.cell3_voltage = (uint16_t)(floatValue*1000); 

                        // set the next voltage to 0 (which can be 0)
                        sbs_data.cell6_voltage = 0;
                    break;
                    case 3: 
                        // set the cell voltage
                        sbs_data.cell4_voltage = (uint16_t)(floatValue*1000); 
                    break; 
                    case 4: 
                        // set the cell voltage
                        sbs_data.cell5_voltage = (uint16_t)(floatValue*1000); 
                    break; 
                    case 5: 
                        // set the cell voltage
                        sbs_data.cell6_voltage = (uint16_t)(floatValue*1000); 
                    break;
                    default:
                        cli_printfError("SMBus ERROR: could not set v-cell%d i == %d!\n", 
                            i, i);
                    break;
                }
                
            }

            /* Write a prepared sbs_data_s struct to the SBS driver.  It needs to be
             * converted to a constant character buffer and the buffer length has to be
             * equal to the size of the sbs_dat_s struct.
             *
             * Note that this write operation can be performed as often as you like.
             * The SBS driver will always provide the most recent data that it received
             * when it receives a request for battery data.
             */

            lvRetValue = write(fd, (const char *)&sbs_data, sizeof(struct sbs_data_s));
            if (lvRetValue != sizeof(struct sbs_data_s))
            {
                /* Something went wrong.  You could try to handle the error here, pass
                 * it on to the calling function, or just ignore it.
                 */

                cli_printfError("SMBus ERROR: could not write new data: %d errno: %d\n",
                    lvRetValue, errno);

                //return -1;

                lvRetValue = -1;
            }
            else
            {
                lvRetValue = 0;
            }

            // close the file descriptor
            close(fd);
        }
    }

    // return to user
    return lvRetValue;
}

/****************************************************************************
 * Private Functions
 ****************************************************************************/
