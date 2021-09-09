/*
 * Copyright 2019 - 2021 NXP
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ** ###################################################################
 **     Filename    : define.h
 **     Project     : SmartBattery_RDDRONE_BMS772
 **     Processor   : S32K144
 **     Version     : 1.00
 **     Compiler    : GNU C Compiler
 **     Date/Time   : 2020-04-01, 11:00, # CodeGen: 0
 **     Abstract    :
 **         Define list.
 **         This file contains all defined values.
 **
 **         N.B.: all '_def' values are default values for global variables.
 **
 ** ###################################################################*/
/*!
 ** @file define.h
 **
 ** @version 01.00
 **
 ** @brief
 **         Define list.
 **         This file contains all defined values.
 **
 ** @b Notes \n
 **         All '_def' values are default values for global variables.
 */

#ifndef BCC_DEFINE_H_
#define BCC_DEFINE_H_


/*******************************************************************************
 * Includes
 ******************************************************************************/

#include "bcc_diagnostics.h"
/*******************************************************************************
 * Communication system
 ******************************************************************************/


/*******************************************************************************
 * Enumerations
 ******************************************************************************/

/* BCC CT filters configuration */

/*!
* @brief CT Filters Components.
*
* Values of external components required for OV & UV functional verification
* and CTx open detection functional verification.
*/
typedef struct
{
    uint32_t rLpf1;        /*!< R_LPF-1 low-pass filter resistor in [Ohm]. */
    uint32_t rLpf2;        /*!< R_LPF-2 low-pass filter resistor in [Ohm]. */
    uint32_t cLpf;         /*!< C_LPF capacitance in [nF]. */
    uint32_t cIn;          /*!< C_IN capacitance in [nF]. */
} ct_filter_t;


/* BCC I-sense configuration */

/*!
* @brief ISENSE Filters Components.
*
* Values of external components required for current measurements and related
* diagnostics.
*/
typedef struct
{
    uint16_t rLpfi;        /*!< R_LPFI resistor (between C_HFI and C_LPFI) in
                                [Ohm]. */
    uint32_t cD;           /*!< C_D capacitor (between ISENSE+ and ISENSE-) in
                                [nF]. */
    uint16_t cLpfi;        /*!< C_LPFI capacitor used to cut off common mode
                                disturbances on ISENSE+/- [nF]. */
    uint32_t rShunt;       /*!< Shunt resistor for ISENSE in [uOhm]. */
    uint16_t iMax;         /*!< Maximum shunt resistor current in [mA]. */
} isense_filter_t;

/* BCC NTC configuration */

/*!
* @brief NTC Configuration.
*
* The device has seven GPIOs which enable temperature measurement.
* NTC thermistor and fixed resistor are external components and must be set
* by the user. These values are used to calculate temperature. Beta parameter
* equation is used to calculate temperature. GPIO port of BCC device must be
* configured as Analog Input to measure temperature.
* This configuration is common for all GPIO ports and all devices (in case of
* daisy chain).
*/
typedef struct
{
    uint32_t beta;         /*!< Beta parameter of NTC thermistor in [K].
                                Admissible range is from 1 to 1000000. */
    uint32_t rntc;         /*!< R_NTC - NTC fixed resistance in [Ohm].
                                Admissible range is from 1 to 1000000. */
    uint32_t refRes;       /*!< NTC Reference Resistance in [Ohm].
                                Admissible range is from 1 to 1000000. */
    uint8_t refTemp;       /*!< NTC Reference Temperature in degrees [Celsius].
                                Admissible range is from 0 to 200. */
} ntc_config_t;



/*******************************************************************************
 * EOF
 ******************************************************************************/

#endif /* BCC_DEFINE_H_ */
