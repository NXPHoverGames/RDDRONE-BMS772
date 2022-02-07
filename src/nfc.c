/****************************************************************************
 * nxp_bms/BMS_v1/src/nfc.c
 *
 * BSD 3-Clause License
 * 
 * Copyright 2020-2022 NXP
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
#include <semaphore.h>
#include <sched.h>
#include <math.h>

#include "nfc.h"
#include "cli.h"
#include "gpio.h"
#include "i2c.h"
#include "data.h"

/****************************************************************************
 * Defines
 ****************************************************************************/
/* The NTAG has session and configuration register. 
 * The session register have _SES_ in their name. 
 * While the config bytes (register) have _CONF_ in their name. 
 */

#define NTAG5_SLAVE_ADR                 0x54 //84

#define I2C_SLAVE_CONF_SES_REG_ADR      0x10A9
#define I2C_SLAVE_CONF_SES_REG_BYTE     0x0
                                    
// The NTAG configuration register
// Keep in mind that not all bits of the session (SES) register can be written.
#define I2C_CONF_SES_REG_ADR            0x10A1
#define I2C_CONF_CONF_REG_ADR           0x1037
#define I2C_CONF_CONF0_REG_BYTE         0

#define I2C_CONF0_SRAM_COPY_EN_BIT      7
#define SRAM_COPY_EN_ENABLED            1 // SRAM copy on POR enabled
#define SRAM_COPY_EN_DISABLED           0 // SRAM copy on POR disabled (default)
#define SRAM_COPY_ENABLED_VAL           SRAM_COPY_EN_DISABLED << I2C_CONF0_SRAM_COPY_EN_BIT

#define I2C_CONF0_EH_MODE_BIT           2
#define EH_MODE_LOW_FIELD               2 // Energy harvesting optimized for low field strength (default)
#define EH_MODE_HIGH_FIELD              3 // Energy harvesting optimized for high field strength
#define EH_MODE_LOW_FIELD_VAL           EH_MODE_LOW_FIELD << I2C_CONF0_EH_MODE_BIT

#define I2C_CONF0_LOCK_SESS_REG_BIT     1
#define LOCK_SESS_REG_NFC_ALL_ACC       0 // NFC Write access to all session register (default)
#define LOCK_SESS_REG_NFC_LIM_ACC       1 // No NFC write access to session registers A3h to A7h
#define NOT_LOCK_SESS_REG_VAL           LOCK_SESS_REG_NFC_ALL_ACC << I2C_CONF0_LOCK_SESS_REG_BIT

#define I2C_CONF0_AUTO_STANDBY_BIT      0
#define AUTO_STANDBY_NORMAL_MODE        0 // Normal Operation Mode (Default)
#define AUTO_STANDBY_STANDBY_EN         1 // IC enters standby mode after boot if there is no RF field present automatically.
#define AUTO_STANDBY_NORMAL_VAL         AUTO_STANDBY_NORMAL_MODE << I2C_CONF0_AUTO_STANDBY_BIT

#define I2C_CONF_CONF0_VALUE            (SRAM_COPY_ENABLED_VAL  | \
                                        EH_MODE_LOW_FIELD_VAL   | \
                                        NOT_LOCK_SESS_REG_VAL   | \
                                        AUTO_STANDBY_NORMAL_VAL   \
                                        )
                                                                        

#define I2C_CONF_CONF1_REG_BYTE         1

#define I2C_CONF1_EH_ARBIT_EN_BIT       7
#define EH_ARBIT_EN_DISABLED            0 // In energy harvesting use case, ARBITER_MODE needs to be set with session registers after startup (default)
//#define EH_ARBIT_EN_ENABLED           1 // ARBITER_MODE is set automatically in any case afterstartup
#define EH_ARBIT_EN_VAL                 EH_ARBIT_EN_DISABLED << I2C_CONF1_EH_ARBIT_EN_BIT

#define I2C_CONF1_AMP_PLM_BIT           6
//#define AMP_PLM_PLM_MODE                0 // PLM
#define AMP_PLM_ALM_MODE                1 // ALM mode when supplied by VCC else PLM (default)
#define AMP_PLM_VAL                     AMP_PLM_ALM_MODE << I2C_CONF1_AMP_PLM_BIT

#define I2C_CONF1_USE_CASE_BIT          4
#define USE_CASE_I2C_SLAVE              0 // I2C slave (default)
// #define USE_CASE_I2C_MASTER             1 // I2C master
// #define USE_CASE_GPIO_PWM               2 // GPIO/PWM
// #define USE_CASE_DISABLED               3 // All host interface functionality disabled and pads are in 3-state mode
#define USE_CASE_I2C_SLAVE_VAL          USE_CASE_I2C_SLAVE << I2C_CONF1_USE_CASE_BIT

#define I2C_CONF1_ARBITER_MODE_BIT      2
#define ARBITER_MODE_NORMAL             0 // Normal Mode (default)
#define ARBITER_MODE_MIRROR             1 // SRAM mirror mode
#define ARBITER_MODE_PASS_THROUGH       2 // SRAM passes through mode
#define ARBITER_MODE_PHDC               3 // SRAM PHDC mode
#define ARBITER_MODE_MIRROR_VAL         ARBITER_MODE_MIRROR << I2C_CONF1_ARBITER_MODE_BIT

#define I2C_CONF1_SRAM_EN_BIT           1
#define SRAM_EN_DISABLED                0 // SRAM is not accessible (default)
#define SRAM_EN_ENABLED                 1 // SRAM is available (when VCC supplied)
#define SRAM_EN_ENABLED_VAL             SRAM_EN_ENABLED << I2C_CONF1_SRAM_EN_BIT

#define I2C_CONF1_PT_TRANS_DIR_BIT      0
#define PT_TRANS_DIR_I2C_TO_NFC         0 // Data transfer direction is I2C to NFC (default)
#define PT_TRANS_DIR_NFC_TO_I2C         1 // Data transfer direction is NFC to I2C
#define PT_TRANS_DIR_I2C_TO_NFC_VAL     PT_TRANS_DIR_I2C_TO_NFC << I2C_CONF1_PT_TRANS_DIR_BIT

#define I2C_CONF_CONF1_VALUE            (EH_ARBIT_EN_VAL            | \
                                        AMP_PLM_VAL                 | \
                                        USE_CASE_I2C_SLAVE_VAL      | \
                                        ARBITER_MODE_MIRROR_VAL     | \
                                        SRAM_EN_ENABLED_VAL         | \
                                        PT_TRANS_DIR_I2C_TO_NFC_VAL   \
                                        )


#define I2C_CONF_CONF2_REG_BYTE         2


// Register to reset the NTAG to apply the changes of the configuration registers to the session registers
#define I2C_RESET_GEN_SES_REG_ADR       0x10AA
#define I2C_RESET_GEN_SES_REG_BYTE      0
#define I2C_RESET_GEN_REG_VAL           0xE7


// The session status register can only be cleared if R/W
#define I2C_STATUS_SES_REG_ADR          0x10A0
#define I2C_STATUS0_REG_BYTE            0
#define STATUS0_EEPROM_WR_BUSY_BIT      7
#define STATUS0_EEPROM_WR_ERROR_BIT     6
#define STATUS0_SRAM_DATA_READY_BIT     5
#define STATUS0_SYNCH_BLOCK_WR_BIT      4
#define STATUS0_SYNCH_BLOCK_RD_BIT      3
#define STATUS0_PT_TRANSFER_DIR_BIT     2
#define STATUS0_VCC_SUPPLY_OK_BIT       1
#define STATUS0_NFC_FIELD_OK_BIT        0

#define STATUS0_NFC_FIELD_OK_PRESENT    1 << STATUS0_NFC_FIELD_OK_BIT
#define STATUS0_NFC_FIELD_OK_ABSENT     0 << STATUS0_NFC_FIELD_OK_BIT


#define I2C_STATUS1_REG_BYTE            1
#define STATUS1_VCC_BOOT_OK_BIT         7
#define STATUS1_NFC_BOOT_OK_BIT         6
#define STATUS1_ACTIVE_NFC_OK_BIT       5
#define STATUS1_GPIO1_IN_STATUS_BIT     4
#define STATUS1_GPIO0_IN_STATUS_BIT     3
#define STATUS1_ALM_PLM_BIT             2
#define STATUS1_I2C_IF_LOCKED_BIT       1 // is the only R/W bit of status1 (to be cleared)
#define STATUS1_NFC_IF_LOCKED_BIT       0

#define STATUS1_I2C_LOCKED_LOCKED       1 << STATUS1_I2C_IF_LOCKED_BIT
#define STATUS1_I2C_LOCKED_UNLOCKED     0 << STATUS1_I2C_IF_LOCKED_BIT

#define STATUS1_NFC_LOCKED_LOCKED       1 << STATUS1_NFC_IF_LOCKED_BIT
#define STATUS1_NFC_LOCKED_UNLOCKED     0 << STATUS1_NFC_IF_LOCKED_BIT


// the event detection configuration register
#define I2C_EH_CONFIG_CONF_REG_ADR      0x103D
#define EH_CONFIG_CONF_BYTE             0
#define ED_CONFIG_CONF_BYTE             2 // (uses same value as other ED_CONFIG)

// the event detection configuration session register 
#define I2C_ED_CONFIG_SES_REG_ADR       0x10A8
#define ED_CONFIG_REG_SES_BYTE          0 // (uses same value as other ED_CONFIG)
#define ED_CONFIG_NFC_FIELD_DET_VAL     0b0001
#define ED_CONFIG_PWM_VAL               0b0010
#define ED_CONFIG_I2C_NFC_PT_VAL        0b0011
#define ED_CONFIG_NFC_I2C_PT_VAL        0b0100
#define ED_CONFIG_ARBITER_LOCK_VAL      0b0101
#define ED_CONFIG_NDEF_MTVL_LEN_VAL     0b0110
#define ED_CONFIG_STANDBY_MODE_VAL      0b0111
#define ED_CONFIG_WRITE_COM_VAL         0b1000
#define ED_CONFIG_READ_COM_VAL          0b1001
#define ED_CONFIG_START_COM_VAL         0b1010
#define ED_CONFIG_READ_SYNCH_VAL        0b1011
#define ED_CONFIG_WRITE_SYNCH_VAL       0b1100
#define ED_CONFIG_SW_INT_VAL            0b1101

#define I2C_ED_CONFIG_VALUE             ED_CONFIG_NFC_FIELD_DET_VAL



// ED pin is cleared/released (to high) when writing 1 to this register
#define I2C_ED_INTR_CLEAR_SES_REG_ADR   0x10AB
#define ED_INTR_CLEAR_REG_BYTE          0
#define ED_INTR_CLEAR_VALUE             1



// The Watch Dog Timer Configuration Register
// Only read the WDT session register
#define I2C_WDT_SES_REG_ADR             0x10A6
#define I2C_WDT_CONF_REG_ADR            0x103C
#define I2C_WDT_CONF_LSB_REG_BYTE       0
#define I2C_WDT_CONF_LSB_VALUE          0x24 // for 10ms

#define I2C_WDT_CONF_MSB_REG_BYTE       1
#define I2C_WDT_CONF_MSB_VALUE          0x04 // for 10ms

#define I2C_WDT_WDT_EN_REG_BYTE         2
#define I2C_WDT_WDT_EN_REG_BIT          0
#define WDT_EN_WDT_ENABLED              1
#define WDT_EN_WDT_DISABLED             0
#define WDT_EN_WDT_ENABLED_VAL          WDT_EN_WDT_ENABLED << I2C_WDT_WDT_EN_REG_BIT
#define WDT_EN_WDT_DISABLED_VAL         WDT_EN_WDT_DISABLED << I2C_WDT_WDT_EN_REG_BIT

#define I2C_WDT_WDT_EN_VALUE            (WDT_EN_WDT_ENABLED_VAL    \
                                        )

#define I2C_WDT_SRAM_COPY_REG_BYTE      3
#define I2C_WDT_SRAM_COPY_BYTES_BIT     0
#define I2C_WDT_SRAM_COPY_REG_VAL       0 << I2C_WDT_SRAM_COPY_BYTES_BIT



// the SRAM_CONF_PROT register
#define I2C_SRAM_CONF_CONF_REG_ADR      0x103F
#define I2C_SRAM_CONF_PROT_BYTE         1
#define SRAM_CONF_I2C_CONFIG_W_BIT      5
#define SRAM_CONF_I2C_CONFIG_R_BIT      4
#define SRAM_CONF_NFC_SRAM_W_BIT        3
#define SRAM_CONF_NFC_SRAM_R_BIT        2
#define SRAM_CONF_NFC_CONFIG_W_BIT      1
#define SRAM_CONF_NFC_CONFIG_R_BIT      0



/* Block Size for NTAG5 SRAM */
#define NTAG_I2C_BLOCK_SIZE             0x04

/* Layout of SRAM */
#define NTAG_MEM_BLOCK_START_SRAM       0x2000
#define NTAG_MEM_BLOCK_END_SRAM         0x203F
#define NTAG_MEM_SRAM_BLOCKS            0x3F
#define NTAG_MEM_SESSION_START          0x10A0
#define NTAG_MEM_SESSION_END            0x10AF
#define NTAG_MEM_EEPROM_END             0x01FF
#define NTAG_MEM_EEPROM_START           0x0000
#define NTAG_MEM_CONFIG_START           0x1000



// defines used for the NDEF message payload
#define NDEF_HEADER_STRING_LEGHT        13
#define V_OUT_STRING                    "v-out: xx.xxxv\n"
#define V_OUT_STRING_BEGIN_INDEX        NDEF_HEADER_STRING_LEGHT                 
#define V_OUT_STRING_DATA_INDEX         7 + V_OUT_STRING_BEGIN_INDEX
#define V_OUT_STRING_LENGHT             15
#define V_OUT_STRING_DATA_LENGHT        V_OUT_STRING_LENGHT - 7 - 1
#define C_BATT_STRING                   "c-batt: xxx.xC\n"
#define C_BATT_STRING_BEGIN_INDEX       V_OUT_STRING_BEGIN_INDEX + V_OUT_STRING_LENGHT
#define C_BATT_STRING_DATA_INDEX        8 + C_BATT_STRING_BEGIN_INDEX
#define C_BATT_STRING_LENGHT            15
#define C_BATT_STRING_DATA_LENGHT       C_BATT_STRING_LENGHT - 8 - 1
#define S_CHARGE_STRING                 "s-charge: xxx%\n"
#define S_CHARGE_STRING_BEGIN_INDEX     C_BATT_STRING_BEGIN_INDEX + C_BATT_STRING_LENGHT
#define S_CHARGE_STRING_DATA_INDEX      10 + S_CHARGE_STRING_BEGIN_INDEX 
#define S_CHARGE_STRING_LENGHT          15
#define S_CHARGE_STRING_DATA_LENGHT     S_CHARGE_STRING_LENGHT - 10 - 1
#define S_HEALTH_STRING                 "s-health: xxx%\n"
#define S_HEALTH_STRING_BEGIN_INDEX     S_CHARGE_STRING_BEGIN_INDEX + S_CHARGE_STRING_LENGHT
#define S_HEALTH_STRING_DATA_INDEX      10 + S_HEALTH_STRING_BEGIN_INDEX
#define S_HEALTH_STRING_LENGHT          15
#define S_HEALTH_STRING_DATA_LENGHT     S_HEALTH_STRING_LENGHT - 10 - 1
#define I_OUT_STRING                    "i-out: xxx.xxxA\n"
#define I_OUT_STRING_BEGIN_INDEX        S_HEALTH_STRING_BEGIN_INDEX + S_HEALTH_STRING_LENGHT
#define I_OUT_STRING_DATA_INDEX         7 + I_OUT_STRING_BEGIN_INDEX
#define I_OUT_STRING_LENGHT             16
#define I_OUT_STRING_DATA_LENGHT        I_OUT_STRING_LENGHT - 7 - 1
#define N_CHARGES_STRING                "n-charges: xxxxx\n"
#define N_CHARGES_STRING_BEGIN_INDEX    I_OUT_STRING_BEGIN_INDEX + I_OUT_STRING_LENGHT
#define N_CHARGES_STRING_DATA_INDEX     11 + N_CHARGES_STRING_BEGIN_INDEX
#define N_CHARGES_STRING_LENGHT         17
#define N_CHARGES_STRING_DATA_LENGHT    N_CHARGES_STRING_LENGHT - 11 
#define BATT_ID_STRING                  "batt-id: xxx\n"
#define BATT_ID_STRING_BEGIN_INDEX      N_CHARGES_STRING_BEGIN_INDEX + N_CHARGES_STRING_LENGHT
#define BATT_ID_STRING_DATA_INDEX       9 + BATT_ID_STRING_BEGIN_INDEX
#define BATT_ID_STRING_LENGHT           13
#define BATT_ID_STRING_DATA_LENGHT      BATT_ID_STRING_LENGHT - 9 
#define MODEL_ID_STRING                 "model-id: xxxxxxxxxxxxxxxxxxxx\n"
#define MODEL_ID_STRING_BEGIN_INDEX     BATT_ID_STRING_BEGIN_INDEX + BATT_ID_STRING_LENGHT
#define MODEL_ID_STRING_DATA_INDEX      10 + MODEL_ID_STRING_BEGIN_INDEX
#define MODEL_ID_STRING_LENGHT          31
#define MODEL_ID_STRING_DATA_LENGHT     MODEL_ID_STRING_LENGHT - 10
#define STATE_STRING                    "state: \0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\n"
#define STATE_STRING_BEGIN_INDEX        MODEL_ID_STRING_BEGIN_INDEX + MODEL_ID_STRING_LENGHT
#define STATE_STRING_DATA_INDEX         7 + STATE_STRING_BEGIN_INDEX
#define STATE_STRING_LENGHT             23
#define STATE_STRING_DATA_LENGHT        STATE_STRING_LENGHT - 7


#define NDEF_TEXT_PROPRIETARY_TLV       0xFD // Proprietary TLV
#define NDEF_TEXT_TERMINATOR_TLV        0xFE // Terminator TLV
#define NDEF_TEXT_END_BYTE              NDEF_TEXT_TERMINATOR_TLV // Terminator TLV
#define NDEF_TEXT_END_STRING_LENGHT     1

#define NDEF_TEXT_RECORD_LENGHT         NDEF_HEADER_STRING_LEGHT + V_OUT_STRING_LENGHT + C_BATT_STRING_LENGHT + \
                                        S_CHARGE_STRING_LENGHT + S_HEALTH_STRING_LENGHT + I_OUT_STRING_LENGHT + \
                                        N_CHARGES_STRING_LENGHT +  BATT_ID_STRING_LENGHT + MODEL_ID_STRING_LENGHT + \
                                        STATE_STRING_LENGHT + NDEF_TEXT_END_STRING_LENGHT
                                                                            
#define NDEF_PAYLOAD_LENGTH             (NDEF_TEXT_RECORD_LENGHT - NDEF_HEADER_STRING_LEGHT) + 3


/* 
 * NDEF record header explained:
 *
 * KEEP IN MIND TO ALWAYS USE THE OFFICIAL NDEF SPECIFICATION
 *
 * Bit 7     6       5       4       3       2       1       0
 * ------  ------  ------  ------  ------  ------  ------  ------ 
 * [                        ADDRESS MODE                        ]
 * [                     VERSION AND ACCESS                     ]
 * [                           MEMLEN                           ]
 * [                   ADDITIONAL FEATURE INFO                  ]
 * [                          T field                           ]
 * [                        MESSAGE SIZE                        ]
 * [ MB ]  [ ME ]  [ CF ]  [ SR ]  [ IL ]  [        TNF         ]
 * [                         TYPE LENGTH                        ]
 * [                       PAYLOAD LENGTH                       ]
 * [                          ID LENGTH                         ]
 * [                         RECORD TYPE                        ]
 * [                              ID                            ]
 * [                           PAYLOAD                          ]
 */

// NDEF header defines
#define AM_1_BYTE_ADDRESS               0XE1 // 1-byte address mode is supported
#define AM_2_BYTE_ADDRESS               0XE2 // 2-byte address mode is supported
#define BMS_STATUS_AM_VAL               AM_1_BYTE_ADDRESS // 1-byte address mode is supported

#define VA_MAV_BIT                      6 // the bit of the major version value (2-bit val)
#define VA_MAV_VAL                      1 // the major version (2-bit val)
#define BMS_STATUS_VA_MAV_VAL           VA_MAV_VAL << VA_MAV_BIT

#define VA_MIV_BIT                      4 // the bit of the minor version value (2-bit val)
#define VA_MIV_VAL                      0 // the minor version (2-bit val)
#define BMS_STATUS_VA_MIV_VAL           VA_MIV_VAL << VA_MIV_BIT

#define VA_RA_BIT                       2 // the bit of the read access value (2-bit val)
#define VA_RA_ALWAYS_VAL                0 // always read access
#define VA_RA_RFU1_VAL                  1 // RFU
#define VA_RA_PROPRIETARY_VAL           2 // Proprietary read access
#define VA_RA_RFU2_VAL                  3 // RFU                                      
#define BMS_STATUS_VA_RA_VAL            VA_RA_ALWAYS_VAL << VA_RA_BIT

#define VA_WA_BIT                       2 // the bit of the write access value (2-bit val)
#define VA_WA_ALWAYS_VAL                0 // always write access
#define VA_WA_RFU1_VAL                  1 // RFU
#define VA_WA_PROPRIETARY_VAL           2 // Proprietary read access
#define VA_WA_NEVER_VAL                 3 // Never write access                                      
#define BMS_STATUS_VA_WA_VAL            VA_WA_ALWAYS_VAL << VA_WA_BIT

#define BMS_STATUS_VA_VAL               BMS_STATUS_VA_MAV_VAL | BMS_STATUS_VA_MIV_VAL | \
                                        BMS_STATUS_VA_RA_VAL | BMS_STATUS_VA_WA_VAL


#define BMS_STATUS_MEMLEN_VAL           0x20 // 256 byts / 8 in hex


#define AFI_SF_BIT                      4
#define AFI_SF_NO_SPECIAL_FRAME_VAL     0x0 // T5T does not need the Special Frame format as defined within [DIGITAL] for Write-Alike commands
#define AFI_SF_SPECIAL_FRAME_VAL        0x1 // T5T needs the Special Frame format as defined within [DIGITAL] for Write-Alike commands
#define BMS_STATUS_AFI_SF_VAL           AFI_SF_NO_SPECIAL_FRAME_VAL << AFI_SF_BIT

#define AFI_LB_BIT                      3
#define AFI_LB_NO_LOCK_BLOCK_VAL        0x0 // T5T does not support LOCK_SINGLE_BLOCK or EXTENDED_LOCK_SINGLE_BLOCK
#define AFI_LB_LOCK_BLOCK_VAL           0x1 // T5T support LOCK_SINGLE_BLOCK or EXTENDED_LOCK_SINGLE_BLOCK
#define BMS_STATUS_AFI_LB_VAL           AFI_LB_NO_LOCK_BLOCK_VAL << AFI_LB_BIT

#define AFI_MB_READ_BIT                 0
#define AFI_MB_READ_NO_MULTI_READ_VAL   0x0 // T5T does not support READ_MULTIPLE_BLOCKS or EXTENDED_READ_MULTIPLE_BLOCKS 
#define AFI_MB_READ_MULTI_READ_VAL      0x1 // T5T does support READ_MULTIPLE_BLOCKS or EXTENDED_READ_MULTIPLE_BLOCKS 
#define BMS_STATUS_AFI_MB_READ_VAL      AFI_MB_READ_MULTI_READ_VAL << AFI_MB_READ_BIT

#define BMS_STATUS_AFI_VAL              BMS_STATUS_AFI_SF_VAL | BMS_STATUS_AFI_LB_VAL | \
                                        BMS_STATUS_AFI_MB_READ_VAL


#define BMS_STATUS_T_FIELD_VAL          0x3 // NDEF Message TLV

#define BMS_STATUS_MESSAGE_SIZE_VAL     NDEF_PAYLOAD_LENGTH + 4

#define TNF_EMPTY                       0x00 // Indicates the record is empty.                                                    
#define TNF_WELL_KNOWN                  0x01 // Indicates the type field contains a well-known RTD type name.                     
#define TNF_MIME_MEDIA                  0x02 // Indicates the type field contains a media-type                                    
#define TNF_ABSOLUTE_URI                0x03 // Indicates the type field contains an absolute-URI                                 
#define TNF_EXTERNAL_TYPE               0x04 // Indicates the type field contains an external type name                           
#define TNF_UNKNOWN                     0x05 // Indicates the payload type is unknown                                             
#define TNF_UNCHANGED                   0x06 // Indicates the payload is an intermediate or final chunk of a chunked NDEF Record. 
#define TNF_BIT                         0
#define BMS_STATUS_TNF_VAL              TNF_WELL_KNOWN << TNF_BIT

#define IL_ID_LENGHT_NOT_PRESENT        0x0 // The IL flag indicates if the ID Length Field is present or not
#define IL_ID_LENGHT_PRESENT            0x1 // The IL flag indicates if the ID Length Field is present or not
#define IL_BIT                          3
#define BMS_STATUS_IL_VAL               IL_ID_LENGHT_NOT_PRESENT << IL_BIT

#define SR_SHORT_RECORD_LONG            0x0 // The SR flag is set to 1 if the payload length field is 1 byte (8 bits/0–255) or less
#define SR_SHORT_RECORD_SHORT           0x1 // The SR flag is set to 1 if the payload length field is 1 byte (8 bits/0–255) or less
#define SR_BIT                          4
#define BMS_STATUS_SR_VAL               SR_SHORT_RECORD_SHORT << SR_BIT

// #define CF_FIRST_RECORD               0x0 // The CF flag indicates if this is the first record chunk or a middle record chunk. For the 1st record of the message, it is set to 0
// #define CF_SUBSEQUENT_RECORD          0x1 // The CF flag indicates if this is for the subsequent records, it is set to 1.
// #define CF_BIT                        5
// #define BMS_STATUS_CF_VAL             CF_FIRST_RECORD << CF_BIT

#define CF_NOT_CHUNKED                  0x0 // The CF flag indicates if this is the first record chunk or a middle record chunk. For the 1st record of the message, it is set to 0
#define CF_CHUNKED                      0x1 // True when this record is chunked.
#define CF_BIT                          5
#define BMS_STATUS_CF_VAL               CF_NOT_CHUNKED << CF_BIT

#define ME_LAST_NOT_RECORD              0x0 // The ME flag indicates if this is the last record in the message. It is set to 1 for the last record.
#define ME_LAST_RECORD                  0x1 // The ME flag indicates if this is the last record in the message. It is set to 1 for the last record.
#define ME_BIT                          6
#define BMS_STATUS_ME_VAL               ME_LAST_RECORD << ME_BIT

#define MB_NOT_FIRST_RECORD             0x0 // The MB flag indicates if this is the first record in the message. It is set to 1 for the first message.
#define MB_FIRST_RECORD                 0x1 // The MB flag indicates if this is the first record in the message. It is set to 1 for the first message.
#define MB_BIT                          7
#define BMS_STATUS_MB_VAL               MB_FIRST_RECORD << MB_BIT

#define BMS_STATUS_HEADER_VALUE         BMS_STATUS_TNF_VAL | BMS_STATUS_IL_VAL | BMS_STATUS_SR_VAL | \
                                        BMS_STATUS_CF_VAL | BMS_STATUS_ME_VAL | BMS_STATUS_MB_VAL
#define BMS_STATUS_TYPE_LENGHT          0x1
#define BMS_STATUS_PAYLOAD_LENGHT       NDEF_PAYLOAD_LENGTH
#define BMS_STATUS_RECORD_TYPE          0x54 // T which means text
#define BMS_STATUS_LANG_EN_1_3          0x2
#define BMS_STATUS_LANG_EN_2_3          0x65
#define BMS_STATUS_LANG_EN_3_3          0x6E

#if ((NDEF_TEXT_RECORD_LENGHT + 1) / 4) > NTAG_MEM_BLOCK_END_SRAM
#   error NDEF_TEXT_RECORD_LENGHT is too large! 
#endif

#define AMOUNT_RETRIES                  1000
#define ERROR_COULD_NOT_WRITE           (-222)
#define ERROR_COULD_NOT_READ            (-333)
#define NTAG_EEPROM_WRITE_DELAY_US      4000
#define NTAG_SRAM_WRITE_DELAY_US        400 

#define NORMAL_TO_MILI                  1000
#define CONVERT_TO_1_10TH               10
//#define DEBUG_NFC
//#define DEBUG_NFC_INIT                                        
//#define DEBUG_NFC_WRITE
/****************************************************************************
 * Private Variables
 ****************************************************************************/
/*! @brief variable to indicate of it is initialized */
static bool gNfcInitialized     = false;  

/*! @brief String that will be placed in the NTAG if the data is outdated (not updated) */
const char gWakingUpString[]    = "Waking up BMS, please tap again!\n";

/*! @brief String that will be placed in the NTAG if the data is outdated (not updated) */
const char gChargeRelaxString[] = "Charge-relaxation state\n";

/*! @brief Variable to disable the NFC if it failed */
static bool gDisableNFC = false;

/****************************************************************************
 * Private Functions
 ****************************************************************************/
/*!
 * @brief   This function can be used to write a data via the I2C to the NTAGs normal register (not session register)
 * @note    It will check if it is allowed to write to the NTAG before writing. 
 *  
 * @param   slaveAdr the slave address of the I2C device
 * @param   regAdr the address of the register to write to (2 bytes)
 * @param   writeReg address of the variable to write
 * @param   writeBytes the amount of bytes to write max 255
 * @param   dontWait if true, it doesn't wait until available, otherwise it waits with timeout
 *
 * @return  0 if ok, -1 if there is an error, ERROR_COULD_NOT_WRITE if it could not write
 */
int nfc_writeI2cData(uint8_t slaveAdr, uint16_t regAdr, uint8_t* writeReg, uint8_t writeBytes, 
    bool dontWait);

/*!
 * @brief   This function can be used to read a data via the I2C from the NTAGs normal register (not session register)
 * @note    It will check if it is allowed to read from the NTAG before reading. 
 *  
 * @param   slaveAdr the slave address of the I2C device
 * @param   regAdr the address of the register to read from (2 bytes)
 * @param   readReg address of the variable to read
 * @param   readBytes the amount of bytes to read max 255
 * @param   reTry if true, it will retry to read if failed (maybe WDT timeout happend)
 *
 * @return  0 if ok, -1 if there is an error
 */
int nfc_readI2cData(uint8_t slaveAdr, uint16_t regAdr, uint8_t* readReg, uint8_t readBytes, 
    bool reTry);

/****************************************************************************
 * Public Functions
 ****************************************************************************/
/*!
 * @brief   This function will initialze the NFC
 *          it will test the i2C connection with the chip and read the slave address
 *
 * @param   skipSelfTest if this is true it will skip the self-test
 *    
 * @return  0 if ok, -1 if there is an error
 * @example 
 *          if(nfc_initialize(false))
 *          {
 *            // do something with the error
 *          }
 */
int nfc_initialize(bool skipSelfTest)
{
    int lvRetValue = 0;
    uint8_t regVal[4];
    uint8_t writeVal[4];
    int i;
    bool applyReset = false;

    // check if the NFC is disabled
    if(gDisableNFC)
    {
        // return OK to not output an error
        return 0;
    }

    // reset the register values
    for(i = 0; i < 4; i++)
    {
        writeVal[i] = 0;
    }

    // check if not initialzed
    if(!gNfcInitialized)
    {
        // Check if the self-test shouldn't be skipped
        if(!skipSelfTest)
        {
            cli_printf("SELF-TEST NFC: START\n");
        }

        // make sure the NFC chip is not in hard power-down mode
        lvRetValue = nfc_setHPD(false);

        // check if there is an error
        if(lvRetValue)
        {
            // Check if the self-test shouldn't be skipped
            if(!skipSelfTest)
            {
                // output that it failed
                cli_printf("SELF-TEST GPIO: \e[31mFAIL\e[39m\n");

                // return with an error
                return -1;
            }
        }

        // read the slave address 
        if(i2c_nfcReadSessionRegByte(NTAG5_SLAVE_ADR, I2C_SLAVE_CONF_SES_REG_ADR, 
            I2C_SLAVE_CONF_SES_REG_BYTE, regVal, 2))
        {
            // output error to user
            cli_printfError("nfc ERROR: Can't read register: 0x%x byte %d\n", 
                I2C_SLAVE_CONF_SES_REG_ADR, I2C_SLAVE_CONF_SES_REG_BYTE);

            return -1;
        }

        // check if the value is equal to the slave address
        if((regVal[0] & 0x7F) != NTAG5_SLAVE_ADR)
        {
            // output to the user
            cli_printfError("nfc ERROR: slave address is not equal!\n");

            cli_printfError("Can't verify NFC chip!\n");

            // set the returnvalue 
            lvRetValue = -1;

            // the expected registervalue is not what it should be

            // return to the user
            return lvRetValue;
        }

        //NFC chip (NTAG5) I2C communication verified!

        // read the STATUS_REG session register
        if(i2c_nfcReadSessionRegByte(NTAG5_SLAVE_ADR, I2C_STATUS_SES_REG_ADR, 
            I2C_STATUS0_REG_BYTE, regVal, 2))
        {
            // output error to the user
            cli_printfError("nfc ERROR: Can't read register: 0x%x byte %d\n", 
                I2C_STATUS_SES_REG_ADR, 0);

            return -1;
        }

#ifdef DEBUG_NFC_INIT

        // print it
        cli_printf("status0: 0x%x, 1: 0x%x\n", regVal[0], regVal[1]);

#endif
        // other check
        // if VCC = 1 (2nd bit of STATUS_REG) and there was a response to register read, then NTAG5 is there
        if(!(regVal[0] >> STATUS0_VCC_SUPPLY_OK_BIT) & 1)
        {
            // output to the user
            cli_printfError("nfc ERROR: a) No Vcc on NTAG b) I²C slave Address is not 0x%x!\n", 
                NTAG5_SLAVE_ADR);

            cli_printfError("Can't verify NFC chip!\n");

            // set the returnvalue 
            lvRetValue = -1;

            gNfcInitialized = false;

            // return to the user
            return lvRetValue;
        }

        // read the configuration register
        if(nfc_readI2cData(NTAG5_SLAVE_ADR, I2C_CONF_CONF_REG_ADR, 
            regVal, 4, true))
        {
            cli_printfError("nfc ERROR: Can't read register: 0x%x \n", 
                I2C_CONF_CONF_REG_ADR);

            return -1;
        }

#ifdef DEBUG_NFC_INIT
        // print the configuration register 
        cli_printf("conf 0: 0x%x, 1: 0x%x, 2: 0x%x\n", regVal[0], regVal[1], regVal[2]);
#endif

        // check if in the right mode
        if((regVal[0] != I2C_CONF_CONF0_VALUE) || (regVal[1] != I2C_CONF_CONF1_VALUE))
        {
#ifdef DEBUG_NFC_INIT

            cli_printf("NFC configuration register has wrong value\n");
#endif
            // set the writevalues
            writeVal[0] = I2C_CONF_CONF0_VALUE;
            writeVal[1] = I2C_CONF_CONF1_VALUE;
            writeVal[2] = regVal[2];
            writeVal[3] = regVal[3];

#ifdef DEBUG_NFC_INIT
            cli_printf("NFC writing correct value, 0: 0x%x, 1: 0x%x\n",
                writeVal[0], writeVal[1]);
#endif
            // write the correct configuration
            lvRetValue = nfc_writeI2cData(NTAG5_SLAVE_ADR, I2C_CONF_CONF_REG_ADR, 
                writeVal, 4, false);

            // check for errors
            if(lvRetValue)
            {
                cli_printfError("nfc ERROR: Can't write to register: 0x%x\n", 
                    I2C_CONF_CONF_REG_ADR);

                return lvRetValue;
            }

            // make sure to do a reset later on to apply the configuration values
            applyReset = true;
        }
            
        // write the session register with the correct value as well

        // check if the session configuration register has the right value 
        if(i2c_nfcReadSessionRegByte(NTAG5_SLAVE_ADR, I2C_CONF_SES_REG_ADR, 
            I2C_CONF_CONF0_REG_BYTE, regVal, 3))
        {
            cli_printfError("nfc ERROR: Can't read session register 0x%x\n",
                    I2C_CONF_SES_REG_ADR);

            return -1;
        }

        // check if it has the correct value
        if((regVal[0] != I2C_CONF_CONF0_VALUE) || (regVal[1] != I2C_CONF_CONF1_VALUE))
        {
            // make sure to do a reset later on to apply the configuration values
            applyReset = true;

#ifdef DEBUG_NFC_INIT
            // print the configuration register 
            cli_printf("ses conf 1: 0x%x, 2: 0x%x, 3: 0x%x\n", regVal[0], regVal[1], regVal[2]);
#endif
        }

        // check if the Event Detect configuration has the correct value
        // read the ED config (conf) register
        if(nfc_readI2cData(NTAG5_SLAVE_ADR, I2C_EH_CONFIG_CONF_REG_ADR, 
            regVal, 4, true))
        {
            cli_printfError("nfc ERROR: Can't read register: 0x%x \n", 
                I2C_EH_CONFIG_CONF_REG_ADR);

            return -1;
        }

#ifdef DEBUG_NFC_INIT
        // print the ED register 
        cli_printf("ed: 0x%x\n", regVal[2]);
#endif

        // if it doesn't have the correct value
        if(regVal[2] != I2C_ED_CONFIG_VALUE)
        {
            // set the correct value
            writeVal[0] = regVal[0];
            writeVal[1] = regVal[1];
            writeVal[2] = I2C_ED_CONFIG_VALUE;
            writeVal[3] = regVal[3];

#ifdef DEBUG_NFC_INIT

            cli_printf("NFC writing correct value, ed: 0x%x\n",
                writeVal[2]);
#endif
            // write the correct configuration
            lvRetValue = nfc_writeI2cData(NTAG5_SLAVE_ADR, I2C_EH_CONFIG_CONF_REG_ADR, 
                writeVal, 4, false);

            // check for errors
            if(lvRetValue)
            {
                cli_printfError("nfc ERROR: Can't write to register: 0x%x\n", 
                    I2C_EH_CONFIG_CONF_REG_ADR);

                return lvRetValue;
            }

            // make sure to do a reset later on to apply the configuration values
            applyReset = true;
        }

        // check if the Event Detect configuration session register has the correct value
        // read the ED config session register
        if(i2c_nfcReadSessionRegByte(NTAG5_SLAVE_ADR, I2C_ED_CONFIG_SES_REG_ADR, 
            ED_CONFIG_REG_SES_BYTE, regVal, 1))
        {
            cli_printfError("nfc ERROR: Can't read session register 0x%x\n",
                    I2C_ED_CONFIG_SES_REG_ADR);

            return -1;
        }

        // if it doesn't have the correct value
        if(regVal[0] != I2C_ED_CONFIG_VALUE)
        {
            // make sure to do a reset later on to apply the configuration values
            applyReset = true;

            // set the correct value

#ifdef DEBUG_NFC_INIT
            // print the configuration register 
            cli_printf("ses ed 1: 0x%x\n", regVal[0]);
#endif

            // set the writevalues
            writeVal[0] = I2C_ED_CONFIG_VALUE;

#ifdef DEBUG_NFC_INIT
            cli_printf("NFC writing correct value, byte%d: 0x%x\n",
                ED_CONFIG_REG_SES_BYTE, writeVal[0]);
#endif

            // write the session configuration register
            if(i2c_nfcWriteSessionRegByte(NTAG5_SLAVE_ADR, I2C_ED_CONFIG_SES_REG_ADR, 
                ED_CONFIG_REG_SES_BYTE, writeVal, 0xFF, 1))
            {
                cli_printfError("nfc ERROR: Can't write session register 0x%x\n",
                    I2C_ED_CONFIG_SES_REG_ADR);

                return -1;
            }
        }

        // Read the watchdog configuration register
        if(nfc_readI2cData(NTAG5_SLAVE_ADR, I2C_WDT_CONF_REG_ADR, 
            regVal, 4, true))
        {
            cli_printfError("nfc ERROR: Can't read register: 0x%x \n", 
                I2C_WDT_CONF_REG_ADR);

            return -1;
        }

        // check if the watchdog register has the correct value
        if((regVal[I2C_WDT_CONF_LSB_REG_BYTE] != I2C_WDT_CONF_LSB_VALUE) ||
            (regVal[I2C_WDT_CONF_MSB_REG_BYTE] != I2C_WDT_CONF_MSB_VALUE) ||
            (regVal[I2C_WDT_WDT_EN_REG_BYTE] != I2C_WDT_WDT_EN_VALUE))
        {
            // set the correct configuration WDT value
            // make the value
            writeVal[I2C_WDT_CONF_LSB_REG_BYTE] = I2C_WDT_CONF_LSB_VALUE;
            writeVal[I2C_WDT_CONF_MSB_REG_BYTE] = I2C_WDT_CONF_MSB_VALUE;
            writeVal[I2C_WDT_WDT_EN_REG_BYTE] = I2C_WDT_WDT_EN_VALUE;
            writeVal[I2C_WDT_SRAM_COPY_REG_BYTE] = regVal[I2C_WDT_SRAM_COPY_REG_BYTE];

#ifdef DEBUG_NFC_INIT
            cli_printf("WDT configuration register has the wrong value!\n 0x%x, 0x%x, 0x%x, 0x%x\n", 
                regVal[0], regVal[1], regVal[2], regVal[3]);
            cli_printf("Writing: 0x%x, 0x%x, 0x%x, 0x%x\n", writeVal[0], writeVal[1],
                writeVal[2], writeVal[3]);
#endif
            // write the correct configuration
            lvRetValue = nfc_writeI2cData(NTAG5_SLAVE_ADR, I2C_WDT_CONF_REG_ADR, 
                writeVal, 4, false);

            // check for errors
            if(lvRetValue)
            {
                cli_printfError("nfc ERROR: Can't write to register: 0x%x\n", 
                    I2C_WDT_CONF_REG_ADR);

                return lvRetValue;
            }

            // make sure to do a reset later on to apply the configuration values
            applyReset = true;
        }

        // read the watchdog session register
        if(i2c_nfcReadSessionRegByte(NTAG5_SLAVE_ADR, I2C_WDT_SES_REG_ADR, 
            I2C_WDT_CONF_LSB_REG_BYTE, regVal, 3))
        {
            cli_printfError("nfc ERROR: Can't read session register 0x%x\n",
                I2C_WDT_SES_REG_ADR);

            return -1;
        }

        // check if the watchdog session register has the correct value
        if((regVal[I2C_WDT_CONF_LSB_REG_BYTE] != I2C_WDT_CONF_LSB_VALUE) ||
            (regVal[I2C_WDT_CONF_MSB_REG_BYTE] != I2C_WDT_CONF_MSB_VALUE) ||
            (regVal[I2C_WDT_WDT_EN_REG_BYTE] != I2C_WDT_WDT_EN_VALUE))
        {
#ifdef DEBUG_NFC_INIT
            cli_printf("WDT session register has the wrong value!\n 0x%x, 0x%x, 0x%x\n", 
                regVal[0], regVal[1], regVal[2]);
#endif
            // make sure to do a reset later on to apply the configuration values
            applyReset = true;
        }

        // check if the NTAG reset is needed to apply the changes
        if(applyReset)
        {
            // make the value that needs to be written to the reset generator register
            writeVal[0] = I2C_RESET_GEN_REG_VAL;

#ifdef DEBUG_NFC_INIT
            cli_printf("resetting NTAG! by writing 0x%x\n", writeVal[0]);
#endif
            // write to the reset generator session register
            lvRetValue = i2c_nfcWriteSessionRegByte(NTAG5_SLAVE_ADR, I2C_RESET_GEN_SES_REG_ADR, 
                I2C_RESET_GEN_SES_REG_BYTE, writeVal, writeVal[0], 1);

            // check for errors
            // The I2C write should fail because with a reset, the NTAG will NACK!
            if(!lvRetValue)
            {
                cli_printfError("nfc ERROR: Can't write to reset session register: 0x%x\n", 
                    I2C_RESET_GEN_SES_REG_ADR);

                // return with an error
                return -1;
            }
            else
            {
                cli_printfGreen("nfc: Ignore I2C write error, this is due to NTAG I2C reset!\n");

                // set the return value to OK
                lvRetValue = 0;
            }

            // wait 5ms for the NTAG to start it up again
            usleep(5*1000u);

            // check all the registers again and fail if one is not correct

            // read the configuration register
            if(nfc_readI2cData(NTAG5_SLAVE_ADR, I2C_CONF_CONF_REG_ADR, 
                regVal, 4, true))
            {
                cli_printfError("nfc ERROR: Can't read register: 0x%x \n", 
                    I2C_CONF_CONF_REG_ADR);

                // return with an error
                return -1;
            }

            // check if it doesn't contain the correct values
            if((regVal[0] != I2C_CONF_CONF0_VALUE) || (regVal[1] != I2C_CONF_CONF1_VALUE))
            {
                cli_printf("nfc ERROR: configuration config registers still have the wrong values! conf 0: 0x%x, 1: 0x%x\n",
                    regVal[0], regVal[1]);
                
                // return with an error
                return -1;
            }

            // read the session configuration register
            if(i2c_nfcReadSessionRegByte(NTAG5_SLAVE_ADR, I2C_CONF_SES_REG_ADR, 
                I2C_CONF_CONF0_REG_BYTE, regVal, 3))
            {
                cli_printfError("nfc ERROR: Can't read configuration session register 0x%x\n",
                        I2C_CONF_SES_REG_ADR);

                // return with an error
                return -1;
            }

            // check if it doesn't contain the correct values
            if((regVal[0] != I2C_CONF_CONF0_VALUE) || (regVal[1] != I2C_CONF_CONF1_VALUE))
            {
                 cli_printf("nfc ERROR: session configuration session registers still have the wrong values! conf 0: 0x%x, 1: 0x%x\n",
                    regVal[0], regVal[1]);

                // return with an error
                return -1;
            }

            // read the configuration event detection register
            if(nfc_readI2cData(NTAG5_SLAVE_ADR, I2C_EH_CONFIG_CONF_REG_ADR, 
                regVal, 4, true))
            {
                cli_printfError("nfc ERROR: Can't read register: 0x%x \n", 
                    I2C_EH_CONFIG_CONF_REG_ADR);

                // return with an error
                return -1;
            }

            // check if it doesn't contain the correct value
            if(regVal[2] != I2C_ED_CONFIG_VALUE)
            {
                cli_printf("nfc ERROR: configuration event detect byte 2 register still has wrong value! 0x%x\n",
                    regVal[2]);

                // return with an error
                return -1;
            }

            // read the ED config session register
            if(i2c_nfcReadSessionRegByte(NTAG5_SLAVE_ADR, I2C_ED_CONFIG_SES_REG_ADR, 
                ED_CONFIG_REG_SES_BYTE, regVal, 1))
            {
                cli_printfError("nfc ERROR: Can't read session register 0x%x\n",
                        I2C_ED_CONFIG_SES_REG_ADR);

                // return with an error
                return -1;
            }

            // check if it doesn't contain the correct value
            if(regVal[0] != I2C_ED_CONFIG_VALUE)
            {
                cli_printf("nfc ERROR: session event detect byte 2 register still has the wrong value! 0x%x\n",
                    regVal[0]);

                // return with an error
                return -1;
            }

            // Read the watchdog configuration register
            if(nfc_readI2cData(NTAG5_SLAVE_ADR, I2C_WDT_CONF_REG_ADR, 
                regVal, 4, true))
            {
                cli_printfError("nfc ERROR: Can't read configuration register: 0x%x \n", 
                    I2C_WDT_CONF_REG_ADR);

                // return with an error
                return -1;
            }

            // check if the watchdog registers doesn't contain the correct values
            if((regVal[I2C_WDT_CONF_LSB_REG_BYTE] != I2C_WDT_CONF_LSB_VALUE) ||
                (regVal[I2C_WDT_CONF_MSB_REG_BYTE] != I2C_WDT_CONF_MSB_VALUE) ||
                (regVal[I2C_WDT_WDT_EN_REG_BYTE] != I2C_WDT_WDT_EN_VALUE))
            {
                cli_printf("nfc ERROR: configuration WDT registers still has the wrong values! 0: 0x%x, 1: 0x%x, 2: 0x%x\n",
                    regVal[0], regVal[1], regVal[2]);

                // return with an error
                return -1;
            }

            // read the watchdog session register
            if(i2c_nfcReadSessionRegByte(NTAG5_SLAVE_ADR, I2C_WDT_SES_REG_ADR, 
                I2C_WDT_CONF_LSB_REG_BYTE, regVal, 3))
            {
                cli_printfError("nfc ERROR: Can't read session register 0x%x\n",
                    I2C_WDT_SES_REG_ADR);

                // return with an error
                return -1;
            }

            // check if the watchdog session register doesn't contain the correct values
            if((regVal[I2C_WDT_CONF_LSB_REG_BYTE] != I2C_WDT_CONF_LSB_VALUE) ||
                (regVal[I2C_WDT_CONF_MSB_REG_BYTE] != I2C_WDT_CONF_MSB_VALUE) ||
                (regVal[I2C_WDT_WDT_EN_REG_BYTE] != I2C_WDT_WDT_EN_VALUE))
            {
                cli_printf("NFC session WDT register still has the wrong values! 0: 0x%x, 1: 0x%x, 2: 0x%x\n",
                    regVal[0], regVal[1], regVal[2]);

                // return with an error
                return -1;
            }
        }

        // say it is initialized
        gNfcInitialized = true;

        // Check if the self-test shouldn't be skipped
        if(!skipSelfTest)
        {
            // output that the self test succeeded
            cli_printf("SELF-TEST NFC: \e[32mPASS\e[39m\n");
        }

        // put the outdated data notice in the NTAG
        nfc_updateBMSStatus(true, true);
    }

    // return to the user
    return lvRetValue;
}

/*!
 * @brief   This function can be used to set the hard power-down (HPD) mode of the NFC 
 *    
 * @param   HPD if true, the microcontroller will set the NFC chip in hard power-down mode. 
 *          if false, it will disable this mode.
 * @return  0 if ok, -1 if there is an error
 * @example 
 *          if(nfc_setHPD())
 *          {
 *            // do something with the error
 *          }
 */
int nfc_setHPD(bool HPD)
{
    int lvRetValue;

    // set the HPD (hard power-down) pin of the NFC high to consume power
    lvRetValue = gpio_writePin(NFC_HPD, HPD);

    // check for errors
    if(lvRetValue)
    {
        cli_printfError("nfc ERROR: could not set the HPD pin to %d!\n", HPD);
    } 

    // return
    return lvRetValue;
}

/*!
 * @brief   This function can be used to update the BMS parameter to the NTAG
 * @note  Blocking
 *    
 * @param   setOutdatedText If true it will set the text "outdated, please tap again!",
 *              If false, it will update the NTAG with the latest BMS data.
 * @param   wakingUpMessage If true it will set the waking up text (for sleep mode)
 *              If false, it will set the charge-relaxation string. 
 *
 * @return  0 if ok, -1 if there is an error
 */
int nfc_updateBMSStatus(bool setOutdatedText, bool wakingUpMessage)
{
    // define local variables here 
    float floatValue1;
    uint8_t uint8Val[4] = {0, 0, 0, 0};
    void* dataReturn;
    int intValue, remainer, lvRetValue = -1;
    uint64_t uint64Val; 

    /*! @brief  This string contains the characters to be send to the NTAG as NDEF record
    *   @note   The first couple of bytes are needed for the type of NDEF record a
    *   @note   NDEFTxtRecord[3] = 0x09 instead of 0x00 means multipage read can be used
    */
    uint8_t NDEFTxtRecord[NDEF_TEXT_RECORD_LENGHT+4] = {};

    // check if the NFC is disabled
    if(gDisableNFC)
    {
        // return OK to not output an error
        return 0;
    }

    // last byte needs to be 0xFE otherwise it will be discarded afterwards

    // construct the NDEF header for the bms status text message 
    NDEFTxtRecord[0]   = BMS_STATUS_AM_VAL; 
    NDEFTxtRecord[1]   = BMS_STATUS_VA_VAL; 
    NDEFTxtRecord[2]   = BMS_STATUS_MEMLEN_VAL; 
    NDEFTxtRecord[3]   = BMS_STATUS_AFI_VAL; 
    NDEFTxtRecord[4]   = BMS_STATUS_T_FIELD_VAL; 
    NDEFTxtRecord[5]   = BMS_STATUS_MESSAGE_SIZE_VAL; 
    NDEFTxtRecord[6]   = BMS_STATUS_HEADER_VALUE; 
    NDEFTxtRecord[7]   = BMS_STATUS_TYPE_LENGHT; 
    NDEFTxtRecord[8]   = BMS_STATUS_PAYLOAD_LENGHT; 
    NDEFTxtRecord[9]   = BMS_STATUS_RECORD_TYPE; 
    NDEFTxtRecord[10]  = BMS_STATUS_LANG_EN_1_3; 
    NDEFTxtRecord[11]  = BMS_STATUS_LANG_EN_2_3;
    NDEFTxtRecord[12]  = BMS_STATUS_LANG_EN_3_3; 

    // check if the outdated text needs to be entered
    if(setOutdatedText)
    {
        // check which text needs to be set in the NTAG
        // if it is the waking up text
        if(wakingUpMessage)
        {
            // Set the lenght values different
            NDEFTxtRecord[8] = (sizeof(gWakingUpString) + 
                0) + 3;

            NDEFTxtRecord[5] = NDEFTxtRecord[8] + 4; 

            // set the "outdated, please tap again!" data
            strcpy((char *)&NDEFTxtRecord[V_OUT_STRING_BEGIN_INDEX], gWakingUpString);

            // write to NTAG's SRAM
            lvRetValue = nfc_writeI2cData(NTAG5_SLAVE_ADR, NTAG_MEM_BLOCK_START_SRAM, 
                NDEFTxtRecord, ((((NDEF_HEADER_STRING_LEGHT + sizeof(gWakingUpString) + 
                0)+4)>>2)<<2), true);

            // check for errors
            if(lvRetValue)
            {
    #ifdef DEBUG_NFC
                cli_printfError("nfc ERROR: Can't write data to register: 0x%x NFC read?\n", 
                    NTAG_MEM_EEPROM_START);
    #endif

                // check if the error is because the NFC is occupied
                if(lvRetValue == ERROR_COULD_NOT_WRITE)
                {
                    cli_printfWarning("NOTICE: NFC is read out, tap again for updated values\n");
                    // set the returnvalue to 0, because this is no error
                    lvRetValue = 0;
                }
            }
            else
            {
                // it went OK
                lvRetValue = 0;
            }
        }
        // if it needs to be the charge-relaxation text
        else
        {
            // Set the lenght values different
            NDEFTxtRecord[8] = (sizeof(gChargeRelaxString) + 
                0) + 3;

            NDEFTxtRecord[5] = NDEFTxtRecord[8] + 4; 

            // set the "outdated, please tap again!" data
            strcpy((char *)&NDEFTxtRecord[V_OUT_STRING_BEGIN_INDEX], gChargeRelaxString);

            // write to NTAG's SRAM
            lvRetValue = nfc_writeI2cData(NTAG5_SLAVE_ADR, NTAG_MEM_BLOCK_START_SRAM, 
                NDEFTxtRecord, ((((NDEF_HEADER_STRING_LEGHT + sizeof(gChargeRelaxString) + 
                0)+4)>>2)<<2), true);

            // check for errors
            if(lvRetValue)
            {
    #ifdef DEBUG_NFC
                cli_printfError("nfc ERROR: Can't write data to register: 0x%x NFC read?\n", 
                    NTAG_MEM_EEPROM_START);
    #endif

                // check if the error is because the NFC is occupied
                if(lvRetValue == ERROR_COULD_NOT_WRITE)
                {
                    cli_printfWarning("NOTICE: NFC is read out, tap again for updated values\n");
                    // set the returnvalue to 0, because this is no error
                    lvRetValue = 0;
                }
            }
            else
            {
                // it went OK
                lvRetValue = 0;
            }
        }
    }
    // if the BMS data need to be written
    else
    {

#ifdef DEBUG_NFC

        // output the header
        cli_printf("Header: \n");

        for(i = 0; i < (NDEF_HEADER_STRING_LEGHT>>2) + 1; i++)
        {
            cli_printf("%d: 0x%x, ", i, NDEFTxtRecord[(i*4)]);
            cli_printf("%x, ", NDEFTxtRecord[(i*4)+1]);
            cli_printf("%x, ", NDEFTxtRecord[(i*4)+2]);
            cli_printf("%x \n", NDEFTxtRecord[(i*4)+3]);    
        }

#endif

        // copy the default payload data in the NDEFTxtRecord
        strcpy((char *)&NDEFTxtRecord[V_OUT_STRING_BEGIN_INDEX], V_OUT_STRING);
        strcpy((char *)&NDEFTxtRecord[C_BATT_STRING_BEGIN_INDEX], C_BATT_STRING);
        strcpy((char *)&NDEFTxtRecord[S_CHARGE_STRING_BEGIN_INDEX], S_CHARGE_STRING);
        strcpy((char *)&NDEFTxtRecord[S_HEALTH_STRING_BEGIN_INDEX], S_HEALTH_STRING);
        strcpy((char *)&NDEFTxtRecord[I_OUT_STRING_BEGIN_INDEX], I_OUT_STRING);
        strcpy((char *)&NDEFTxtRecord[N_CHARGES_STRING_BEGIN_INDEX], N_CHARGES_STRING);
        strcpy((char *)&NDEFTxtRecord[BATT_ID_STRING_BEGIN_INDEX], BATT_ID_STRING);
        strcpy((char *)&NDEFTxtRecord[MODEL_ID_STRING_BEGIN_INDEX], MODEL_ID_STRING);
        strcpy((char *)&NDEFTxtRecord[STATE_STRING_BEGIN_INDEX], STATE_STRING);

        // add the last character
        NDEFTxtRecord[(((sizeof(NDEFTxtRecord)/4)*4) - 1)] = NDEF_TEXT_END_BYTE;

        // check if initialized
        if(!gNfcInitialized)
        {
            // set the error to exit failure
            lvRetValue = EXIT_FAILURE;

            // output error
            cli_printfError("NFC ERROR: NFC not initialized!\n");

            // output warning to disable the NFC
            cli_printfWarning("NFC WARNING: NFC will be disabled!\n");

            // disable the NFC
            gDisableNFC = true;
        }
        else
        {
            // get the output voltage
            dataReturn = (int32_t*)data_getParameter(V_OUT, &floatValue1, NULL);

            // check for error 
            if(dataReturn == NULL)
            {
                // set the default value
                floatValue1 = V_OUT_DEFAULT;

                // error output
                cli_printfError("NFC ERROR: could not get v-out!\n");

                // return with an error
                return lvRetValue;
            }

            // convert the floating point value to an integer and multiply by 1000
            intValue = (int)(floatValue1*NORMAL_TO_MILI);

            // check if it is a negative value
            if(intValue < 0)
            {
                // make sure to make the remainer positive
                remainer = (-1*intValue)%NORMAL_TO_MILI;
            }
            else
            {
                // make the remainer
                remainer = (intValue)%NORMAL_TO_MILI;
            }

            // convert the float value to a 6 digit string value
            snprintf((char *)&NDEFTxtRecord[V_OUT_STRING_DATA_INDEX], V_OUT_STRING_DATA_LENGHT, 
                "%02d.%03u", intValue/1000, remainer);

            // overwrite the NULL character
            NDEFTxtRecord[V_OUT_STRING_DATA_INDEX+6] = 'v';

            // get batt temperature (if sensor is enabled)
            // check sensor enable variable
            dataReturn = (int32_t*)data_getParameter(SENSOR_ENABLE, &uint8Val[0], NULL);

            // check for error 
            if(dataReturn == NULL)
            {
                // set the default value
                uint8Val[0] = SENSOR_ENABLE_DEFAULT;

                // error output
                cli_printfError("NFC ERROR: could not get sensor-enable!\n");

                // return with an error
                return lvRetValue;
            }

            // if enabled 
            if(uint8Val[0] & 1)
            {
                // print the battery temperature
                dataReturn = (int32_t*)data_getParameter(C_BATT, &floatValue1, NULL);

                // check for error 
                if(dataReturn == NULL)
                {
                    // set the default value
                    floatValue1 = C_BATT_DEFAULT;

                    // error output
                    cli_printfError("NFC ERROR: could not get c-batt!\n");

                    // return with an error
                    return lvRetValue;
                }

                // convert the floating point value to an integer and multiply by 10
                intValue = (int)(floatValue1*CONVERT_TO_1_10TH);

                // check if it is a negative value
                if(intValue < 0)
                {
                    // make sure to make the remainer positive
                    remainer = (-1*intValue)%CONVERT_TO_1_10TH;
                }
                else
                {
                    // make the remainer
                    remainer = (intValue)%CONVERT_TO_1_10TH;
                }

                // convert the float value to a 6/7 digit string value
                snprintf((char *)&NDEFTxtRecord[C_BATT_STRING_DATA_INDEX], C_BATT_STRING_DATA_LENGHT, 
                    "%03d.%01u", intValue/10, remainer);

                // overwrite the NULL character
                NDEFTxtRecord[C_BATT_STRING_DATA_INDEX+5] = 'C';
            }

            // get the state of charge
            dataReturn = (int32_t*)data_getParameter(S_CHARGE, &uint8Val[0], NULL);

            // check for error 
            if(dataReturn == NULL)
            {
                // set the default value
                uint8Val[0] = S_CHARGE_DEFAULT;

                // error output
                cli_printfError("NFC ERROR: could not get s-charge!\n");

                // return with an error
                return lvRetValue;
            }

            // convert the float value to a 6 digit string value
            snprintf((char *)&NDEFTxtRecord[S_CHARGE_STRING_DATA_INDEX], S_CHARGE_STRING_DATA_LENGHT, 
                "%03u", uint8Val[0]);

            // overwrite the NULL character
            NDEFTxtRecord[S_CHARGE_STRING_DATA_INDEX+3] = '%';

            // get the state of health
            dataReturn = (int32_t*)data_getParameter(S_HEALTH, &uint8Val[0], NULL);

            // check for error 
            if(dataReturn == NULL)
            {
                // set the default value
                uint8Val[0] = S_HEALTH_DEFAULT;

                // error output
                cli_printfError("NFC ERROR: could not get s-health!\n");

                // return with an error
                return lvRetValue;
            }

            // convert the float value to a 6 digit string value
            snprintf((char *)&NDEFTxtRecord[S_HEALTH_STRING_DATA_INDEX], S_HEALTH_STRING_DATA_LENGHT, 
                "%03u%", uint8Val[0]);

            // overwrite the NULL character
            NDEFTxtRecord[S_HEALTH_STRING_DATA_INDEX+3] = '%';

            // get the average current
            dataReturn = (int32_t*)data_getParameter(I_BATT_AVG, &floatValue1, NULL);

            // check for error 
            if(dataReturn == NULL)
            {
                // set the default value
                floatValue1 = I_BATT_AVG_DEFAULT;

                // error output
                cli_printfError("NFC ERROR: could not get i-batt-avg!\n");

                // return with an error
                return lvRetValue;
            }

            // convert the floating point value to an integer and multiply by 1000
            intValue = (int)(floatValue1*NORMAL_TO_MILI);

            // check if it is a negative value
            if(intValue < 0)
            {
                // make sure to make the remainer positive
                remainer = (-1*intValue)%NORMAL_TO_MILI;
            }
            else
            {
                // make the remainer
                remainer = (intValue)%NORMAL_TO_MILI;
            }

            // convert the float value to a 6/7 digit string value
            snprintf((char *)&NDEFTxtRecord[I_OUT_STRING_DATA_INDEX], I_OUT_STRING_DATA_LENGHT, 
                "%03d.%03u", intValue/1000, remainer);

            // overwrite the NULL character
            NDEFTxtRecord[I_OUT_STRING_DATA_INDEX+7] = 'A';

            // get the n-charges
            dataReturn = (int32_t*)data_getParameter(N_CHARGES, &intValue, NULL);

            // check for error 
            if(dataReturn == NULL)
            {
                // set the default value
                intValue = N_CHARGES_DEFAULT;

                // error output
                cli_printfError("NFC ERROR: could not get n-charges!\n");

                // return with an error
                return lvRetValue;
            }

            // limit the intValue to uint16_t
            intValue &= UINT16_MAX;

            // convert the float value to a 6 digit string value
            snprintf((char *)&NDEFTxtRecord[N_CHARGES_STRING_DATA_INDEX], N_CHARGES_STRING_DATA_LENGHT, 
                "%05u%", intValue);

            // overwrite the NULL character
            NDEFTxtRecord[N_CHARGES_STRING_DATA_INDEX+5] = '\n';

            // get the batt-id
            dataReturn = (int32_t*)data_getParameter(BATT_ID, &uint8Val[0], NULL);

            // check for error 
            if(dataReturn == NULL)
            {
                // set the default value
                uint8Val[0] = BATT_ID_DEFAULT;

                // error output
                cli_printfError("NFC ERROR: could not get batt-id!\n");

                // return with an error
                return lvRetValue;
            }

            // convert the float value to a 6 digit string value
            snprintf((char *)&NDEFTxtRecord[BATT_ID_STRING_DATA_INDEX], BATT_ID_STRING_DATA_LENGHT, 
                "%03u%", uint8Val[0]);

            // overwrite the NULL character
            NDEFTxtRecord[BATT_ID_STRING_DATA_INDEX+3] = '\n';

            // get the model-id
            dataReturn = (int32_t*)data_getParameter(MODEL_ID, &uint64Val, NULL);

            // check for error 
            if(dataReturn == NULL)
            {
                // set the default value
                uint64Val = MODEL_ID_DEFAULT;

                // error output
                cli_printfError("NFC ERROR: could not get model-id!\n");

                // return with an error
                return lvRetValue;
            }

            // convert the float value to a 6 digit string value
            snprintf((char *)&NDEFTxtRecord[MODEL_ID_STRING_DATA_INDEX], MODEL_ID_STRING_DATA_LENGHT, 
                "%020llu", uint64Val);

            // overwrite the NULL character
            NDEFTxtRecord[MODEL_ID_STRING_DATA_INDEX+20] = '\n';

            // get the state
            intValue = (int)data_getMainState();

            // check if in the charge state
            if((states_t)intValue == CHARGE)
            {
                // get the charge state
                intValue = (uint8_t)data_getChargeState();

                // put the charge state string at the correct location
                cli_getStateString(false, (uint8_t)intValue, 
                    (char *)&NDEFTxtRecord[STATE_STRING_DATA_INDEX]);
            }
            // if not in the charge state
            else
            {
                // put the main state string at the correct location
                cli_getStateString(true, (uint8_t)intValue, 
                    (char *)&NDEFTxtRecord[STATE_STRING_DATA_INDEX]);
            }

            // check if the size is not too much
            if((sizeof(NDEFTxtRecord)/4) > NTAG_MEM_BLOCK_END_SRAM)
            {
                // error
                cli_printfError("nfc ERROR: NDEFTxtRecord too large for SRAM!\n");

                // return with an error
                return lvRetValue;
            }

            // write to NTAG's SRAM
            lvRetValue = nfc_writeI2cData(NTAG5_SLAVE_ADR, NTAG_MEM_BLOCK_START_SRAM, NDEFTxtRecord, 
                (((NDEF_TEXT_RECORD_LENGHT+4)>>2)<<2), true);
            // check for errors
            if(lvRetValue)
            {
#ifdef DEBUG_NFC
                cli_printfError("nfc ERROR: Can't write data to register: 0x%x NFC read?\n", 
                    NTAG_MEM_EEPROM_START);
#endif

                // check if the error is because the NFC is occupied
                if(lvRetValue == ERROR_COULD_NOT_WRITE)
                {
                    cli_printfWarning("WARNING: NFC is not updated due to user interaction\n");
                    // set the returnvalue to 0, because this is no error
                    lvRetValue = 0;
                }

                // return with an error
                return lvRetValue;
            }
            else
            {
                // it went OK
                lvRetValue = 0;
            }
        }

//  this commented code shows how to update the NTAG 4 bytes at a time for RAM and EEPROM. 
//  this is needed if the user want to update it to the EEPROM of the NTAG (slower update rate is needed as well)
//     for(i = 0; i < (sizeof(NDEFTxtRecord)/4); i++)
//     {
//       // make the data to write to the EEPROM
//       uint8Val[0] = NDEFTxtRecord[(i*4)];
//       uint8Val[1] = NDEFTxtRecord[(i*4)+1];
//       uint8Val[2] = NDEFTxtRecord[(i*4)+2];
//       uint8Val[3] = NDEFTxtRecord[(i*4)+3];

// #ifdef DEBUG_NFC
//       cli_printf("writing %2d: %c %c %c %c\n", i, uint8Val[0], uint8Val[1], uint8Val[2], uint8Val[3]);
// #endif

//       // write the data to the NTAG if allowed (don't wait if the NTAG is locked/used by NFC)
//       // Other option: 255 bytes may be written to SRAM in one I2C transfer
//       if(nfc_writeI2cData(NTAG5_SLAVE_ADR, NTAG_MEM_BLOCK_START_SRAM + i, uint8Val, 4, true))
//       {
// #ifdef DEBUG_NFC
//         cli_printfError("nfc ERROR: Can't write data to register: 0x%x NFC read?\n", 
//           NTAG_MEM_EEPROM_START + i);
// #endif
//       }

//       // Wait for 4ms before writing to EEPROM (or 0.4ms for SRAM) again
//       // Other option: 255 bytes may be written to SRAM in one I2C transfer
//       usleep(NTAG_SRAM_WRITE_DELAY_US);
//     }
    }

    // return the value
    return lvRetValue;
}

/****************************************************************************
 * private Functions
 ****************************************************************************/

/*!
 * @brief   This function can be used to write a data via the I2C to the NTAGs normal register (not session register)
 * @note    It will check if it is allowed to write to the NTAG before writing. 
 *  
 * @param   slaveAdr the slave address of the I2C device
 * @param   regAdr the address of the register to write to (2 bytes)
 * @param   writeReg address of the variable to write
 * @param   writeBytes the amount of bytes to write max 255
 * @param   dontWait if true, it doesn't wait until available, otherwise it waits with timeout
 *
 * @return  0 if ok, -1 if there is an error
 */
int nfc_writeI2cData(uint8_t slaveAdr, uint16_t regAdr, uint8_t* writeReg, uint8_t writeBytes, 
    bool dontWait)
{
    int ret;
    uint8_t regVal[2];
    uint16_t tries = 0;
    bool valuesWritten = false;

    // check if the number of bytes is a division of 4
    if(writeBytes % 4 == 0)
    {
        // do this at least once 
        do
        {
            // check the session register if the NTAG is busy
            // read the STATUS_REG session register
            ret = i2c_nfcReadSessionRegByte(NTAG5_SLAVE_ADR, I2C_STATUS_SES_REG_ADR, 
                I2C_STATUS0_REG_BYTE, regVal, 2);

            // check for errors
            if(ret)
            {
                // output error to the user
                cli_printfError("nfc ERROR: Can't read register: 0x%x byte %d E%d\n", 
                    I2C_STATUS_SES_REG_ADR, 0, ret);
            }
            else
            {
                // check if there is not an NFC field, 
                // if the arbitter is not locked on NFC (if allowed to write)
                // and if the event detection GPIO is not active
                if(((regVal[I2C_STATUS0_REG_BYTE] & STATUS0_NFC_FIELD_OK_PRESENT) == STATUS0_NFC_FIELD_OK_ABSENT) &&
                    ((regVal[I2C_STATUS1_REG_BYTE] & STATUS1_NFC_LOCKED_LOCKED) == STATUS1_NFC_LOCKED_UNLOCKED)
                    && (gpio_readPin(NFC_ED) == NFC_ED_PIN_INACTIVE))
                {
                    // write the data 
                    ret |= i2c_writeData(slaveAdr, regAdr, writeReg, writeBytes);

                    // state that you've written the value(s)
                    valuesWritten = true;
                }
                else if(dontWait)
                {
                    // error back 
                    ret = ERROR_COULD_NOT_WRITE;
                }
                else
                {
                    // sleep for a little while
                    usleep(1);
                }
  #ifdef DEBUG_NFC_WRITE
                if(gpio_readPin(NFC_ED) != NFC_ED_PIN_INACTIVE)
                {
                    cli_printfGreen("NFC_ED active\n");
                }

                if(((regVal[I2C_STATUS0_REG_BYTE] & STATUS0_NFC_FIELD_OK_PRESENT) == STATUS0_NFC_FIELD_OK_PRESENT))
                {
                    cli_printf("NFC field present! 0x%x\n", regVal[I2C_STATUS0_REG_BYTE]);
                }

                if(((regVal[I2C_STATUS1_REG_BYTE] & STATUS1_NFC_LOCKED_LOCKED) == STATUS1_NFC_LOCKED_LOCKED))
                {
                    cli_printf("NFC locked! 0x%x\n", regVal[I2C_STATUS1_REG_BYTE]);
                }
  #endif
            }

        // loop if dontWait is false and while the tries have not been done yet and there is no error
        }while((!dontWait) && ((++tries) < AMOUNT_RETRIES) && 
            (!ret) && (!valuesWritten));
    }
    // not writing a multiple of 4 bytes
    else
    {
        // ouptut to the user
        cli_printfError("nfc ERROR: Not writing a multiple of 4 bytes to 0x%x!\n", regAdr);

        // return an error
        ret = -1;
    }

    // check if timeout hapend 
    if(tries >= AMOUNT_RETRIES)
    {
        // error back 
        ret = ERROR_COULD_NOT_WRITE;
    }

    // return
    return ret;
}

/*!
 * @brief   This function can be used to read a data via the I2C from the NTAGs normal register (not session register)
 * @note    It will check if it is allowed to read from the NTAG before reading. 
 *  
 * @param   slaveAdr the slave address of the I2C device
 * @param   regAdr the address of the register to read from (2 bytes)
 * @param   readReg address of the variable to read
 * @param   readBytes the amount of bytes to read max 255
 * @param   reTry if true, it will retry to read if failed (maybe WDT timeout happend)
 *
 * @return  0 if ok, -1 if there is an error
 */
int nfc_readI2cData(uint8_t slaveAdr, uint16_t regAdr, uint8_t* readReg, uint8_t readBytes, 
    bool reTry)
{
    int ret;
    uint8_t regVal[2];
    uint16_t tries = 0;
    bool valuesRead = false;

    // do this at least once 
    do
    {
        // check the session register if the NTAG is busy
        // read the STATUS_REG session register
        ret = i2c_nfcReadSessionRegByte(NTAG5_SLAVE_ADR, I2C_STATUS_SES_REG_ADR, 
            I2C_STATUS0_REG_BYTE, regVal, 2);

        // check for errors
        if(ret)
        {
            // output error to the user
            cli_printfError("nfc ERROR: Can't read register: 0x%x byte %d E%d\n", 
                I2C_STATUS_SES_REG_ADR, 0, ret);
        }
        else
        {
            // check if there is not an NFC field, 
            // if the arbitter is not locked on NFC (if allowed to write)
            // and if the event detection GPIO is not active
            if(((regVal[I2C_STATUS0_REG_BYTE] & STATUS0_NFC_FIELD_OK_PRESENT) == STATUS0_NFC_FIELD_OK_ABSENT) &&
                ((regVal[I2C_STATUS1_REG_BYTE] & STATUS1_NFC_LOCKED_LOCKED) == STATUS1_NFC_LOCKED_UNLOCKED)
                && (gpio_readPin(NFC_ED) == NFC_ED_PIN_INACTIVE))
            {
                // Read the watchdog configuration register
                ret = i2c_readData(slaveAdr, regAdr, readReg, readBytes, false);
                if(ret)
                {
                    if(((tries) >= AMOUNT_RETRIES) || !reTry)
                    {
                        cli_printfError("nfc ERROR: Can't read register: 0x%x \n", 
                            regAdr);
                    }
                    else
                    {
                        cli_printf("nfc: re-trying read to 0x%x, could be cut off by WDT!\n",
                            regAdr);
                        // keep trying
                        ret = 0;
                    }
                }
                else
                {
                    valuesRead = true;
                }
            }
            else if(reTry)
            {
                // sleep for a little time
                usleep(1);
            }
            else
            {
                // if reTry is disabled state that read is not possible
                ret = ERROR_COULD_NOT_READ;
            }
        }
    // loop if retry is true and while the tries have not been done yet and there is no error
    }while((reTry) && ((++tries) < AMOUNT_RETRIES) && 
        (!ret) && (!valuesRead));

    // check if timeout hapend 
    if(tries >= AMOUNT_RETRIES)
    {
        // error back 
        ret = ERROR_COULD_NOT_READ;
    }

    // return
    return ret;
}
