/****************************************************************************
 * nxp_bms/BMS_v1/src/BCC/bcc_spiwrapper.c
 *
 * BSD 3-Clause License
 * 
 * Copyright 2021 NXP
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

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "bcc_spiwrapper.h"
#include "spi.h"
#include "cli.h"

/*******************************************************************************
 * Defines
 ******************************************************************************/

/*******************************************************************************
 * Private functions declerations
 ******************************************************************************/

/*******************************************************************************
 * Public functions
 ******************************************************************************/

/* @brief This function initializes the Battery Cell Controller device(s),
 * configures its registers, assigns CID and initializes internal driver data.
 *
 * @note  Thread safe, it will lock on the thread
 * @note  Assumes you have the bcc SPI lock to execute, otherwise it will wait.
 *
 * @param drvConfig Pointer to driver instance configuration.
 * @param devConf Initialization values of BCC device registers specified
 *                by BCC_INIT_CONF_REG_ADDR. If NULL, registers are not
 *                initialized.
 *                devConf[0][x] belongs to device with CID 1,
 *                devConf[1][x] belongs to device with CID 2, etc.
 *
 * @return bcc_status_t Error code.
 */
bcc_status_t bcc_spiwrapper_BCC_Init(bcc_drv_config_t* const drvConfig,
    const uint16_t devConf[][BCC_INIT_CONF_REG_CNT])
{
    bcc_status_t ret;

    // lock the spi
    if(spi_lockNotUnlockBCCSpi(true))
    {
        // error
        cli_printfError("spiwrapper ERROR: Could not lock the spi in init!\n");
    }

    // lock on this thread (no task switch!)
    sched_lock();

    // Do the init
    ret = BCC_Init(drvConfig, devConf);

    // unlock this thread (enable task switch!)
    sched_unlock();

    // unlock the spi again
    if(spi_lockNotUnlockBCCSpi(false))
    {
        // error
        cli_printfError("spiwrapper ERROR: Could not unlock the spi in init!\n");
    }

    // return
    return ret;
}

/*!
 * @brief This function uses No Operation command of BCC to verify communication
 * with device specified by CID without performing any operation.
 *
 * @note  Thread safe, it will lock on the thread
 * @note  Assumes you have the bcc SPI lock to execute, otherwise it will wait.
 *
 * @param drvConfig Pointer to driver instance configuration.
 * @param cid Cluster Identification Address.
 *
 * @return bcc_status_t Error code.
 */
bcc_status_t bcc_spiwrapper_BCC_VerifyCom(bcc_drv_config_t* const drvConfig, bcc_cid_t cid)
{
    bcc_status_t ret;

    // lock the spi
    if(spi_lockNotUnlockBCCSpi(true))
    {
        // error
        cli_printfError("spiwrapper ERROR: Could not lock the spi in verify!\n");
    }

    // lock on this thread (no task switch!)
    sched_lock();

    // Do the verify
    ret = BCC_VerifyCom(drvConfig, cid);

    // unlock this thread (enable task switch!)
    sched_unlock();

    // unlock the spi again
    if(spi_lockNotUnlockBCCSpi(false))
    {
        // error
        cli_printfError("spiwrapper ERROR: Could not unlock the spi in verify!\n");
    }

    // return
    return ret;
}

/*!
 * @brief This function reads a value from addressed register (or desired
 * number of registers) of selected Battery Cell Controller device.
 *
 * In case of simultaneous read of more registers, address is incremented
 * in ascending manner.
 *
 * @note  Thread safe, it will lock on the thread
 * @note  Assumes you have the bcc SPI lock to execute, otherwise it will wait.
 * @note  Assumes you have the bcc SPI lock to execute, otherwise it will wait.
 *
 * @param drvConfig Pointer to driver instance configuration.
 * @param cid Cluster Identification Address.
 * @param regAddr Register address. See BCC header file with register map for
 *                possible values.
 * @param regCnt Number of registers to read.
 * @param regVal Pointer to memory where content of selected 16 bit registers
 *               is stored.
 *
 * @return bcc_status_t Error code.
 */
bcc_status_t bcc_spiwrapper_BCC_Reg_Read(bcc_drv_config_t* const drvConfig, bcc_cid_t cid,
    uint8_t regAddr, uint8_t regCnt, uint16_t* regVal)
{
    bcc_status_t ret;

    // lock the spi to be sure to do all the reads
    if(spi_lockNotUnlockBCCSpi(true))
    {
        // error
        cli_printfError("spiwrapper ERROR: Could not lock the spi!\n");
    }

    // lock on this thread (no task switch!)
    sched_lock();

    // Do all the reads from the BCC
    ret = BCC_Reg_Read(drvConfig, cid, regAddr, regCnt, regVal);

    // unlock this thread (enable task switch!)
    sched_unlock();

    // unlock the spi again
    if(spi_lockNotUnlockBCCSpi(false))
    {
        // error
        cli_printfError("spiwrapper ERROR: Could not unlock the spi!\n");
    }

    // return
    return ret;
}

/*!
 * @brief This function writes a value to addressed register of selected Battery
 * Cell Controller device.
 *
 * @note  Thread safe, it will lock on the thread
 * @note  Assumes you have the bcc SPI lock to execute, otherwise it will wait.
 *
 * @param drvConfig Pointer to driver instance configuration.
 * @param cid Cluster Identification Address.
 * @param regAddr Register address. See BCC header file with register map for
 *                possible values.
 * @param regVal New value of selected register.
 * @param retReg Automatic response of BCC, which contains updated register
 *               (TPL mode) or a register addressed in previous access
 *               (SPI mode). You can pass NULL when you do not care about the
 *               response.
 *
 * @return bcc_status_t Error code.
 */
bcc_status_t bcc_spiwrapper_BCC_Reg_Write(bcc_drv_config_t* const drvConfig, bcc_cid_t cid,
    uint8_t regAddr, uint16_t regVal, uint16_t* retReg)
{
    bcc_status_t ret;

    // lock the spi to be sure to do all the writes
    if(spi_lockNotUnlockBCCSpi(true))
    {
        // error
        cli_printfError("spiwrapper ERROR: Could not lock the spi in write!\n");
    }

    // lock on this thread (no task switch!)
    sched_lock();

    // Do all the writes to the BCC
    ret = BCC_Reg_Write(drvConfig, cid, regAddr, regVal, retReg);

    // unlock this thread (enable task switch!)
    sched_unlock();

    // unlock the spi again
    if(spi_lockNotUnlockBCCSpi(false))
    {
        // error
        cli_printfError("spiwrapper ERROR: Could not unlock the spi in write!\n");
    }

    // return
    return ret;
}

/*!
 * @brief This function updates content of a selected register. It affects bits
 * specified by a bit mask only.
 *
 * @note  Thread safe, it will lock on the thread
 * @note  Assumes you have the bcc SPI lock to execute, otherwise it will wait.
 *
 * @param drvConfig Pointer to driver instance configuration.
 * @param cid Cluster Identification Address.
 * @param regAddr Register address. See BCC header file with register map for
 *                possible values.
 * @param regMask Bit mask. Bits set to 1 will be updated.
 * @param regVal  New value of register bits defined by bit mask.
 *
 * @return bcc_status_t Error code.
 */
bcc_status_t bcc_spiwrapper_BCC_Reg_Update(bcc_drv_config_t* const drvConfig, bcc_cid_t cid,
    uint8_t regAddr, uint16_t regMask, uint16_t regVal)
{
    bcc_status_t ret;

    // lock the spi to be sure to do the whole update first
    if(spi_lockNotUnlockBCCSpi(true))
    {
        // error
        cli_printfError("spiwrapper ERROR: Could not lock the spi in update!\n");
    }

    // lock on this thread (no task switch!)
    sched_lock();

    // Do the update to the BCC
    ret = BCC_Reg_Update(drvConfig, cid, regAddr, regMask, regVal);

    // unlock this thread (enable task switch!)
    sched_unlock();

    // unlock the spi again
    if(spi_lockNotUnlockBCCSpi(false))
    {
        // error
        cli_printfError("spiwrapper ERROR: Could not unlock the spi in update!\n");
    }

    // return
    return ret;
}

/*!
 * @brief This function starts ADC conversion. It sets Start of Conversion bit
 * and new value of TAG ID in ADC_CFG register.
 *
 * @note  Thread safe, it will lock on the thread
 * @note  Assumes you have the bcc SPI lock to execute, otherwise it will wait.
 *
 * TAG ID is incremented for each conversion. You can use function
 * BCC_Meas_IsConverting to check conversion status.
 *
 * @param drvConfig Pointer to driver instance configuration.
 * @param cid Cluster Identification Address.
 *
 * @return bcc_status_t Error code.
 */
bcc_status_t bcc_spiwrapper_BCC_Meas_StartConversion(bcc_drv_config_t* const drvConfig,
    bcc_cid_t cid)
{
    bcc_status_t ret;

    // lock the spi 
    if(spi_lockNotUnlockBCCSpi(true))
    {
        // error
        cli_printfError("spiwrapper ERROR: Could not lock the spi in StartConversion!\n");
    }

    // lock on this thread (no task switch!)
    sched_lock();

    // Do all the start conversion with the BCC
    ret = BCC_Meas_StartConversion(drvConfig, cid);

    // unlock this thread (enable task switch!)
    sched_unlock();

    // unlock the spi again
    if(spi_lockNotUnlockBCCSpi(false))
    {
        // error
        cli_printfError("spiwrapper ERROR: Could not unlock the spi in StartConversion!\n");
    }

    // return
    return ret;
}


/*!
 * @brief This function checks status of conversion defined by End of Conversion
 * bit in ADC_CFG register.
 *
 * @note  Thread safe, it will lock on the thread
 * @note  Assumes you have the bcc SPI lock to execute, otherwise it will wait.
 *
 * @param drvConfig Pointer to driver instance configuration.
 * @param cid Cluster Identification Address.
 * @param completed Pointer to check result. True if a conversion is complete.
 *
 * @return bcc_status_t Error code.
 */
bcc_status_t bcc_spiwrapper_BCC_Meas_IsConverting(bcc_drv_config_t* const drvConfig,
    bcc_cid_t cid, bool *completed)
{
    bcc_status_t ret;

    // lock the spi 
    if(spi_lockNotUnlockBCCSpi(true))
    {
        // error
        cli_printfError("spiwrapper ERROR: Could not lock the spi in isConverting!\n");
    }

    // lock on this thread (no task switch!)
    sched_lock();

    // Check if conversion is complete
    ret = BCC_Meas_IsConverting(drvConfig, cid, completed);

    // unlock this thread (enable task switch!)
    sched_unlock();

    // unlock the spi again
    if(spi_lockNotUnlockBCCSpi(false))
    {
        // error
        cli_printfError("spiwrapper ERROR: Could not unlock the spi in isConverting!\n");
    }

    // return
    return ret;
}

/*!
 * @brief This function reads the status registers and returns raw values.
 * You can use constants defined in bcc_mc3377x.h file.
 *
 * @note  Thread safe, it will lock on the thread
 * @note  Assumes you have the bcc SPI lock to execute, otherwise it will wait.
 *
 * @param drvConfig Pointer to driver instance configuration.
 * @param cid Cluster Identification Address.
 * @param status Array containing all fault status information provided by BCC.
 *               Indexes into the array are defined in bcc_fault_status_t
 *               enumeration placed in BCC header file. Required size of the
 *               array is 11. You can use macro BCC_STAT_CNT defined in BCC
 *               header file, which contains appropriate value.
 *
 * @return bcc_status_t Error code.
 */
bcc_status_t bcc_spiwrapper_BCC_Fault_GetStatus(bcc_drv_config_t* const drvConfig,
    bcc_cid_t cid, uint16_t status[])
{
    bcc_status_t ret;

    // lock the spi 
    if(spi_lockNotUnlockBCCSpi(true))
    {
        // error
        cli_printfError("spiwrapper ERROR: Could not lock the spi in getStatus!\n");
    }

    // lock on this thread (no task switch!)
    sched_lock();

    // Get the fault status
    ret = BCC_Fault_GetStatus(drvConfig, cid, status);

    // unlock this thread (enable task switch!)
    sched_unlock();

    // unlock the spi again
    if(spi_lockNotUnlockBCCSpi(false))
    {
        // error
        cli_printfError("spiwrapper ERROR: Could not unlock the spi in getStatus!\n");
    }

    // return
    return ret;
}

/*!
 * @brief This function clears selected fault status register.
 *
 * @note  Thread safe, it will lock on the thread
 * @note  Assumes you have the bcc SPI lock to execute, otherwise it will wait.
 *
 * @param drvConfig Pointer to driver instance configuration.
 * @param cid Cluster Identification Address.
 * @param statSel Selection of a fault status register to be cleared. See
 *                definition of this enumeration in BCC header file.
 *                COM_STATUS register is read only and cannot be cleared.
 *
 * @return bcc_status_t Error code.
 */
bcc_status_t bcc_spiwrapper_BCC_Fault_ClearStatus(bcc_drv_config_t* const drvConfig,
    bcc_cid_t cid, bcc_fault_status_t statSel)
{
    bcc_status_t ret;

    // lock the spi 
    if(spi_lockNotUnlockBCCSpi(true))
    {
        // error
        cli_printfError("spiwrapper ERROR: Could not lock the spi in clearStatus!\n");
    }

    // lock on this thread (no task switch!)
    sched_lock();

    // Clear the fault status
    ret = BCC_Fault_ClearStatus(drvConfig, cid, statSel);

    // unlock this thread (enable task switch!)
    sched_unlock();

    // unlock the spi again
    if(spi_lockNotUnlockBCCSpi(false))
    {
        // error
        cli_printfError("spiwrapper ERROR: Could not unlock the spi in clearStatus!\n");
    }

    // return
    return ret;
}

/*!
 * @brief This function enables or disables the cell balancing via
 * SYS_CFG1[CB_DRVEN] bit.
 *
 * @note  Thread safe, it will lock on the thread
 * @note  Assumes you have the bcc SPI lock to execute, otherwise it will wait.
 *
 * Note that each cell balancing driver needs to be setup separately, e.g. by
 * BCC_CB_SetIndividual function.
 *
 * @param drvConfig Pointer to driver instance configuration.
 * @param cid Cluster Identification Address.
 * @param enable Drivers state. False (all drivers are disabled) or true
 *               (drivers are enabled).
 *
 * @return bcc_status_t Error code.
 */
bcc_status_t bcc_spiwrapper_BCC_CB_Enable(bcc_drv_config_t* const drvConfig, bcc_cid_t cid,
    bool enable)
{
    bcc_status_t ret;

    // lock the spi 
    if(spi_lockNotUnlockBCCSpi(true))
    {
        // error
        cli_printfError("spiwrapper ERROR: Could not lock the spi in CB enable!\n");
    }

    // lock on this thread (no task switch!)
    sched_lock();

    // Enable or disable the CB
    ret = BCC_CB_Enable(drvConfig, cid, enable);

    // unlock this thread (enable task switch!)
    sched_unlock();

    // unlock the spi again
    if(spi_lockNotUnlockBCCSpi(false))
    {
        // error
        cli_printfError("spiwrapper ERROR: Could not unlock the spi in CB enable!\n");
    }

    // return
    return ret;
}

/*!
 * @brief This function enables or disables cell balancing for a specified cell
 * and sets its timer.
 *
 * @note  Thread safe, it will lock on the thread
 * @note  Assumes you have the bcc SPI lock to execute, otherwise it will wait.
 *
 * @param drvConfig Pointer to driver instance configuration.
 * @param cid Cluster Identification Address.
 * @param cellIndex Index of the cell. Note the cells are indexed from 0.
 * @param enable True for enabling of CB, False otherwise.
 * @param timer Timer for enabled CB driver in minutes. Note that a zero
 *              value represents 30 seconds.
 *
 * @return bcc_status_t Error code.
 */
bcc_status_t bcc_spiwrapper_BCC_CB_SetIndividual(bcc_drv_config_t* const drvConfig,
    bcc_cid_t cid, uint8_t cellIndex, bool enable, uint16_t timer)
{
    bcc_status_t ret;

    // lock the spi 
    if(spi_lockNotUnlockBCCSpi(true))
    {
        // error
        cli_printfError("spiwrapper ERROR: Could not lock the spi in CB enable!\n");
    }

    // lock on this thread (no task switch!)
    sched_lock();

    // Enable or disable the CB
    ret = BCC_CB_SetIndividual(drvConfig, cid, cellIndex, enable, timer);

    // unlock this thread (enable task switch!)
    sched_unlock();

    // unlock the spi again
    if(spi_lockNotUnlockBCCSpi(false))
    {
        // error
        cli_printfError("spiwrapper ERROR: Could not unlock the spi in CB enable!\n");
    }

    // return
    return ret;
}

/*!
 * @brief This function can be used to manual pause cell balancing before on
 * demand conversion. As a result more precise measurement can be done. Note
 * that it is user obligation to re-enable cell balancing after measurement
 * ends.
 *
 * @note  Thread safe, it will lock on the thread
 * @note  Assumes you have the bcc SPI lock to execute, otherwise it will wait.
 *
 * @param drvConfig Pointer to driver instance configuration.
 * @param cid Cluster Identification Address.
 * @param pause True (pause) / false (unpause).
 *
 * @return bcc_status_t Error code.
 */
bcc_status_t bcc_spiwrapper_BCC_CB_Pause(bcc_drv_config_t* const drvConfig, bcc_cid_t cid,
    bool pause)
{
    bcc_status_t ret;

    // lock the spi 
    if(spi_lockNotUnlockBCCSpi(true))
    {
        // error
        cli_printfError("spiwrapper ERROR: Could not lock the spi in CB pause!\n");
    }

    // lock on this thread (no task switch!)
    sched_lock();

    // pause the CB
    ret = BCC_CB_Pause(drvConfig, cid, pause);

    // unlock this thread (enable task switch!)
    sched_unlock();

    // unlock the spi again
    if(spi_lockNotUnlockBCCSpi(false))
    {
        // error
        cli_printfError("spiwrapper ERROR: Could not unlock the spi in CB pause!\n");
    }

    // return
    return ret;
}

/*******************************************************************************
 * Private functions
 ******************************************************************************/

 /*******************************************************************************
 * EOF
 ******************************************************************************/
