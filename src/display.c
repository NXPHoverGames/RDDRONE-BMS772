/****************************************************************************
 * nxp_bms/BMS_v1/src/display.c
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

#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <fcntl.h>
#include <errno.h>
#include <assert.h>

#include <nuttx/video/fb.h>

#include <nuttx/nx/nx.h>
#include <nuttx/nx/nxtk.h>
#include <nuttx/nx/nxbe.h>
#include <nuttx/nx/nxfonts.h>
#include <nuttx/ascii.h>

#include "cli.h"
#include "display.h"
#include "data.h"

/****************************************************************************
 * Defines
 ****************************************************************************/
//#define DEBUG_DISPLAY

#define DEGREE_SIGN_ASCII_CODE     "\xB0"
#define ALL_SPACES_16              "                "
#define UNKNOWN_SOH                "???"
#define UNKNOWN_TEMP_VALUE         (-99999)
#define SOC_DATA_INDEX_X           1 // 001
#define SOC_DATA_INDEX_Y           0 // 001
#define SOC_DATA_LENGHT            3
#define VOLTAGE_DATA_INDEX_X       7 // 12.1
#define VOLTAGE_DATA_INDEX_Y       0 // 12.1
#define VOLTAGE_DATA_LENGHT        5
#define SOH_DATA_INDEX_X           1 // 096
#define SOH_DATA_INDEX_Y           1 // 096
#define SOH_DATA_LENGHT            3
#define CURRENT_DATA_INDEX_X       6 // -13.1
#define CURRENT_DATA_INDEX_Y       1 // -13.1
#define CURRENT_DATA_LENGHT        6
#define BATT_ID_DATA_INDEX_X       13 // 000
#define BATT_ID_DATA_INDEX_Y       1  // 000
#define BATT_ID_DATA_LENGHT        3
#define OUTPUT_STATUS_DATA_INDEX_X 0 // "ON " / "OFF"
#define OUTPUT_STATUS_DATA_INDEX_Y 2 // "ON " / "OFF"
#define OUTPUT_STATUS_DATA_LENGHT  3
#define TEMPERATURE_DATA_INDEX_X   6 // -10.1
#define TEMPERATURE_DATA_INDEX_Y   2 // -10.1
#define TEMPERATURE_DATA_LENGHT    5
#define STATE_DATA_INDEX_X         0 // CHARGE_COMPLETE
#define STATE_DATA_INDEX_Y         3 // CHARGE_COMPLETE
#define STATE_DATA_LENGHT          16

/****************************************************************************
 * Types
 ****************************************************************************/
struct fb_state_s
{
    int                   fd;
    struct fb_videoinfo_s vinfo;
    struct fb_planeinfo_s pinfo;
#ifdef CONFIG_FB_OVERLAY
    struct fb_overlayinfo_s oinfo;
#endif
    FAR void *fbmem;
};

//! struct that holds the old display value to compare if something changed.
typedef struct
{
    uint8_t socValue;
    int32_t voltageValue;
    uint8_t sohValue;
    int32_t currentValue;
    uint8_t battIdValue;
    uint8_t outputStatusValue;
    int32_t temperatureValue;
    uint8_t mainStateValue;
    uint8_t chargeStateValue;
} displayValues_t;

//! struct that holds the old display value to compare if something changed.
displayValues_t g_oldDisplayValue = 
{
    .socValue          = UINT8_MAX,
    .voltageValue      = INT32_MIN,
    .sohValue          = UINT8_MAX,
    .currentValue      = INT32_MIN,
    .battIdValue       = UINT8_MAX,
    .outputStatusValue = UINT8_MAX,
    .temperatureValue  = INT32_MIN,
    .mainStateValue    = UINT8_MAX,
    .chargeStateValue  = UINT8_MAX,
};

/****************************************************************************
 * private data
 ****************************************************************************/
//! this variable will be used to update the display or not.
static bool gDoNotUpdateDisplay = true;

//! This variable will be used to save the display on/off state
static bool gDisplayOn = false;

//! @brief  mutex for the display
static pthread_mutex_t gDisplayLock;

//! File descriptor for the frame buffer
const static char gFbDev[] = "/dev/fb0";

//! The NuttX font handle
NXHANDLE g_hfont;
// struct nx_font_s *fontset;
FAR const struct nx_font_s *g_fontset;
struct fb_state_s           g_state;

//! This is the area of the whole display to update
static const struct fb_area_s g_areaAll = {
    .x = 0 * 8,  /* x-offset of the area */
    .y = 0 * 8,  /* y-offset of the area */
    .w = 8 * 16, /* Width of the area */
    .h = 8 * 4,  /* Height of the area */
};

/****************************************************************************
 * private Functions declerations
 ****************************************************************************/
/*!
 * @brief   Function to write values to the display (SSD1306 display)
 *
 * @param startPosX The place on the 16 character wide x position (0 - 15)
 * @param startPosY The place on the 4 character high y position (0 - 3)
 * @param line This is the string (max 16 characters) that will written to the display
 * @param size The length of the string
 * @param state_p the address to the fb_state_s with framebuffer, display info and mmap
 * @param updateDisplay If this is true, it will update the line in the display right away

 * @return 0 if succeeded, failure otherwise
 */
int display_writeLine(uint8_t startPosX, uint8_t startPosY, char *line, uint8_t size,
    struct fb_state_s *state_p, bool updateDisplay);

/*!
 * @brief   Function to write the framebuffer values to the whole display (update it) (SSD1306 display)
 *
 * @param area_p address of the fb_area_s containing the area to update.
 * @param times8 If true, all variables of area_p will be multiplied by 8.
 * @param state_p the address to the fb_state_s with framebuffer, display info and mmap
 *
 * @return 0 if succeeded, failure otherwise
 */
int display_updateDisplayArea(struct fb_area_s *area_p, bool times8, struct fb_state_s *state_p);

/****************************************************************************
 * main
 ****************************************************************************/
/*!
 * @brief   This function is used to initialize the display part
 *
 * @param   skipSelfTest if this is true it will skip the self-test
 *
 * @return  0 if ok, -1 if there is an error
 */
int display_initialize(bool skipSelfTest)
{
    int retValue;
    int errcode;

    // Check if the self-test shouldn't be skipped
    if(!skipSelfTest)
    {
        cli_printf("SELF-TEST DISPLAY: START\n");
    }

    // initialze the mutex
    pthread_mutex_init(&gDisplayLock, NULL);

    /* Open the framebuffer driver */

    g_state.fd = open(gFbDev, O_RDWR);
    if(g_state.fd < 0)
    {
        errcode = errno;
        cli_printfError("ERROR: Failed to open %s: %d\n", gFbDev, errcode);
        return EXIT_FAILURE;
    }

    /* Set the power off to make sure it will re-configure the display (if reconfigure is enabled) */
    retValue = ioctl(g_state.fd, FBIOSET_POWER, (int)0);

    // check for errors
    if(retValue < 0)
    {
        errcode = errno;
        cli_printfError("ERROR: ioctl(FBIOSET_POWER, 0) failed: %d\n", errcode);
        close(g_state.fd);
        pthread_mutex_unlock(&gDisplayLock);
        return EXIT_FAILURE;
    }

    /* Set the power on and configure the display if enabled */
    retValue = ioctl(g_state.fd, FBIOSET_POWER, (int)1);

    // check for errors
    if(retValue < 0)
    {
        errcode = errno;
        cli_printfError("ERROR: ioctl(FBIOSET_POWER, 1) failed: %d\n", errcode);
        close(g_state.fd);
        pthread_mutex_unlock(&gDisplayLock);
        return EXIT_FAILURE;
    }

    /* Set the variable */
    gDisplayOn = true;

    /* Get the characteristics of the framebuffer */

    retValue = ioctl(g_state.fd, FBIOGET_VIDEOINFO, (unsigned long)((uintptr_t)&g_state.vinfo));
    if(retValue < 0)
    {
        errcode = errno;
        cli_printfError("ERROR: ioctl(FBIOGET_VIDEOINFO) failed: %d\n", errcode);
        close(g_state.fd);
        return EXIT_FAILURE;
    }

#ifdef DEBUG_DISPLAY
    cli_printf("VideoInfo:\n");
    cli_printf("      fmt: %u\n", g_state.vinfo.fmt);
    cli_printf("     xres: %u\n", g_state.vinfo.xres);
    cli_printf("     yres: %u\n", g_state.vinfo.yres);
    cli_printf("  nplanes: %u\n", g_state.vinfo.nplanes);
#endif

// frame buffer overlay is not supported
#ifdef CONFIG_FB_OVERLAY
#    error add more, frame buffer overlay is currently not supported
#endif
    // do the call to get the plane info
    retValue = ioctl(g_state.fd, FBIOGET_PLANEINFO, (unsigned long)((uintptr_t)&g_state.pinfo));
    if(retValue < 0)
    {
        errcode = errno;
        cli_printfError("ERROR: ioctl(FBIOGET_PLANEINFO) failed: %d\n", errcode);
        close(g_state.fd);
        return EXIT_FAILURE;
    }

#ifdef DEBUG_DISPLAY
    cli_printf("PlaneInfo (plane 0):\n");
    cli_printf("    fbmem: %p\n", g_state.pinfo.fbmem);
    cli_printf("    fblen: %lu\n", (unsigned long)g_state.pinfo.fblen);
    cli_printf("   stride: %u\n", g_state.pinfo.stride);
    cli_printf("  display: %u\n", g_state.pinfo.display);
    cli_printf("      bpp: %u\n", g_state.pinfo.bpp);
#endif

    /* mmap() the framebuffer.
     *
     * NOTE: In the FLAT build the frame buffer address returned by the
     * FBIOGET_PLANEINFO IOCTL command will be the same as the framebuffer
     * address.  mmap(), however, is the preferred way to get the framebuffer
     * address because in the KERNEL build, it will perform the necessary
     * address mapping to make the memory accessible to the application.
     */

    g_state.fbmem =
        mmap(NULL, g_state.pinfo.fblen, PROT_READ | PROT_WRITE, MAP_SHARED | MAP_FILE, g_state.fd, 0);
    if(g_state.fbmem == MAP_FAILED)
    {
        errcode = errno;
        cli_printfError("ERROR: ioctl(FBIOGET_PLANEINFO) failed: %d\n", errcode);
        close(g_state.fd);
        return EXIT_FAILURE;
    }

    // get the font handle
    g_hfont = nxf_getfonthandle(NXFONT_DEFAULT);

    /* Get information about the font set being used and save this in the
     * g_state structure
     */

    g_fontset = nxf_getfontset(g_hfont);

#ifdef DEBUG_DISPLAY
    cli_printf("mxheight: %d\n", g_fontset->mxheight);
    cli_printf("mxwidth: %d\n", g_fontset->mxwidth);
    cli_printf("spwidth: %d\n", g_fontset->spwidth);
#endif

    // write the default text to the display
    retValue = display_writeLine(0, 0, "C---%  --.--V ID", 16, &g_state, false);
    // Check for an error
    if(retValue)
    {
        cli_printfError("ERROR: Couldn't write to display!\n");

        // close the file descriptor
        close(g_state.fd);

        // check if it was in the wrong state
        if(retValue == -2)
        {
            return EXIT_SUCCESS;
        }
        else
        {
            return EXIT_FAILURE;
        }
    }

    retValue = display_writeLine(0, 1, "H---% ---.--A  -", 16, &g_state, false);
    // Check for an error
    if(retValue)
    {
        cli_printfError("ERROR: Couldn't write to display!\n");

        // close the file descriptor
        close(g_state.fd);

        // check if it was in the wrong state
        if(retValue == -2)
        {
            return EXIT_SUCCESS;
        }
        else
        {
            return EXIT_FAILURE;
        }
    }

    retValue = display_writeLine(0, 2, "OFF   ---.-" DEGREE_SIGN_ASCII_CODE "C   ", 16, &g_state, false);

    // Check for an error
    if(retValue)
    {
        cli_printfError("ERROR: Couldn't write to display!\n");

        // close the file descriptor
        close(g_state.fd);

        // check if it was in the wrong state
        if(retValue == -2)
        {
            return EXIT_SUCCESS;
        }
        else
        {
            return EXIT_FAILURE;
        }
    }

    retValue = display_writeLine(0, 3, "SELF_TEST       ", 16, &g_state, false);
    // Check for an error
    if(retValue)
    {
        cli_printfError("ERROR: Couldn't write to display!\n");

        // close the file descriptor
        close(g_state.fd);

        // check if it was in the wrong state
        if(retValue == -2)
        {
            return EXIT_SUCCESS;
        }
        else
        {
            return EXIT_FAILURE;
        }
    }

    // update the whole framebuffer with the written values
    retValue = display_updateDisplayArea((struct fb_area_s *)&g_areaAll, false, &g_state);

    // Check for an error
    if(retValue)
    {
        cli_printfError("ERROR: Couldn't update the framebuffer display!\n");

        // close the file descriptor
        close(g_state.fd);

        return EXIT_FAILURE;
    }

    // close the file descriptor
    close(g_state.fd);

    // Check if the self-test shouldn't be skipped
    if(!skipSelfTest)
    {
        // output that the self test succeeded
        cli_printf("SELF-TEST DISPLAY: \e[32mPASS\e[39m\n");
    }

    // set the return value to success
    retValue = EXIT_SUCCESS;

    // make sure to update the display
    gDoNotUpdateDisplay = false;

    // return
    return retValue;
}

/*!
 * @brief   This function is used to uninitialize the display part
 *
 * @return  0 if ok, -1 if there is an error
 */
int display_uninitialize(void)
{
    int errcode;
    int retValue;

    pthread_mutex_lock(&gDisplayLock);

    // set the varaible to true to stop the update of the display
    // the pm handler will make sure it won't keep changing the power of the display
    gDoNotUpdateDisplay = true;

    /* Uninitialze the display */
    /* Open the framebuffer driver */

    g_state.fd = open(gFbDev, O_RDWR);
    if(g_state.fd < 0)
    {
        errcode = errno;
        cli_printfError("ERROR: Failed to open %s: %d\n", gFbDev, errcode);
        pthread_mutex_unlock(&gDisplayLock);
        return EXIT_FAILURE;
    }

    /* Set the power to off */
    retValue = ioctl(g_state.fd, FBIOSET_POWER, (int)false);

    // check for errors
    if(retValue < 0)
    {
        errcode = errno;
        cli_printfError("ERROR: ioctl(FBIOSET_POWER) failed: %d\n", errcode);
        close(g_state.fd);
        pthread_mutex_unlock(&gDisplayLock);
        return EXIT_FAILURE;
    }

    // close the file descriptor
    close(g_state.fd);

    /* Save the variable */
    gDisplayOn = false;

    pthread_mutex_unlock(&gDisplayLock);

    // return
    return !gDoNotUpdateDisplay;
}

/*!
 * @brief   This function is used to turn on or off the power to the display
 *
 * @param   on If true, it will turn on the display, false otherwise.
 *
 * @return  0 if ok, -1 if there is an error
 */
int display_setPower(bool on)
{
    int errcode;
    int retValue = 0;

    pthread_mutex_lock(&gDisplayLock);

    // If the display should be updated
    if(!gDoNotUpdateDisplay)
    {
        /* Open the framebuffer driver */

        g_state.fd = open(gFbDev, O_RDWR);
        if(g_state.fd < 0)
        {
            errcode = errno;
            cli_printfError("ERROR: Failed to open %s: %d\n", gFbDev, errcode);
            pthread_mutex_unlock(&gDisplayLock);
            return EXIT_FAILURE;
        }

        /* Set the power to off */
        retValue = ioctl(g_state.fd, FBIOSET_POWER, (int)on);

        // check for errors
        if(retValue < 0)
        {
            errcode = errno;
            cli_printfError("ERROR: ioctl(FBIOSET_POWER) failed: %d\n", errcode);
            close(g_state.fd);
            pthread_mutex_unlock(&gDisplayLock);
            return EXIT_FAILURE;
        }

        // close the file descriptor
        close(g_state.fd);

        /* Save the state */
        gDisplayOn = on;

        retValue = 0;
    }

    pthread_mutex_unlock(&gDisplayLock);

    return retValue;
}

/*!
 * @brief   This function is used to get the state of the display, on or off.
 *
 * @param   none.
 *
 * @return  true if on, false otherwise.
 */
bool display_getPower(void)
{
    bool retValue;

    pthread_mutex_lock(&gDisplayLock);

    retValue = gDisplayOn;

    pthread_mutex_unlock(&gDisplayLock);

    return retValue;
}

/*!
 * @brief   This function is used to update the values of the display with the actual information
 *
 * @param   pCommonBatteryVariables pointer to the commonBatteryVariables_t to update the information
 * @param   pCalcBatteryVariables pointer to the calcBatteryVariables_t to update the information
 *
 * @return  0 If successful, otherwise an error will indicate the error
 */
int display_updateValues(
    commonBatteryVariables_t *pCommonBatteryVariables, calcBatteryVariables_t *pCalcBatteryVariables)
{
    // define local variables here
    int              retValue = -1;
    int              errcode, bmsFault;
    char             valueString[17];
    bool             outputStatusChanged    = false;
    struct fb_area_s areaValue              = { .x = 0, .y = 0, .w = 0, .h = 1 };
    uint8_t          segmentsToUpdateInLine = 0;

    // check if initialized
    if(!gDoNotUpdateDisplay)
    {
        // Open the framebuffer driver
        g_state.fd = open(gFbDev, O_RDWR);
        if(g_state.fd < 0)
        {
            errcode = errno;
            cli_printfError("ERROR: Failed to open %s: %d\n", gFbDev, errcode);
            return EXIT_FAILURE;
        }

        // check if the value changed or if it is the first time
        if((pCalcBatteryVariables->s_charge != g_oldDisplayValue.socValue))
        {
            // get the state of charge
            // write the value in the value string
            sprintf(valueString, "%3d", pCalcBatteryVariables->s_charge);

            // write the value to the display
            retValue = display_writeLine(
                SOC_DATA_INDEX_X, SOC_DATA_INDEX_Y, valueString, SOC_DATA_LENGHT, &g_state, false);

            // Check for an error
            if(retValue)
            {
                cli_printfError("ERROR: Couldn't write to display!\n");

                // close the file descriptor
                close(g_state.fd);

                if(retValue == -2)
                {
                    return EXIT_SUCCESS;
                }
                else
                {
                    return EXIT_FAILURE;
                }
            }

            // save the segements for the display update
            areaValue.x = SOC_DATA_INDEX_X;
            areaValue.y = SOC_DATA_INDEX_Y;
            areaValue.w = SOC_DATA_LENGHT;
            segmentsToUpdateInLine |= 1 << 0;

            // save the new value in old value
            g_oldDisplayValue.socValue = pCalcBatteryVariables->s_charge;
        }

        // check if the output is enabled
        if(pCalcBatteryVariables->s_out & 1)
        {
            // check if the value changed or if it is the first time
            if(((int)(pCommonBatteryVariables->V_out * 100) != g_oldDisplayValue.voltageValue))
            {
                // get the output voltage
                // write the value in the value string
                sprintf(valueString, "%5.2f", pCommonBatteryVariables->V_out);

                // write the value to the display
                retValue = display_writeLine(VOLTAGE_DATA_INDEX_X, VOLTAGE_DATA_INDEX_Y, valueString,
                    VOLTAGE_DATA_LENGHT, &g_state, false);

                if(retValue)
                {
                    cli_printfError("ERROR: Couldn't write to display!\n");

                    // close the file descriptor
                    close(g_state.fd);

                    if(retValue == -2)
                    {
                        return EXIT_SUCCESS;
                    }
                    else
                    {
                        return EXIT_FAILURE;
                    }
                }

                // check if this is the first segment
                if(segmentsToUpdateInLine == 0)
                {
                    // save the segements startpos for the display update
                    areaValue.x = VOLTAGE_DATA_INDEX_X;
                    areaValue.y = VOLTAGE_DATA_INDEX_Y;
                    areaValue.w = VOLTAGE_DATA_LENGHT;
                    segmentsToUpdateInLine |= 1 << 1;
                }
                else
                {
                    // add the voltage segment to the area to update
                    areaValue.w +=
                        (VOLTAGE_DATA_INDEX_X - (SOC_DATA_INDEX_X + SOC_DATA_LENGHT)) + VOLTAGE_DATA_LENGHT;
                    segmentsToUpdateInLine |= 1 << 1;
                }

                g_oldDisplayValue.voltageValue = (int)(pCommonBatteryVariables->V_out * 100);
            }

            // check if the value changed or if it is the first time
            if(((pCalcBatteryVariables->s_out & 1) != g_oldDisplayValue.outputStatusValue))
            {
                // write the output ON to the display
                retValue = display_writeLine(OUTPUT_STATUS_DATA_INDEX_X, OUTPUT_STATUS_DATA_INDEX_Y, "ON ",
                    OUTPUT_STATUS_DATA_LENGHT, &g_state, false);

                if(retValue)
                {
                    cli_printfError("ERROR: Couldn't write to display!\n");

                    // close the file descriptor
                    close(g_state.fd);

                    if(retValue == -2)
                    {
                        return EXIT_SUCCESS;
                    }
                    else
                    {
                        return EXIT_FAILURE;
                    }
                }

                // make sure to add the output status to the area to update
                outputStatusChanged = true;

                g_oldDisplayValue.outputStatusValue = pCalcBatteryVariables->s_out & 1;
            }
        }
        // if the output is not enabled
        else
        {
            // check if the value changed or if it is the first time
            if(((int)(pCommonBatteryVariables->V_batt * 100) != g_oldDisplayValue.voltageValue))
            {
                // write the value in the value string
                sprintf(valueString, "%5.2f", pCommonBatteryVariables->V_batt);

                // write the value to the display
                retValue = display_writeLine(VOLTAGE_DATA_INDEX_X, VOLTAGE_DATA_INDEX_Y, valueString,
                    VOLTAGE_DATA_LENGHT, &g_state, false);

                if(retValue)
                {
                    cli_printfError("ERROR: Couldn't write to display!\n");

                    // close the file descriptor
                    close(g_state.fd);

                    if(retValue == -2)
                    {
                        return EXIT_SUCCESS;
                    }
                    else
                    {
                        return EXIT_FAILURE;
                    }
                }

                // check if this is the first segment
                if(segmentsToUpdateInLine == 0)
                {
                    // save the segements startpos for the display update
                    areaValue.x = VOLTAGE_DATA_INDEX_X;
                    areaValue.y = VOLTAGE_DATA_INDEX_Y;
                    areaValue.w = VOLTAGE_DATA_LENGHT;
                    segmentsToUpdateInLine |= 1 << 1;
                }
                else
                {
                    // add the voltage segment to the area to update
                    areaValue.w +=
                        (VOLTAGE_DATA_INDEX_X - (SOC_DATA_INDEX_X + SOC_DATA_LENGHT)) + VOLTAGE_DATA_LENGHT;
                    segmentsToUpdateInLine |= 1 << 1;
                }

                g_oldDisplayValue.voltageValue = (int)(pCommonBatteryVariables->V_batt * 100);
            }

            // check if the value changed or if it is the first time
            if(((pCalcBatteryVariables->s_out & 1) != g_oldDisplayValue.outputStatusValue))
            {
                // write the output ON to the display
                retValue = display_writeLine(OUTPUT_STATUS_DATA_INDEX_X, OUTPUT_STATUS_DATA_INDEX_Y, "OFF",
                    OUTPUT_STATUS_DATA_LENGHT, &g_state, false);

                if(retValue)
                {
                    cli_printfError("ERROR: Couldn't write to display!\n");

                    // close the file descriptor
                    close(g_state.fd);

                    if(retValue == -2)
                    {
                        return EXIT_SUCCESS;
                    }
                    else
                    {
                        return EXIT_FAILURE;
                    }
                }

                // make sure to add the output status to the area to update
                outputStatusChanged = true;

                g_oldDisplayValue.outputStatusValue = pCalcBatteryVariables->s_out & 1;
            }
        }

        // check if the first row needs to be updated
        if(segmentsToUpdateInLine)
        {
            // update the first row
            if(display_updateDisplayArea(&(areaValue), true, &g_state))
            {
                cli_printfError("Display ERROR: Could not update row 1!\n");

                // close the file descriptor
                close(g_state.fd);

                return EXIT_FAILURE;
            }

            areaValue.h = 1;
        }

        // reset the segmentsToUpdateInLine for the next row
        segmentsToUpdateInLine = 0;

        // check if the value changed or if it is the first time
        if(pCalcBatteryVariables->s_health != g_oldDisplayValue.sohValue)
        {
            // check if the SoH is known
            if(pCalcBatteryVariables->s_health != 127)
            {
                // write the value in the value string
                sprintf(valueString, "%3d", pCalcBatteryVariables->s_health);
            }
            // if it is not known
            else
            {
                // write ??? to it to indicate it is not known yet
                sprintf(valueString, UNKNOWN_SOH);
            }

            // write the value to the display
            retValue = display_writeLine(
                SOH_DATA_INDEX_X, SOH_DATA_INDEX_Y, valueString, SOH_DATA_LENGHT, &g_state, false);

            // Check for an error
            if(retValue)
            {
                cli_printfError("ERROR: Couldn't write to display!\n");

                // close the file descriptor
                close(g_state.fd);

                if(retValue == -2)
                {
                    return EXIT_SUCCESS;
                }
                else
                {
                    return EXIT_FAILURE;
                }
            }

            // save the segements for the display update
            areaValue.x = SOH_DATA_INDEX_X;
            areaValue.y = SOH_DATA_INDEX_Y;
            areaValue.w = SOH_DATA_LENGHT;
            segmentsToUpdateInLine |= 1 << 0;

            g_oldDisplayValue.sohValue = pCalcBatteryVariables->s_health;
        }

        // check if the value changed or if it is the first time
        if(((int)(pCommonBatteryVariables->I_batt_avg * 100)) != g_oldDisplayValue.currentValue)
        {
            // write the value in the value string
            sprintf(valueString, "%6.2f", pCommonBatteryVariables->I_batt_avg);

            // write the value to the display
            retValue = display_writeLine(CURRENT_DATA_INDEX_X, CURRENT_DATA_INDEX_Y, valueString,
                CURRENT_DATA_LENGHT, &g_state, false);

            // Check for an error
            if(retValue)
            {
                cli_printfError("ERROR: Couldn't write to display!\n");

                // close the file descriptor
                close(g_state.fd);

                if(retValue == -2)
                {
                    return EXIT_SUCCESS;
                }
                else
                {
                    return EXIT_FAILURE;
                }
            }

            // check if this is the first segment
            if(segmentsToUpdateInLine == 0)
            {
                // save the segements startpos for the display update
                areaValue.x = CURRENT_DATA_INDEX_X;
                areaValue.y = CURRENT_DATA_INDEX_Y;
                areaValue.w = CURRENT_DATA_LENGHT;
                segmentsToUpdateInLine |= 1 << 1;
            }
            else
            {
                // add the voltage segment to the area to update
                areaValue.w +=
                    (CURRENT_DATA_INDEX_X - (SOH_DATA_INDEX_X + SOH_DATA_LENGHT)) + CURRENT_DATA_LENGHT;
                segmentsToUpdateInLine |= 1 << 1;
            }

            g_oldDisplayValue.currentValue = (int)(pCommonBatteryVariables->I_batt_avg * 100);
        }

        // check if the value changed or if it is the first time
        if(pCalcBatteryVariables->batt_id != g_oldDisplayValue.battIdValue)
        {
            // write the value in the value string
            sprintf(valueString, "%3d", pCalcBatteryVariables->batt_id);

            // write the value to the display
            retValue = display_writeLine(BATT_ID_DATA_INDEX_X, BATT_ID_DATA_INDEX_Y, valueString,
                BATT_ID_DATA_LENGHT, &g_state, false);

            // Check for an error
            if(retValue)
            {
                cli_printfError("ERROR: Couldn't write to display!\n");

                // close the file descriptor
                close(g_state.fd);

                if(retValue == -2)
                {
                    return EXIT_SUCCESS;
                }
                else
                {
                    return EXIT_FAILURE;
                }
            }

            // check if this is the first segment
            if(segmentsToUpdateInLine == 0)
            {
                // save the segements startpos for the display update
                areaValue.x = BATT_ID_DATA_INDEX_X;
                areaValue.y = BATT_ID_DATA_INDEX_Y;
                areaValue.w = BATT_ID_DATA_LENGHT;
            }
            // if the previous is added
            else if(segmentsToUpdateInLine == 1 << 0)
            {
                // add this segment to the area to update
                areaValue.w +=
                    (BATT_ID_DATA_INDEX_X - (SOH_DATA_INDEX_X + SOH_DATA_LENGHT)) + BATT_ID_DATA_LENGHT;
            }
            // if the previous segment is added
            else
            {
                // add this segment to the area to update
                areaValue.w += (BATT_ID_DATA_INDEX_X - (CURRENT_DATA_INDEX_X + CURRENT_DATA_LENGHT)) +
                    BATT_ID_DATA_LENGHT;
            }

            segmentsToUpdateInLine |= 1 << 2;

            g_oldDisplayValue.battIdValue = pCalcBatteryVariables->batt_id;
        }

        // check if the second row needs to be updated
        if(segmentsToUpdateInLine)
        {
            // update the first row
            if(display_updateDisplayArea(&(areaValue), true, &g_state))
            {
                cli_printfError("Display ERROR: Could not update row 2!\n");

                // close the file descriptor
                close(g_state.fd);

                return EXIT_FAILURE;
            }

            areaValue.h = 1;
        }

        // reset the segmentsToUpdateInLine for the next row
        segmentsToUpdateInLine = 0;

        // check if the output status changed
        if(outputStatusChanged)
        {
            // save the segements for the display update
            areaValue.x = OUTPUT_STATUS_DATA_INDEX_X;
            areaValue.y = OUTPUT_STATUS_DATA_INDEX_Y;
            areaValue.w = OUTPUT_STATUS_DATA_LENGHT;
            segmentsToUpdateInLine |= 1 << 0;
        }

        // check if the battery temperature sensor is used
        if(pCommonBatteryVariables->sensor_enable & 1)
        {
            // check if the value changed or if it is the first time
            if(((int)(pCommonBatteryVariables->C_batt * 10) != g_oldDisplayValue.temperatureValue))
            {
                // get the battery temperature
                // write the value in the value string
                sprintf(valueString, "%5.1f", pCommonBatteryVariables->C_batt);

                // write the value to the display
                retValue = display_writeLine(TEMPERATURE_DATA_INDEX_X, TEMPERATURE_DATA_INDEX_Y, valueString,
                    TEMPERATURE_DATA_LENGHT, &g_state, false);

                // Check for an error
                if(retValue)
                {
                    cli_printfError("ERROR: Couldn't write to display!\n");

                    // close the file descriptor
                    close(g_state.fd);

                    if(retValue == -2)
                    {
                        return EXIT_SUCCESS;
                    }
                    else
                    {
                        return EXIT_FAILURE;
                    }
                }

                // check if this is the first segment
                if(segmentsToUpdateInLine == 0)
                {
                    // save the segements startpos for the display update
                    areaValue.x = TEMPERATURE_DATA_INDEX_X;
                    areaValue.y = TEMPERATURE_DATA_INDEX_Y;
                    areaValue.w = TEMPERATURE_DATA_LENGHT;
                    segmentsToUpdateInLine |= 1 << 1;
                }
                else
                {
                    // add the voltage segment to the area to update
                    areaValue.w += (TEMPERATURE_DATA_INDEX_X -
                                       (OUTPUT_STATUS_DATA_INDEX_X + OUTPUT_STATUS_DATA_LENGHT)) +
                        TEMPERATURE_DATA_LENGHT;
                    segmentsToUpdateInLine |= 1 << 1;
                }

                g_oldDisplayValue.temperatureValue = (int)(pCommonBatteryVariables->C_batt * 10);
            }
        }
        // if the battery temp sensor is not enabled
        else
        {
            // check if the value changed or if it is the first time
            if(UNKNOWN_TEMP_VALUE != g_oldDisplayValue.temperatureValue)
            {
                // set it to ---
                // write the value to the display
                retValue = display_writeLine(TEMPERATURE_DATA_INDEX_X, TEMPERATURE_DATA_INDEX_Y, " --.-",
                    TEMPERATURE_DATA_LENGHT, &g_state, false);

                if(retValue)
                {
                    cli_printfError("ERROR: Couldn't write to display!\n");

                    // close the file descriptor
                    close(g_state.fd);

                    if(retValue == -2)
                    {
                        return EXIT_SUCCESS;
                    }
                    else
                    {
                        return EXIT_FAILURE;
                    }
                }

                // check if this is the first segment
                if(segmentsToUpdateInLine == 0)
                {
                    // save the segements startpos for the display update
                    areaValue.x = TEMPERATURE_DATA_INDEX_X;
                    areaValue.y = TEMPERATURE_DATA_INDEX_Y;
                    areaValue.w = TEMPERATURE_DATA_LENGHT;
                    segmentsToUpdateInLine |= 1 << 1;
                }
                else
                {
                    // add the voltage segment to the area to update
                    areaValue.w += (TEMPERATURE_DATA_INDEX_X -
                                       (OUTPUT_STATUS_DATA_INDEX_X + OUTPUT_STATUS_DATA_LENGHT)) +
                                       TEMPERATURE_DATA_LENGHT;
                    segmentsToUpdateInLine |= 1 << 1;
                }

                g_oldDisplayValue.temperatureValue = UNKNOWN_TEMP_VALUE;
            }
        }

        // check if the third row needs to be updated
        if(segmentsToUpdateInLine)
        {
            // update the first row
            if(display_updateDisplayArea(&(areaValue), true, &g_state))
            {
                cli_printfError("Display ERROR: Could not update row 3!\n");

                // close the file descriptor
                close(g_state.fd);

                return EXIT_FAILURE;
            }

            areaValue.h = 1;
        }

        // reset the value string to spaces
        sprintf(valueString, ALL_SPACES_16);

        // get the state in a uint8 value
        retValue = (int)data_getMainState();

        // check if the state changed
        if(((retValue) != g_oldDisplayValue.mainStateValue) ||
            (retValue == CHARGE && ((uint8_t)data_getChargeState() != g_oldDisplayValue.chargeStateValue)))
        {
            // check if the charge state needs to be outputted as well
            if((states_t)retValue == CHARGE)
            {
                // get the charge state string and put it in the value string
                cli_getStateString(false, (uint8_t)data_getChargeState(), valueString);

                g_oldDisplayValue.chargeStateValue = (uint8_t)data_getChargeState();
            }
            else if(((states_t)retValue == FAULT_OFF) || ((states_t)retValue == FAULT_ON))
            {
                // get the state string and put it in the value string
                cli_getStateString(true, (uint8_t)data_getMainState(), valueString);

                // get the bms fault
                bmsFault = data_getBmsFault();

                // check which fault is active and add the reason to the state sting
                if(bmsFault & BMS_OV_FAULT)
                {
                    // add the indication to the string
                    sprintf(&(valueString[10]), "OV");
                }
                if(bmsFault & BMS_UV_FAULT)
                {
                    // add the indication to the string
                    sprintf(&(valueString[10]), "UV");
                }
                if(bmsFault & BMS_OT_FAULT)
                {
                    // add the indication to the string
                    sprintf(&(valueString[12]), "OT");
                }
                if(bmsFault & BMS_UT_FAULT)
                {
                    // add the indication to the string
                    sprintf(&(valueString[12]), "UT");
                }
                if(bmsFault & BMS_OC_FAULT)
                {
                    // add the indication to the string
                    sprintf(&(valueString[14]), "OC");
                }
            }
            else
            {
                // get the state string and put it in the value string
                cli_getStateString(true, (uint8_t)data_getMainState(), valueString);
            }

            g_oldDisplayValue.mainStateValue = (uint8_t)retValue;

            // write the value to the display and update
            retValue = display_writeLine(
                STATE_DATA_INDEX_X, STATE_DATA_INDEX_Y, valueString, STATE_DATA_LENGHT, &g_state, true);

            // Check for an error
            if(retValue)
            {
                cli_printfError("ERROR: Couldn't write to display!\n");

                if(retValue == -2)
                {
                    retValue = EXIT_SUCCESS;
                }
            }
        }

        // close the file descriptor
        close(g_state.fd);
    }

    return 0;
}

/****************************************************************************
 * private Functions
 ****************************************************************************/
/*!
 * @brief   Function to write values to the display (SSD1306 display)
 *
 * @param startPosX The place on the 16 character wide x position (0 - 15)
 * @param startPosY The place on the 4 character high y position (0 - 3)
 * @param line This is the string (max 16 characters) that will written to the display
 * @param size The length of the string
 * @param state_p the address to the fb_state_s with framebuffer, display info and mmap
 * @param updateDisplay If this is true, it will update the line in the display right away

 * @return 0 if succeeded, failure otherwise
 */
int display_writeLine(uint8_t startPosX, uint8_t startPosY, char *line, uint8_t size,
    struct fb_state_s *state_p, bool updateDisplay)
{
    FAR const struct nx_fontbitmap_s *fbm;
    uint8_t                           i, y;
    struct nxgl_size_s                fsize;
    struct fb_area_s                  area;
    int                               retValue = -1;
    int                               errcode;

    // check for error value
    if(startPosX > 15)
    {
        cli_printfError("ERROR: startPosX > 15\n");
        return retValue;
    }
    if(startPosY > 3)
    {
        cli_printfError("ERROR: startPosY > 3\n");
        return retValue;
    }

    // check if in the charge relaxation state
    if(data_getChargeState() == RELAXATION && data_getMainState() == CHARGE)
    {
        // return -2 to indicate the state
        return -2;
    }

    // loop through the size of the array
    for(i = 0; i < (size); i++)
    {
        // get the bitmap of the character ()
        fbm = nxf_getbitmap(g_hfont, line[i]);
        if(fbm != NULL)
        {
            // calculate the hight of the character
            fsize.h = fbm->metric.height + fbm->metric.yoffset;

            // loop through the letter from top to bottom to fill the rows
            for(y = 0; y < fsize.h; y++)
            {
                // add each line of the charactor to the framebuffer to send it to the display
                // each byte is a row (of 8) of a character, there are 16 character per line
                ((uint8_t *)state_p->fbmem)[((startPosY * 8 + y) * state_p->pinfo.stride) + i + startPosX] =
                    fbm->bitmap[y];
            }
        }
        // it could be a ' '
        // threat it like a ' '
        else
        {
            // use the max height
            fsize.h = g_fontset->mxheight;

            // loop through the space from top to bottom to fill the rows
            for(y = 0; y < fsize.h; y++)
            {
                // set the frame buffer to 0 (nothing)
                ((uint8_t *)state_p->fbmem)[((startPosY * 8 + y) * state_p->pinfo.stride) + i + startPosX] =
                    0;
            }
        }
    }
// if the area needs to be updated
#ifdef CONFIG_FB_UPDATE
    // If you need to update the framebuffer right away
    if(updateDisplay)
    {
        // create the area to update
        area.x = startPosX * 8; /* x-offset of the area */
        area.y = startPosY * 8; /* y-offset of the area */
        area.w = 8 * size;      /* Width of the area */
        area.h = 8;             /* Height of the area */

        // update the area with the framebuffer
        retValue = ioctl(state_p->fd, FBIO_UPDATE, (unsigned long)((uintptr_t)&area));

        // check for errors
        if(retValue < 0)
        {
            errcode = errno;
            cli_printfError("ERROR: ioctl(FBIO_UPDATE) failed: %d\n", errcode);
        }
    }
    else
#endif
    {
        // Make sure to return OK
        retValue = 0;
    }

    return retValue;
}

/*!
 * @brief   Function to write the framebuffer values to the whole display (update it) (SSD1306 display)
 *
 * @param area_p address of the fb_area_s containing the area to update.
 * @param times8 If true, all variables of area_p will be multiplied by 8.
 * @param state_p the address to the fb_state_s with framebuffer, display info and mmap
 *
 * @return 0 if succeeded, failure otherwise
 */
int display_updateDisplayArea(struct fb_area_s *area_p, bool times8, struct fb_state_s *state_p)
{
    int retValue = 0, errcode;

    DEBUGASSERT(area_p != NULL);
    DEBUGASSERT(state_p != NULL);

// if the area needs to be updated
#ifdef CONFIG_FB_UPDATE

    // check if it needs to be multiplied by 8
    if(times8)
    {
        DEBUGASSERT(area_p->x <= 15);
        DEBUGASSERT(area_p->y <= 3);
        DEBUGASSERT((area_p->x + area_p->w) <= 16);
        DEBUGASSERT((area_p->y + area_p->h) <= 4);
        area_p->x *= 8;
        area_p->y *= 8;
        area_p->w *= 8;
        area_p->h *= 8;
    }

    // update the whole area with the framebuffer
    retValue = ioctl(state_p->fd, FBIO_UPDATE, (unsigned long)((uintptr_t)area_p));

    // check for errors
    if(retValue < 0 || retValue > 0)
    {
        errcode = errno;
        cli_printfError("ERROR: ioctl(FBIO_UPDATE) failed: %d %d\n", errcode, retValue);
    }
#endif

    return retValue;
}

