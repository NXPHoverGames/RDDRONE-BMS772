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

#define DEGREE_SIGN_ASCII_CODE      "\xB0"
#define ALL_SPACES_16               "                "
#define UNKNOWN_SOH                 "???"

#define SOC_DATA_INDEX_X            1   // 001
#define SOC_DATA_INDEX_Y            0   // 001
#define SOC_DATA_LENGHT             3
#define VOLTAGE_DATA_INDEX_X        7   // 12.1
#define VOLTAGE_DATA_INDEX_Y        0   // 12.1
#define VOLTAGE_DATA_LENGHT         5
#define SOH_DATA_INDEX_X            1  // 096
#define SOH_DATA_INDEX_Y            1  // 096
#define SOH_DATA_LENGHT             3
#define CURRENT_DATA_INDEX_X        6  // -13.1
#define CURRENT_DATA_INDEX_Y        1  // -13.1
#define CURRENT_DATA_LENGHT         6
#define BATT_ID_DATA_INDEX_X        13  // 000
#define BATT_ID_DATA_INDEX_Y        1  // 000
#define BATT_ID_DATA_LENGHT         3
#define OUTPUT_STATUS_DATA_INDEX_X  0  // "ON " / "OFF"
#define OUTPUT_STATUS_DATA_INDEX_Y  2  // "ON " / "OFF"
#define OUTPUT_STATUS_DATA_LENGHT   3
#define TEMPERATURE_DATA_INDEX_X    6  // -10.1
#define TEMPERATURE_DATA_INDEX_Y    2  // -10.1
#define TEMPERATURE_DATA_LENGHT     5
#define STATE_DATA_INDEX_X          0  // CHARGE_COMPLETE
#define STATE_DATA_INDEX_Y          3  // CHARGE_COMPLETE
#define STATE_DATA_LENGHT           16

/****************************************************************************
 * Types
 ****************************************************************************/
struct fb_state_s
{
  int fd;
  struct fb_videoinfo_s vinfo;
  struct fb_planeinfo_s pinfo;
#ifdef CONFIG_FB_OVERLAY
  struct fb_overlayinfo_s oinfo;
#endif
  FAR void *fbmem;
};

/****************************************************************************
 * private data
 ****************************************************************************/
//! Variable to keep track if the display is initialzed
bool gDisplayInitialized = false;

//! Variable to keep track if the display should be initialized
bool gDoNotInitializeDisplay = false;

//! File descriptor for the frame buffer
const static char gFbDev[] = "/dev/fb0";

//! The NuttX font handle 
NXHANDLE g_hfont;
//struct nx_font_s *fontset;
FAR const struct nx_font_s *g_fontset;
struct fb_state_s g_state;

/****************************************************************************
 * private Functions declerations 
 ****************************************************************************/
int display_writeLine(uint8_t startPosX, uint8_t startPosY, char *line, uint8_t size, 
    struct fb_state_s *state_p);

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
    int lvRetValue;

    // check if already initialzed or should be initialized
    if(!gDisplayInitialized && !gDoNotInitializeDisplay)
    {
        // do not initialize display anymore if failed once, it shouldn't be done anymore
        gDoNotInitializeDisplay = true;

        // Check if the self-test shouldn't be skipped
        if(!skipSelfTest)
        {
            cli_printf("SELF-TEST DISPLAY: START\n");
        }

        /* Open the framebuffer driver */

        g_state.fd = open(gFbDev, O_RDWR);
        if(g_state.fd < 0)
        {
            int errcode = errno;
            cli_printfError("ERROR: Failed to open %s: %d\n", gFbDev, errcode);
            return EXIT_FAILURE;
        }

        /* Get the characteristics of the framebuffer */

        lvRetValue = ioctl(g_state.fd, FBIOGET_VIDEOINFO, (unsigned long)((uintptr_t)&g_state.vinfo));
        if(lvRetValue < 0)
        {
            int errcode = errno;
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
#error add more, frame buffer overlay is currently not supported
#endif
        // do the call to get the plane info
        lvRetValue = ioctl(g_state.fd, FBIOGET_PLANEINFO, (unsigned long)((uintptr_t)&g_state.pinfo));
        if(lvRetValue < 0)
        {
            int errcode = errno;
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

        g_state.fbmem = mmap(NULL, g_state.pinfo.fblen, PROT_READ | PROT_WRITE,
                         MAP_SHARED | MAP_FILE, g_state.fd, 0);
        if(g_state.fbmem == MAP_FAILED)
        {
            int errcode = errno;
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
        lvRetValue = display_writeLine(0, 0, "C---%  --.--V ID", 16, &g_state);
        // Check for an error
        if(lvRetValue)
        {
            cli_printfError("ERROR: Couldn't write to display!\n");

            // close the file descriptor
            close(g_state.fd);

            // check if it was in the wrong state
            if(lvRetValue == -2)
            {
                return EXIT_SUCCESS;
            }
            else
            {
                return EXIT_FAILURE;
            }
        }

        lvRetValue = display_writeLine(0, 1, "H---% ---.--A  -", 16, &g_state);
        // Check for an error
        if(lvRetValue)
        {
            cli_printfError("ERROR: Couldn't write to display!\n");

            // close the file descriptor
            close(g_state.fd);

            // check if it was in the wrong state
            if(lvRetValue == -2)
            {
                return EXIT_SUCCESS;
            }
            else
            {
                return EXIT_FAILURE;
            }
        }

        //lvRetValue = display_writeLine(0, 2, "ON    -23.0" "\xB0" "C   ", 16, &g_state))
        lvRetValue = display_writeLine(0, 2, "OFF   ---.-" DEGREE_SIGN_ASCII_CODE "C   ",
            16, &g_state);

        // Check for an error
        if(lvRetValue)
        {
            cli_printfError("ERROR: Couldn't write to display!\n");

            // close the file descriptor
            close(g_state.fd);

            // check if it was in the wrong state
            if(lvRetValue == -2)
            {
                return EXIT_SUCCESS;
            }
            else
            {
                return EXIT_FAILURE;
            }  
        }

        lvRetValue = display_writeLine(0, 3, "SELF_TEST       ", 16, &g_state);
        // Check for an error
        if(lvRetValue)
        {
            cli_printfError("ERROR: Couldn't write to display!\n");

            // close the file descriptor
            close(g_state.fd);

            // check if it was in the wrong state
            if(lvRetValue == -2)
            {
                return EXIT_SUCCESS;
            }
            else
            {
                return EXIT_FAILURE;
            }            
        }

        // close the file descriptor
        close(g_state.fd);

        // state it is initialized
        gDisplayInitialized = true;

        // Check if the self-test shouldn't be skipped
        if(!skipSelfTest)
        {
            // output that the self test succeeded
            cli_printf("SELF-TEST DISPLAY: \e[32mPASS\e[39m\n");
        }
    }

    // set the return value to success
    lvRetValue = EXIT_SUCCESS;

    // return
    return lvRetValue;
}

/*!
 * @brief   This function is used to uninitialize the display part  
 *
 * @return  0 if ok, -1 if there is an error
 */
int display_uninitialize(void)
{
    // set the varaible to false to stop the update of the display
    // the pm handler will make sure it won't keep changing the power of the display
    gDisplayInitialized = false;

    // return
    return gDisplayInitialized;
}

/*!
 * @brief   This function is used to update the values of the display with the actual information  
 *
 * @return  0 if ok, -1 if there is an error
 */
int display_updateValues(void)
{
    // define local variables here 
    float floatValue1;
    void* dataReturn;
    uint8_t uint8Val;
    int lvRetValue = -1;
    char valueString[16];

    // check if initialized
    if(gDisplayInitialized)
    {
        // Open the framebuffer driver
        g_state.fd = open(gFbDev, O_RDWR);
        if(g_state.fd < 0)
        {
            int errcode = errno;
            cli_printfError("ERROR: Failed to open %s: %d\n", gFbDev, errcode);
            return EXIT_FAILURE;
        }

        // get the state of charge
        dataReturn = (int32_t*)data_getParameter(S_CHARGE, &uint8Val, NULL);

        // check for error 
        if(dataReturn == NULL)
        {
            // set the default value
            uint8Val = S_CHARGE_DEFAULT;

            // error output
            cli_printfError("Display ERROR: could not get s_charge!\n");

            // close the file descriptor
            close(g_state.fd);

            // return with an error
            return lvRetValue;
        }

        // write the value in the value string
        sprintf(valueString, "%3d", uint8Val);

        // write the value to the display
        lvRetValue = display_writeLine(SOC_DATA_INDEX_X, SOC_DATA_INDEX_Y, valueString, 
            SOC_DATA_LENGHT, &g_state);

        // Check for an error
        if(lvRetValue)
        {
            cli_printfError("ERROR: Couldn't write to display!\n");
            
            // close the file descriptor
            close(g_state.fd);

            if(lvRetValue == -2)
            {
                return EXIT_SUCCESS;
            }
            else
            {
                return EXIT_FAILURE;
            }
        }

        // get the output status
        dataReturn = (int32_t*)data_getParameter(S_OUT, &uint8Val, NULL);

        // check for error
        if(dataReturn == NULL)
        {
            // set the default value
            uint8Val = S_OUT_DEFAULT;

            // error output
            cli_printfError("Display ERROR: could not get s-out!\n");

            // close the file descriptor
            close(g_state.fd);

            // return with an error
            return lvRetValue;
        }

        // check if the output is enabled
        if(uint8Val & 1)
        {
            // get the output voltage
            dataReturn = (int32_t*)data_getParameter(V_OUT, &floatValue1, NULL);

            // check for error 
            if(dataReturn == NULL)
            {
                // set the default value
                floatValue1 = V_OUT_DEFAULT;

                // error output
                cli_printfError("Display ERROR: could not get v-out!\n");

                // close the file descriptor
                close(g_state.fd);

                // return with an error
                return lvRetValue;
            }

            // write the value in the value string
            sprintf(valueString, "%5.2f", floatValue1);

            // write the value to the display
            lvRetValue = display_writeLine(VOLTAGE_DATA_INDEX_X, VOLTAGE_DATA_INDEX_Y, valueString, 
                VOLTAGE_DATA_LENGHT, &g_state);

            if(lvRetValue)
            {
                cli_printfError("ERROR: Couldn't write to display!\n");
            
                // close the file descriptor
                close(g_state.fd);

                if(lvRetValue == -2)
                {
                    return EXIT_SUCCESS;
                }
                else
                {
                    return EXIT_FAILURE;
                }
            }

            // write the output ON to the display
            lvRetValue = display_writeLine(OUTPUT_STATUS_DATA_INDEX_X, OUTPUT_STATUS_DATA_INDEX_Y, "ON ", 
                OUTPUT_STATUS_DATA_LENGHT, &g_state);

            if(lvRetValue)
            {
                cli_printfError("ERROR: Couldn't write to display!\n");
            
                // close the file descriptor
                close(g_state.fd);
            
                if(lvRetValue == -2)
                {
                    return EXIT_SUCCESS;
                }
                else
                {
                    return EXIT_FAILURE;
                }
            }
        }
        // if the output is not enabled
        else
        {
            // get the battery voltage
            dataReturn = (int32_t*)data_getParameter(V_BATT, &floatValue1, NULL);

            // check for error 
            if(dataReturn == NULL)
            {
                // set the default value
                floatValue1 = V_BATT_DEFAULT;

                // error output
                cli_printfError("Display ERROR: could not get v-batt!\n");

                // close the file descriptor
                close(g_state.fd);

                // return with an error
                return lvRetValue;
            }

            // write the value in the value string
            sprintf(valueString, "%5.2f", floatValue1);

            // write the value to the display
            lvRetValue = display_writeLine(VOLTAGE_DATA_INDEX_X, VOLTAGE_DATA_INDEX_Y, valueString, 
                VOLTAGE_DATA_LENGHT, &g_state);

            if(lvRetValue)
            {
                cli_printfError("ERROR: Couldn't write to display!\n");
            
                // close the file descriptor
                close(g_state.fd);

                if(lvRetValue == -2)
                {
                    return EXIT_SUCCESS;
                }
                else
                {           
                    return EXIT_FAILURE;
                }
            }

            // write the output ON to the display
            lvRetValue = display_writeLine(OUTPUT_STATUS_DATA_INDEX_X, OUTPUT_STATUS_DATA_INDEX_Y, "OFF", 
                OUTPUT_STATUS_DATA_LENGHT, &g_state);

            if(lvRetValue)
            {
                cli_printfError("ERROR: Couldn't write to display!\n");
            
                // close the file descriptor
                close(g_state.fd);
            
                if(lvRetValue == -2)
                {
                    return EXIT_SUCCESS;
                }
                else
                {
                    return EXIT_FAILURE;
                }
            }
        }

        // get the battery ID
        dataReturn = (int32_t*)data_getParameter(BATT_ID, &uint8Val, NULL);

        // check for error 
        if(dataReturn == NULL)
        {
            // set the default value
            uint8Val = BATT_ID_DEFAULT;

            // error output
            cli_printfError("Display ERROR: could not get batt-id!\n");

            // close the file descriptor
            close(g_state.fd);

            // return with an error
            return lvRetValue;
        }

        // write the value in the value string
        sprintf(valueString, "%3d", uint8Val);

        // write the value to the display
        lvRetValue = display_writeLine(BATT_ID_DATA_INDEX_X, BATT_ID_DATA_INDEX_Y, valueString, 
            BATT_ID_DATA_LENGHT, &g_state);

        // Check for an error
        if(lvRetValue)
        {
            cli_printfError("ERROR: Couldn't write to display!\n");
            
            // close the file descriptor
            close(g_state.fd);

            if(lvRetValue == -2)
            {
                return EXIT_SUCCESS;
            }
            else
            {
                return EXIT_FAILURE;
            }
        }

        // get the state of health
        dataReturn = (int32_t*)data_getParameter(S_HEALTH, &uint8Val, NULL);

        // check for error 
        if(dataReturn == NULL)
        {
            // set the default value
            uint8Val = S_HEALTH_DEFAULT;

            // error output
            cli_printfError("Display ERROR: could not get s-health!\n");

            // close the file descriptor
            close(g_state.fd);

            // return with an error
            return lvRetValue;
        }

        // check if the SoH is known
        if(uint8Val != 127)
        {
            // write the value in the value string
            sprintf(valueString, "%3d", uint8Val);
        }
        // if it is not known
        else
        {
            // write ??? to it to indicate it is not known yet
            sprintf(valueString, UNKNOWN_SOH);
        }

        // write the value to the display
        lvRetValue = display_writeLine(SOH_DATA_INDEX_X, SOH_DATA_INDEX_Y, valueString, 
            SOH_DATA_LENGHT, &g_state);

        // Check for an error
        if(lvRetValue)
        {
            cli_printfError("ERROR: Couldn't write to display!\n");
            
            // close the file descriptor
            close(g_state.fd);
            
            if(lvRetValue == -2)
            {
                return EXIT_SUCCESS;
            }
            else
            {
                return EXIT_FAILURE;
            }
        }

        // get the battery voltage
        dataReturn = (int32_t*)data_getParameter(I_BATT_AVG, &floatValue1, NULL);

        // check for error 
        if(dataReturn == NULL)
        {
            // set the default value
            floatValue1 = I_BATT_AVG_DEFAULT;

            // error output
            cli_printfError("Display ERROR: could not get i-batt-avg!\n");

            // close the file descriptor
            close(g_state.fd);

            // return with an error
            return lvRetValue;
        }

        // write the value in the value string
        sprintf(valueString, "%6.2f", floatValue1);

        // write the value to the display
        lvRetValue = display_writeLine(CURRENT_DATA_INDEX_X, CURRENT_DATA_INDEX_Y, valueString, 
            CURRENT_DATA_LENGHT, &g_state);

        // Check for an error
        if(lvRetValue)
        {
            cli_printfError("ERROR: Couldn't write to display!\n");
            
            // close the file descriptor
            close(g_state.fd);
            
            if(lvRetValue == -2)
            {
                return EXIT_SUCCESS;
            }
            else
            {
                return EXIT_FAILURE;
            }
        }

        // get the sensor enable variable
        dataReturn = (int32_t*)data_getParameter(SENSOR_ENABLE, &uint8Val, NULL);

        // check for error 
        if(dataReturn == NULL)
        {
            // set the default value
            uint8Val = SENSOR_ENABLE_DEFAULT;

            // error output
            cli_printfError("Display ERROR: could not get sensor-enable!\n");

            // close the file descriptor
            close(g_state.fd);

            // return with an error
            return lvRetValue;
        }

        // check if the battery temperature sensor is used
        if(uint8Val & 1)
        {
            // get the battery temperature
            dataReturn = (int32_t*)data_getParameter(C_BATT, &floatValue1, NULL);

            // check for error 
            if(dataReturn == NULL)
            {
                // set the default value
                floatValue1 = C_BATT_DEFAULT;

                // error output
                cli_printfError("Display ERROR: could not get c-batt!\n");

                // close the file descriptor
                close(g_state.fd);

                // return with an error
                return lvRetValue;
            }

            // write the value in the value string
            sprintf(valueString, "%5.1f", floatValue1);

            // write the value to the display
            lvRetValue = display_writeLine(TEMPERATURE_DATA_INDEX_X, TEMPERATURE_DATA_INDEX_Y, valueString, 
                TEMPERATURE_DATA_LENGHT, &g_state);

            // Check for an error
            if(lvRetValue)
            {
                cli_printfError("ERROR: Couldn't write to display!\n");
            
                // close the file descriptor
                close(g_state.fd);
            
                if(lvRetValue == -2)
                {
                    return EXIT_SUCCESS;
                }
                else
                {
                    return EXIT_FAILURE;
                }
            }
        }
        // if the battery temp sensor is not enabled
        else
        {
            // set it to ---
            // write the value to the display
            lvRetValue = display_writeLine(TEMPERATURE_DATA_INDEX_X, TEMPERATURE_DATA_INDEX_Y, " --.-", 
                TEMPERATURE_DATA_LENGHT, &g_state);

            if(lvRetValue)
            {
                cli_printfError("ERROR: Couldn't write to display!\n");
            
                // close the file descriptor
                close(g_state.fd);
            
                if(lvRetValue == -2)
                {
                    return EXIT_SUCCESS;
                }
                else
                {
                    return EXIT_FAILURE;
                }
            }
        }

        // reset the value string to spaces
        sprintf(valueString, ALL_SPACES_16);

        // get the state in a uint8 value
        uint8Val = (uint8_t)data_getMainState();

        // check if the charge state needs to be outputted as well
        if((states_t)uint8Val == CHARGE)
        {
            // get the charge state string and put it in the value string 
            cli_getStateString(false, (uint8_t)data_getChargeState(), valueString);
        }
        else
        {
            // get the state string and put it in the value string 
            cli_getStateString(true, (uint8_t)data_getMainState(), valueString);
        }

        // write the value to the display
        lvRetValue = display_writeLine(STATE_DATA_INDEX_X, STATE_DATA_INDEX_Y, valueString, 
            STATE_DATA_LENGHT, &g_state);

        // Check for an error
        if(lvRetValue)
        {
            cli_printfError("ERROR: Couldn't write to display!\n");
            
            // close the file descriptor
            close(g_state.fd);
            
            if(lvRetValue == -2)
            {
                return EXIT_SUCCESS;
            }
            else
            {
                return EXIT_FAILURE;
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

 * @return 0 if succeeded, failure otherwise
 */
int display_writeLine(uint8_t startPosX, uint8_t startPosY, char *line, uint8_t size, 
    struct fb_state_s *state_p)
{
    FAR const struct nx_fontbitmap_s *fbm;
    uint8_t i, y;
    struct nxgl_size_s fsize;
    struct fb_area_s area;
    int lvRetValue = -1;

    // check for error value
    if(startPosX > 15)
    {
        cli_printfError("ERROR: startPosX > 15\n");
        return lvRetValue;
    }
    if(startPosY > 3)
    {
        cli_printfError("ERROR: startPosY > 3\n");
        return lvRetValue;
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
                ((uint8_t*)state_p->fbmem)[((startPosY*8+y)*state_p->pinfo.stride)+i+startPosX] = fbm->bitmap[y];
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
                ((uint8_t*)state_p->fbmem)[((startPosY*8+y)*state_p->pinfo.stride)+i+startPosX] = 0;
            }
        }
    }

// if the area needs to be updated
#ifdef CONFIG_FB_UPDATE
    // create the area to update
    area.x = startPosX*8;      /* x-offset of the area */
    area.y = startPosY*8;      /* y-offset of the area */
    area.w = 8*size;           /* Width of the area */
    area.h = 8;                /* Height of the area */

    // update the area with the framebuffer
    lvRetValue = ioctl(state_p->fd, FBIO_UPDATE, (unsigned long)((uintptr_t)&area));

    // check for errors
    if(lvRetValue < 0)
    {
      int errcode = errno;
      fprintf(stderr, "ERROR: ioctl(FBIO_UPDATE) failed: %d\n", errcode);
    }
#endif

    return lvRetValue;
}

