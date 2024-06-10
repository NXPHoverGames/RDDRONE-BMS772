############################################################################
# apps/nxp_bms/bms/Makefile
#
# BSD 3-Clause License
# 
# Copyright 2020-2023 NXP
# 
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
# 
# 1. Redistributions of source code must retain the above copyright notice, this
#    list of conditions and the following disclaimer.
# 
# 2. Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.
# 
# 3. Neither the name of the copyright holder nor the names of its
#    contributors may be used to endorse or promote products derived from
#    this software without specific prior written permission.
# 
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#
############################################################################

-include $(APPDIR)/Make.defs

# BMS! built-in application info

PROGNAME  = $(CONFIG_NXP_BMS_PROGNAME)
PRIORITY  = $(CONFIG_NXP_BMS_PRIORITY)
STACKSIZE = $(CONFIG_NXP_BMS_STACKSIZE)
MODULE    = $(CONFIG_NXP_BMS)

# BMS application

CSRCS   = src/data.c
CSRCS   += src/cli.c
CSRCS   += src/ledState.c
CSRCS   += src/gpio.c
CSRCS   += src/batManagement.c
CSRCS   += src/spi.c
CSRCS   += src/cyphalcan.c
CSRCS   += src/dronecan.c

CSRCS   += src/BCC/bcc_wait.c
CSRCS   += src/BCC/bcc_peripheries.c
CSRCS   += src/BCC/bcc_monitoring.c
# CSRCS     += src/BCC/bcc_diag.c
CSRCS   += src/BCC/bcc_configuration.c
CSRCS   += src/BCC/bcc_spiwrapper.c
CSRCS   += src/BCC/Derivatives/bcc.c
CSRCS   += src/BCC/Derivatives/bcc_communication.c
# CSRCS     += src/BCC/Derivatives/bcc_diagnostics.c
CSRCS   += src/BCC/Derivatives/bcc_spi.c
CSRCS   += src/BCC/Derivatives/bcc_tpl.c
CSRCS   += src/CAN/o1heap.c
CSRCS   += src/CAN/socketcan.c
CSRCS   += src/CAN/pnp.c
CSRCS   += src/CAN/portid.c
CSRCS   += src/CAN/timestamp.c
CSRCS   += src/sbc.c
CSRCS   += src/nfc.c
CSRCS   += src/a1007.c
CSRCS   += src/SMBus.c
CSRCS   += src/i2c.c
CSRCS   += src/power.c
CSRCS   += src/display.c
CSRCS   += src/balancing.c

MAINSRC = src/main.c
CFLAGS  += -I inc
CFLAGS  += -I inc/BCC
CFLAGS  += -I inc/BCC/Derivatives
CFLAGS  += -I inc/CAN
CFLAGS  += -I inc/dronecan
CFLAGS  += -std=c11 -I$(APPDIR)/include/canutils
CFLAGS  += -I $(TOPDIR)/arch/arm/src/common
CFLAGS  += -I $(TOPDIR)/boards/arm/s32k1xx/rddrone-bms772/src
CFLAGS  += -I $(TOPDIR)/include/arch/chip
CFLAGS  += -I $(TOPDIR)/include/arch/board
CFLAGS  += -I $(TOPDIR)/arch/arm/src/s32k1xx
CFLAGS  += -I $(TOPDIR)/boards/arm/s32k1xx/drivers


include $(APPDIR)/Application.mk
