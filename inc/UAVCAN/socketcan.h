/****************************************************************************
 * nxp_bms/BMS_v1/inc/UAVCAN/socketcan.h
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
 **     Filename    : socketcan.h
 **     Project     : SmartBattery_RDDRONE_BMS772
 **     Processor   : S32K144
 **     Version     : 1.00
 **     Date   		: 2020-10-29
 **     Abstract    :
 **        socketcan module.
 **
 ** ###################################################################*/
/*!
 ** @file socketcan.h
 **
 ** @version 01.00
 **
 ** @brief
 **        socketcan module. this module implements the socketcan interface
 **
 */

#ifndef SOCKETCAN_H_INCLUDED
#define SOCKETCAN_H_INCLUDED

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include <sys/time.h>
#include <sys/socket.h>

#include <nuttx/can.h>
#include <netpacket/can.h>

#include <canard.h>


#ifdef __cplusplus
extern "C" {
#endif

typedef struct CanardSocketInstance CanardSocketInstance;
typedef int fd_t;

struct CanardSocketInstance {
	fd_t               s;
	bool               can_fd;

	//// Send msg structure
	struct iovec       send_iov;
	struct canfd_frame send_frame;
	struct msghdr      send_msg;
	struct cmsghdr     *send_cmsg;
	struct timeval     *send_tv;   /* TX deadline timestamp */
	uint8_t            send_control[sizeof(struct cmsghdr) + sizeof(struct timeval)];

	//// Receive msg structure
	struct iovec       recv_iov;
	struct canfd_frame recv_frame;
	struct msghdr      recv_msg;
	struct cmsghdr     *recv_cmsg;
	uint8_t            recv_control[sizeof(struct cmsghdr) + sizeof(struct timeval)];
};

/// Creates a SocketCAN socket for corresponding iface can_iface_name
/// Also sets up the message structures required for socketcanTransmit & socketcanReceive
/// can_fd determines to use CAN FD frame when is 1, and classical CAN frame when is 0
/// The return value is 0 on succes and -1 on error
int16_t socketcanOpen(CanardSocketInstance *ins, const char *const can_iface_name, const bool can_fd);

/// Send a CanardFrame to the CanardSocketInstance socket
/// This function is blocking
/// The return value is number of bytes transferred, negative value on error.
int16_t socketcanTransmit(CanardSocketInstance *ins, const CanardFrame *txf);

/// Receive a CanardFrame from the CanardSocketInstance socket
/// This function is blocking
/// The return value is number of bytes received, negative value on error.
int16_t socketcanReceive(CanardSocketInstance *ins, CanardFrame *rxf);

// TODO implement ioctl for CAN filter
int16_t socketcanConfigureFilter(const fd_t fd, const size_t num_filters, const struct can_filter *filters);

/*!
 * @brief 	this function is used to set the bitrate of both classical CAN as CAN FD
 * 			
 * @param 	ins the canard socket instance
 * @param 	can_iface_name the name of the device (see canOpen())
 * @param 	arbit_bitrate the classical CAN bitrate in bit/s
 * @param 	data_bitrate the CAN FD bitrate in bit/s
 *
 * @return 	If successful, the function will return zero (OK), -1 otherwise.
 */
int16_t socketcanSetBitrate(CanardSocketInstance *ins, const char *const can_iface_name,
	 int32_t arbit_bitrate, int32_t data_bitrate);

#ifdef __cplusplus
}
#endif

#endif //SOCKETCAN_H_INCLUDED
