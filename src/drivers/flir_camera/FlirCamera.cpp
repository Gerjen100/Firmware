/****************************************************************************
 *
 *   Copyright (c) 2013 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file FlirCamera.cpp
 *
 * FlirCamera Motor Driver
 *
 * references:
 * http://downloads.orionrobotics.com/downloads/datasheets/motor_controller_robo_claw_R0401.pdf
 *
 */

#include "FlirCamera.hpp"
#include <poll.h>
#include <stdio.h>
#include <math.h>
#include <string.h>
#include <fcntl.h>
#include <termios.h>

#include <systemlib/err.h>
#include <systemlib/mavlink_log.h>


#include <uORB/Publication.hpp>
#include <uORB/topics/debug_key_value.h>
#include <uORB/topics/thermal_cam.h>
#include <drivers/drv_hrt.h>
#include <math.h>

// The FlirCamera has a serial communication timeout of 10ms.
// Add a little extra to account for timing inaccuracy
#define TIMEOUT_US 10500

// If a timeout occurs during serial communication, it will immediately try again this many times
#define TIMEOUT_RETRIES 1

// If a timeout occurs while disarmed, it will try again this many times. This should be a higher number,
// because stopping when disarmed is pretty important.
#define STOP_RETRIES 10

// Number of bytes returned by the Roboclaw when sending command 78, read both encoders
#define ENCODER_MESSAGE_SIZE 10

// Number of bytes for commands 18 and 19, read speeds.
#define ENCODER_SPEED_MESSAGE_SIZE 7

bool FlirCamera::taskShouldExit = false;

FlirCamera::FlirCamera(const char *deviceName, const char *baudRateParam):
	_uart(0),
	_uart_set(),
	_uart_timeout{.tv_sec = 0, .tv_usec = TIMEOUT_US},
	_thermal_cam_pub{ORB_ID(thermal_cam)}
{
	_param_handles.serial_baud_rate = 			param_find(baudRateParam);
	_parameters_update();


		const char *uart_name = "/dev/ttyS3";
		PX4_INFO("opening port %s", uart_name);


		_uart = open(uart_name,  O_RDWR | O_NOCTTY | O_NDELAY);
		// unsigned speed = B115200;

		if (_uart < 0) {
			err(1, "failed to open port: %s", uart_name);
		}

		/* Try to set baud rate */
		struct termios uart_config;
		int termios_state;

		/* Back up the original uart configuration to restore it after exit */
		if ((termios_state = tcgetattr(_uart, &uart_config)) < 0) {
			PX4_INFO("ERR GET CONF %s: %d\n", uart_name, termios_state);
			close(_uart);
			// return -1;
		}

		/* Clear ONLCR flag (which appends a CR for every LF) */
		uart_config.c_oflag =  CS8 | CLOCAL | CREAD;
		uart_config.c_iflag = IGNCR;    //ignore CR, other options off
		uart_config.c_iflag |= IGNBRK;  //ignore break condition

		uart_config.c_oflag = 0;        //all options off
		uart_config.c_lflag = ICANON;     //process input as lines

		if (cfsetispeed(&uart_config, B115200) < 0 || cfsetospeed(&uart_config, B115200) < 0) {
			PX4_INFO("ERR SET BAUD %s: %d\n", uart_name, termios_state);
			close(_uart);
			// return -1;
		}


		if ((termios_state = tcsetattr(_uart, TCSANOW, &uart_config)) < 0) {
			PX4_INFO("ERR SET CONF %s\n", uart_name);
			close(_uart);
			// return -1;
		}

}

FlirCamera::~FlirCamera()
{
	close(_uart);
}

void set_uart_params(){

}

void FlirCamera::taskMain()
{
	// thread_running = true;
  char buffer[200];
	PX4_INFO("Flir camera thread starting...");
	while(!taskShouldExit){
		int n = read(_uart, buffer, sizeof buffer);
		if (n > 0) {
				if(buffer[0] == 36){
					int seperator_loc = 0;
					int end_disriptor_loc = 0;
					for(int i = 0; i < (int)sizeof(buffer);i++){
						//59 = ; which is the seperator sign
						if(buffer[i] == 59){
							seperator_loc = i;
						}

						//42 is * which is the end discriptor of the message
						if(buffer[i] == 42){
							end_disriptor_loc = i;
						}
					}
					if(seperator_loc != 0 && end_disriptor_loc != 0){

						int x_loc = 0;
						int y_loc = 0;

						if(sscanf(buffer, "$%d;%d*", &x_loc, &y_loc) == 2){
							thermal_cam_s therm_cam_msg;
							therm_cam_msg.x = x_loc;
							therm_cam_msg.y = y_loc;

							_thermal_cam_pub.publish(therm_cam_msg);
						}
					}
				}
			}
			px4_usleep(TIMEOUT_US);
		}
}

void FlirCamera::_parameters_update()
{
	int baudRate;
	param_get(_param_handles.serial_baud_rate, &baudRate);

	PX4_INFO("Parameters are being updated, baudrate %d", _param_handles.serial_baud_rate);

	switch (baudRate) {
	case 2400:
		_parameters.serial_baud_rate = B2400;
		break;

	case 9600:
		_parameters.serial_baud_rate = B9600;
		break;

	case 19200:
		_parameters.serial_baud_rate = B19200;
		break;

	case 38400:
		_parameters.serial_baud_rate = B38400;
		break;

	case 57600:
		_parameters.serial_baud_rate = B57600;
		break;

	case 115200:
		_parameters.serial_baud_rate = B115200;
		break;

	case 230400:
		_parameters.serial_baud_rate = B230400;
		break;

	case 460800:
		_parameters.serial_baud_rate = B460800;
		break;

	default:
		_parameters.serial_baud_rate = B2400;
	}
}
