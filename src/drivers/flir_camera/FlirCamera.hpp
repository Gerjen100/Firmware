/****************************************************************************
 *
 *   Copyright (C) 2013 PX4 Development Team. All rights reserved.
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
 * @file RoboClas.hpp
 *
 * RoboClaw Motor Driver
 *
 * references:
 * http://downloads.orionrobotics.com/downloads/datasheets/motor_controller_robo_claw_R0401.pdf
 *
 */

#pragma once

#include <poll.h>
#include <stdio.h>
#include <termios.h>
#include <lib/parameters/param.h>
#include <uORB/PublicationMulti.hpp>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/thermal_cam.h>
#include <drivers/device/i2c.h>
#include <sys/select.h>
#include <sys/time.h>
#include <pthread.h>
#include <string.h>

/**
 * This is a driver for the RoboClaw motor controller
 */
class FlirCamera
{
public:

	void taskMain();
	static bool taskShouldExit;

	/**
	 * constructor
	 * @param deviceName the name of the
	 * 	serial port e.g. "/dev/ttyS2"
	 * @param address the adddress  of the motor
	 * 	(selectable on roboclaw)
	 * @param baudRateParam Name of the parameter that holds the baud rate of this serial port
	 */
	FlirCamera(const char *deviceName, const char *baudRateParam);

	/**
	 * deconstructor
	 */
	virtual ~FlirCamera();

	void printStatus(char *string, size_t n);

private:

	struct {
		speed_t serial_baud_rate;
	} _parameters{};

	struct {
		param_t serial_baud_rate;
	} _param_handles{};

	int _uart;
	fd_set _uart_set;
	struct timeval _uart_timeout;

	int _paramSub{-1};
	parameter_update_s _paramUpdate;

	uORB::PublicationMultiData<thermal_cam_s>	_thermal_cam_pub;

	void _parameters_update();
};
