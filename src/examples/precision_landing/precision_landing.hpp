/****************************************************************************
 *
 *   Copyright (C) 2015 Mark Charlebois. All rights reserved.
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
 * @file hello_.h
 *  app for Linux
 *
 * @author Mark Charlebois <charlebm@gmail.com>
 */
#pragma once

#include <px4_platform_common/app.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/tasks.h>
#include <px4_platform_common/posix.h>
#include <unistd.h>
#include <stdio.h>
#include <poll.h>
#include <string.h>
#include <math.h>

#include <uORB/uORB.h>
#include <uORB/PublicationMulti.hpp>
#include <uORB/topics/home_position.h>
#include <uORB/topics/vehicle_local_position_setpoint.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/thermal_cam.h>

#define C_PI (double)3.141592653589793

#define SLOW_DOWN_HEIGHT 3.0
#define LANDING_HEIGHT 2.0
#define LADNING_HEIGHT_LOWER_LIMIT 1.3

#define CAMERA_RESOLUTION_X 336		//pixels
#define CAMERA_RESOLUTION_Y 256
#define CAMERA_FOV_X 35			//degrees
#define CAMERA_FOV_Y 27

#define MAXHORIZONTALSPEED 0.8 //set the maximum speed in m/s of autonomous flying.
#define MAXVERICALSPEED 2.1

class PrecisionLanding
{
public:
	PrecisionLanding();
	//
	~PrecisionLanding();

	int main();
	double toRadians(double degrees);

	static px4::AppState appState; /* track requests to terminate app */

private:
	uORB::PublicationMulti<vehicle_local_position_setpoint_s>  _vehicle_local_position_setpoint_pub;

	struct thermal_cam_s therm_cam;
	struct vehicle_local_position_s vehicle_local_position;
	struct home_position_s home_position;
};
