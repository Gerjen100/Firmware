
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
 * @file hello_.cpp
 *  for Linux
 *
 * @author Mark Charlebois <charlebm@gmail.com>
 */

#include "precision_landing.h"
#include <px4_platform_common/time.h>
#include <unistd.h>
#include <stdio.h>

px4::AppState PrecisionLanding::appState;

int  PrecisionLanding::main()
{
	appState.setRunning(true);

	int thermal_cam_sub_fd = orb_subscribe(ORB_ID(thermal_cam));
	int vehicle_local_pos_sub_fd = orb_subscribe(ORB_ID(vehicle_local_position));
	int home_pos_sub_fd = orb_subscribe(ORB_ID(home_position));

	orb_set_interval(thermal_cam_sub_fd, 200);
	orb_set_interval(vehicle_local_pos_sub_fd, 200);
	orb_set_interval(home_pos_sub_fd, 200);


	px4_pollfd_struct_t fds[] = {
		{ .fd = thermal_cam_sub_fd,   .events = POLLIN },
		{ .fd = vehicle_local_pos_sub_fd, .events = POLLIN},
		{ .fd = home_pos_sub_fd, .events = POLLIN},

	};

	struct thermal_cam_s therm_cam;
	struct vehicle_local_position_s vehicle_local_position;
	struct home_position_s home_position;

	vehicle_local_position.z = 0;
	home_position.z = 0;

	while(!appState.exitRequested()){
  	int poll_ret = px4_poll(fds, 3, 1000);

		//retreive subscribers
		if (fds[0].revents & POLLIN) {
			if(poll_ret != 0){
				orb_copy(ORB_ID(thermal_cam), thermal_cam_sub_fd, &therm_cam);
			}
		}

		if (fds[1].revents & POLLIN) {
			if(poll_ret != 0){
				orb_copy(ORB_ID(vehicle_local_position), vehicle_local_pos_sub_fd, &vehicle_local_position);
			}
		}

		if (fds[2].revents & POLLIN) {
			if(poll_ret != 0){
				orb_copy(ORB_ID(home_position), home_pos_sub_fd, &home_position);
			}
		}

		//calculate current altitude
		double rel_height = 0;
		if (home_position.valid_alt) {
			rel_height = -(vehicle_local_position.z - home_position.z);

		} else {
			rel_height = -vehicle_local_position.z;
		}

		PX4_INFO("Debug home pose z: %f, curr_alt z: %f, rel_height %f", (double)home_position.z, (double)vehicle_local_position.z, rel_height);
		px4_sleep(0.1); //not sure if necessary
	}

	return 0;
}
