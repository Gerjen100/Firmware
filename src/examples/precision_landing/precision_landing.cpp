
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

#include "precision_landing.hpp"
#include <px4_platform_common/time.h>
#include <unistd.h>
#include <stdio.h>

px4::AppState PrecisionLanding::appState;

PrecisionLanding::PrecisionLanding():
		_vehicle_local_position_setpoint_pub{ORB_ID(vehicle_local_position_setpoint)}
{

}

PrecisionLanding::~PrecisionLanding(){}

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

	// struct thermal_cam_s therm_cam;
	// struct vehicle_local_position_s vehicle_local_position;
	// struct home_position_s home_position;

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

		// PX4_INFO("Debug home pose z: %f, curr_alt z: %f, rel_height %f", (double)home_position.z, (double)vehicle_local_position.z, rel_height);




		//precision landing


	  double height = rel_height;  //heigt above takeoff

		// double land_slow_range = SLOW_DOWN_HEIGHT - LANDING_HEIGHT; //Range between the landing and from which altitude the drone is slowing down

		height = rel_height;

		PX4_INFO("thermal_cam x %d, thermal_cam y %d", therm_cam.x, therm_cam.y);

		double dY = (height * tan(toRadians(CAMERA_RESOLUTION_X)) * therm_cam.x) / CAMERA_RESOLUTION_X; //dx at ground level in meters  view angle radians is half the FOV
		double dX = (height * tan(toRadians(CAMERA_RESOLUTION_Y)) * therm_cam.y) / CAMERA_RESOLUTION_Y; //dy at ground level in meters, view angle radians is half the FOV

		double dY_max = (height * tan(toRadians(CAMERA_RESOLUTION_X)) * 0.5 * CAMERA_RESOLUTION_X) / CAMERA_RESOLUTION_X; //Max dx
		double dX_max = (height * tan(toRadians(CAMERA_RESOLUTION_Y)) * 0.5 * CAMERA_RESOLUTION_Y) / CAMERA_RESOLUTION_Y; //Max dy

    double slow_down = 1;

		//			TODO 				//
    // if(height < SLOW_DOWN_HEIGHT && height > LANDING_HEIGHT){
    //   slow_down = (1 - 0.5*((SLOW_DOWN_HEIGHT - rel_height) / land_slow_range)); //If in slow down range, gradually slow down more
    // }else if(height >= SLOW_DOWN_HEIGHT){
    //   slow_down = 1; //If above slow down range, no modifiers
    // }else{
    //   slow_down = 0.5; //If below slow down range, speed is always halved
    // }
		//			TODO 				//

		double vX = (dX / dX_max) * MAXHORIZONTALSPEED * slow_down;
		double vY = -(dY / dY_max) * MAXHORIZONTALSPEED * slow_down;

		// Camera X = Vehicle Y and vice versa
		// double vYaw = 0.0;
		double vZ = 0.0;
		if (dX > -1 && dY < 1 && dY > -1 && dX < 1) {
			vZ = fmax(-MAXVERICALSPEED*(height / 50), -MAXVERICALSPEED); //gradually slow down as the landing platform is being approached
		} else {
			vZ = 0;
		}

		vehicle_local_position_setpoint_s setpoint;
		setpoint.vx = vX;
		setpoint.vy = vY;
		setpoint.vz = vZ;
		// setpoint.yaw = 0;

		PX4_INFO("vX %f, vY %f, vZ %f", vX, vY, vZ);

		_vehicle_local_position_setpoint_pub.publish(setpoint);

    //added for test purposeses
    // mavrick::TestInfo test_info;
    // test_info.dX = dX;
    // test_info.dY = dY;
    // test_info.vZ = vZ;
    // test_info.vXlocal = vX;
    // test_info.vYlocal = vY;
    // test_info.height = height;

		//todo uorb

		// publishBodyCmdvel(vX, vY, vZ, vYaw);
		//
    // test_info.vXBody = vX;
    // test_info.vYBody = vY;

    // test_info_pub.publish(test_info);
		//
    // if(info_counter >= 25){ //Once every 25 publishes give an update
    //   ROS_INFO("Height : %f, Vx: %f, Vy: %f, Vz: %f, Vyaw: %f",height, vX, vY, vZ, vYaw);
    //   info_counter = 0;
    // }
    // info_counter++;
		//
		// ros::spinOnce();
		// publish_rate.sleep(); // sleep to make sure we're publishing at 50Hz
		// ros::spinOnce();
	//
  // if ((hotspotX < 30) && (hotspotY < 30) && (height < LANDING_HEIGHT)) {
  //   cout << " Initialize landing... hotspotX,Y: [" << hotspotX << " , " << hotspotY << "height: " << height << endl;
  //   land();
  // }







		px4_sleep(0.1); //not sure if necessary
	}

	return 0;
}

double PrecisionLanding::toRadians(double degrees){
	return degrees / 180.0 * C_PI;
}
