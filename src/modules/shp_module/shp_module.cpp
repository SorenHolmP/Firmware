/****************************************************************************
 *
 *   Copyright (c) 2018 PX4 Development Team. All rights reserved.
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

#include "shp_module.h"

#include <px4_getopt.h>
#include <px4_log.h>
#include <px4_posix.h>

#include <uORB/topics/parameter_update.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/actuator_outputs.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/exogenous_kf.h>
#include <uORB/topics/shp_output.h>

#include <math.h>
#include <iostream>
#include <eigen3/Eigen/Dense>  //Linear algebra library
#include <drivers/drv_hrt.h>  
#include <fstream>



int shp_module::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Section that describes the provided module functionality.
This is a template for a module running as a task in the background with start/stop/status functionality.
### Implementation
Section describing the high-level implementation of this module.
### Examples
CLI usage example:
$ module start -f -p 42
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("module", "template");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_PARAM_FLAG('f', "Optional example flag", true);
	PRINT_MODULE_USAGE_PARAM_INT('p', 0, 0, 1000, "Optional example parameter", true);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

int shp_module::print_status()
{
	PX4_INFO("Running");
	// TODO: print additional runtime information about the state of the module

	return 0;
}

int shp_module::custom_command(int argc, char *argv[])
{
	/*
	if (!is_running()) {
		print_usage("not running");
		return 1;
	}
	// additional custom commands can be handled like this:
	if (!strcmp(argv[0], "do-something")) {
		get_instance()->do_something();
		return 0;
	}
	 */

	return print_usage("unknown command");
}


int shp_module::task_spawn(int argc, char *argv[])
{
	_task_id = px4_task_spawn_cmd("shp_module",
				      SCHED_DEFAULT,
				      SCHED_PRIORITY_DEFAULT,
				      1024,
				      (px4_main_t)&run_trampoline,
				      (char *const *)argv);

	if (_task_id < 0) {
		_task_id = -1;
		return -errno;
	}

	return 0;
}

shp_module *shp_module::instantiate(int argc, char *argv[])
{
	int example_param = 0;
	bool example_flag = false;
	bool error_flag = false;

	int myoptind = 1;
	int ch;
	const char *myoptarg = nullptr;

	// parse CLI arguments
	while ((ch = px4_getopt(argc, argv, "p:f", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'p':
			example_param = (int)strtol(myoptarg, nullptr, 10);
			break;

		case 'f':
			example_flag = true;
			break;

		case '?':
			error_flag = true;
			break;

		default:
			PX4_WARN("unrecognized flag");
			error_flag = true;
			break;
		}
	}

	if (error_flag) {
		return nullptr;
	}

	shp_module *instance = new shp_module(example_param, example_flag);

	if (instance == nullptr) {
		PX4_ERR("alloc failed");
	}

	return instance;
}

shp_module::shp_module(int example_param, bool example_flag)
	: ModuleParams(nullptr)
{
}

void shp_module::run()
{
	//Setup vectors and matrices:
	Eigen::VectorXd y(9); y.setZero(); 			//Initialize output vector. Containing simulation states
	Eigen::VectorXd yhat(9); y.setZero();		//output from state estimator
	Eigen::VectorXd xhat(12); xhat.setZero();
	Eigen::VectorXd xhatdot(12); xhatdot.setZero();

	Eigen::MatrixXd PThau(12,9); PThau.setZero();
	PThau(0,0) = 1.5;
	PThau(0,3) = 0.5;
	PThau(1,1) = 1.5;
	PThau(1,4) = 0.5;	
	PThau(2,2) = 1.5;
	PThau(2,5) = 0.5;	
	PThau(3,0) = 0.5;
	PThau(3,3) = 0.5;
	PThau(4,1) = 0.5; 
	PThau(4,4) = 0.5;
	PThau(5,2) = 0.5; 
	PThau (5,5) = 0.5;
	PThau(6,6) = 1.0;
	PThau(7,7) = 1.0;
	PThau(8,8) = 1.0;
	PThau(9,6) = 2.0;
	PThau(10,7) = 2.0;
	PThau(11,8) = 2.0;
	PThau(2,5) = 0.5;
	double Gain = 100.0;
	PThau = Gain * PThau;

	//Physical properties:
	float b = 3e-6; 	//motor constant
	float d = 1e-7;	  	//drag factor
	float l = 0.26; 	//arm length
	double Ix = 0.0347563; //From sdf file
	double Iy = 0.0458929; 
	double Iz = 0.0977;
	double g = 9.82;
	double m = 1.5; //From sdf file

	//float roll_old, pitch_old, yaw_old;
	//Init variables:
	bool updated = false;
	bool flying = false;
	double dt = 0;
	double taux = 0, tauy = 0, tauz = 0, ft = 0;

	//dt variables:
	double timestamp = 0;
	double timestamp_old = 0;

	// Example: run the loop synchronized to the sensor_combined topic publication
	int sensor_combined_sub = orb_subscribe(ORB_ID(sensor_combined));

	//Subscribe to actuator topic
	int actuator_outputs_fd = orb_subscribe(ORB_ID(actuator_outputs));

	int pos_fd = orb_subscribe(ORB_ID(vehicle_local_position_groundtruth));

	int attitude_fd = orb_subscribe(ORB_ID(vehicle_attitude_groundtruth));
	px4_pollfd_struct_t fds[1];
	//fds[0].fd = sensor_combined_sub;
	//fds[0].events = POLLIN;
	fds[0].fd = pos_fd;
	fds[0].events = POLLIN;

	// initialize parameters
	int parameter_update_sub = orb_subscribe(ORB_ID(parameter_update));

	parameters_update(parameter_update_sub, true);


	//Structs for holding data:
	struct sensor_combined_s sensor_combined;
	struct vehicle_attitude_s att;
	struct vehicle_local_position_s pos;
	struct actuator_outputs_s act;
	
	//Struct for publishing xkf estimate:
	struct shp_output_s shp_out;

	memset(&shp_out, 0, sizeof(shp_out));
    orb_advert_t shp_pub_fd = orb_advertise(ORB_ID(shp_output), &shp_out);

	std::ofstream myfile;
    myfile.open("data_shp.txt");
	myfile << "roll " << "pitch " << "yaw " <<  "x " << "y " << "z " << "taux" << " " << "tauy" << " " << "tauz " << "ft" << std::endl;

	while (!should_exit()) {

		// wait for up to 1000ms for data
		int pret = px4_poll(fds, (sizeof(fds) / sizeof(fds[0])), 1000);

		if (pret == 0) {
			// Timeout: let the loop run anyway, don't do `continue` here
			PX4_INFO("Timeout");
		} else if (pret < 0) {
			// this is undesirable but not much we can do
			PX4_ERR("poll error %d, %d", pret, errno);
			usleep(50000);
			continue;

		} else if (fds[0].revents & POLLIN) {
			orb_check(actuator_outputs_fd, &updated);
			if(updated)
			{	
				flying = true;
				//PX4_INFO("I am updated");
				// for(int i = 0; i < 6; i++)
				// {
				// 	std::cout << actuator_outputs.output[i] << std::endl;
				// }

			}
			if(flying)
			{
				//Copy data into structs:
				orb_copy(ORB_ID(sensor_combined), sensor_combined_sub, &sensor_combined);
				orb_copy(ORB_ID(vehicle_attitude_groundtruth), attitude_fd, &att);
				orb_copy(ORB_ID(vehicle_local_position_groundtruth), pos_fd, &pos);			
				orb_copy(ORB_ID(actuator_outputs), actuator_outputs_fd, &act);

				//Calculate input based on actuator_outputs
				ft = b * (act.output[2]*act.output[2] + act.output[0]*act.output[0] + act.output[3]*act.output[3] + act.output[1]*act.output[1]);
				taux = b * l * (act.output[3]*act.output[3] - act.output[2]*act.output[2]);
				tauy = b * l * (act.output[1]*act.output[1] - act.output[0]*act.output[0]);
				tauz = d * (act.output[0]*act.output[0] + act.output[1]*act.output[1] - act.output[2]*act.output[2] - act.output[3]*act.output[3]);

				//PX4_INFO("ft: %f \t taux: %f \t tauy: %f \t tauz: %f",ft, taux, tauy, tauz);

				// xhatdot(0) = xhat(3)+xhat(5)*xhat(1)+xhat(4)*xhat(0)*xhat(1);
				// xhatdot(1) = xhat(4)-xhat(5)*xhat(0);
				// xhatdot(2) = xhat(5)+xhat(4)*xhat(0);
				// xhatdot(3) = ((Iy-Iz)/Ix)*xhat(5)*xhat(4)+taux/Ix;
				// xhatdot(4) = ((Iz-Ix)/Iy)*xhat(3)*xhat(5)+tauy/Iy;
				// xhatdot(5) = ((Ix-Iy)/Iz)*xhat(3)*xhat(4)+tauz/Iz;
				// xhatdot(6) = xhat(5)*xhat(7)-xhat(6)*xhat(10)-g*xhat(1);
				// xhatdot(7) = xhat(3)*xhat(8)-xhat(5)*xhat(6)+g*xhat(0);
				// xhatdot(8) = xhat(4)*xhat(6)-xhat(3)*xhat(7)+ g - (ft/m);
				// xhatdot(9) = xhat(8)*(xhat(0)*xhat(2)+xhat(1))-xhat(7)*(xhat(2)-xhat(0)*xhat(1))+xhat(6);
				// xhatdot(10) = xhat(7)*(1+xhat(0)*xhat(1)*xhat(2))-xhat(8)*(xhat(0)-xhat(1)*xhat(2))+xhat(6)*xhat(2);
				// xhatdot(11) = xhat(8)-xhat(6)*xhat(1)+xhat(7)*xhat(0);

				xhatdot(0) = 0;
				xhatdot(1) = 0;
				xhatdot(2) = 0;
				xhatdot(3) = 0;
				xhatdot(4) = 0;
				xhatdot(5) = 0;
				xhatdot(6) = 0;
				xhatdot(7) = 0;
				xhatdot(8) = 0;
				xhatdot(9) = 0;
				xhatdot(10) = 0;
				xhatdot(11) = 0;

				float q[4] = {0};
				q[0] = att.q[0]; q[1] = att.q[1]; q[2] = att.q[2]; q[3] = att.q[3];
				
				float roll = atan2(2.0f * (q[0] * q[1] + q[2] * q[3]), 1 - 2*(q[1] * q[1] + q[2] * q[2])); //roll quaternion conversion
				float pitch = asin(2*(q[0]*q[2] - q[3]*q[1]));												 //pitch
				float yaw = atan2(2.0f * (q[0] * q[3] + q[1] * q[2]), 1 - 2*(q[2] * q[2] + q[3] * q[3])); //yaw
				
				
				y(0) = roll;
				y(1) = pitch;
				y(2) = yaw;
				y(3) = att.rollspeed; - att.yawspeed * sin(pitch); 							//p, formula from p.12 Thomas S. Alderete.
				y(4) = att.pitchspeed * cos(roll) + att.yawspeed * cos(pitch) * sin(roll);	//q
				y(5) = att.yawspeed * cos(pitch) * cos(roll) - att.pitchspeed * sin(roll);	//r
				y(6) = pos.x;
				y(7) = pos.y;
				y(8) = pos.z;
				yhat(0) = xhat(0);
				yhat(1) = xhat(1);
				yhat(2) = xhat(2);
				yhat(3) = xhat(3);
				yhat(4) = xhat(4);
				yhat(5) = xhat(5);
				yhat(6) = xhat(9);
				yhat(7) = xhat(10);
				yhat(8) = xhat(11);		
				
				timestamp = hrt_absolute_time();
				//dt = (timestamp - timestamp_old) / 1e6;
				dt = 0.001;

				
				xhatdot = xhatdot + PThau * (y-yhat);
				xhat = xhat + xhatdot * dt; 
				//std::cout << "dt: " <<  dt << std::endl;
				myfile << xhat(0) << " " << xhat(1)<<  " " << xhat(2)<< " " << xhat(9) <<" " << xhat(10) <<" " << xhat(11) << " " << taux << " " << tauy << " " << tauz << " " << ft << std::endl;
				std::cout << xhat << std::endl;
				timestamp_old = timestamp;

				shp_out.timestamp = timestamp;
				shp_out.roll = xhat(0);
				shp_out.pitch = xhat(1);
				shp_out.yaw = xhat(2);
				shp_out.x = xhat(9);
				shp_out.y = xhat(10);
				shp_out.z = xhat(11);

				orb_publish(ORB_ID(shp_output), shp_pub_fd, &shp_out);

				orb_check(pos_fd, &updated);
				
				// TODO: do something with the data...

			}

		}
		parameters_update(parameter_update_sub);
	}

	orb_unsubscribe(sensor_combined_sub);
	orb_unsubscribe(parameter_update_sub);
}

void shp_module::parameters_update(int parameter_update_sub, bool force)
{
	bool updated;
	struct parameter_update_s param_upd;

	orb_check(parameter_update_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(parameter_update), parameter_update_sub, &param_upd);
	}

	if (force || updated) {
		updateParams();
	}
}


int shp_module_main(int argc, char *argv[])
{

	PX4_INFO("%f", cos(0));
	PX4_INFO("atan2: %f", atan2(1,1));	

	Eigen::VectorXd b(3);
	b(0) = atan2(1,1) * 2;


	//Matrix experimentation with eigen library:
	//Eigen::Matrix<float, 12, 9> L;
	//L.setIdentity();

	// Eigen::VectorXd xhatdot(12);
	// xhatdot(0) = xhat(3)+xhat(5)*xhat(1)+xhat(4)*xhat(0)*xhat(1);
	// xhatdot(1) = xhat(4)-xhat(5)*xhat(0);
	// xhatdot(2) = xhat(5)+xhat(4)*xhat(0);
	// xhatdot(3) = ((Iy-Iz)/Ix)*xhat(5)*xhat(4)+taux/Ix;
	// xhatdot(4) = ((Iz-Ix)/Iy)*xhat(3)*xhat(5)+tauy/Iy;
	// xhatdot(5) = ((Ix-Iy)/Iz)*xhat(3)*xhat(4)+tauz/Iz;
	// xhatdot(6) = xhat(5)*xhat(7)-xhat(6)*xhat(10)-g*xhat(1);
	// xhatdot(7) = xhat(3)*xhat(8)-xhat(5)*xhat(6)+g*xhat(0);
	// xhatdot(8) = xhat(4)*xhat(6)-xhat(3)*xhat(7)+g*(ft/m);
	// xhatdot(9) = xhat(8)*(xhat(0)*xhat(2)+xhat(1))-xhat(7)*(xhat(2)-xhat(0)*xhat(1))+xhat(6);
	// xhatdot(10) = xhat(7)*(1+xhat(0)*xhat(1)*xhat(2))-xhat(8)*(xhat(0)-xhat(1)*xhat(2))+xhat(6)*xhat(2);
	// xhatdot(11) = xhat(8)-xhat(6)*xhat(1)+xhat(7)*xhat(0);
	// std::cout << xhatdot;


	// yhat(0) = xhat(0);
	// yhat(1) = xhat(1);
	// yhat(2) = xhat(2);
	// yhat(3) = xhat(3);
	// yhat(4) = xhat(4);
	// yhat(5) = xhat(5);
	// yhat(6) = xhat(9);
	// yhat(7) = xhat(10);
	// yhat(8) = xhat(11);

	// xhatdot = xhatdot + NLO_gain * (y-C*xhatdot);

	/*float roll_vel =(roll - roll_old) / dt;			//Calculate angular velocity
			float pitch_vel	 = / (pitch - pitch_old) / dt;
			float yaw_vel    = (yaw - yaw_old) / dt;*/
				//roll_old = roll; pitch_old = pitch; yaw_old = yaw; //Update angles for next calculation of angular velocities


	return shp_module::main(argc, argv);
}
