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

#include <uORB/topics/xkf_output.h>
#include <uORB/topics/shp_output.h>
#include <uORB/topics/xkf_residual_output.h>
#include <uORB/topics/NLO_output.h>

#include <math.h>
#include <iostream>
#include <eigen3/Eigen/Dense>  //Linear algebra library
#include <drivers/drv_hrt.h>  
#include <fstream>
#include <chrono>
#include <random>



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

	bool flying = false;

void shp_module::run()
{
	//Physical properties:
	float b = 1.5e-06;		//motor constant
	float d = 1e-14;	  	//drag factor
	float l = 0.26; 		//arm length
	double Ix = 0.0347563; 	//From sdf file
	double Iy = 0.0458929; 
	double Iz = 0.0977;
	double g = 9.8066; 		//From iris.world file
	double m = 1.5; 		//From sdf file

	//Copy from Agus:
	double pro_noise     = 0.01;
	double meas_noise    = 0.2;
	Eigen::MatrixXd Q(12,12); Q = pro_noise * pro_noise *  Q.setIdentity();
	Eigen::MatrixXd R(9,9); R = meas_noise * meas_noise * R.setIdentity();
	Eigen::MatrixXd P(12,12); P.setIdentity();

	Eigen::MatrixXd C(9,12);
	C(0,0) = 1; C(1,1) = 1; C(2,2) = 1; C(3,3) = 1; C(4,4) = 1; C(5,5) = 1; C(6,9) = 1; C(7,10) = 1; C(8,11) = 1;
	Eigen::MatrixXd H = C;
	Eigen::MatrixXd K(12,9);
	Eigen::MatrixXd Pdot(12,12);


	Eigen::MatrixXd F(12,12); F.setZero(); //Linear model:
	F(0,3) = 1; F(1,4) = 1; F(2,5) = 1; F(6,1) = -g; F(7,0) = g; F(9,6) = 1; F(10,7) = 1; F(11,8) = 1;

	//Adding sensor noise to ground truth outputs:
	unsigned seed1 = std::chrono::system_clock::now().time_since_epoch().count(); 	
	std::default_random_engine generator(seed1);
 	std::normal_distribution<double> distribution(0.0, meas_noise); //(mean, std deviation) std normal distribution


	//Init variables:
	bool updated = false;

	double dt = 0;
	double taux = 0, tauy = 0, tauz = 0, ft = 0;

	//dt variables:
	double timestamp = 0;
	double timestamp_old = 0;
	double time_flying = 0;
	
	//Setup vectors and matrices:
	Eigen::VectorXd y(9); y.setZero(); 			//Initialize output vector. Containing simulation states
	Eigen::VectorXd yhat(9); y.setZero();		//output from state estimator
	Eigen::VectorXd z(9); z.setZero(); 			//Output from thau observer
	Eigen::VectorXd xhat(12); xhat.setZero();
	Eigen::VectorXd xhatdot(12); xhatdot.setZero();
	Eigen::VectorXd Qxdot(12); Qxdot.setZero();
	Eigen::VectorXd Qx(12); Qx.setZero();


	Eigen::MatrixXd PThau(12,9); PThau.setZero();	//Non linear observer gain
	PThau(0,0) = 1.5;	PThau(0,3) = 0.5;	PThau(1,1) = 1.5;
	PThau(1,4) = 0.5;	PThau(2,2) = 1.5;	PThau(2,5) = 0.5;	PThau(3,0) = 0.5;
	PThau(3,3) = 0.5;	PThau(4,1) = 0.5;	PThau(4,4) = 0.5;	PThau(5,2) = 0.5; 
	PThau (5,5) = 0.5;	PThau(6,6) = 1.0;	PThau(7,7) = 1.0;	PThau(8,8) = 1.0;
	PThau(9,6) = 2.0;	PThau(10,7) = 2.0;	PThau(11,8) = 2.0;	PThau(2,5) = 0.5;
	double Gain = 20.0; //Sketchy
	PThau = Gain * PThau;

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
	struct xkf_output_s xkf_out;
	struct xkf_residual_output_s res_out;
	struct NLO_output_s nlo_out;

	memset(&xkf_out, 0, sizeof(xkf_out));
    orb_advert_t xkf_pub_fd = orb_advertise(ORB_ID(xkf_output), &xkf_out);
	memset(&shp_out, 0, sizeof(shp_out));
    orb_advert_t shp_pub_fd = orb_advertise(ORB_ID(shp_output), &shp_out);
	memset(&res_out, 0, sizeof(res_out));
    orb_advert_t res_pub_fd = orb_advertise(ORB_ID(xkf_residual_output), &res_out);
	memset(&nlo_out, 0, sizeof(nlo_out));
    orb_advert_t nlo_pub_fd = orb_advertise(ORB_ID(NLO_output), &nlo_out);


	std::ofstream myfile;
	std::ofstream actfile;
	actfile.open("actoutput.txt");
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
				if(!flying)
					timestamp_old = hrt_absolute_time();

				flying = true;
				time_flying = hrt_absolute_time() / 1e6;
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

				float q[4] = {0};
				q[0] = att.q[0]; q[1] = att.q[1]; q[2] = att.q[2]; q[3] = att.q[3];
				
				//Wikipedia:
				float roll = atan2(2.0f * (q[0] * q[1] + q[2] * q[3]), 1 - 2*(q[1] * q[1] + q[2] * q[2])); //roll quaternion conversion
				float pitch = asin(2*(q[0]*q[2] - q[3]*q[1]));											   //pitch
				float yaw = atan2(2.0f * (q[0] * q[3] + q[1] * q[2]), 1 - 2*(q[2] * q[2] + q[3] * q[3])); //yaw

				//Haukur og Valthor:
				// float yaw   = -atan2(2.0f * (q[1] * q[2] + q[0] * q[3]), q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3]);   
				// float pitch = asin(2.0f * (q[1] * q[3] - q[0] * q[2]));
				// float roll  = atan2(2.0f * (q[0] * q[1] + q[2] * q[3]), q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]);
							
				y(0) = roll;
				y(1) = pitch;
				y(2) = yaw;
				y(3) = att.rollspeed;//; - att.yawspeed * sin(pitch); 							//p, formula from p.12 Thomas S. Alderete.
				y(4) = att.pitchspeed;// * cos(roll) + att.yawspeed * cos(pitch) * sin(roll);	//Maybe change to just: att.roll-/pitch-/yawspeed
				y(5) = att.yawspeed;// * cos(pitch) * cos(roll) - att.pitchspeed * sin(roll);	//Because small angle approx, gives T = I_3x3
				y(6) = pos.x;
				y(7) = pos.y;
				y(8) = -pos.z;// + distribution(generator); //Should it be minus here?	
				
				timestamp = hrt_absolute_time();
				dt = (timestamp - timestamp_old) / 1e6; //dt = 0.0005;
				//dt = 0.0005;
				timestamp_old = timestamp;	
	
				Qxdot(0) = Qx(3)+Qx(5)*Qx(1)+Qx(4)*Qx(0)*Qx(1);
				Qxdot(1) = Qx(4)-Qx(5)*Qx(0);
				Qxdot(2) = Qx(5)+Qx(4)*Qx(0);
				Qxdot(3) = ((Iy-Iz)/Ix)*Qx(5)*Qx(4)+taux/Ix;
				Qxdot(4) = ((Iz-Ix)/Iy)*Qx(3)*Qx(5)+tauy/Iy;
				Qxdot(5) = ((Ix-Iy)/Iz)*Qx(3)*Qx(4)+tauz/Iz;
				Qxdot(6) = Qx(5)*Qx(7)-Qx(6)*Qx(10)-g*Qx(1);
				Qxdot(7) = Qx(3)*Qx(8)-Qx(5)*Qx(6)+g*Qx(0);
				Qxdot(8) = Qx(4)*Qx(6)-Qx(3)*Qx(7)+ g - (ft/m);
				Qxdot(9) = Qx(8)*(Qx(0)*Qx(2)+Qx(1))-Qx(7)*(Qx(2)-Qx(0)*Qx(1))+Qx(6);
				Qxdot(10) = Qx(7)*(1+Qx(0)*Qx(1)*Qx(2))-Qx(8)*(Qx(0)-Qx(1)*Qx(2))+Qx(6)*Qx(2);
				Qxdot(11) = Qx(8)-Qx(6)*Qx(1)+Qx(7)*Qx(0);

				Qxdot = Qxdot + PThau*(y-C*Qx);
				Qx = Qx + Qxdot * dt;

				xhatdot(0) = Qx(3); //change all the way
				xhatdot(1) = Qx(4);
				xhatdot(2) = Qx(5);
				xhatdot(3) = taux/Ix;
				xhatdot(4) = tauy/Iy;
				xhatdot(5) = tauz/Iz;
				xhatdot(6) = -g*Qx(1);
				xhatdot(7) = g*Qx(0);
				xhatdot(8) = -(ft/m);
				xhatdot(9) = Qx(6);
				xhatdot(10) = Qx(7);
				xhatdot(11) = Qx(8);

				//Messing up model still works: ???
				// xhatdot(0) = 0;  xhatdot(1) = 0; xhatdot(2) = 0;  xhatdot(3) = 0; xhatdot(4) = 0; xhatdot(5) = 0; xhatdot(6) = 0;	
				// xhatdot(7) = 0; xhatdot(8) = 0; xhatdot(9) = 0;xhatdot(10) = 0; xhatdot(11) = 0;

				// yhat(0) = xhat(0);
				// yhat(1) = xhat(1);
				// yhat(2) = xhat(2);
				// yhat(3) = xhat(3);
				// yhat(4) = xhat(4);
				// yhat(5) = xhat(5);
				// yhat(6) = xhat(9);
				// yhat(7) = xhat(10);
				// yhat(8) = xhat(11);		
				yhat = C * xhat;

				//xhatdot = xhatdot + PThau * (y-yhat); //Pure nonlinear observer
				//xhat = xhat + xhatdot * dt; 
				z = C * Qx;

				//Kalman filter from Agus:
				K = P * C.transpose() * R.inverse();
				xhatdot = xhatdot + K * (z - H * xhat);	
				xhat = xhat + xhatdot * dt;
				Pdot = F * P + P * F.transpose() + Q - P * H.transpose() * R.inverse() * H * P;
				P = P + Pdot * dt;

				//Publish collected data:
				//Custom output shp, for test purposes
				shp_out.timestamp 		= timestamp;
				shp_out.roll 			= y(0) + distribution(generator); //Noisy outputs
				shp_out.pitch 			= y(1) + distribution(generator);
				shp_out.yaw 			= y(2) + distribution(generator);
				shp_out.x 				= y(6) + distribution(generator);
				shp_out.y 				= y(7) + distribution(generator);
				shp_out.z 				= y(8) + distribution(generator);
				shp_out.roll_ground_truth	= roll;
				shp_out.pitch_ground_truth 	= pitch;
				shp_out.yaw_ground_truth 	= yaw;

				//NLO observer outputs:
				nlo_out.timestamp = timestamp;
				nlo_out.roll 		= Qx(0);
				nlo_out.pitch		= Qx(1);
				nlo_out.yaw 		= Qx(2);
				nlo_out.x 			= Qx(9);
				nlo_out.y 			= Qx(10);
				nlo_out.z 			= Qx(11);

				//XKF filter outputs:
				xkf_out.timestamp 	= timestamp;
				xkf_out.roll 		= xhat(0);
				xkf_out.pitch		= xhat(1);
				xkf_out.yaw 		= xhat(2);
				xkf_out.x 			= xhat(9);
				xkf_out.y 			= xhat(10);
				xkf_out.z 			= xhat(11);

				//Residual outputs XKF:
				res_out.timestamp 	= timestamp;
				res_out.roll_r 		= y(0) - xhat(0);
				res_out.pitch_r 	= y(1) - xhat(1);
				res_out.yaw_r 		= y(2) - xhat(2);
				res_out.x_r 		= y(6) - xhat(9);
				res_out.y_r 		= y(7) - xhat(10);
				res_out.z_r 		= y(8) - xhat(11);

				orb_publish(ORB_ID(shp_output), shp_pub_fd, &shp_out);
				orb_publish(ORB_ID(xkf_output), xkf_pub_fd, &xkf_out);
				orb_publish(ORB_ID(xkf_residual_output), res_pub_fd, &res_out);
				orb_publish(ORB_ID(NLO_output), nlo_pub_fd, &nlo_out);


				orb_check(pos_fd, &updated);

				//Debugging:
				//std::cout << "dt: " <<  dt << std::endl;
				//std::cout << xhatdot << std::endl;
				//std::cout << act.output[0] << "   " << act.output[1] << "    " << act.output[2] << "    " << act.output[3] << std::endl; 
				//myfile << time_flying << " " << xhat(0) << " " << xhat(1)<<  " " << xhat(2)<< " " << xhat(9) <<" " << xhat(10) <<" " << xhat(11) << std::endl;// << " " << taux << " " << tauy << " " << tauz << " " << ft << std::endl;
				//std::cout << xhat << std::endl;
				//std::cout << y;
				//time_flying += dt;
				//actfile << time_flying << " " << act.output[0] << " " << act.output[1] << " " << act.output[2] << " " << act.output[3] << std::endl; 
				
				
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


	Eigen::VectorXd vec(3);
	vec(0) = 0;
	vec(1) = 1;
	vec(2) = 2;

	Eigen::VectorXd n_vec(3);
	n_vec = 2 * vec;
	std::cout << n_vec;

	return shp_module::main(argc, argv);
}
