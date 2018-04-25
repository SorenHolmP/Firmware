/****************************************************************************
 *
 *   Copyright (c) 2012-2016 PX4 Development Team. All rights reserved.
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
 * @file px4_simple_app.c
 * Minimal application example for PX4 autopilot
 *
 * @author Example User <mail@example.com>
 */


#include "xkf.h"
#include <fstream>
#include <iostream>
#include <unistd.h>


#include <chrono>
#include <thread>

extern "C" __EXPORT int xkf_main(int argc, char *argv[]); //Et eller andet med C vs cpp filer

xkf::xkf()
{
    local_position_fd = orb_subscribe(ORB_ID(vehicle_local_position));
    attitude_fd = orb_subscribe(ORB_ID(vehicle_attitude));
    sensor_fd = orb_subscribe(ORB_ID(sensor_combined));
    orb_set_interval(local_position_fd,200); //update rate 1ms
    orb_set_interval(attitude_fd, 200);
    orb_set_interval(sensor_fd,500);

    memset(&shp_info, 0, sizeof(shp_info));
    shp_pub_fd = orb_advertise(ORB_ID(shp_output), &shp_info);
}

void xkf::begin()
{
    float q[4] = {0};
    float roll = 0;
    float pitch = 0;
    float yaw = 0;
    std::ofstream myfile;
    myfile.open("data.txt");
    // px4_pollfd_struct_t fds[] = {
	// 	{.fd = sensor_fd, .events = POLLIN}//, { .fd = attitude_fd,   .events = POLLIN }, {.fd = local_position_fd, .events = POLLIN}
	// };
    
    //  int error_counter = 0;
    //     for(int i = 0; i < 10; i++) {  
    //     //while(!should_exit()){
    //     /* wait for sensor update of 1 file descriptor for 1000 ms (1 second) */
    //     int poll_ret = px4_poll(fds, 1, 1000);

    //     /* handle the poll result */
    //     if (poll_ret == 0) {
    //         /* this means none of our providers is giving us data */
    //         PX4_ERR("Got no data within a second");

    //     } else if (poll_ret < 0) {
    //         /* this is seriously bad - should be an emergency */
    //         if (error_counter < 10 || error_counter % 50 == 0) {
    //             /* use a counter to prevent flooding (and slowing us down) */
    //             PX4_ERR("ERROR return value from poll(): %d", poll_ret);
    //         }

    //         error_counter++;

    //     } else {

    //         if (fds[0].revents & POLLIN) {
    //             /* obtained data for the first file descriptor */
    //             /* copy sensors raw data into local buffer */
    //             orb_copy(ORB_ID(vehicle_local_position), local_position_fd, &pos);
    //             orb_copy(ORB_ID(vehicle_attitude), attitude_fd, &att);

    //             q[0] = att.q[0]; q[1] = att.q[1]; q[2] = att.q[2]; q[3] = att.q[3];

    //             yaw         = atan2(2.0f * (q[0] * q[3] + q[1] * q[2]), 1 - 2*(q[2] * q[2] + q[3] * q[3]));
    //             pitch       = asin(2*(q[0]*q[2] - q[3]*q[1]));
    //             roll        = atan2(2.0f * (q[0] * q[1] + q[2] * q[3]), 1 - 2*(q[1] * q[1] + q[2] * q[2]));
                

    //             shp_info.timestamp = hrt_absolute_time();
    //             shp_info.x = 1;
    //             shp_info.y = 2;
    //             shp_info.z = 3;

    //             shp_info.yaw = yaw;
    //             shp_info.pitch = pitch;
    //             shp_info.roll = roll;

    //             orb_publish(ORB_ID(shp_output), shp_pub_fd, &shp_info);
    //             PX4_INFO("I am alive");
    //             myfile << hrt_absolute_time() << " " << yaw << " " << pitch << " " << " " << roll << std::endl;
    //         }

    //         /* there could be more file descriptors here, in the form like:
    //          * if (fds[1..n].revents & POLLIN) {}
    //          */
    //     }
    // }



   // myfile << "Hello dad";


 
    
    for(int i = 0; i < 2; i++)
    {
        orb_copy(ORB_ID(vehicle_local_position), local_position_fd, &pos);
        orb_copy(ORB_ID(vehicle_attitude), attitude_fd, &att);

        q[0] = att.q[0]; q[1] = att.q[1]; q[2] = att.q[2]; q[3] = att.q[3];

        yaw         = atan2(2.0f * (q[0] * q[3] + q[1] * q[2]), 1 - 2*(q[2] * q[2] + q[3] * q[3]));
        pitch       = asin(2*(q[0]*q[2] - q[3]*q[1]));
        roll        = atan2(2.0f * (q[0] * q[1] + q[2] * q[3]), 1 - 2*(q[1] * q[1] + q[2] * q[2]));
        
        roll = roll;
        pitch = pitch;
        yaw = yaw;


        shp_info.timestamp = hrt_absolute_time();
        shp_info.x = 1;
        shp_info.y = 2;
        shp_info.z = 3;

        shp_info.yaw = yaw;
        shp_info.pitch = pitch;
        shp_info.roll = roll;
        myfile << shp_info.timestamp << std::endl;

        //orb_publish(ORB_ID(shp_output), shp_pub_fd, &shp_info);

        std::this_thread::sleep_for(std::chrono::milliseconds(10000));

        myfile << hrt_absolute_time() << " " << yaw << " " << pitch << " " << " " << roll << std::endl;

        PX4_INFO("I am alive from regular loop");
    }
    
 }

int xkf_main(int argc, char *argv[])
{
    if(!strcmp(argv[1], "start"))
    {
        PX4_INFO("Hello Sky!");
        xkf xkf_obj;
        xkf_obj.begin();
    }
//    int task_id =  px4_task_spawn_cmd("xkf",
//                         SCHED_DEFAULT,
//                         SCHED_PRIORITY_DEFAULT,
//                         1024,
//                         (px4_main_t)&run_trampoline,
//                         (char *const *)argv);


    return 0;
}
