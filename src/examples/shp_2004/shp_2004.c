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

#include <px4_log.h>
#include <poll.h>

#include <px4_getopt.h>
#include <px4_log.h>
#include <px4_posix.h>
#include <px4_config.h>
#include <px4_tasks.h>
#include <px4_module.h>

#include <drivers/drv_hrt.h>


#include <uORB/uORB.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/shp_output.h>

#include <math.h>


__EXPORT int shp_2004_main(int argc, char *argv[]);


int shp_2004_main(int argc, char *argv[])
{
    PX4_INFO("Hello Sky!");

    int local_pos_sub = orb_subscribe(ORB_ID(vehicle_local_position));
    int attitude_fd = orb_subscribe(ORB_ID(vehicle_attitude));

    orb_set_interval(local_pos_sub,100);
    orb_set_interval(attitude_fd, 100);


    struct vehicle_local_position_s pos;
    struct vehicle_attitude_s att;
    struct shp_output_s shp_info;

    orb_copy(ORB_ID(vehicle_local_position), local_pos_sub, &pos);
    orb_copy(ORB_ID(vehicle_attitude), attitude_fd, &att);


    memset(&shp_info, 0, sizeof(shp_info));
    orb_advert_t shp_pub_fd = orb_advertise(ORB_ID(shp_output), &shp_info);

  
    float roll = -1;
    float pitch = -2;
    float yaw = -3;

    orb_publish(ORB_ID(shp_output), shp_pub_fd, &shp_info);
  

    float q[4] = {0};
    
    int i = 0;
   
    while(i < 10e6)
    {
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

        orb_publish(ORB_ID(shp_output), shp_pub_fd, &shp_info);

        i++;
    }

   //roll = atan2(2*(att.q[0] * att.q[1] + att.q[2] * att.q[3],1-2*(att.q[1] * att.q[1] + att.q[2] * att.q[2]);
  //  double omfg = yaw * 2;
   // omfg = omfg;
    //double roll = atan2f(2*)

    PX4_INFO("z position: %10f\t: ", (double)pos.z);

     /* PX4_INFO("Accelerometer:%10f\t%10f\t%10f",
                     (double)raw.accelerometer_m_s2[0],
                     (double)raw.accelerometer_m_s2[1],
                     (double)raw.accelerometer_m_s2[2]);
    */
           
    return OK;
}