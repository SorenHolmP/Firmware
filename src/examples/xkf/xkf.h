#include <px4_module.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/shp_output.h>

#include <px4_log.h>
#include <px4_config.h>
#include <px4_tasks.h>
#include <px4_posix.h>
#include <unistd.h>
#include <stdio.h>
#include <poll.h>
#include <string.h>

#include <math.h>
#include <drivers/drv_hrt.h> //hrt_absolute_time()



class xkf  : public ModuleBase<xkf>
{
    public:
    xkf();
    void initialize();
    void begin();

    
    int local_position_fd;
    int attitude_fd;

    struct vehicle_local_position_s pos;
    struct vehicle_attitude_s att;
    struct shp_output_s shp_info;

    orb_advert_t shp_pub_fd; 

    private:

    //px4_pollfd_struct_t fds[1];

};