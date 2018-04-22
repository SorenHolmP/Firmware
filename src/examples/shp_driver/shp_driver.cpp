#include <px4_log.h>

extern "C" __EXPORT int shp_driver(int argc, char *argv[]); //Et eller andet med C vs cpp filer



int shp_driver_main(int argc, char *argv[])
{
    PX4_INFO("Hello Sky!");
    return OK;
}


