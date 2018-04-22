#include <px4_log.h>

extern "C" __EXPORT int shp_xkf_main(int argc, char *argv[]); //Et eller andet med C vs cpp filer

int shp_xkf_main(int argc, char *argv[])
{
    PX4_INFO("Hello Sky!");
    return OK;
}

