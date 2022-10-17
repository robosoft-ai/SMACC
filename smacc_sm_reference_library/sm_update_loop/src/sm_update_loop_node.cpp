#include <sm_update_loop/sm_update_loop.h>

//--------------------------------------------------------------------
int main(int argc, char **argv)
{
    ros::init(argc, argv, "sm_update_loop");
    ros::NodeHandle nh;

    smacc::run<sm_update_loop::SmUpdateLoop>();
}
