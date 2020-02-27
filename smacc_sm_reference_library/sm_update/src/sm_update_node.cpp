#include <sm_update/sm_update.h>

//--------------------------------------------------------------------
int main(int argc, char **argv)
{
    ros::init(argc, argv, "sm_update");
    ros::NodeHandle nh;

    smacc::run<sm_update::SmUpdate>();
}