#include <sm_test3/sm_test3.h>

//--------------------------------------------------------------------
int main(int argc, char **argv)
{
    ros::init(argc, argv, "sm_test3");
    ros::NodeHandle nh;

    smacc::run<sm_test3::SmTest3>();
}