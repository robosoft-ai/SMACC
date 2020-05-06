#include <sm_coretest_x_y_3/sm_coretest_x_y_3.h>

//--------------------------------------------------------------------
int main(int argc, char **argv)
{
    ros::init(argc, argv, "sm_coretest_x_y_3");
    ros::NodeHandle nh;

    smacc::run<sm_coretest_x_y_3::SmCoreTestXY3>();
}