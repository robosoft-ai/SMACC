#include <sm_coretest_x_y_1/sm_coretest_x_y_1.h>

//--------------------------------------------------------------------
int main(int argc, char **argv)
{
    ros::init(argc, argv, "sm_coretest_x_y_1");
    ros::NodeHandle nh;

    smacc::run<sm_coretest_x_y_1::SmCoreTestXY1>();
}
