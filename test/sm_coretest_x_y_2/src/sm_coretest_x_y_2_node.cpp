#include <sm_coretest_x_y_2/sm_coretest_x_y_2.h>

//--------------------------------------------------------------------
int main(int argc, char **argv)
{
    ros::init(argc, argv, "sm_coretest_x_y_2");
    ros::NodeHandle nh;

    smacc::run<sm_coretest_x_y_2::SmCoreTestXY2>();
}
