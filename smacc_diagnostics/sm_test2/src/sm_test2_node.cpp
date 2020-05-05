#include <sm_test2/sm_test2.h>

//--------------------------------------------------------------------
int main(int argc, char **argv)
{
    ros::init(argc, argv, "sm_test2");
    ros::NodeHandle nh;

    smacc::run<sm_test2::SmTest2>();
}