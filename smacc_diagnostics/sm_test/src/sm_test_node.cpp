#include <sm_test/sm_test.h>

//--------------------------------------------------------------------
int main(int argc, char **argv)
{
    ros::init(argc, argv, "sm_test");
    ros::NodeHandle nh;

    smacc::run<sm_test::SmTest>();
}