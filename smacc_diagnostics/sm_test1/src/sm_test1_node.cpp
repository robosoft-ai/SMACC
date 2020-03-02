#include <sm_test1/sm_test1.h>

//--------------------------------------------------------------------
int main(int argc, char **argv)
{
    ros::init(argc, argv, "sm_test1");
    ros::NodeHandle nh;

    smacc::run<sm_test1::SmTest1>();
}