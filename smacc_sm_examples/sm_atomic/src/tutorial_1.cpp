#include <sm_atomic/sm_atomic.h>

//--------------------------------------------------------------------
int main(int argc, char **argv)
{
    ros::init(argc, argv, "tutorial");
    ros::NodeHandle nh;

    smacc::run<sm_atomic::BaseStateMachine>();
}