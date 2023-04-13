#include <sm_atomic_mode_states/sm_atomic_mode_states.h>

//--------------------------------------------------------------------
int main(int argc, char **argv)
{
    ros::init(argc, argv, "sm_atomic_mode_states");
    ros::NodeHandle nh;

    smacc::run<sm_atomic_mode_states::SmAtomic>();
}
