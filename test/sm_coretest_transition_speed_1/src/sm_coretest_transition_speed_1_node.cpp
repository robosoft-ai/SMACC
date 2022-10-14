#include <sm_coretest_transition_speed_1/sm_coretest_transition_speed_1.h>

//--------------------------------------------------------------------
int main(int argc, char **argv)
{
    ros::init(argc, argv, "sm_coretest_transition_speed_1");
    ros::NodeHandle nh;

    smacc::run<sm_coretest_transition_speed_1::SmCoreTestTransistionSpeed1>();
}
