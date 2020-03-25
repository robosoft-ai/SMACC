#include <ssm/SimpleStateMachine.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ssm");
    ros::NodeHandle nh;

    smacc::run<simple_state_machine::SimpleStateMachine>();
}