#include <sm_radial_motion/radial_motion.h>

int main(int argc, char **argv) 
{
  ros::init(argc, argv, "radial_test_state_machine");
  ros::NodeHandle nh;

  ros::Duration(5).sleep();
  smacc::run<RadialMotionStateMachine>();
}