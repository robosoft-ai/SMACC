#include <sensor_state_machine.h>

int main(int argc, char **argv) 
{
  ros::init(argc, argv, "sensor_state_machine");
  smacc::run<SensorStateMachine>();
}