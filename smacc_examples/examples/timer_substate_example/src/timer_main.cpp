#include <timer_state_machine.h>

int main(int argc, char **argv) 
{
  ros::init(argc, argv, "timer_state_machine");
  smacc::run<TimerStateMachine>();
}