#include <keyboard_state_machine.h>

int main(int argc, char **argv) 
{
  ros::init(argc, argv, "keyboard_state_machine");
  smacc::run<KeyboardStateMachine>();
}