#pragma once

#include <keyboard_state_machine.h>
#include <orthogonals/or_input_device.h>
#include <client_behaviors/input_device/keyboard_substate.h>

//--------------------------------------------
struct KeyboardState: smacc::SmaccState<KeyboardState, KeyboardStateMachine>
{
  typedef smacc::transition<smacc::KeyPressEvent<'k'>, KeyboardState> reactions; 

public:
  using SmaccState::SmaccState;

  void onInitialize()
  {
    this->configure<OrInputDevice, smacc::Keyboard>();
  }

  void onEntry()
  {
    ROS_INFO("pressed key 'k'");
  }

  void onExit()
  {
  }
};
