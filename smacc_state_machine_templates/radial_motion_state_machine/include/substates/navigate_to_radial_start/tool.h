#pragma once

#include <radial_motion.h>
#include <thread>

namespace NavigateToRadialStart 
{
using namespace smacc;


struct ToolSubstateMiniState
    : SmaccState<ToolSubstateMiniState, ToolSubstate> 
{  
public:
  using SmaccState::SmaccState;

  void onEntry()
  {
  
  }
};

}