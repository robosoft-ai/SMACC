#pragma once

#include <smacc_core/smacc.h>

class SensorState;

struct SensorStateMachine
    : public smacc::SmaccStateMachineBase<SensorStateMachine,SensorState> 
{
    SensorStateMachine(my_context ctx, smacc::SignalDetector *signalDetector)
      : SmaccStateMachineBase<SensorStateMachine,SensorState>(ctx, signalDetector) 
      {
      }      

};

#include <states/sensor_state.h>

