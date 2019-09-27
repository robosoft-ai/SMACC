#pragma once

#include <smacc/smacc.h>
#include <smacc/impl/smacc_state_machine_base_impl.h>

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

