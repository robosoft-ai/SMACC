#pragma once

#include <smacc/smacc.h>
#include <smacc/impl/smacc_state_machine_base_impl.h>

class SensorOrthogonal;
class SensorState;
class ObstaclePerceptionOrthogonal;

struct SensorStateMachine
    : public smacc::SmaccStateMachineBase<SensorStateMachine,SensorState> 
{
    SensorStateMachine(my_context ctx, smacc::SignalDetector *signalDetector)
      : SmaccStateMachineBase<SensorStateMachine,SensorState>(ctx, signalDetector) 
      {
          this->createOrthogonal<SensorOrthogonal>();
          this->createOrthogonal<ObstaclePerceptionOrthogonal>();
      }      
};

#include <states/sensor_state.h>

