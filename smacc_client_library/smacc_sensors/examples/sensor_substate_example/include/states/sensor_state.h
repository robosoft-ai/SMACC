#pragma once

#include <sensor_state_machine.h>
#include <orthogonals/sensor_orthogonal.h>
#include <orthogonals/obstacle_perception_orthogonal.h>

#include <sensor_msgs/LaserScan.h>
#include <substates_behaviors/sensor/custom_condition_temperature_sensor.h>
#include <smacc_interface_components/substate_behaviors/sensor_substate.h>

using namespace smacc;

class SbLidarSensor: public SensorTopic<SbLidarSensor, sensor_msgs::LaserScan>
{

};

struct SensorState: smacc::SmaccState<SensorState, SensorStateMachine>
{
  public:
  typedef mpl::list<
                smacc::transition<EvTopicInitialMessage<SbLidarSensor>, SensorState>, 
                smacc::transition<EvTopicMessage<SbLidarSensor>, SensorState>, 
                smacc::transition<EvTopicMessageTimeout<SbLidarSensor>, SensorState>,
                
                smacc::transition<EvTopicInitialMessage<CustomConditionTemperatureSensor>, SensorState>, 
                smacc::transition<EvTopicMessage<CustomConditionTemperatureSensor>, SensorState>, 
                smacc::transition<EvTopicMessageTimeout<CustomConditionTemperatureSensor>, SensorState>
                
                //smacc::transition<EvCustomTemperatureAlert<SensorState>, SensorState>
                > reactions; 


  using SmaccState::SmaccState;

  void onInitialize()
  {
    this->configure<ObstaclePerceptionOrthogonal>(std::make_shared<SbLidarSensor>());
    this->configure<SensorOrthogonal>(std::make_shared<CustomConditionTemperatureSensor>());
  }

  void onEntry()
  {
    ROS_INFO("sensor state onEntry" );
  }

  void onExit()
  {
  }
};
