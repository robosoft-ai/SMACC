#pragma once

#include <sensor_state_machine.h>
#include <orthogonals/sensor_orthogonal.h>
#include <orthogonals/obstacle_perception_orthogonal.h>

#include <sensor_msgs/LaserScan.h>
#include <substates_behaviors/sensor/custom_condition_temperature_sensor.h>
#include <smacc_interface_components/substate_behaviors/sensor_substate.h>

using namespace smacc;

typedef smacc::SensorTopic<sensor_msgs::LaserScan> LidarSensor ;

struct SensorState: smacc::SmaccState<SensorState, SensorStateMachine>
{
  public:
  typedef mpl::list<
                sc::transition<EvTopicInitialMessage<LidarSensor>, SensorState>, 
                sc::transition<EvTopicMessage<LidarSensor>, SensorState>, 
                sc::transition<EvTopicMessageTimeout<LidarSensor>, SensorState>,
                
                sc::transition<EvTopicInitialMessage<CustomConditionTemperatureSensor>, SensorState>, 
                sc::transition<EvTopicMessage<CustomConditionTemperatureSensor>, SensorState>, 
                sc::transition<EvTopicMessageTimeout<CustomConditionTemperatureSensor>, SensorState>,
                
                sc::transition<EvCustomTemperatureAlert, SensorState>> reactions; 


  using SmaccState::SmaccState;

  void onInitialize()
  {
    this->configure<ObstaclePerceptionOrthogonal>(std::make_shared<smacc::SensorTopic<sensor_msgs::LaserScan>>("/scan", 1, ros::Duration(10)));
    this->configure<SensorOrthogonal>(std::make_shared<CustomConditionTemperatureSensor>("/temperature"));
  }

  void onEntry()
  {
    ROS_INFO("sensor state onEntry" );
  }

  void onExit()
  {
  }
};
