#pragma once

#include <smacc_interface_components/substate_behaviors/sensor_substate.h>
#include <sensor_msgs/LaserScan.h>
#include <sm_dance_bot/substate_behaviors/lidar_sensor/lidar_client.h>

namespace sm_dance_bot
{
struct SbLidarSensor : smacc::SensorTopic<LaserSensor>
{
public:
  SbLidarSensor()
  {
    ROS_INFO("SbLidarSensor Constructor");
  }

  virtual void onEntry() override
  {
    ROS_INFO("SbLidarSensor onEntry");
    smacc::SensorTopic<LaserSensor>::onEntry();
  }
  
  virtual void onMessageCallback(const sensor_msgs::LaserScan &msg) override
  {
  }
};
} // namespace sm_dance_bot
