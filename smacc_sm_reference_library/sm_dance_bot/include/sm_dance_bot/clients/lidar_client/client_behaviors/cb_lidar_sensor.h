#pragma once

#include <smacc_interface_components/client_behaviors/cb_sensor_base.h>
#include <sensor_msgs/LaserScan.h>
#include <sm_dance_bot/clients/lidar_client/cl_lidar.h>

namespace sm_dance_bot
{
namespace lidar_client
{
struct CbLidarSensor : smacc::SensorTopic<ClLaserSensor>
{
public:
  CbLidarSensor()
  {
    ROS_INFO("CbLidarSensor Constructor");
  }

  virtual void onEntry() override
  {
    ROS_INFO("CbLidarSensor onEntry");
    smacc::SensorTopic<ClLaserSensor>::onEntry();
  }

  virtual void onMessageCallback(const sensor_msgs::LaserScan &msg) override
  {
  }
};
} // namespace lidar_client
} // namespace sm_dance_bot
