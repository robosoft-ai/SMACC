#pragma once

#include <multirole_sensor_client/client_behaviors/cb_default_multirole_sensor_behavior.h>
#include <sensor_msgs/LaserScan.h>
#include <sm_dance_bot_3/clients/cl_lidar/cl_lidar.h>

namespace sm_dance_bot_3
{
namespace cl_lidar
{
struct CbLidarSensor : multirole_sensor_client::CbDefaultMultiRoleSensorBehavior<sm_dance_bot_3::cl_lidar::ClLidarSensor>
{
public:
  CbLidarSensor()
  {
    ROS_INFO("CbLidarSensor Constructor");
  }

  virtual void onEntry() override
  {
    ROS_INFO("CbLidarSensor onEntry");
    multirole_sensor_client::CbDefaultMultiRoleSensorBehavior<ClLidarSensor>::onEntry();
  }

  virtual void onMessageCallback(const sensor_msgs::LaserScan &msg) override
  {
  }
};
} // namespace cl_lidar
} // namespace sm_dance_bot
