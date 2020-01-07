#pragma once

#include <multirole_sensor_client/client_behaviors/cb_default_multirole_sensor_behavior.h>
#include <sensor_msgs/LaserScan.h>
#include <sm_dance_bot/clients/cl_lidar/cl_lidar.h>

namespace sm_dance_bot_2
{
namespace cl_lidar
{
struct CbLidarSensor : multirole_sensor_client::SmaccSubscriberClient<ClLaserSensor>
{
public:
  CbLidarSensor()
  {
    ROS_INFO("CbLidarSensor Constructor");
  }

  virtual void onEntry() override
  {
    ROS_INFO("CbLidarSensor onEntry");
    multirole_sensor_client::SmaccSubscriberClient<ClLaserSensor>::onEntry();
  }

  virtual void onMessageCallback(const sensor_msgs::LaserScan &msg) override
  {
  }
};
} // namespace cl_lidar
} // namespace sm_dance_bot
