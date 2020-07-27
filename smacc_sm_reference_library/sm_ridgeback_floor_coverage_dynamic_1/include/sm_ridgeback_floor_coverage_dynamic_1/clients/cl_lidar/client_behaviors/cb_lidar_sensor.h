#pragma once

#include <multirole_sensor_client/client_behaviors/cb_default_multirole_sensor_behavior.h>
#include <sensor_msgs/LaserScan.h>
#include <sm_ridgeback_floor_coverage_dynamic_1/clients/cl_lidar/cl_lidar.h>

namespace sm_ridgeback_floor_coverage_dynamic_1
{
  namespace cl_lidar
  {
    struct CbLidarSensor : cl_multirole_sensor::CbDefaultMultiRoleSensorBehavior<sm_ridgeback_floor_coverage_dynamic_1::cl_lidar::ClLidarSensor>
    {
    public:
      CbLidarSensor()
      {
        ROS_INFO("CbLidarSensor Constructor");
      }

      virtual void onEntry() override
      {
        ROS_INFO("CbLidarSensor onEntry");
        cl_multirole_sensor::CbDefaultMultiRoleSensorBehavior<ClLidarSensor>::onEntry();
      }

      virtual void onMessageCallback(const sensor_msgs::LaserScan &msg) override
      {
      }
    };
  } // namespace cl_lidar
} // namespace sm_ridgeback_floor_coverage_dynamic_1
