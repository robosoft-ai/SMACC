#pragma once

#include <multirole_sensor_client/cl_multirole_sensor.h>
#include <std_msgs/String.h>
#include <sensor_msgs/LaserScan.h>

namespace sm_ridgeback_floor_coverage_dynamic_1
{
namespace cl_lidar
{

class ClLidarSensor : public cl_multirole_sensor::ClMultiroleSensor<sensor_msgs::LaserScan>
{
public:
    ClLidarSensor(std::string topicname, ros::Duration timeout)
    {

        this->topicName = topicname;
        this->timeout_ = timeout;
    }
};
} // namespace cl_lidar
} // namespace sm_dance_bot_2