#pragma once

#include <smacc_interface_components/clients/sensor_client.h>
#include <std_msgs/String.h>
#include <sensor_msgs/LaserScan.h>

namespace sm_dance_bot
{
namespace cl_lidar
{

class ClLaserSensor : public smacc::SensorClient<sensor_msgs::LaserScan>
{
public:
    ClLaserSensor()
    {
    }
};
} // namespace cl_lidar
} // namespace sm_dance_bot