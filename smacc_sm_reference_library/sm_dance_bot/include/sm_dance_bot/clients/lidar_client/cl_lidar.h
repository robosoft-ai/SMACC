#pragma once

#include <smacc_interface_components/clients/sensor_client.h>
#include <std_msgs/String.h>
#include <sensor_msgs/LaserScan.h>

namespace sm_dance_bot
{
namespace lidar_client
{

class ClLaserSensor : public smacc::SensorClient<sensor_msgs::LaserScan>
{
public:
    ClLaserSensor()
    {
    }
};
} // namespace lidar_client
} // namespace sm_dance_bot