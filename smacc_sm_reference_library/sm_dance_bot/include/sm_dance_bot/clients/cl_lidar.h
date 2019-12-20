#pragma once

#include <smacc_interface_components/clients/sensor_client.h>
#include <std_msgs/String.h>
#include <sensor_msgs/LaserScan.h>

namespace sm_dance_bot
{

class ClLaserSensor : public smacc::SensorClient<sensor_msgs::LaserScan>
{
public:
    ClLaserSensor()
    {
    }
};
} // namespace sm_dance_bot