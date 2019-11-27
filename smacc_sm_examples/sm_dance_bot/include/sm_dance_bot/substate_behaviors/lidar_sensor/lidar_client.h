#pragma once

#include <smacc_interface_components/clients/sensor_client.h>
#include <std_msgs/String.h>
#include <sensor_msgs/LaserScan.h>

namespace sm_dancebot
{

class LaserSensor : public smacc::SensorClient<LaserSensor, sensor_msgs::LaserScan>
{
public:
    LaserSensor()
    {
    }
};
} // namespace sm_dancebot