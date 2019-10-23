#pragma once

#include <smacc_interface_components/clients/sensor_client.h>
#include <std_msgs/String.h>
#include <sensor_msgs/LaserScan.h>

namespace dance_bot
{
    
class LaserSensor: public smacc::SensorClient<LaserSensor, sensor_msgs::LaserScan>
{
    public:
    LaserSensor()
    {
        
    }
};
}