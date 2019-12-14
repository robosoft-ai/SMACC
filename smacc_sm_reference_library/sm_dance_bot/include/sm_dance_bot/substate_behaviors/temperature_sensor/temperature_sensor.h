#pragma once

#include <smacc_interface_components/clients/sensor_client.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Temperature.h>

namespace sm_dance_bot
{

class TemperatureSensor : public smacc::SensorClient<sensor_msgs::Temperature>
{
public:
    TemperatureSensor()
    {
    }
};
} // namespace sm_dance_bot