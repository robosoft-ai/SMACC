#pragma once

#include <smacc_interface_components/clients/sensor_client.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Temperature.h>

namespace sm_dancebot
{

class TemperatureSensor : public smacc::SensorClient<TemperatureSensor, sensor_msgs::Temperature>
{
public:
    TemperatureSensor()
    {
    }
};
} // namespace sm_dancebot