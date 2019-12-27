#pragma once

#include <smacc_interface_components/clients/sensor_client.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Temperature.h>

namespace sm_dance_bot
{
namespace temperature_sensor_client
{
class ClTemperatureSensor : public smacc::SensorClient<sensor_msgs::Temperature>
{
public:
    ClTemperatureSensor()
    {
    }
};
} // namespace temperature_sensor_client
} // namespace sm_dance_bot