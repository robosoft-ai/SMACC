#pragma once

#include <smacc/smacc_orthogonal.h>
#include <sensor_msgs/Temperature.h>
#include <smacc_interface_components/clients/sensor_client.h>
#include <sm_dance_bot/substate_behaviors/temperature_sensor/temperature_sensor.h>

namespace sm_dance_bot
{
class SensorOrthogonal : public smacc::Orthogonal
{
public:
    virtual void onInitialize() override
    {
        auto temperatureSensor = this->createClient<SensorOrthogonal, sm_dance_bot::TemperatureSensor>();

        temperatureSensor->topicName = "/temperature";
        //temperatureSensor->queueSize = "/front/scan";
        temperatureSensor->timeout_ = ros::Duration(10);

        temperatureSensor->initialize();
    }
};
} // namespace sm_dance_bot