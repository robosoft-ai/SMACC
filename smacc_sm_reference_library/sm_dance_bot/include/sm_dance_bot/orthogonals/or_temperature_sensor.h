#pragma once

#include <smacc/smacc_orthogonal.h>
#include <sensor_msgs/Temperature.h>
#include <smacc_interface_components/clients/sensor_client.h>
#include <sm_dance_bot/clients/cl_temperature_sensor.h>

namespace sm_dance_bot
{
class OrClTemperatureSensor : public smacc::Orthogonal
{
public:
    virtual void onInitialize() override
    {
        auto ClTemperatureSensor = this->createClient<OrClTemperatureSensor, sm_dance_bot::ClTemperatureSensor>();

        ClTemperatureSensor->topicName = "/temperature";
        //ClTemperatureSensor->queueSize = "/front/scan";
        ClTemperatureSensor->timeout_ = ros::Duration(10);

        ClTemperatureSensor->initialize();
    }
};
} // namespace sm_dance_bot