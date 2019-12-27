#pragma once

#include <smacc/smacc_orthogonal.h>
#include <sensor_msgs/Temperature.h>
#include <smacc_interface_components/clients/sensor_client.h>
#include <sm_dance_bot/clients/temperature_sensor_client/cl_temperature_sensor.h>

namespace sm_dance_bot
{
class OrTemperatureSensor : public smacc::Orthogonal
{
public:
    virtual void onInitialize() override
    {
        auto clTemperatureSensor = this->createClient<OrTemperatureSensor, ClTemperatureSensor>();

        clTemperatureSensor->topicName = "/temperature";
        //ClTemperatureSensor->queueSize = "/front/scan";
        clTemperatureSensor->timeout_ = ros::Duration(10);

        clTemperatureSensor->initialize();
    }
};
} // namespace sm_dance_bot