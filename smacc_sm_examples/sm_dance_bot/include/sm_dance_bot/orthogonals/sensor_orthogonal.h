#pragma once

#include <smacc/orthogonal.h>
#include <sensor_msgs/Temperature.h>
#include <smacc_interface_components/clients/sensor_client.h>

class SensorOrthogonal : public smacc::Orthogonal
{
public:
    virtual void onInitialize() override
    {
        auto* temperatureSensor = this->createClient<smacc::SensorClient<sensor_msgs::Temperature>>();
        
        temperatureSensor->topicName = "/temperature";
        //temperatureSensor->queueSize = "/front/scan";
        temperatureSensor->timeout_ =  ros::Duration(10);

        temperatureSensor->initialize();
    }
};