#pragma once

#include <smacc/orthogonal.h>
#include <sensor_msgs/LaserScan.h>
#include <smacc_interface_components/clients/sensor_client.h>

class ObstaclePerceptionOrthogonal : public smacc::Orthogonal
{
public:
    virtual void onInitialize() override
    {
        auto* lidarClient = this->createClient<smacc::SensorClient<sensor_msgs::LaserScan>>();
        
        lidarClient->topicName = "/front/scan";
        //lidarClient->queueSize = "/front/scan";
        lidarClient->timeout_ =  ros::Duration(10);

        lidarClient->initialize();
    }
};