#pragma once

#include <sm_dance_bot/substate_behaviors/lidar_sensor/lidar_client.h>
#include <smacc/orthogonal.h>

class ObstaclePerceptionOrthogonal : public smacc::Orthogonal
{
public:
    virtual void onInitialize() override
    {
        auto *lidarClient = this->createClient<dance_bot::LaserSensor>();

        lidarClient->topicName = "/front/scan";
        //lidarClient->queueSize = "/front/scan";
        lidarClient->timeout_ = ros::Duration(10);

        lidarClient->initialize();
    }
};