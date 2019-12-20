#pragma once

#include <sm_dance_bot/client_behaviors/lidar_sensor/lidar_client.h>
#include <smacc/smacc_orthogonal.h>

namespace sm_dance_bot
{
class OrObstaclePerception : public smacc::Orthogonal
{
public:
    virtual void onInitialize() override
    {
        auto lidarClient = this->createClient<OrObstaclePerception, sm_dance_bot::LaserSensor>();

        lidarClient->topicName = "/front/scan";
        //lidarClient->queueSize = "/front/scan";
        lidarClient->timeout_ = ros::Duration(10);

        lidarClient->initialize();
    }
};
}