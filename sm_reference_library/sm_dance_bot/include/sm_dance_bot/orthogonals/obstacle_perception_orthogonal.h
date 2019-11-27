#pragma once

#include <sm_dance_bot/substate_behaviors/lidar_sensor/lidar_client.h>
#include <smacc/smacc_orthogonal.h>

namespace sm_dancebot
{
class ObstaclePerceptionOrthogonal : public smacc::Orthogonal
{
public:
    virtual void onInitialize() override
    {
        auto *lidarClient = this->createClient<sm_dancebot::LaserSensor>();

        lidarClient->topicName = "/front/scan";
        //lidarClient->queueSize = "/front/scan";
        lidarClient->timeout_ = ros::Duration(10);

        lidarClient->initialize();
    }
};
}