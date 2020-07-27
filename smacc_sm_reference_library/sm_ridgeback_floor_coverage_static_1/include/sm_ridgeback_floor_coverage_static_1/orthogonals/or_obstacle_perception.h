#pragma once

#include <sm_ridgeback_floor_coverage_static_1/clients/cl_lidar/cl_lidar.h>
#include <smacc/smacc_orthogonal.h>

namespace sm_ridgeback_floor_coverage_static_1
{
class OrObstaclePerception : public smacc::Orthogonal<OrObstaclePerception>
{
public:
    virtual void onInitialize() override
    {
        auto lidarClient = this->createClient<ClLidarSensor>();

        lidarClient->topicName = "/front/scan";
        //lidarClient->queueSize = "/front/scan";
        lidarClient->timeout_ = ros::Duration(10);

        lidarClient->initialize();
    }
};
}