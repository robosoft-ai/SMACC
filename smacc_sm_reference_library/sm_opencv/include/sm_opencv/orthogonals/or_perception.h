#pragma once

#include <smacc/smacc_orthogonal.h>

namespace sm_opencv
{
class OrPerception : public smacc::Orthogonal<OrPerception>
{
public:
    virtual void onInitialize() override
    {
        // auto lidarClient =
        //     this->createClient<ClLidarSensor>("/front/scan", ros::Duration(10));

        // lidarClient->initialize();

        // lidarClient->createComponent<CpLidarSensorData>();
    }
};
}