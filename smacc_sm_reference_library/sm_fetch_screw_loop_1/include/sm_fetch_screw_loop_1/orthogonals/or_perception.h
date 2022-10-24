#pragma once

#include <smacc/smacc_orthogonal.h>
#include <sm_fetch_screw_loop_1/clients/perception_system_client/cl_perception_system.h>

using namespace sm_fetch_screw_loop_1::cl_perception_system;

namespace sm_fetch_screw_loop_1
{
class OrPerception : public smacc::Orthogonal<OrPerception>
{

public:
    virtual void onInitialize() override
    {
        ROS_INFO("Or perception initialization");
        auto perceptionClient = this->createClient<ClPerceptionSystem>();
        perceptionClient->initialize();
    }
};
} // namespace sm_fetch_screw_loop_1
