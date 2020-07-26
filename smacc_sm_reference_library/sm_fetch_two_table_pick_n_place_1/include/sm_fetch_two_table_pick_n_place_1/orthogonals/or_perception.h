#pragma once

#include <smacc/smacc_orthogonal.h>
#include <sm_fetch_two_table_pick_n_place_1/clients/perception_system_client/cl_perception_system.h>

using namespace sm_fetch_two_table_pick_n_place_1::cl_perception_system;

namespace sm_fetch_two_table_pick_n_place_1
{
class OrPerception : public smacc::Orthogonal<OrPerception>
{
public:
    virtual void onInitialize() override
    {
        ROS_INFO("Or perception initialization");
        auto perceptionClient = this->createClient<ClPerceptionSystem>();

        perceptionClient->detectedCubePose0 = perceptionClient->createNamedComponent<cl_move_base_z::Pose>("cube_0", "cube_0", "map");
        perceptionClient->detectedCubePose1 = perceptionClient->createNamedComponent<cl_move_base_z::Pose>("cube_1", "cube_1", "map");
        perceptionClient->detectedCubePose2 = perceptionClient->createNamedComponent<cl_move_base_z::Pose>("cube_2", "cube_2", "map");
    }
};
} // namespace sm_fetch_two_table_pick_n_place_1