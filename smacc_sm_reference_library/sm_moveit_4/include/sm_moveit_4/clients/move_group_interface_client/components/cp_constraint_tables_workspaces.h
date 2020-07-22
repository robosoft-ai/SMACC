#pragma once

#include <smacc/component.h>
#include <move_group_interface_client/cl_movegroup.h>
#include <geometry_msgs/Vector3.h>
#include <sm_moveit_4/clients/perception_system_client/cl_perception_system.h>
#include <sm_moveit_4/clients/perception_system_client/components/cp_scene_state.h>

namespace sm_moveit_4
{
    namespace move_group_interface_client
    {
        using namespace cl_perception_system;
        // Adds two simetric collision virtual walls for the moveit planning
        class CpConstraintTableWorkspaces : public smacc::ISmaccComponent, public smacc::ISmaccUpdatable
        {
        private:
            // required component
            moveit::planning_interface::PlanningSceneInterface *planningSceneInterface_;

            // required component
            CpSceneState *sceneState_;
            double safeTableHeightOffsetForCubeCollisions = 0;

        public:
            void setBigTableCollisionVolume();

            void setSmallTableCollisionVolume();

            void disableTableCollisionVolume();

            virtual void onInitialize() override;

            virtual void update();
        };

    } // namespace move_group_interface_client

} // namespace sm_moveit_4