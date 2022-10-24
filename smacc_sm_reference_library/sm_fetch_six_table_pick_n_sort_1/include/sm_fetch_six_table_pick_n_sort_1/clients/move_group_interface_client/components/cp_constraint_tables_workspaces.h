#pragma once

#include <smacc/component.h>
#include <move_group_interface_client/cl_movegroup.h>
#include <geometry_msgs/Vector3.h>
#include <sm_fetch_six_table_pick_n_sort_1/clients/perception_system_client/cl_perception_system.h>
#include <sm_fetch_six_table_pick_n_sort_1/clients/perception_system_client/components/cp_scene_state.h>

namespace sm_fetch_six_table_pick_n_sort_1
{
    namespace cl_move_group_interface
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

    } // namespace cl_move_group_interface

} // namespace sm_fetch_six_table_pick_n_sort_1
