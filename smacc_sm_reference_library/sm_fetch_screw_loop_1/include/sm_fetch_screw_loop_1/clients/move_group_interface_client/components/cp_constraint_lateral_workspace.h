#pragma once

#include <smacc/component.h>
#include <move_group_interface_client/cl_movegroup.h>
#include <geometry_msgs/Vector3.h>
#include <ros/time.h>

namespace sm_fetch_screw_loop_1
{
    namespace cl_move_group_interface
    {
        // Adds two simetric collision virtual walls for the moveit planning
        class CpConstraintLateralWorkspace : public smacc::ISmaccComponent, public smacc::ISmaccUpdatable
        {
        private:
            moveit::planning_interface::PlanningSceneInterface *planningSceneInterface_;
            ::cl_move_group_interface::ClMoveGroup *movegroupclient_;

            std::string referenceFrame_;
            float lateralDistance_;
            geometry_msgs::Vector3 offset_;
            geometry_msgs::Vector3 size_;
            bool enabled_ = true;
            bool alreadyRemoved_ = true;

        public:
            void enable();

            void disable();

            CpConstraintLateralWorkspace(std::string referenceFrame, float lateralDistance, geometry_msgs::Vector3 size, geometry_msgs::Vector3 offset = geometry_msgs::Vector3());

            virtual void onInitialize() override;

            virtual void update();

            void createCollisionBox(float x, float y, float z, float xl, float yl, float zl, std::string id, std::string frameid, moveit_msgs::CollisionObject &collision, const ros::Time &time, int addOrRemove);

            void createVirtualCollisionWalls(std::vector<moveit_msgs::CollisionObject> &collisions, const ros::Time &time, int addOrRemove);
        };

    } // namespace cl_move_group_interface

} // namespace sm_fetch_screw_loop_1