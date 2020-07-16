#pragma once

#include <smacc/smacc_orthogonal.h>
#include <moveit_z_client/cl_movegroup.h>
#include <sm_moveit_4/clients/cl_moveit_z_client/components/cp_constraint_lateral_workspace.h>
#include <sm_moveit_4/clients/cl_moveit_z_client/components/cp_constraint_virtual_side_wall.h>
#include <sm_moveit_4/clients/cl_moveit_z_client/components/cp_constraint_tables_workspaces.h>

namespace sm_moveit_4
{
    using namespace moveit_z_client;

    class OrArm : public smacc::Orthogonal<OrArm>
    {
    public:
        virtual void onInitialize() override
        {
            auto moveGroupClient = this->createClient<ClMoveGroup>("arm_with_torso");
            moveGroupClient->initialize();

            // (Constraint workspace) create obstacles around table surfaces (optionally covering the cubes volume)
            moveGroupClient->createComponent<cl_moveit_z_client::CpConstraintTableWorkspaces>();

            // (Constraint workspace) create two simetrics virtual-collision side walls from the robot base reference frame
            {
                std::string referenceFrame = "base_link";
                double lateralDistance = 0.38;

                geometry_msgs::Vector3 size;
                size.x = 1.0;
                size.y = 0.2;
                size.z = 0.6;

                geometry_msgs::Vector3 offset;
                offset.z = 0.3;

                moveGroupClient->createComponent<cl_moveit_z_client::CpConstraintLateralWorkspace>(referenceFrame, lateralDistance, size, offset);
            }

            // (Constraint workspace) create one tall virtual-wall-collision at the left side (from the robot base reference frame)
            {
                std::string referenceFrame = "base_link";

                geometry_msgs::Vector3 size;
                size.x = 0.5;
                size.y = 0.1;
                size.z = 1.2;

                geometry_msgs::Vector3 offset;
                offset.x = -0.1;
                offset.y = 0.48;
                offset.z = 0.6;

                moveGroupClient->createComponent<cl_moveit_z_client::CpConstraintVirtualSideWall>(referenceFrame, size, offset);
            }
        }
    };
} // namespace sm_moveit_4