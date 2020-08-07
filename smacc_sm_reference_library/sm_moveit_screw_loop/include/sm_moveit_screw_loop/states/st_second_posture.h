#pragma once

#include <smacc/smacc.h>
namespace sm_moveit_screw_loop
{
    // STATE DECLARATION
    struct StSecondPosture : smacc::SmaccState<StSecondPosture, SmFetchSixTablePickNSort1>
    {
        using SmaccState::SmaccState;

        // TRANSITION TABLE
        typedef mpl::list<
            Transition<EvCbSuccess<CbCircularPouringMotion, OrArm>, StInitialPosture, SUCCESS>,

            /*POSIBLE ERRORS*/
            Transition<EvIncorrectInitialPosition<CbCircularPouringMotion, OrArm>, SS3::SsRecoveryScrew, ABORT>,
            Transition<EvJointDiscontinuity<CbCircularPouringMotion, OrArm>, StSecondPosture, ABORT> /*Retry*/, 
            Transition<EvMoveGroupMotionExecutionFailed<CbCircularPouringMotion, OrArm>, StSecondPosture, ABORT> /*retry motion failure*/
            >
            reactions;

        // STATE FUNCTIONS
        static void staticConfigure()
        {
            geometry_msgs::PointStamped pivot;
            pivot.header.frame_id = "map";
            pivot.header.stamp = ros::Time::now();
            pivot.point.x = 0.9;
            pivot.point.y = 0;
            pivot.point.z = 0.8;

            // configure_orthogonal<OrArm, CbCircularPivotMotion>(pivot, M_PI, "gripper_link");

            // geometry_msgs::Pose initialOffset;
            // initialOffset.position.y = 0.05;
            // initialOffset.orientation.w = 1;

            configure_orthogonal<OrArm, CbCircularPouringMotion>(pivot, -0.3, "gripper_link");
            configure_orthogonal<OrGripper, CbCloseGripper>();
        }

        void runtimeConfigure()
        {
            auto cbpivotMotion = this->getOrthogonal<OrArm>()->getClientBehavior<CbCircularPouringMotion>();
            //cbpivotMotion->linearSpeed_m_s_ = 0.05; // m/s
            cbpivotMotion->linearSpeed_m_s_ = 0.1; // m/s
            //cbpivotMotion->allowInitialTrajectoryStateJointDiscontinuity_ = true;
        }

        void onEntry()
        {
        }

        void onExit()
        {
            ros::Duration(1.0).sleep();
        }
    };
} // namespace sm_moveit_screw_loop