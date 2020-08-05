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
            Transition<EvCbSuccess<CbEndEffectorRotate, OrArm>, StInitialPosture, SUCCESS>,

            /*POSIBLE ERRORS*/
            Transition<EvIncorrectInitialPosition<CbEndEffectorRotate, OrArm>, SS3::SsRecoveryScrew, ABORT>,
            Transition<EvJointDiscontinuity<CbEndEffectorRotate, OrArm>, StSecondPosture, ABORT> /*Retry*/, 
            Transition<EvMoveGroupMotionExecutionFailed<CbEndEffectorRotate, OrArm>, StSecondPosture, ABORT> /*retry motion failure*/
            >
            reactions;

        // STATE FUNCTIONS
        static void staticConfigure()
        {
            geometry_msgs::PoseStamped pivot;
            pivot.pose.position.x = 0.9;
            pivot.pose.position.y = 0;
            pivot.pose.position.z = 0.8;
            pivot.pose.orientation.w = 1;
            pivot.header.frame_id = "map";

            // configure_orthogonal<OrArm, CbCircularPivotMotion>(pivot, M_PI, "gripper_link");

            // geometry_msgs::Pose initialOffset;
            // initialOffset.position.y = 0.05;
            // initialOffset.orientation.w = 1;

            configure_orthogonal<OrArm, CbEndEffectorRotate>(-M_PI / 2, "gripper_link");
            configure_orthogonal<OrGripper, CbCloseGripper>();
        }

        void runtimeConfigure()
        {
            auto cbpivotMotion = this->getOrthogonal<OrArm>()->getClientBehavior<CbCircularPivotMotion>();
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