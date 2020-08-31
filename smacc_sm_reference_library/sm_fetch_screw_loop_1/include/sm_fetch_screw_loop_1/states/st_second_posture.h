#pragma once

#include <smacc/smacc.h>
namespace sm_fetch_screw_loop_1
{
    // STATE DECLARATION
    struct StSecondPosture : smacc::SmaccState<StSecondPosture, SmFetchScrewLoop1>
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
            geometry_msgs::Point pivot;
            pivot.x = 0.4;
            pivot.y = 0;
            pivot.z = 0.6;

            // configure_orthogonal<OrArm, CbCircularPivotMotion>(pivot, M_PI, "gripper_link");

            // geometry_msgs::Pose initialOffset;
            // initialOffset.position.y = 0.05;
            // initialOffset.orientation.w = 1;

            configure_orthogonal<OrArm, CbCircularPouringMotion>(pivot, -0.35, "gripper_link", "map");
            configure_orthogonal<OrGripper, CbCloseGripper>();
        }

        void runtimeConfigure()
        {
            auto cbpivotMotion = this->getOrthogonal<OrArm>()->getClientBehavior<CbCircularPouringMotion>();
            //cbpivotMotion->linearSpeed_m_s_ = 0.05; // m/s
            cbpivotMotion->linearSpeed_m_s_ = 0.1; // m/s

            cbpivotMotion->pointerRelativePose_.position.z = 0.1;
            cbpivotMotion->pointerRelativePose_.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, -M_PI * 0.5, 0);
        }

        void onEntry()
        {
        }

        void onExit()
        {
            ros::Duration(1.0).sleep();
        }
    };
} // namespace sm_fetch_screw_loop_1