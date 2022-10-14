#pragma once

#include <smacc/smacc.h>
namespace sm_fetch_two_table_whiskey_pour
{
    // STATE DECLARATION
    struct StPouringPosture : smacc::SmaccState<StPouringPosture, SmFetchTwoTableWhiskeyPour>
    {
        using SmaccState::SmaccState;

        // TRANSITION TABLE
        typedef mpl::list<
            Transition<EvCbSuccess<CbCircularPouringMotion, OrArm>, StPouringUndoPosture, SUCCESS>

            /*POSSIBLE ERRORS*/
            //Transition<EvIncorrectInitialPosition<CbCircularPouringMotion, OrArm>, SS3::SsRecoveryScrew, ABORT>,
            //Transition<EvJointDiscontinuity<CbCircularPouringMotion, OrArm>, SS3::SsRecoveryScrew, ABORT> /*Retry*/,
            //Transition<EvMoveGroupMotionExecutionFailed<CbCircularPouringMotion, OrArm>, StInitialPosture, ABORT> /*retry motion failure*/
        >
            reactions;

        // STATE FUNCTIONS
        static void staticConfigure()
        {
            geometry_msgs::Point pivot;
            pivot.y = -0.08;
            pivot.x = 0.00;
            pivot.z = 0.0;

            // configure_orthogonal<OrArm, CbCircularPivotMotion>(pivot, M_PI, "gripper_link");

            // geometry_msgs::Pose initialOffset;
            // initialOffset.position.y = 0.05;
            // initialOffset.orientation.w = 1;

            configure_orthogonal<OrArm, CbCircularPouringMotion>(pivot, -0.24, "gripper_link", "map");
            //configure_orthogonal<OrGripper, CbCloseGripper>();
        }

        void runtimeConfigure()
        {
            auto cbpivotMotion = this->getOrthogonal<OrArm>()->getClientBehavior<CbCircularPouringMotion>();
            //cbpivotMotion->linearSpeed_m_s_ = 0.05; // m/s
            cbpivotMotion->linearSpeed_m_s_ = 0.05; // m/s

            cbpivotMotion->pointerRelativePose_.position.z = 0.2; /*distance from the grasping point to the lid of the bottle*/
            cbpivotMotion->pointerRelativePose_.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, -M_PI*0.5, 0);
            //cbpivotMotion->pointerRelativePose_.orientation.w =1;

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
} // namespace sm_fetch_two_table_whiskey_pour
