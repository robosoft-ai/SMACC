#pragma once

#include <smacc/smacc.h>
namespace sm_fetch_two_table_whiskey_pour
{
    // STATE DECLARATION
    struct StPouringUndoPosture : smacc::SmaccState<StPouringUndoPosture, SmFetchTwoTableWhiskeyPour>
    {
        using SmaccState::SmaccState;

        // TRANSITION TABLE
        typedef mpl::list<
            Transition<EvCbSuccess<CbUndoLastTrajectory, OrArm>, StBringBottleBackBackwardNavigation, SUCCESS>

            /*POSIBLE ERRORS*/
            //Transition<EvIncorrectInitialPosition<CbCircularPouringMotion, OrArm>, SS3::SsRecoveryScrew, ABORT>,
            //Transition<EvJointDiscontinuity<CbCircularPouringMotion, OrArm>, SS3::SsRecoveryScrew, ABORT> /*Retry*/,
            //Transition<EvMoveGroupMotionExecutionFailed<CbCircularPouringMotion, OrArm>, StInitialPosture, ABORT> /*retry motion failure*/
        >
            reactions;

        // STATE FUNCTIONS
        static void staticConfigure()
        {
            configure_orthogonal<OrArm, CbUndoLastTrajectory>();
        }

        void runtimeConfigure()
        {
            /*
            auto cbpivotMotion = this->getOrthogonal<OrArm>()->getClientBehavior<CbCircularPouringMotion>();
            //cbpivotMotion->linearSpeed_m_s_ = 0.05; // m/s
            cbpivotMotion->linearSpeed_m_s_ = 0.1; // m/s

            cbpivotMotion->pointerRelativePose_.position.z = 0.1;
            cbpivotMotion->pointerRelativePose_.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, -M_PI*0.5, 0);
            //cbpivotMotion->pointerRelativePose_.orientation.w =1;

            //cbpivotMotion->allowInitialTrajectoryStateJointDiscontinuity_ = true;
            */
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