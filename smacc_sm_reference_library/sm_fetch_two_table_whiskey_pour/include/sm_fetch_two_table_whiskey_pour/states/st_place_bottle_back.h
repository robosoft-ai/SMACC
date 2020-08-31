#pragma once

#include <smacc/smacc.h>
namespace sm_fetch_two_table_whiskey_pour
{
    // STATE DECLARATION
    struct StPlaceBottleBack : smacc::SmaccState<StPlaceBottleBack, SmFetchTwoTableWhiskeyPour>
    {
        using SmaccState::SmaccState;

        // TRANSITION TABLE
        typedef mpl::list<
            Transition<EvCbSuccess<CbUndoLastTrajectory, OrArm>, StReleaseGripper, SUCCESS>

            /*POSIBLE ERRORS*/
            //Transition<EvIncorrectInitialPosition<CbCircularPouringMotion, OrArm>, SS3::SsRecoveryScrew, ABORT>,
            //Transition<EvJointDiscontinuity<CbCircularPouringMotion, OrArm>, SS3::SsRecoveryScrew, ABORT> /*Retry*/,
            //Transition<EvMoveGroupMotionExecutionFailed<CbCircularPouringMotion, OrArm>, StInitialPosture, ABORT> /*retry motion failure*/
            >
            reactions;

        // STATE FUNCTIONS
        static void staticConfigure()
        {
            configure_orthogonal<OrArm, CbUndoLastTrajectory>(1);
        }

        void runtimeConfigure()
        {
            /*for the case of abor/retry cartesian retreat --*/
            /*ClMoveGroup *moveGroup;
            this->requiresClient(moveGroup);
            moveGroup->getComponent<CpConstraintTableWorkspaces>()->disableTableCollisionVolume();
            ros::Duration(1).sleep();

            auto moveCartesianRelative = this->getOrthogonal<OrArm>()
                                             ->getClientBehavior<CbMoveCartesianRelative>();

            moveCartesianRelative->offset_.z = -0.15;*/
        }

        void onExit()
        {
            ros::Duration(1.0).sleep();
        }
    };
} // namespace sm_fetch_two_table_whiskey_pour