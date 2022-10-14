#include <smacc/smacc.h>
namespace sm_fetch_two_table_pick_n_place_1
{
    // STATE DECLARATION
    struct StRotate180 : smacc::SmaccState<StRotate180, SmFetchTwoTablePickNPlace1>
    {
        using SmaccState::SmaccState;

        // TRANSITION TABLE
        typedef mpl::list<

            Transition<EvCbSuccess<CbAbsoluteRotate, OrNavigation>, StForwardNextTable, SUCCESS>,
            Transition<EvCbFailure<CbAbsoluteRotate, OrNavigation>, StRotate180, ABORT> /*retry*/
            >
            reactions;

        // STATE FUNCTIONS
        static void staticConfigure()
        {
            configure_orthogonal<OrNavigation, CbAbsoluteRotate>(180 /*-M_PI*/);
        }

        void runtimeConfigure()
        {
            ClMoveBaseZ *moveBaseClient_;
            this->requiresClient(moveBaseClient_);

            auto pose = moveBaseClient_->getComponent<Pose>();
            auto currentPose = pose->toPoseMsg();

            double targetAbsoluteAngleDegrees;
            if (currentPose.position.x < -0.2) // robot is at negative x-axis side
            {
                targetAbsoluteAngleDegrees = 0;
            }
            else // the robot in at positive x-axis side
            {
                targetAbsoluteAngleDegrees = 180;
            }

            auto absoluteRotateBehavior = this->getOrthogonal<OrNavigation>()->getClientBehavior<CbAbsoluteRotate>();
            absoluteRotateBehavior->absoluteGoalAngleDegree = targetAbsoluteAngleDegrees;
            absoluteRotateBehavior->spinningPlanner = SpiningPlanner::PureSpinning;
        }
    };
} // namespace sm_fetch_two_table_pick_n_place_1
