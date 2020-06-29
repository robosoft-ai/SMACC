#include <smacc/smacc.h>
namespace sm_moveit
{
// STATE DECLARATION
struct StRotate180 : smacc::SmaccState<StRotate180, SmMoveIt>
{
    using SmaccState::SmaccState;

    // TRANSITION TABLE
    typedef mpl::list<

        Transition<EvActionSucceeded<ClMoveBaseZ, OrNavigation>, StForwardNextTable, SUCCESS>,
        Transition<EvActionAborted<ClMoveBaseZ, OrNavigation>, StRotate180, ABORT> /*retry*/
        >
        reactions;

    // STATE FUNCTIONS
    static void staticConfigure()
    {
        configure_orthogonal<OrNavigation, CbAbsoluteRotate>(180/*-M_PI*/);
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
        absoluteRotateBehavior->spinningPlanner = CbAbsoluteRotate::SpiningPlanner::PureSpinning;
    }
};
} // namespace sm_moveit