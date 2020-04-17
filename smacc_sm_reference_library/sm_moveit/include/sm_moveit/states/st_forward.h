#include <smacc/smacc.h>
namespace sm_moveit
{
// STATE DECLARATION
struct StForwardNextTable : smacc::SmaccState<StForwardNextTable, SmMoveIt>
{
    using SmaccState::SmaccState;

    // TRANSITION TABLE
    typedef mpl::list<

        // Transition<EvActionSucceeded<ClMoveBaseZ, OrNavigation>, StRotate180, SUCCESS>
        Transition<EvActionSucceeded<ClMoveBaseZ, OrNavigation>, SS2::SsPlaceObject, SUCCESS>,
        Transition<EvActionAborted<ClMoveBaseZ, OrNavigation>, StUndoIncorrectForward, ABORT>
        >
        reactions;

    // STATE FUNCTIONS
    static void staticConfigure()
    {
        //configure_orthogonal<OrNavigation, CbNavigateGlobalPosition>(-1, 0, M_PI);
        configure_orthogonal<OrNavigation, CbNavigateForward>();
    }

    void runtimeConfigure()
    {          
        ClMoveBaseZ *moveBaseClient_;
        this->requiresClient(moveBaseClient_);

        auto pose = moveBaseClient_->getComponent<Pose>();
        auto currentPose = pose->toPoseMsg();

        auto offset = 0.2;
        double dist;
        if (currentPose.position.x < -0.2) // robot is at negative x-axis side
        {
                // target is at positive side, at point x=0
                dist = fabs(0 - currentPose.position.x) + offset;
        }
        else // the robot in at positive x-axis side
        {
                // target is at negative x-axis side at point x = -1
                dist = fabs(-1. - currentPose.position.x) + offset;
             
        }

        auto forwardBh = this->getOrthogonal<OrNavigation>()->getClientBehavior<CbNavigateForward>();
        forwardBh->forwardDistance = dist;
    }
};
} // namespace sm_moveit