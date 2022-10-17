#include <smacc/smacc.h>
namespace sm_fetch_two_table_pick_n_place_1
{
  // STATE DECLARATION
  struct StForwardNextTable : smacc::SmaccState<StForwardNextTable, SmFetchTwoTablePickNPlace1>
  {
    using SmaccState::SmaccState;

    // TRANSITION TABLE
    typedef mpl::list<
        Transition<EvCbSuccess<CbNavigateGlobalPosition, OrNavigation>, SS2::SsPlaceObject, SUCCESS>,
        Transition<EvCbFailure<CbNavigateGlobalPosition, OrNavigation>, StUndoIncorrectForward, ABORT>>
        reactions;

    // STATE FUNCTIONS
    static void staticConfigure()
    {
      configure_orthogonal<OrNavigation, CbNavigateGlobalPosition>();
      //configure_orthogonal<OrNavigation, CbNavigateForward>();
    }

    void runtimeConfigure()
    {
      configureGlobalNavigation();
      //configureForwardBehavior();
    }

    void configureGlobalNavigation()
    {
      ClMoveBaseZ *moveBaseClient_;
      this->requiresClient(moveBaseClient_);

      auto pose = moveBaseClient_->getComponent<Pose>();
      auto currentPose = pose->toPoseMsg();

      geometry_msgs::Point goalPosition;
      goalPosition.z = 0;
      goalPosition.y = 0;
      double yaw = 0;
      double forwardOffset = 1.2;
      if (currentPose.position.x < -0.2) // robot is at negative x-axis side
      {
        // target is at positive side, at point x=0
        goalPosition.x = forwardOffset;
        yaw = 0;
      }
      else // the robot in at positive x-axis side
      {
        // target is at negative x-axis side at point x = -1
        goalPosition.x = -forwardOffset;
        yaw = -M_PI;
      }

      auto globalNavigationBh = this->getOrthogonal<OrNavigation>()->getClientBehavior<CbNavigateGlobalPosition>();
      globalNavigationBh->goalPosition = goalPosition;
      globalNavigationBh->goalYaw = yaw;
    }

    void configureForwardBehavior()
    {
      ClMoveBaseZ *moveBaseClient_;
      this->requiresClient(moveBaseClient_);

      auto pose = moveBaseClient_->getComponent<Pose>();
      auto currentPose = pose->toPoseMsg();

      auto offset = 0.15;
      auto forwardDistance = 2.0;
      double dist;

      dist = forwardDistance + offset;

      auto forwardBh = this->getOrthogonal<OrNavigation>()->getClientBehavior<CbNavigateForward>();
      forwardBh->forwardDistance = dist;
    }
  };
} // namespace sm_fetch_two_table_pick_n_place_1
