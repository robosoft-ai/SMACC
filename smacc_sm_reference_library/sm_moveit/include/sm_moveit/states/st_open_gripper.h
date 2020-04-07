#pragma once
#include <smacc/smacc.h>
namespace sm_moveit
{
// STATE DECLARATION
struct StOpenGripper : smacc::SmaccState<StOpenGripper, SmMoveIt>
{
   using SmaccState::SmaccState;

   // TRANSITION TABLE
   typedef mpl::list<

       //    Transition<EvActionSucceeded<ClMoveBaseZ, OrNavigation>, StRotateDegrees3>,
       //    Transition<EvActionAborted<ClMoveBaseZ, OrNavigation>, StNavigateToWaypointsX>

       >
       reactions;

   // STATE FUNCTIONS
   static void staticConfigure()
   {
      //   configure_orthogonal<OrNavigation, CbNavigateBackwards>(2);
      //   configure_orthogonal<OrLED, CbLEDOff>();
      //   configure_orthogonal<OrObstaclePerception, CbLidarSensor>();
   }
};
} // namespace sm_moveit