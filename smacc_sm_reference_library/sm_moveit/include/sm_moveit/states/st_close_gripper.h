#pragma once

#include <smacc/smacc.h>
#include <sm_moveit/clients/gripper_client/cl_gripper.h>

namespace sm_moveit
{
// STATE DECLARATION
struct StCloseGripper : smacc::SmaccState<StCloseGripper, SmMoveIt>
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