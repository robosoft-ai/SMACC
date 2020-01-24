#include <smacc/smacc.h>
namespace sm_dance_bot_3
{
struct StNavigateReverse3 : smacc::SmaccState<StNavigateReverse3, MsDanceBotRunMode>
{
   using SmaccState::SmaccState;

   typedef mpl::list<
       // Expected event
       Transition<EvActionSucceeded<ClMoveBaseZ, OrNavigation>, StNavigateToWaypoint1>,

       // Error events
       //Transition<smacc::EvTopicMessageTimeout<CbLidarSensor>, StAcquireSensors>,
       Transition<EvActionAborted<ClMoveBaseZ, OrNavigation>, StNavigateToWaypointsX>>
       reactions;

   static void staticConfigure()
   {
      configure_orthogonal<OrNavigation, CbNavigateBackwards>(2);
      configure_orthogonal<OrLED, CbLEDOff>();
      configure_orthogonal<OrObstaclePerception, CbLidarSensor>();
   }

   void runtimeConfiguration()
   {
   }
};
}