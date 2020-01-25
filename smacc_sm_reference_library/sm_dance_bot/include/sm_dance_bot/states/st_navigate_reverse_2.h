#include <smacc/smacc.h>
namespace sm_dance_bot
{
struct StNavigateReverse2 : smacc::SmaccState<StNavigateReverse2, MsDanceBotRunMode>
{
   using SmaccState::SmaccState;

   typedef mpl::list<
       // Expected event
       Transition<EvActionSucceeded<ClMoveBaseZ, OrNavigation>, StNavigateToWaypointsX>,

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

   void runtimeConfigure()
   {
   }
};
}