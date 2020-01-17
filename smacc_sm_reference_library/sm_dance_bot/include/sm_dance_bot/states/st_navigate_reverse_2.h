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

   static void onDefinition()
   {
      static_configure<OrNavigation, CbNavigateBackwards>(2);
      static_configure<OrLED, CbLEDOff>();
      static_configure<OrObstaclePerception, CbLidarSensor>();
   }

   void onInitialize()
   {
   }
};
}