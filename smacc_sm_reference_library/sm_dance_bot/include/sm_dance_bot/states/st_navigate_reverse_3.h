#include <smacc/smacc.h>
namespace sm_dance_bot
{
struct StNavigateReverse3 : smacc::SmaccState<StNavigateReverse3, MsDanceBotRunMode>
{
   using SmaccState::SmaccState;

   typedef mpl::list<
       // Expected event
       smacc::Transition<EvActionSucceeded<ClMoveBaseZ, OrNavigation>, StNavigateToWaypoint1>,

       // Error events
       //smacc::Transition<smacc::EvTopicMessageTimeout<CbLidarSensor>, StAcquireSensors>,
       smacc::Transition<EvActionAborted<ClMoveBaseZ, OrNavigation>, StNavigateToWaypointsX>>
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