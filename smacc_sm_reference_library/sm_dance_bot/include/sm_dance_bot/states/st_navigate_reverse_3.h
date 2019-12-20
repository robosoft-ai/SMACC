#include <smacc/smacc.h>
namespace sm_dance_bot
{
struct StNavigateReverse3 : smacc::SmaccState<StNavigateReverse3, MsDanceBotRunMode>
{
   using SmaccState::SmaccState;

   typedef mpl::list<
       // Expected event
       smacc::transition<EvActionSucceeded<smacc::SmaccMoveBaseActionClient, NavigationOrthogonal>, StNavigateToWaypoint1>,

       // Error events
       //smacc::transition<smacc::EvTopicMessageTimeout<CbLidarSensor>, StAcquireSensors>,
       smacc::transition<EvActionAborted<smacc::SmaccMoveBaseActionClient, NavigationOrthogonal>, StNavigateToWaypointsX>>
       reactions;

   static void onDefinition()
   {
      static_configure<NavigationOrthogonal, CbNavigateBackwards>(2);
      static_configure<ToolOrthogonal, CbToolStop>();
      static_configure<ObstaclePerceptionOrthogonal, CbLidarSensor>();
   }

   void onInitialize()
   {
   }
};
}