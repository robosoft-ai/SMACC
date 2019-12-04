#include <smacc/smacc.h>
namespace sm_dance_bot
{
struct StNavigateReverse3 : smacc::SmaccState<StNavigateReverse3, MsDanceBotRunMode>
{
   using SmaccState::SmaccState;

   typedef mpl::list<
       // Expected event
       smacc::transition<EvActionSucceeded<smacc::SmaccMoveBaseActionClient>, StNavigateToWaypoint1>,

       // Error events
       //smacc::transition<smacc::EvTopicMessageTimeout<SbLidarSensor>, StAcquireSensors>,
       smacc::transition<EvActionAborted<smacc::SmaccMoveBaseActionClient>, StNavigateToWaypointsX>>
       reactions;

   static void onDefinition()
   {
      static_configure<NavigationOrthogonal, SbNavigateBackwards>(2);
      static_configure<ToolOrthogonal, SbToolStop>();
      static_configure<ObstaclePerceptionOrthogonal, SbLidarSensor>();
   }

   void onInitialize()
   {
   }
};
}