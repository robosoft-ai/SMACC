#include <smacc/smacc.h>
namespace sm_dance_bot
{
struct StNavigateReverse1 : smacc::SmaccState<StNavigateReverse1, MsDanceBotRunMode>
{
   using SmaccState::SmaccState;

   typedef mpl::list<
       // Expected event
       smacc::transition<EvActionSucceeded<SmaccMoveBaseActionClient, NavigationOrthogonal>, StRotateDegrees3>,

       // Sensor events
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