struct StNavigateReverse2 : smacc::SmaccState<StNavigateReverse2, MsDanceBotRunMode>
{
   using SmaccState::SmaccState;

   typedef mpl::list<
       // Expected event
       smacc::transition<EvActionSucceeded<smacc::SmaccMoveBaseActionClient>, StNavigateToWaypointsX>,

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
