struct StNavigateReverse3 : smacc::SmaccState<StNavigateReverse3, MsDanceBotRunMode>
{
   using SmaccState::SmaccState;

   typedef mpl::list<
       // Expected event
       smacc::transition<EvActionSucceded<smacc::SmaccMoveBaseActionClient>, StNavigateToWaypoint1>,

       // Keyboard events
       smacc::transition<EvKeyPressP<SbKeyboard>, StRotateDegrees6>,
       smacc::transition<EvKeyPressN<SbKeyboard>, StNavigateToWaypoint1>,

       // Error events
       //smacc::transition<smacc::EvTopicMessageTimeout<SbLidarSensor>, StAcquireSensors>,
       smacc::transition<EvActionAborted<smacc::SmaccMoveBaseActionClient>, StNavigateToWaypointsX>>
       reactions;

   static void onDefinition()
   {
      static_configure<NavigationOrthogonal, SbNavigateBackwards>(2);
      static_configure<ToolOrthogonal, SbToolStop>();
      static_configure<KeyboardOrthogonal, SbKeyboard>();
      static_configure<ObstaclePerceptionOrthogonal, SbLidarSensor>();
   }

   void onInitialize()
   {
   }
};
