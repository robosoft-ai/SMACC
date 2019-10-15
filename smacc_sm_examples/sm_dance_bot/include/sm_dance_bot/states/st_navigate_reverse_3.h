struct StNavigateReverse3 : smacc::SmaccState<StNavigateReverse3, SmDanceBot>
{
   using SmaccState::SmaccState;

   typedef mpl::list<
       // Expected event
       sc::transition<EvActionSucceded<smacc::SmaccMoveBaseActionClient>, StNavigateToWaypoint1>,

       // Keyboard events
       sc::transition<EvKeyPressP<SbKeyboard>, StRotateDegrees6>,
       sc::transition<EvKeyPressN<SbKeyboard>, StNavigateToWaypoint1>,

       // Error events
       sc::transition<smacc::EvTopicMessageTimeout<LidarSensor>, StAcquireSensors>,
       sc::transition<EvActionAborted<smacc::SmaccMoveBaseActionClient>, StNavigateToWaypointsX>>
       reactions;

   static void onDefinition()
   {
      static_configure<NavigationOrthogonal, SbNavigateBackwards>(2);
      static_configure<ToolOrthogonal, SbToolStop>();
      static_configure<KeyboardOrthogonal, SbKeyboard>();
   }

   void onInitialize()
   {
   }
};
