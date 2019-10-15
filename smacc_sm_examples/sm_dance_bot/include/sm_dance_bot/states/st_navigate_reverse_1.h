struct StNavigateReverse1 : smacc::SmaccState<StNavigateReverse1, SmDanceBot>
{
   using SmaccState::SmaccState;

   typedef mpl::list<
       // Expected event
       sc::transition<EvActionSucceded<SmaccMoveBaseActionClient>, StRotateDegrees3>,

       // Keyboard event
       sc::transition<EvKeyPressP<SbKeyboard>, SS2::SsRadialPattern2>,
       sc::transition<EvKeyPressN<SbKeyboard>, StRotateDegrees3>,

       // Sensor events
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
