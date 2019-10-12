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

   void onInitialize()
   {
      this->configure<NavigationOrthogonal>(std::make_shared<SbNavigateBackwards>(2));
      this->configure<ToolOrthogonal>(std::make_shared<SbToolStop>());
      this->configure<KeyboardOrthogonal>(std::make_shared<SbKeyboard>());
   }
};
