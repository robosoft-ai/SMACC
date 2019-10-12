struct StNavigateReverse2 : smacc::SmaccState<StNavigateReverse2, SmDanceBot>
{
   using SmaccState::SmaccState;

   typedef mpl::list<
       // Expected event
       sc::transition<EvActionSucceded<smacc::SmaccMoveBaseActionClient>, StNavigateToWaypointsX>,

       // Keyboard events
       sc::transition<EvKeyPressP<SbKeyboard>, StRotateDegrees4>,
       sc::transition<EvKeyPressN<SbKeyboard>, StNavigateToWaypointsX>,

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
