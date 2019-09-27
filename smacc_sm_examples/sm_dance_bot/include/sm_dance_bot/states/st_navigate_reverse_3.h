struct StNavigateReverse3 : smacc::SmaccState<StNavigateReverse3, SmDanceBot>
{
   using SmaccState::SmaccState;

   typedef mpl::list<
            // Expected event
            sc::transition<EvActionSucceded<smacc::SmaccMoveBaseActionClient::Result>, StNavigateToWaypoint1>,

            // Keyboard events
            sc::transition<EvKeyPressP<SbKeyboard>,StRotateDegrees6>,
            sc::transition<smacc::EvKeyPressN<SbKeyboard>, StNavigateToWaypoint1>,

            // Error events
            sc::transition<smacc::EvTopicMessageTimeout<LidarSensor>, StAcquireSensors>,
            sc::transition<EvActionAborted<smacc::SmaccMoveBaseActionClient::Result>, StNavigateToWaypointsX>
            > reactions;

   void onInitialize()
   {
      this->configure<NavigationOrthogonal>(std::make_shared<SbNavigateBackwards>(2));
      this->configure<ToolOrthogonal>(std::make_shared<SbToolStop>());
      this->configure<KeyboardOrthogonal>(std::make_shared<SbKeyboard>());
   }
};
