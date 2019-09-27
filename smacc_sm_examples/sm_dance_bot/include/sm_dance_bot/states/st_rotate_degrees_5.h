struct StRotateDegrees5: smacc::SmaccState<StRotateDegrees5,SmDanceBot>
{
  using SmaccState::SmaccState;

  typedef mpl::list<
            // Expected event
            sc::transition<EvActionSucceded<smacc::SmaccMoveBaseActionClient::Result>, StNavigateToWaypointsX>,
            
            // Keyboard events
            sc::transition<EvKeyPressP<SbKeyboard>,StNavigateForward2>,
            sc::transition<smacc::EvKeyPressN<SbKeyboard>, StNavigateToWaypointsX>,

            // Error events
            sc::transition<smacc::EvTopicMessageTimeout<LidarSensor>, StAcquireSensors>,
            sc::transition<EvActionAborted<smacc::SmaccMoveBaseActionClient::Result>, StNavigateToWaypointsX>
            > reactions;

  void onInitialize()
  {
    this->configure<NavigationOrthogonal>(std::make_shared<SbRotate>(/*30*/-180));
    this->configure<ToolOrthogonal>(std::make_shared<SbToolStop>());
    this->configure<KeyboardOrthogonal>(std::make_shared<SbKeyboard>());
  }
};
