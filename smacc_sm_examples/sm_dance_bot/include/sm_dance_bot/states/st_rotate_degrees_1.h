struct StRotateDegrees1 : smacc::SmaccState<StRotateDegrees1, SmDanceBot>
{
  using SmaccState::SmaccState;

  typedef mpl::list<
            // Expected event
            sc::transition<EvActionSucceded<smacc::SmaccMoveBaseActionClient::Result>, StNavigateForward1>,

            // Keyboard event
            sc::transition<EvKeyPressP<SbKeyboard>,SS1::SsRadialPattern1>,
            sc::transition<EvKeyPressN<SbKeyboard>, StNavigateForward1>,

            // Error events
            sc::transition<smacc::EvTopicMessageTimeout<LidarSensor>, StAcquireSensors>,
            sc::transition<EvActionAborted<smacc::SmaccMoveBaseActionClient::Result>, StNavigateToWaypointsX>
          > reactions;

  void onInitialize()
  {
    this->configure<NavigationOrthogonal>(std::make_shared<SbRotate>(/*30*/90));
    this->configure<ToolOrthogonal>(std::make_shared<SbToolStop>());
    this->configure<KeyboardOrthogonal>(std::make_shared<SbKeyboard>());
  }
};
