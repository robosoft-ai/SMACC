struct StRotateDegrees3 : smacc::SmaccState<StRotateDegrees3, SmDanceBot>
{
  using SmaccState::SmaccState;

  typedef mpl::list<
              // Expected event
              sc::transition<EvActionSucceded<smacc::SmaccMoveBaseActionClient::Result>, StNavigateToWaypointsX>,
              
              // Keyboard events
              sc::transition<EvKeyPressP<SbKeyboard>,StNavigateReverse1>,
              sc::transition<smacc::EvKeyPressN<SbKeyboard>, StNavigateToWaypointsX>,

              // Error events
              sc::transition<smacc::EvTopicMessageTimeout<LidarSensor>, StAcquireSensors>,
              sc::transition<EvActionAborted<smacc::SmaccMoveBaseActionClient::Result>, StNavigateToWaypointsX>
              > reactions;

  void onInitialize()
  {
    this->configure<NavigationOrthogonal>(new SbRotate(/*30*/180));
    this->configure<ToolOrthogonal>(new SbToolStop());
    this->configure<KeyboardOrthogonal>(new SbKeyboard());
  }
};
