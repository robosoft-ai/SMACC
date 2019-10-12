struct StNavigateToWaypoint1 : smacc::SmaccState<StNavigateToWaypoint1, SmDanceBot>
{
  using SmaccState::SmaccState;

  typedef mpl::list<
      // Expected event
      sc::transition<EvActionSucceded<smacc::SmaccMoveBaseActionClient>, StNavigateToWaypointsX>,

      // Keyboard events
      sc::transition<EvKeyPressP<SbKeyboard>, StNavigateReverse2>,

      // Error events
      sc::transition<smacc::EvTopicMessageTimeout<LidarSensor>, StAcquireSensors>,
      sc::transition<EvActionAborted<smacc::SmaccMoveBaseActionClient>, StNavigateToWaypointsX>>
      reactions;

  void onInitialize()
  {
    this->configure<NavigationOrthogonal>(std::make_shared<SbNavigateGlobalPosition>(0, 0, 0));
    this->configure<ToolOrthogonal>(std::make_shared<SbToolStart>());
    this->configure<KeyboardOrthogonal>(std::make_shared<SbKeyboard>());
    this->configure<PublisherOrthogonal>(std::make_shared<SbStringPublisher>("All Done!"));
  }
};
