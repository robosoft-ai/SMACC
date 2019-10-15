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

  static void onDefinition()
  {
    static_configure<NavigationOrthogonal, SbNavigateGlobalPosition>(0, 0, 0);
    static_configure<ToolOrthogonal, SbToolStart>();
    static_configure<KeyboardOrthogonal, SbKeyboard>();
    static_configure<PublisherOrthogonal, SbStringPublisher>("All Done!");
  }

  void onInitialize()
  {
    
  }
};
