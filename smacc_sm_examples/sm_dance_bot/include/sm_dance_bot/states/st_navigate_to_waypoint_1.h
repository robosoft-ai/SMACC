struct StNavigateToWaypoint1 : smacc::SmaccState<StNavigateToWaypoint1, MsDanceBotRunMode>
{
  using SmaccState::SmaccState;

  typedef mpl::list<
      // Expected event
      smacc::transition<EvActionSucceded<smacc::SmaccMoveBaseActionClient>, StNavigateToWaypointsX>,

      // Keyboard events
      smacc::transition<EvKeyPressP<SbKeyboard>, StNavigateReverse2>,

      // Error events
      //smacc::transition<smacc::EvTopicMessageTimeout<SbLidarSensor>, StAcquireSensors>,
      smacc::transition<EvActionAborted<smacc::SmaccMoveBaseActionClient>, StNavigateToWaypointsX>>
      reactions;

  static void onDefinition()
  {
    static_configure<NavigationOrthogonal, SbNavigateGlobalPosition>(0, 0, 0);
    static_configure<ToolOrthogonal, SbToolStart>();
    static_configure<KeyboardOrthogonal, SbKeyboard>();
    static_configure<PublisherOrthogonal, SbStringPublisher>("All Done!");
    static_configure<ObstaclePerceptionOrthogonal, SbLidarSensor>();
  }

  void onInitialize()
  {
    
  }
};
