struct StNavigateForward2 : smacc::SmaccState<StNavigateForward2, MsDanceBotRunMode>
{
  typedef mpl::list<
      // Expected event
      smacc::transition<EvActionSucceded<SmaccMoveBaseActionClient>, StRotateDegrees5>,

      // Keyboard events
      smacc::transition<EvKeyPressP<SbKeyboard>, SS4::SsFPattern1>,
      smacc::transition<EvKeyPressN<SbKeyboard>, StRotateDegrees5>,

      // Error events
      //smacc::transition<smacc::EvTopicMessageTimeout<SbLidarSensor>, StAcquireSensors>,
      smacc::transition<EvActionAborted<smacc::SmaccMoveBaseActionClient>, StNavigateToWaypointsX>
      >
      reactions;

  using SmaccState::SmaccState;

  static void onDefinition()
  {
    static_configure<NavigationOrthogonal,SbNavigateForward>(1);
    static_configure<ToolOrthogonal,SbToolStop>();
    static_configure<KeyboardOrthogonal,SbKeyboard>();
    static_configure<ObstaclePerceptionOrthogonal, SbLidarSensor>();
  }

  // Key N -> next state
  void onInitialize()
  {
  }
};