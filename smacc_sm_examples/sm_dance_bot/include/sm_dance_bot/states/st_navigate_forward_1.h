struct StNavigateForward1 : smacc::SmaccState<StNavigateForward1, SmDanceBot>
{
  typedef mpl::list<
      // Expected event
      smacc::transition<EvActionSucceded<SmaccMoveBaseActionClient>, StRotateDegrees2>,

      // Keyboard events
      smacc::transition<EvKeyPressP<SbKeyboard>, StRotateDegrees1>,
      smacc::transition<EvKeyPressN<SbKeyboard>, StRotateDegrees2>,

      // Error events
      smacc::transition<smacc::EvTopicMessageTimeout<SbLidarSensor>, StAcquireSensors>,
      smacc::transition<EvActionAborted<smacc::SmaccMoveBaseActionClient>, StNavigateToWaypointsX>>
      reactions;

  using SmaccState::SmaccState;

  static void onDefinition()
  {
    static_configure<NavigationOrthogonal, SbNavigateForward>(1);
    static_configure<ToolOrthogonal, SbToolStop>();
    static_configure<KeyboardOrthogonal, SbKeyboard>();
    static_configure<ObstaclePerceptionOrthogonal, SbLidarSensor>();
  }

  void onInitialize()
  {
  }
};