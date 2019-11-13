struct StNavigateForward1 : smacc::SmaccState<StNavigateForward1, MsDanceBotRunMode>
{
  typedef mpl::list<
      // Expected event
      transition<EvActionSucceded<SmaccMoveBaseActionClient>, StRotateDegrees2>,

      // Keyboard events
      transition<EvKeyPressP<SbKeyboard>, StRotateDegrees1, SUCCESS >,
      transition<EvKeyPressN<SbKeyboard>, StRotateDegrees2, ABORT>,

      // Error events
      //smacc::transition<smacc::EvTopicMessageTimeout<SbLidarSensor>, StAcquireSensors>,
      smacc::transition<EvActionAborted<smacc::SmaccMoveBaseActionClient>, StNavigateToWaypointsX, ABORT>,
      smacc::transition<EvActionPreempted<smacc::SmaccMoveBaseActionClient>, StNavigateToWaypointsX, PREEMPT>>
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