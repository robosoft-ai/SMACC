struct StNavigateForward1 : smacc::SmaccState<StNavigateForward1, SmDanceBot>
{
  typedef mpl::list<
      // Expected event
      sc::transition<EvActionSucceded<SmaccMoveBaseActionClient>, StRotateDegrees2>,

      // Keyboard events
      sc::transition<EvKeyPressP<SbKeyboard>, StRotateDegrees1>,
      sc::transition<EvKeyPressN<SbKeyboard>, StRotateDegrees2>,

      // Error events
      sc::transition<smacc::EvTopicMessageTimeout<LidarSensor>, StAcquireSensors>,
      sc::transition<EvActionAborted<smacc::SmaccMoveBaseActionClient>, StNavigateToWaypointsX>>
      reactions;

  using SmaccState::SmaccState;

  static void onDefinition()
  {
    static_configure<NavigationOrthogonal, SbNavigateForward>(1);
    static_configure<ToolOrthogonal, SbToolStop>();
    static_configure<KeyboardOrthogonal, SbKeyboard>();
  }

  void onInitialize()
  {
  }
};