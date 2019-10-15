struct StNavigateForward2 : smacc::SmaccState<StNavigateForward2, SmDanceBot>
{
  typedef mpl::list<
      // Expected event
      sc::transition<EvActionSucceded<SmaccMoveBaseActionClient>, StRotateDegrees5>,

      // Keyboard events
      sc::transition<EvKeyPressP<SbKeyboard>, SS4::SsFPattern1>,
      sc::transition<EvKeyPressN<SbKeyboard>, StRotateDegrees5>,

      // Error events
      sc::transition<smacc::EvTopicMessageTimeout<LidarSensor>, StAcquireSensors>,
      sc::transition<EvActionAborted<smacc::SmaccMoveBaseActionClient>, StNavigateToWaypointsX>

      >
      reactions;

  using SmaccState::SmaccState;

  static void onDefinition()
  {
    static_configure<NavigationOrthogonal,SbNavigateForward>(1);
    static_configure<ToolOrthogonal,SbToolStop>();
    static_configure<KeyboardOrthogonal,SbKeyboard>();
  }

  // Key N -> next state
  void onInitialize()
  {
  }
};