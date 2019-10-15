struct StRotateDegrees6 : smacc::SmaccState<StRotateDegrees6, SmDanceBot>
{
  using SmaccState::SmaccState;

  typedef mpl::list<
      // Expected event
      sc::transition<EvActionSucceded<smacc::SmaccMoveBaseActionClient>, StNavigateReverse3>,

      // Keyboard events
      sc::transition<EvKeyPressP<SbKeyboard>, SS5::SsSPattern1>,
      sc::transition<EvKeyPressN<SbKeyboard>, StNavigateReverse3>,

      // Error events
      sc::transition<smacc::EvTopicMessageTimeout<LidarSensor>, StAcquireSensors>,
      sc::transition<EvActionAborted<smacc::SmaccMoveBaseActionClient>, StNavigateToWaypointsX>>
      reactions;

  static void onDefinition()
  {
    static_configure<NavigationOrthogonal,SbRotate>(/*30*/ -180);
    static_configure<ToolOrthogonal, SbToolStop>();
    static_configure<KeyboardOrthogonal,SbKeyboard>();
  }

  void onInitialize()
  {
  }
};
