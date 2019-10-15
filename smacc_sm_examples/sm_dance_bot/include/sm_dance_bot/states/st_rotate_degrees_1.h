struct StRotateDegrees1 : smacc::SmaccState<StRotateDegrees1, SmDanceBot>
{
  using SmaccState::SmaccState;

  typedef mpl::list<
      // Expected event
      sc::transition<EvActionSucceded<smacc::SmaccMoveBaseActionClient>, StNavigateForward1>,

      // Keyboard event
      sc::transition<EvKeyPressP<SbKeyboard>, SS1::SsRadialPattern1>,
      sc::transition<EvKeyPressN<SbKeyboard>, StNavigateForward1>,

      // Error events
      sc::transition<smacc::EvTopicMessageTimeout<LidarSensor>, StAcquireSensors>,
      sc::transition<EvActionAborted<smacc::SmaccMoveBaseActionClient>, StNavigateToWaypointsX>>
      reactions;

  static void onDefinition()
  {
    static_configure<NavigationOrthogonal, SbRotate>(/*30*/ 90);
    static_configure<ToolOrthogonal, SbToolStop>();
    static_configure<KeyboardOrthogonal, SbKeyboard>();
  }

  void onInitialize()
  {
    
  }
};
