struct StRotateDegrees4 : smacc::SmaccState<StRotateDegrees4, SmDanceBot>
{
  using SmaccState::SmaccState;

  typedef mpl::list<
      // Expected event
      sc::transition<EvActionSucceded<smacc::SmaccMoveBaseActionClient>, StNavigateReverse2>,

      // Keyboard events
      sc::transition<EvKeyPressP<SbKeyboard>, SS3::SsRadialPattern3>,
      sc::transition<EvKeyPressN<SbKeyboard>, StNavigateReverse2>,

      // Error events
      sc::transition<smacc::EvTopicMessageTimeout<LidarSensor>, StAcquireSensors>,
      sc::transition<EvActionAborted<smacc::SmaccMoveBaseActionClient>, StNavigateToWaypointsX>>
      reactions;

  static void onDefinition()
  {
    static_configure<NavigationOrthogonal,SbRotate>(/*30*/ -180);
    static_configure<ToolOrthogonal,SbToolStop>();
    static_configure<KeyboardOrthogonal,SbKeyboard>();
  }

  void onInitialize()
  {
    
  }
};
