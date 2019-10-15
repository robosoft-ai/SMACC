struct StRotateDegrees2 : smacc::SmaccState<StRotateDegrees2, SmDanceBot>
{
  using SmaccState::SmaccState;

  typedef mpl::list<
      // Expected event
      sc::transition<EvActionSucceded<smacc::SmaccMoveBaseActionClient>, StNavigateToWaypointsX>,

      // Keyboard events
      sc::transition<EvKeyPressP<SbKeyboard>, StNavigateForward1>,
      sc::transition<EvKeyPressN<SbKeyboard>, StNavigateToWaypointsX>,

      // Error events
      sc::transition<smacc::EvTopicMessageTimeout<LidarSensor>, StAcquireSensors>,
      sc::transition<EvActionAborted<smacc::SmaccMoveBaseActionClient>, StNavigateToWaypointsX>>
      reactions;

  static void onDefinition()
  {
    static_configure<NavigationOrthogonal, SbRotate>(/*30*/ -90);
    static_configure<ToolOrthogonal, SbToolStop>();
    static_configure<KeyboardOrthogonal, SbKeyboard>();
  }
  
  void onInitialize()
  {
    
  }
};
