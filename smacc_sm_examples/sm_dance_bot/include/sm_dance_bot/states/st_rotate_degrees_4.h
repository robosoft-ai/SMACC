struct StRotateDegrees4: smacc::SmaccState<StRotateDegrees4,SmDanceBot>
{
  using SmaccState::SmaccState;

  typedef mpl::list<
            // Expected event
            sc::transition<EvActionSucceded<smacc::SmaccMoveBaseActionClient::Result>, StNavigateReverse2>,
            
            // Keyboard events
            sc::transition<EvKeyPressP<SbKeyboard>,SS3::SsRadialPattern3>,
            sc::transition<smacc::EvKeyPressN<SbKeyboard>, StNavigateReverse2>,

            // Error events
            sc::transition<smacc::EvTopicMessageTimeout<LidarSensor>, StAcquireSensors>,
            sc::transition<EvActionAborted<smacc::SmaccMoveBaseActionClient::Result>, StNavigateToWaypointsX>
            > reactions;

  void onInitialize()
  {
    this->configure<NavigationOrthogonal>(new SbRotate(/*30*/-180));
    this->configure<ToolOrthogonal>(new SbToolStop());
    this->configure<KeyboardOrthogonal>(new SbKeyboard());
  }
};
