struct StNavigateForward1 : smacc::SmaccState<StNavigateForward1, SmDanceBot>
{
  typedef mpl::list<
                    // Expected event
                    sc::transition<EvActionSucceded<smacc::SmaccMoveBaseActionClient::Result>, StRotateDegrees2>,
                    
                    // Keyboard events
                    sc::transition<EvKeyPressP<SbKeyboard>,StRotateDegrees1>,
                    sc::transition<smacc::EvKeyPressN<SbKeyboard>, StRotateDegrees2>,
                    
                    // Error events
                    sc::transition<smacc::EvTopicMessageTimeout<LidarSensor>, StAcquireSensors>,
                    sc::transition<EvActionAborted<smacc::SmaccMoveBaseActionClient::Result>, StNavigateToWaypointsX>

                    > reactions; 

  using SmaccState::SmaccState;

  // Key N -> next state
  void onInitialize()
  {
    this->configure<NavigationOrthogonal>(new SbNavigateForward(1));
    this->configure<ToolOrthogonal>(new SbToolStop());
    this->configure<KeyboardOrthogonal>(new SbKeyboard());
  }
};