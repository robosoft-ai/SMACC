struct StNavigateForward2 : smacc::SmaccState<StNavigateForward2, SmDanceBot>
{
  typedef mpl::list<
                    // Expected event
                    sc::transition<EvActionSucceded<smacc::SmaccMoveBaseActionClient::Result>, StRotateDegrees5>,
                    
                    // Keyboard events
                    sc::transition<EvKeyPressP<SbKeyboard>,SS4::SsFPattern1>,
                    sc::transition<smacc::EvKeyPressN<SbKeyboard>, StRotateDegrees5>,
                    
                    // Error events
                    sc::transition<smacc::EvTopicMessageTimeout<LidarSensor>, StAcquireSensors>,
                    sc::transition<EvActionAborted<smacc::SmaccMoveBaseActionClient::Result>, StNavigateToWaypointsX>

                    > reactions; 

  using SmaccState::SmaccState;

  // Key N -> next state
  void onInitialize()
  {
    this->configure<NavigationOrthogonal>(std::make_shared<SbNavigateForward>(1));
    this->configure<ToolOrthogonal>(std::make_shared<SbToolStop>());
    this->configure<KeyboardOrthogonal>(std::make_shared<SbKeyboard>());
  }
};