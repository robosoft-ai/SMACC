struct StNavigateForward1 : smacc::SmaccState<StNavigateForward1, SmDanceBot>
{
  typedef mpl::list<sc::transition<smacc::SmaccMoveBaseActionClient::SuccessEv, StRotateDegrees2>,
                    sc::transition<smacc::KeyPressEvent<'n'>, StRotateDegrees2>,

                    sc::transition<smacc::EvSensorMessageTimeout<LidarSensor>, StAcquireSensors>,
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