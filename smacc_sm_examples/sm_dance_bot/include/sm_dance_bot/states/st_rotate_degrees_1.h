struct StRotateDegrees1 : smacc::SmaccState<StRotateDegrees1, SmDanceBot>
{
  using SmaccState::SmaccState;

  typedef mpl::list<
            // Expected event
            sc::transition<EvActionSucceded<smacc::SmaccMoveBaseActionClient::Result>, StNavigateForward1>,

            // Keyboard event
            sc::transition<KeyPressEvent<'p'>,SS1::SsRadialPattern1>,
            sc::transition<KeyPressEvent<'n'>, StNavigateForward1>,

            // Error events
            sc::transition<smacc::EvSensorMessageTimeout<LidarSensor>, StAcquireSensors>,
            sc::transition<EvActionAborted<smacc::SmaccMoveBaseActionClient::Result>, StNavigateToWaypointsX>
          > reactions;

  void onInitialize()
  {
    this->configure<NavigationOrthogonal>(new SbRotate(/*30*/90));
    this->configure<ToolOrthogonal>(new SbToolStop());
  }
};
