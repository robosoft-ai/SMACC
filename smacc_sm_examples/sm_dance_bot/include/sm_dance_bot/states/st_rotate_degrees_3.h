struct StRotateDegrees3 : smacc::SmaccState<StRotateDegrees3, SmDanceBot>
{
  using SmaccState::SmaccState;

  typedef mpl::list<
              // Expected event
              sc::transition<EvActionSucceded<smacc::SmaccMoveBaseActionClient::Result>, StNavigateToWaypointsX>,
              
              // Keyboard events
              sc::transition<KeyPressEvent<'p'>,StNavigateReverse1>,
              sc::transition<smacc::KeyPressEvent<'n'>, StNavigateToWaypointsX>,

              // Error events
              sc::transition<smacc::EvSensorMessageTimeout<LidarSensor>, StAcquireSensors>,
              sc::transition<EvActionAborted<smacc::SmaccMoveBaseActionClient::Result>, StNavigateToWaypointsX>
              > reactions;

  void onInitialize()
  {
    this->configure<NavigationOrthogonal>(new SbRotate(/*30*/180));
    this->configure<ToolOrthogonal>(new SbToolStop());
  }
};
