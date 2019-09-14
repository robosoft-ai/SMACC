struct StRotateDegrees3 : smacc::SmaccState<StRotateDegrees3, SmDanceBot>
{
  using SmaccState::SmaccState;

  typedef mpl::list<
              sc::transition<smacc::SmaccMoveBaseActionClient::SuccessEv, StNavigateToWaypointsX>,
              
              sc::transition<smacc::EvSensorMessageTimeout<LidarSensor>, StAcquireSensors>,
              sc::transition<EvActionAborted<smacc::SmaccMoveBaseActionClient::Result>, StNavigateToWaypointsX>
              > reactions;

  void onInitialize()
  {
    this->configure<NavigationOrthogonal>(new SbRotate(/*30*/180));
    this->configure<ToolOrthogonal>(new SbToolStop());
  }
};
