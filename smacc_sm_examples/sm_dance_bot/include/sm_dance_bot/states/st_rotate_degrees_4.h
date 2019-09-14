struct StRotateDegrees4: smacc::SmaccState<StRotateDegrees4,SmDanceBot>
{
  using SmaccState::SmaccState;

  typedef mpl::list<
            sc::transition<smacc::SmaccMoveBaseActionClient::SuccessEv, StNavigateReverse2>,
            
            sc::transition<smacc::EvSensorMessageTimeout<LidarSensor>, StAcquireSensors>,
            sc::transition<EvActionAborted<smacc::SmaccMoveBaseActionClient::Result>, StNavigateToWaypointsX>
            > reactions;

  void onInitialize()
  {
    this->configure<NavigationOrthogonal>(new SbRotate(/*30*/-180));
    this->configure<ToolOrthogonal>(new SbToolStop());
  }
};
