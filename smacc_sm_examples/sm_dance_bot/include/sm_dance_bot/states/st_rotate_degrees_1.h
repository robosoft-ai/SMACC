struct StRotateDegrees1 : smacc::SmaccState<StRotateDegrees1, SmDanceBot>
{
  using SmaccState::SmaccState;

  typedef mpl::list<
            sc::transition<smacc::SmaccMoveBaseActionClient::SuccessEv, StNavigateForward1>,

            sc::transition<smacc::EvSensorMessageTimeout<LidarSensor>, StAcquireSensors>,
            sc::transition<EvActionAborted<smacc::SmaccMoveBaseActionClient::Result>, StNavigateToWaypointsX>
          > reactions;

  void onInitialize()
  {
    this->configure<NavigationOrthogonal>(new SbRotate(/*30*/90));
    this->configure<ToolOrthogonal>(new SbToolStop());
  }
};
