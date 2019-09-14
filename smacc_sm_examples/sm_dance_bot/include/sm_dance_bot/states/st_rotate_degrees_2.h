struct StRotateDegrees2: smacc::SmaccState<StRotateDegrees2,SmDanceBot>
{
  using SmaccState::SmaccState;

  typedef mpl::list<sc::transition<smacc::SmaccMoveBaseActionClient::SuccessEv, StNavigateToWaypointsX>,
                    
                    sc::transition<smacc::EvSensorMessageTimeout<LidarSensor>, StAcquireSensors>,
                    sc::transition<EvActionAborted<smacc::SmaccMoveBaseActionClient::Result>, StNavigateToWaypointsX>
            > reactions; 

  void onInitialize()
  {
    this->configure<NavigationOrthogonal>(new SbRotate(/*30*/-90));
    this->configure<ToolOrthogonal>(new SbToolStop());
  }
};
