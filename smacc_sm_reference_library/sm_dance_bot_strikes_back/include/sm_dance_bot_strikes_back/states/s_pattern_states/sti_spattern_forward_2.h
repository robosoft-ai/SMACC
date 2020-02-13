namespace sm_dance_bot_strikes_back
{
namespace s_pattern_states
{
// STATE DECLARATION
struct StiSPatternForward2 : public smacc::SmaccState<StiSPatternForward2, SS>
{
  using SmaccState::SmaccState;

// TRANSITION TABLE
  typedef mpl::list<
  
  Transition<EvActionSucceeded<ClMoveBaseZ, OrNavigation>, StiSPatternRotate3>,
  Transition<EvActionAborted<ClMoveBaseZ, OrNavigation>, StiSPatternRotate2>
  
  >reactions;

// STATE FUNCTIONS
  static void staticConfigure()
  {
  }

  void runtimeConfigure()
  {
    double extrasecurityMargin = 0.2;
    auto forwardBehavior = this->configure<OrNavigation, CbNavigateForward>();
    this->configure<OrLED, CbLEDOn>();

    cl_lidar::ClLidarSensor *lidarClient;
    this->requiresClient(lidarClient);
    auto lidarData = lidarClient->getComponent<CpLidarSensorData>();

    forwardBehavior->forwardDistance = lidarData->forwardObstacleDistance - extrasecurityMargin /*extra security margin for easy dynamic implementation of dynamic-smotion*/;
  }
};
} // namespace s_pattern_states
} // namespace sm_dance_bot_strikes_back