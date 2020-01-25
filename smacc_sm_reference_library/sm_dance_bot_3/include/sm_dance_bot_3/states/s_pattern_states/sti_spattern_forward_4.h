namespace sm_dance_bot_3
{
namespace s_pattern_states
{
struct StiSPatternForward4 : public smacc::SmaccState<StiSPatternForward4, SS>
{
  using SmaccState::SmaccState;

  typedef mpl::list<smacc::Transition<EvActionSucceeded<ClMoveBaseZ, OrNavigation>, StiSPatternLoopStart>,
                    smacc::Transition<EvActionAborted<ClMoveBaseZ, OrNavigation>, StiSPatternRotate4>>
      reactions;

  static void staticConfigure()
  {
  }

  void runtimeConfigure()
  {
    double extrasecurityMargin = 0.2;
    
    this->configure<OrNavigation, CbNavigateForward>();
    this->configure<OrLED, CbLEDOn>();

     auto forwardBehavior = this->configure<OrNavigation, CbNavigateForward>();
    this->configure<OrLED, CbLEDOn>();

    cl_lidar::ClLidarSensor *lidarClient;
    this->requiresClient(lidarClient);
    auto lidarData = lidarClient->getComponent<CpLidarSensorData>();

    forwardBehavior->forwardDistance = lidarData->forwardObstacleDistance - extrasecurityMargin /*extra security margin for easy dynamic implementation of dynamic-smotion*/;
  }
};
} // namespace s_pattern_states
} // namespace sm_dance_bot_3