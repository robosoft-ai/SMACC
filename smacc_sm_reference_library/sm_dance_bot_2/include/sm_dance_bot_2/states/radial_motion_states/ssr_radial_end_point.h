namespace sm_dance_bot_2
{
namespace radial_motion_states
{

struct SsrRadialEndPoint : smacc::SmaccState<SsrRadialEndPoint, SS>
{
  using SmaccState::SmaccState;

  typedef mpl::list<
      smacc::transition<EvActionSucceeded<ClMoveBaseZ, OrNavigation>, SsrRadialReturn, SUCCESS>,
      smacc::transition<EvActionAborted<ClMoveBaseZ, OrNavigation>, SsrRadialRotate, ABORT>>
      reactions;

  static void onDefinition()
  {
    ROS_INFO("ssr radial end point, distance in meters: %lf", SS::ray_length_meters());
    static_configure<OrNavigation, CbNavigateForward>();  }

  void onInitialize()
  {
    cl_lidar::ClLaserSensor *lidarClient;
    this->requiresClient(lidarClient);

    auto lidarData = lidarClient->getComponent<CpLidarSensorData>();

    auto forwardBehavior = this->getStateMachine()
                               .getOrthogonal<OrNavigation>()
                               ->getClientBehavior<CbNavigateForward>();

    forwardBehavior->forwardDistance = lidarData->forwardObstacleDistance;
  }
};
} // namespace radial_motion_states
} // namespace sm_dance_bot_2