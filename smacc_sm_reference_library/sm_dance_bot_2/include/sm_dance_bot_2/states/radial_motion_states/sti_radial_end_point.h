namespace sm_dance_bot_2
{
namespace radial_motion_states
{

struct StiRadialEndPoint : smacc::SmaccState<StiRadialEndPoint, SS>
{
  using SmaccState::SmaccState;

  typedef mpl::list<
      Transition<EvActionSucceeded<ClMoveBaseZ, OrNavigation>, StiRadialReturn, SUCCESS>,
      Transition<EvActionAborted<ClMoveBaseZ, OrNavigation>, StiRadialReturn, ABORT>>
      reactions;

  static void onDefinition()
  {
    configure_orthogonal<OrNavigation, CbNavigateForward>();
  }

  void onInitialize()
  {
    cl_lidar::ClLaserSensor *lidarClient;
    this->requiresClient(lidarClient);

    auto lidarData = lidarClient->getComponent<CpLidarSensorData>();

    auto forwardBehavior = this->getOrthogonal<OrNavigation>()
                               ->getClientBehavior<CbNavigateForward>();

    forwardBehavior->forwardDistance = lidarData->forwardObstacleDistance;
  }
};
} // namespace radial_motion_states
} // namespace sm_dance_bot_2