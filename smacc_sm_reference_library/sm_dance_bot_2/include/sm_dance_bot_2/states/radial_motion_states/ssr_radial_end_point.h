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
    static_configure<OrNavigation, CbNavigateForward>();
  }

  void onInitialize()
  {
    cl_lidar::ClLaserSensor *lidarClient;
    this->requiresClient(lidarClient);

    auto lidarData = lidarClient->getComponent<CpLidarSensorData>();

    auto forwardBehavior = this->getStateMachine()
                               .getOrthogonal<OrNavigation>()
                               ->getClientBehavior<CbNavigateForward>();

    double forwarddist = 10;

    if (lidarData->forwardObstacleDistance == std::numeric_limits<float>::infinity() || lidarData->forwardObstacleDistance != lidarData->forwardObstacleDistance) // check not is a nan (sensor max dist)
    {
      ROS_INFO("Distance to forward obstacle is not a number, setting default value to: %lf", forwarddist);
    }
    else
    {
      forwarddist = lidarData->forwardObstacleDistance - 0.8 /*meters*/;
    }

    forwardBehavior->forwardDistance = forwarddist;
  }
};
} // namespace radial_motion_states
} // namespace sm_dance_bot_2