#include <smacc/smacc.h>
namespace sm_opencv_2
{
// STATE DECLARATION
struct StDetectItems : smacc::SmaccState<StDetectItems, SmOpenCV2>
{
  using SmaccState::SmaccState;

  // TRANSITION TABLE
  typedef mpl::list<smacc::Transition<EvTopicMessage<cl_opencv_perception::ClOpenCVPerception, OrPerception>, StNavigateToWaypointX>>
      reactions;

  // STATE FUNCTIONS
  static void staticConfigure()
  {
    //   configure_orthogonal<OrObstaclePerception, CbLidarSensor>();
  }
};
}  // namespace sm_opencv_2