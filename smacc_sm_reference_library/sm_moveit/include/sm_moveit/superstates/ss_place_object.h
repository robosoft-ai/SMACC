#include <smacc/smacc.h>

namespace sm_moveit
{
namespace SS2
{
namespace sm_moveit
{
namespace place_states
{

//FORWARD DECLARATION OF INNER STATES
class StMovePrePlacePose;
class StPlaceApproach;
class StOpenGripper;
class StPlaceRetreat;
} // namespace place_states
} // namespace sm_moveit

using namespace sm_moveit::place_states;

// STATE DECLARATION
struct SsPlaceObject : smacc::SmaccState<SsPlaceObject, SmMoveIt, StMovePrePlacePose>
{
public:
    using SmaccState::SmaccState;

    // TRANSITION TABLE
    typedef mpl::list<
            Transition<EvSequenceFinished<SS2::StPlaceRetreat>, SS1::SsPickObject, SUCCESS>> reactions;

    // STATE FUNCTIONS
    static void staticConfigure()
    {
        //configure_orthogonal<OrObstaclePerception, CbLidarSensor>();
    }

    void runtimeConfigure()
    {
    }
};

using SS = SsPlaceObject;

#include <sm_moveit/states/place_states/st_open_gripper.h>
#include <sm_moveit/states/place_states/st_place_approach.h>
#include <sm_moveit/states/place_states/st_place_retreat.h>
#include <sm_moveit/states/place_states/st_move_preplace_pose.h>

} // namespace SS2
} // namespace sm_moveit