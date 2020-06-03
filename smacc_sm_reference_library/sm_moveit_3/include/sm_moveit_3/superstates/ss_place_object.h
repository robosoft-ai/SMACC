#include <smacc/smacc.h>

namespace sm_moveit_3
{
namespace SS2
{
namespace sm_moveit_3
{
namespace place_states
{

//FORWARD DECLARATION OF INNER STATES
class StMovePrePlacePose;
class StPlaceApproach;
class StOpenGripper;
class StPlaceRetreat;
} // namespace place_states
} // namespace sm_moveit_3

using namespace sm_moveit_3::place_states;

// STATE DECLARATION
struct SsPlaceObject : smacc::SmaccState<SsPlaceObject, SmMoveit3, StMovePrePlacePose>
{
public:
    using SmaccState::SmaccState;

    // TRANSITION TABLE
    typedef mpl::list<
        Transition<EvSequenceFinished<SS2::StPlaceRetreat>, SS1::SsPickObject, SUCCESS>>
        reactions;

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

#include <sm_moveit_3/states/place_states/st_open_gripper.h>
#include <sm_moveit_3/states/place_states/st_place_approach.h>
#include <sm_moveit_3/states/place_states/st_place_retreat.h>
#include <sm_moveit_3/states/place_states/st_move_preplace_pose.h>

} // namespace SS2
} // namespace sm_moveit_3