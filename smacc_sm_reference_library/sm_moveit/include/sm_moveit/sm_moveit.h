#include <smacc/smacc.h>

// ORTHOGONALS

// CLIENT BEHAVIORS
#include <sm_moveit/clients/movegroup_client/client_behaviors/cb_goto_cube.h>
#include <sm_moveit/clients/gripper_client/client_behaviors/cb_open_gripper.h>
// #include <sm_moveit/clients/gripper_client/client_behaviors/cb_close_gripper.h>

// CLIENT NAMESPACES (to improve readability in state transitions and behavior configurations)
using namespace sm_moveit::cl_movegroup;
using namespace sm_moveit::cl_gripper;

//STATE REACTORS

using namespace smacc::state_reactors;

// ORTHOGONALS
#include <sm_moveit/orthogonals/or_gripper.h>
#include <sm_moveit/orthogonals/or_arm.h>

namespace sm_moveit
{
//STATE FORWARD DECLARATIONS
class StCloseGripper;
class StGraspRetreat;
class StMovePregraspPose;

//SUPERSTATE FORWARD DECLARATIONS

// MODE STATES FORWARD DECLARATIONS
} // namespace sm_moveit

using namespace smacc;

namespace sm_moveit
{
/// \brief Advanced example of state machine with smacc that shows multiple techniques
///  for the development of state machines
struct SmMoveIt
    : public smacc::SmaccStateMachineBase<SmMoveIt, StMovePregraspPose>
{
    using SmaccStateMachineBase::SmaccStateMachineBase;

    virtual void onInitialize() override
    {
        this->createOrthogonal<OrGripper>();
        this->createOrthogonal<OrArm>();
    }
};

} // namespace sm_moveit

//MODESTATES

//SUPERSTATES

//STATES
// #include <sm_moveit/states/st_close_gripper.h>
// #include <sm_moveit/states/st_open_gripper.h>
#include <sm_moveit/states/st_move_pregrasp_pose.h>
#include <sm_moveit/states/st_grasp_retreat.h>
