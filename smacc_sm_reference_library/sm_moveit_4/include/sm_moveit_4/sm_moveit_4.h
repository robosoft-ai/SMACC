#include <smacc/smacc.h>

// ORTHOGONALS

// CLIENT BEHAVIORS
#include <moveit_z_client/client_behaviors.h>

#include <move_base_z_client_plugin/client_behaviors.h>
#include <move_base_z_client_plugin/components/pose/cp_pose.h>

#include <sm_moveit_4/clients/gripper_client/client_behaviors/cb_close_gripper.h>
#include <sm_moveit_4/clients/gripper_client/client_behaviors/cb_open_gripper.h>

// CLIENT NAMESPACES (to improve readability in state transitions and behavior configurations)
using namespace moveit_z_client;
using namespace sm_moveit_4::cl_gripper;

//STATE REACTORS
using namespace smacc::state_reactors;

// ORTHOGONALS
#include <sm_moveit_4/orthogonals/or_gripper.h>
#include <sm_moveit_4/orthogonals/or_arm.h>
#include <sm_moveit_4/orthogonals/or_perception.h>
#include <sm_moveit_4/orthogonals/or_navigation.h>

namespace sm_moveit_4
{
//STATE FORWARD DECLARATIONS

namespace SS1
{
class SsPickObject;
}

namespace SS2
{
class SsPlaceObject;
}

class StRotate180;
class StForwardNextTable;
class StInitialPosture;
class StUndoIncorrectForward;

//SUPERSTATE FORWARD DECLARATIONS

// MODE STATES FORWARD DECLARATIONS
} // namespace sm_moveit_4

using namespace smacc;

namespace sm_moveit_4
{
/// \brief Advanced example of state machine with smacc that shows multiple techniques
///  for the development of state machines
struct SmMoveIt44
    : public smacc::SmaccStateMachineBase<SmMoveIt44, /*StRotate180*/ StInitialPosture>
{
    using SmaccStateMachineBase::SmaccStateMachineBase;

    virtual void onInitialize() override
    {
        this->createOrthogonal<OrGripper>();
        this->createOrthogonal<OrArm>();
        this->createOrthogonal<OrPerception>();
        this->createOrthogonal<OrNavigation>();
    }
};

} // namespace sm_moveit_4

//MODESTATES

//SUPERSTATES
#include <sm_moveit_4/superstates/ss_pick_object.h>
#include <sm_moveit_4/superstates/ss_place_object.h>

//STATES
#include <sm_moveit_4/states/st_initial_forward.h>
#include <sm_moveit_4/states/st_initial_posture.h>
#include <sm_moveit_4/states/st_forward.h>
#include <sm_moveit_4/states/st_undo_incorrect_forward.h>
#include <sm_moveit_4/states/st_rotate180.h>

// #include <sm_moveit_4/states/st_open_gripper.h>
// #include <sm_moveit_4/states/st_move_pregrasp_pose.h>
// #include <sm_moveit_4/states/st_grasp_approach.h>
// #include <sm_moveit_4/states/st_close_gripper.h>
// #include <sm_moveit_4/states/st_grasp_retreat.h>