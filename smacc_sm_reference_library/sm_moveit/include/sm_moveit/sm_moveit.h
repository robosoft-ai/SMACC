#include <smacc/smacc.h>

// ORTHOGONALS

// CLIENT BEHAVIORS
#include <sm_moveit/clients/movegroup_client/client_behaviors/cb_move_absolute.h>
#include <sm_moveit/clients/movegroup_client/client_behaviors/cb_move_cartesian_relative.h>
#include <move_base_z_client_plugin/components/pose/cp_pose.h>

#include <sm_moveit/clients/gripper_client/client_behaviors/cb_close_gripper.h>
#include <sm_moveit/clients/gripper_client/client_behaviors/cb_open_gripper.h>

// CLIENT NAMESPACES (to improve readability in state transitions and behavior configurations)
using namespace sm_moveit::cl_movegroup;
using namespace sm_moveit::cl_gripper;

//STATE REACTORS

using namespace smacc::state_reactors;

// ORTHOGONALS
#include <sm_moveit/orthogonals/or_gripper.h>
#include <sm_moveit/orthogonals/or_arm.h>
#include <sm_moveit/orthogonals/or_perception.h>

namespace sm_moveit
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

//SUPERSTATE FORWARD DECLARATIONS

// MODE STATES FORWARD DECLARATIONS
} // namespace sm_moveit

using namespace smacc;

namespace sm_moveit
{
/// \brief Advanced example of state machine with smacc that shows multiple techniques
///  for the development of state machines
struct SmMoveIt
    : public smacc::SmaccStateMachineBase<SmMoveIt, SS1::SsPickObject>
{
    using SmaccStateMachineBase::SmaccStateMachineBase;

    virtual void onInitialize() override
    {
        this->createOrthogonal<OrGripper>();
        this->createOrthogonal<OrArm>();
        this->createOrthogonal<OrPerception>();
    }
};

} // namespace sm_moveit

//MODESTATES

//SUPERSTATES
#include <sm_moveit/superstates/ss_pick_object.h>
#include <sm_moveit/superstates/ss_place_object.h>

//STATES

// #include <sm_moveit/states/st_open_gripper.h>

// #include <sm_moveit/states/st_move_pregrasp_pose.h>
// #include <sm_moveit/states/st_grasp_approach.h>
// #include <sm_moveit/states/st_close_gripper.h>
// #include <sm_moveit/states/st_grasp_retreat.h>