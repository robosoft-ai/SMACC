#include <smacc/smacc.h>

// ORTHOGONALS

// CLIENT BEHAVIORS
#include <move_group_interface_client/client_behaviors.h>
#include <move_group_interface_client/cl_movegroup.h>

#include <move_base_z_client_plugin/client_behaviors.h>
#include <move_base_z_client_plugin/components/pose/cp_pose.h>

#include <sm_fetch_two_table_pick_n_place_1/clients/gripper_client/client_behaviors/cb_close_gripper.h>
#include <sm_fetch_two_table_pick_n_place_1/clients/gripper_client/client_behaviors/cb_open_gripper.h>

// CLIENT NAMESPACES (to improve readability in state transitions and behavior configurations)
using namespace cl_move_group_interface;
using namespace sm_fetch_two_table_pick_n_place_1::cl_gripper;

//STATE REACTORS
using namespace smacc::state_reactors;

// ORTHOGONALS
#include "orthogonals/or_gripper.h"
#include "orthogonals/or_arm.h"
#include "orthogonals/or_perception.h"
#include "orthogonals/or_navigation.h"

namespace sm_fetch_two_table_pick_n_place_1
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
} // namespace sm_fetch_two_table_pick_n_place_1

using namespace smacc;

namespace sm_fetch_two_table_pick_n_place_1
{
/// \brief Advanced example of state machine with smacc that shows multiple techniques
///  for the development of state machines
struct SmFetchTwoTablePickNPlace1
    : public smacc::SmaccStateMachineBase<SmFetchTwoTablePickNPlace1, /*StRotate180*/ StInitialPosture>
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

} // namespace sm_fetch_two_table_pick_n_place_1

//MODESTATES

//SUPERSTATES
#include "superstates/ss_pick_object.h"
#include "superstates/ss_place_object.h"

//STATES
#include "states/st_initial_forward.h"
#include "states/st_initial_posture.h"
#include "states/st_forward.h"
#include "states/st_undo_incorrect_forward.h"
#include "states/st_rotate180.h"

// #include <sm_fetch_two_table_pick_n_place_1/states/st_open_gripper.h>
// #include <sm_fetch_two_table_pick_n_place_1/states/st_move_pregrasp_pose.h>
// #include <sm_fetch_two_table_pick_n_place_1/states/st_grasp_approach.h>
// #include <sm_fetch_two_table_pick_n_place_1/states/st_close_gripper.h>
// #include <sm_fetch_two_table_pick_n_place_1/states/st_grasp_retreat.h>