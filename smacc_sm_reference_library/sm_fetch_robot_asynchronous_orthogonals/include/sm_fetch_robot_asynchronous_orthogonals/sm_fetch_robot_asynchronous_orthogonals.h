#include <smacc/smacc.h>

// ORTHOGONALS

// CLIENT BEHAVIORS
#include <move_group_interface_client/client_behaviors.h>

#include <move_base_z_client_plugin/client_behaviors.h>
#include <move_base_z_client_plugin/components/pose/cp_pose.h>

#include "clients/gripper_client/client_behaviors/cb_close_gripper.h"
#include "clients/gripper_client/client_behaviors/cb_open_gripper.h"

// CLIENT NAMESPACES (to improve readability in state transitions and behavior configurations)
using namespace cl_move_group_interface;
using namespace sm_fetch_robot_asynchronous_orthogonals::cl_gripper;

// ORTHOGONALS
#include "orthogonals/or_gripper.h"
#include "orthogonals/or_arm.h"
#include "orthogonals/or_perception.h"
#include "orthogonals/or_navigation.h"

namespace sm_fetch_robot_asynchronous_orthogonals
{ 
    class StInitialPosture;
    class StNavigateForwardsAndHandsUp;
    class StNavigateOriginAndHandsDown;
} // namespace sm_fetch_robot_asynchronous_orthogonals

using namespace smacc;

namespace sm_fetch_robot_asynchronous_orthogonals
{
    /// \brief Advanced example of state machine with smacc that shows multiple techniques
    ///  for the development of state machines
    struct SmFetchRobotAsynchronousOrthogonals
        : public smacc::SmaccStateMachineBase<SmFetchRobotAsynchronousOrthogonals, StInitialPosture>
    {
        using SmaccStateMachineBase::SmaccStateMachineBase;

        virtual void onInitialize() override
        {
            this->createOrthogonal<OrPerception>();
            this->createOrthogonal<OrGripper>();
            this->createOrthogonal<OrArm>();
            this->createOrthogonal<OrNavigation>();
        }
    };

} // namespace sm_fetch_robot_asynchronous_orthogonals

//STATES
#include "states/st_initial_posture.h"
#include "states/st_navigate_forwards_and_hands_up.h"
#include "states/st_navigate_origin_and_hands_down.h"