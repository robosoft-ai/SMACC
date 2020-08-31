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
using namespace sm_fetch_screw_loop_1::cl_gripper;

// ORTHOGONALS
#include "orthogonals/or_gripper.h"
#include "orthogonals/or_arm.h"
#include "orthogonals/or_perception.h"
#include "orthogonals/or_navigation.h"

using namespace sm_fetch_screw_loop_1::cl_move_group_interface;

namespace sm_fetch_screw_loop_1
{
    //STATE FORWARD DECLARATIONS

    namespace SS3
    {
        class SsRecoveryScrew;
    }

    class StInitialPosture;
    class StSecondPosture;
    class StFinalRaiseHandsUp;

    //SUPERSTATE FORWARD DECLARATIONS

    // MODE STATES FORWARD DECLARATIONS
} // namespace sm_fetch_screw_loop_1

using namespace smacc;

namespace sm_fetch_screw_loop_1
{
    /// \brief Advanced example of state machine with smacc that shows multiple techniques
    ///  for the development of state machines
    struct SmFetchScrewLoop1
        : public smacc::SmaccStateMachineBase<SmFetchScrewLoop1, StInitialPosture>
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

} // namespace sm_fetch_screw_loop_1

//MODESTATES

//SUPERSTATES

#include "superstates/ss_recovery_screw.h"


#include "states/st_initial_posture.h"
#include "states/st_second_posture.h"
#include "states/st_final_raise_hands_up.h"
