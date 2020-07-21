#include <smacc/smacc.h>

// ORTHOGONALS

// CLIENT BEHAVIORS
#include <move_group_interface_client/client_behaviors.h>

#include <move_base_z_client_plugin/client_behaviors.h>
#include <move_base_z_client_plugin/components/pose/cp_pose.h>

#include "clients/gripper_client/client_behaviors/cb_close_gripper.h"
#include "clients/gripper_client/client_behaviors/cb_open_gripper.h"

// CLIENT NAMESPACES (to improve readability in state transitions and behavior configurations)
using namespace move_group_interface_client;
using namespace sm_moveit_4::cl_gripper;

// ORTHOGONALS
#include "orthogonals/or_gripper.h"
#include "orthogonals/or_arm.h"
#include "orthogonals/or_perception.h"
#include "orthogonals/or_navigation.h"

using namespace sm_moveit_4::move_group_interface_client;

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

    class StInitialPosture;
    class StUndoIncorrectForward;
    class StNavigateToSourceTable;
    class StNavigateToDestinyTable;
    class StNavigationTableRetreat;
    class StNavigateFinalPose;
    class StFinalRaiseHandsUp;

    //SUPERSTATE FORWARD DECLARATIONS

    // MODE STATES FORWARD DECLARATIONS
} // namespace sm_moveit_4

using namespace smacc;

namespace sm_moveit_4
{
    /// \brief Advanced example of state machine with smacc that shows multiple techniques
    ///  for the development of state machines
    struct SmMoveIt4
        : public smacc::SmaccStateMachineBase<SmMoveIt4, StInitialPosture>
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

} // namespace sm_moveit_4

//MODESTATES

//SUPERSTATES
#include "superstates/ss_pick_object.h"
#include "superstates/ss_place_object.h"

//STATES
#include "states/st_navigate_to_source_table.h"
#include "states/st_navigate_to_destiny_table.h"
#include "states/st_initial_posture.h"
#include "states/st_navigation_table_retreat.h"
#include "states/st_navigate_final_pose.h"
#include "states/st_final_raise_hands_up.h"
