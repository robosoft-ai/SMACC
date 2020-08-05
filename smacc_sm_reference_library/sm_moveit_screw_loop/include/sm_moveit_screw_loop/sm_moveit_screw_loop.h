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
using namespace sm_moveit_screw_loop::cl_gripper;

// ORTHOGONALS
#include "orthogonals/or_gripper.h"
#include "orthogonals/or_arm.h"
#include "orthogonals/or_perception.h"
#include "orthogonals/or_navigation.h"

using namespace sm_moveit_screw_loop::cl_move_group_interface;

namespace sm_moveit_screw_loop
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

    namespace SS3
    {
        class SsRecoveryScrew;
    }

    class StInitialPosture;
    class StSecondPosture;
    class StUndoIncorrectForward;
    class StNavigateToSourceTable;
    class StNavigateToDestinyTable;
    class StNavigationTableRetreat;
    class StNavigateFinalPose;
    class StFinalRaiseHandsUp;

    //SUPERSTATE FORWARD DECLARATIONS

    // MODE STATES FORWARD DECLARATIONS
} // namespace sm_moveit_screw_loop

using namespace smacc;

namespace sm_moveit_screw_loop
{
    /// \brief Advanced example of state machine with smacc that shows multiple techniques
    ///  for the development of state machines
    struct SmFetchSixTablePickNSort1
        : public smacc::SmaccStateMachineBase<SmFetchSixTablePickNSort1, StInitialPosture>
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

} // namespace sm_moveit_screw_loop

//MODESTATES

//SUPERSTATES
#include "superstates/ss_pick_object.h"
#include "superstates/ss_place_object.h"
#include "superstates/ss_recovery_screw.h"

//STATES
#include "states/st_navigate_to_source_table.h"
#include "states/st_navigate_to_destiny_table.h"
#include "states/st_initial_posture.h"
#include "states/st_second_posture.h"
#include "states/st_navigation_table_retreat.h"
#include "states/st_navigate_final_pose.h"
#include "states/st_final_raise_hands_up.h"
