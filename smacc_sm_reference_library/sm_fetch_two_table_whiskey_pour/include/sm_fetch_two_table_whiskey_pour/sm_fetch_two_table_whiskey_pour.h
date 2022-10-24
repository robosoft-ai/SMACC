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
using namespace sm_fetch_two_table_whiskey_pour::cl_gripper;

// ORTHOGONALS
#include "orthogonals/or_gripper.h"
#include "orthogonals/or_arm.h"
#include "orthogonals/or_perception.h"
#include "orthogonals/or_navigation.h"

using namespace sm_fetch_two_table_whiskey_pour::cl_move_group_interface;

namespace sm_fetch_two_table_whiskey_pour
{
    //STATE FORWARD DECLARATIONS
    namespace SS1
    {
        class recovery_screw;
    } // namespace SS1

    namespace SS3
    {
        class SsRecoveryScrew;
    }

    namespace SS2
    {
        class SsPlaceObject;
    }

    class StInitialPosture;
    class StPouringPosture;
    class StPouringUndoPosture;
    class StFinalRaiseHandsUp;
    class StNavigateToSourceTable;
    class StMoveCartesianBottleToContainer;
    class StPlaceBottleBack;
    class StReleaseGripper;
    class StPlaceRetreat;
    class StNavigateToDestinyTable;
    class StPlaceGlassBack;
    class StRetreatBackwards;
    class StReleaseGlass;
    class StNavigationPosture;
    class StDecideDestinyTableAction;
    class StBringBottleBackNavigateSourceTable;
    class StBringBottleBackBackwardNavigation;

    //SUPERSTATE FORWARD DECLARATIONS

    // MODE STATES FORWARD DECLARATIONS
} // namespace sm_fetch_two_table_whiskey_pour

using namespace smacc;

namespace sm_fetch_two_table_whiskey_pour
{
    /// \brief Advanced example of state machine with smacc that shows multiple techniques
    ///  for the development of state machines
    struct SmFetchTwoTableWhiskeyPour
        : public smacc::SmaccStateMachineBase<SmFetchTwoTableWhiskeyPour, StNavigateToSourceTable>
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

} // namespace sm_fetch_two_table_whiskey_pour

//MODESTATES

//SUPERSTATES

#include "superstates/ss_recovery_screw.h"
#include "superstates/ss_pick_object.h"

#include "states/st_initial_posture.h"
#include "states/st_pouring_posture.h"
#include "states/st_move_cartesian_bottle_to_container.h"
#include "states/st_undo_pouring_posture.h"
#include "states/st_place_bottle_back.h"
#include "states/st_place_release_gripper.h"
#include "states/st_place_retreat.h"
#include "states/st_final_raise_hands_up.h"
#include "states/st_navigate_to_source_table.h"
#include "states/st_navigate_to_destiny_table.h"
#include "states/st_place_glass_back.h"
#include "states/st_retreat_backwards.h"
#include "states/st_release_glass.h"
#include "states/st_navigation_posture.h"
#include "states/st_decide_destiny_table_action.h"
#include "states/st_bring_bottle_back_backward_navigation.h"
#include "states/st_bring_bottle_back_navigate_source_table.h"
