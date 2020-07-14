#include <smacc/smacc.h>

namespace sm_moveit_4
{
    namespace SS2
    {
        namespace sm_moveit_4
        {
            namespace place_states
            {
                //FORWARD DECLARATION OF INNER STATES
                class StMovePrePlacePose;
                class StPlaceApproach;
                class StOpenGripper;
                class StPlaceRetreat;
                class StNavigationPosture;
            } // namespace place_states
        }     // namespace sm_moveit_4

        using namespace sm_moveit_4::place_states;

        // STATE DECLARATION
        struct SsPlaceObject : smacc::SmaccState<SsPlaceObject, SmMoveIt4, StMovePrePlacePose>
        {
        public:
            using SmaccState::SmaccState;

            // TRANSITION TABLE
            typedef mpl::list<
                Transition<EvSequenceFinished<SS2::StNavigationPosture>, StNavigationTableRetreat, SUCCESS>>
                reactions;

            // STATE FUNCTIONS
            static void staticConfigure()
            {
            }

            void runtimeConfigure()
            {
            }
        };

        using SS = SsPlaceObject;

#include <sm_moveit_4/states/place_states/st_navigation_posture.h>
#include <sm_moveit_4/states/place_states/st_open_gripper.h>
#include <sm_moveit_4/states/place_states/st_place_approach.h>
#include <sm_moveit_4/states/place_states/st_place_retreat.h>
#include <sm_moveit_4/states/place_states/st_move_preplace_pose.h>

    } // namespace SS2
} // namespace sm_moveit_4