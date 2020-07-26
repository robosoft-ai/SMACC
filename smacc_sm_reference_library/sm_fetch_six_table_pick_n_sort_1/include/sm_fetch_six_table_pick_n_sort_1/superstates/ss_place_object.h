#include <smacc/smacc.h>

namespace sm_fetch_six_table_pick_n_sort_1
{
    namespace SS2
    {
        namespace sm_fetch_six_table_pick_n_sort_1
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
        }     // namespace sm_fetch_six_table_pick_n_sort_1

        using namespace sm_fetch_six_table_pick_n_sort_1::place_states;

        // STATE DECLARATION
        struct SsPlaceObject : smacc::SmaccState<SsPlaceObject, SmFetchSixTablePickNSort1, StMovePrePlacePose>
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

#include <sm_fetch_six_table_pick_n_sort_1/states/place_states/st_navigation_posture.h>
#include <sm_fetch_six_table_pick_n_sort_1/states/place_states/st_open_gripper.h>
#include <sm_fetch_six_table_pick_n_sort_1/states/place_states/st_place_approach.h>
#include <sm_fetch_six_table_pick_n_sort_1/states/place_states/st_place_retreat.h>
#include <sm_fetch_six_table_pick_n_sort_1/states/place_states/st_move_preplace_pose.h>

    } // namespace SS2
} // namespace sm_fetch_six_table_pick_n_sort_1