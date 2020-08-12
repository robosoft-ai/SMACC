#include <smacc/smacc.h>

namespace sm_moveit_wine_serve
{
    namespace SS1
    {
        namespace sm_moveit_wine_serve
        {
            namespace pick_states
            {
                //FORWARD DECLARATION OF INNER STATES
                class StCloseGripper;
                class StGraspApproach;
                class StGraspRetreat;
                class StMovePregraspPose;
                class StNavigationPosture;
            } // namespace pick_states
        }     // namespace sm_fetch_six_table_pick_n_sort_1

        using namespace sm_moveit_wine_serve::pick_states;

        // STATE DECLARATION
        struct SsPickObject : smacc::SmaccState<SsPickObject, SmFetchSixTablePickNSort1, StMovePregraspPose>
        {
        public:
            using SmaccState::SmaccState;

            // TRANSITION TABLE
            typedef mpl::list<

                Transition<EvSequenceFinished<SS1::StGraspRetreat>, StPouringPosture, SUCCESS>
            >
            reactions;

            // STATE FUNCTIONS
            static void staticConfigure()
            {
            }

            void runtimeConfigure()
            {
                ROS_INFO("picking object superstate");
            }
        };

        // FORWARD DECLARATION FOR THE SUPERSTATE
        using SS = SsPickObject;
#include <sm_moveit_wine_serve/states/pick_states/st_close_gripper.h>
#include <sm_moveit_wine_serve/states/pick_states/st_grasp_approach.h>
#include <sm_moveit_wine_serve/states/pick_states/st_grasp_retreat.h>
#include <sm_moveit_wine_serve/states/pick_states/st_move_pregrasp_pose.h>
#include <sm_moveit_wine_serve/states/pick_states/st_navigation_posture.h>

    } // namespace SS1
} // namespace sm_fetch_six_table_pick_n_sort_1