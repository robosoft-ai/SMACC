#include <smacc/smacc.h>

namespace sm_moveit_4
{
    namespace SS1
    {
        namespace sm_moveit_4
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
        }     // namespace sm_moveit_4

        using namespace sm_moveit_4::pick_states;

        // STATE DECLARATION
        struct SsPickObject : smacc::SmaccState<SsPickObject, SmMoveIt4, StMovePregraspPose>
        {
        public:
            using SmaccState::SmaccState;

            // TRANSITION TABLE
            typedef mpl::list<

                Transition<EvSequenceFinished<SS1::StNavigationPosture>, StNavigateToDestinyTable, SUCCESS>>
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
#include <sm_moveit_4/states/pick_states/st_close_gripper.h>
#include <sm_moveit_4/states/pick_states/st_grasp_approach.h>
#include <sm_moveit_4/states/pick_states/st_grasp_retreat.h>
#include <sm_moveit_4/states/pick_states/st_move_pregrasp_pose.h>
#include <sm_moveit_4/states/pick_states/st_navigation_posture.h>

    } // namespace SS1
} // namespace sm_moveit_4