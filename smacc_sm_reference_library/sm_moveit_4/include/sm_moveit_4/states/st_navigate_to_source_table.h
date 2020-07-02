#include <smacc/smacc.h>
namespace sm_moveit_4
{
    // STATE DECLARATION
    struct StNavigateToSourceTable : smacc::SmaccState<StNavigateToSourceTable, SmMoveIt4>
    {
        using SmaccState::SmaccState;

        // TRANSITION TABLE
        typedef mpl::list<

            // Transition<EvActionSucceeded<ClMoveBaseZ, OrNavigation>, StRotate180, SUCCESS>
            Transition<EvActionSucceeded<ClMoveBaseZ, OrNavigation>, SS1::SsPickObject, SUCCESS>,
            Transition<EvActionAborted<ClMoveBaseZ, OrNavigation>, StNavigateToSourceTable, ABORT>>
            reactions;

        // STATE FUNCTIONS
        static void staticConfigure()
        {
            configure_orthogonal_fn<OrNavigation, CbNavigateGlobalPosition>
            (
                [](auto& navigateGlobalPosition, auto& state)
                {
                    ClPerceptionSystem* perceptionSystem;
                    state.requiresClient(perceptionSystem);
                    
                    auto mainTablePose = perceptionSystem->getMainTablePose().pose;
                    mainTablePose.position.x -= 1.2;

                    navigateGlobalPosition.setGoal(mainTablePose);
                }
            );
        }

        void runtimeConfigure()
        {
            ROS_INFO("runtime");
        }

        void OnEntry()
        {
            ROS_INFO("state on entry");
        }
    };
} // namespace sm_moveit_4