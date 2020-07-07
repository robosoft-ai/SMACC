#include <smacc/smacc.h>
#include <tf/tf.h>
namespace sm_moveit_4
{
    // STATE DECLARATION
    struct StNavigateToDestinyTable : smacc::SmaccState<StNavigateToDestinyTable, SmMoveIt4>
    {
        using SmaccState::SmaccState;

        // TRANSITION TABLE
        typedef mpl::list<

            Transition<EvActionSucceeded<ClMoveBaseZ, OrNavigation>, SS2::SsPlaceObject, SUCCESS>,
            Transition<EvActionAborted<ClMoveBaseZ, OrNavigation>, StNavigateToDestinyTable, ABORT> /*retry*/
            >
            reactions;

        // STATE FUNCTIONS
        static void staticConfigure()
        {
            configure_orthogonal_fn<OrNavigation, CbNavigateGlobalPosition>(
                [](auto &navigateGlobalPosition) {
                    ClPerceptionSystem *perceptionSystem;
                    navigateGlobalPosition.requiresClient(perceptionSystem);

                    geometry_msgs::PoseStamped nextCubePose;
                    perceptionSystem->decidePickCubePose(nextCubePose);
                    auto targetTablePose = perceptionSystem->getTargetTablePose().pose;

                    if (targetTablePose.position.x > 0)
                    {
                        targetTablePose.position.x -= 0.85;
                        
                    }
                    else
                    {
                        targetTablePose.position.x += 0.85;
                        auto quat = tf::createQuaternionFromYaw(M_PI);
                        tf::quaternionTFToMsg(quat, targetTablePose.orientation);
                    }

                    // align with the cube in the y axis
                    // targetTablePose.position.y = nextCubePose.pose.position.y;

                    navigateGlobalPosition.setGoal(targetTablePose);
                });
        }

        void runtimeConfigure()
        {
        }
    };
} // namespace sm_moveit_4