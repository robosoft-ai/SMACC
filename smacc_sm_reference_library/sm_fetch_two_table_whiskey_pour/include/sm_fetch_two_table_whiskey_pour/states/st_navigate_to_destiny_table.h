#pragma once

#include <smacc/smacc.h>
#include <tf/tf.h>
namespace sm_fetch_two_table_whiskey_pour
{
    // STATE DECLARATION
    struct StNavigateToDestinyTable : smacc::SmaccState<StNavigateToDestinyTable, SmFetchTwoTableWhiskeyPour>
    {
        using SmaccState::SmaccState;

        // TRANSITION TABLE
        typedef mpl::list<

            Transition<EvCbSuccess<CbNavigateGlobalPosition, OrNavigation>, StDecideDestinyTableAction /*SS2::SsPlaceObject*/ /*StPouringPosture*/, SUCCESS>,
            Transition<EvCbFailure<CbNavigateGlobalPosition, OrNavigation>, StNavigateToDestinyTable, ABORT> /*retry*/
            >
            reactions;

        // STATE FUNCTIONS
        static void staticConfigure()
        {
            configure_orthogonal<OrNavigation, CbNavigateGlobalPosition>();
        }

        void runtimeConfigure()
        {
            auto navigateGlobalPosition = this->getOrthogonal<OrNavigation>()->getClientBehavior<CbNavigateGlobalPosition>();
            ClPerceptionSystem *perceptionSystem;
            navigateGlobalPosition->requiresClient(perceptionSystem);

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

            navigateGlobalPosition->setGoal(targetTablePose);
        }
    };
} // namespace sm_fetch_two_table_whiskey_pour