#pragma once
namespace sm_moveit_4
{
    namespace place_states
    {
        // STATE DECLARATION
        struct StOpenGripper : smacc::SmaccState<StOpenGripper, SS>
        {
            using SmaccState::SmaccState;

            // TRANSITION TABLE
            typedef mpl::list<
                Transition<EvActionSucceeded<ClGripper, OrGripper>, StPlaceRetreat>>
                reactions;

            // STATE FUNCTIONS
            static void staticConfigure()
            {
                configure_orthogonal<OrGripper, CbOpenGripper>();
            }

            void onEntry()
            {
                ros::WallDuration(2).sleep();
            }

            void onExit()
            {
                ClMoveGroup *moveGroupClient;
                this->requiresClient(moveGroupClient);

                auto &planningSceneInterface = moveGroupClient->planningSceneInterface;

                moveGroupClient->moveGroupClientInterface.detachObject("collisioncube");
                planningSceneInterface.removeCollisionObjects({"collisioncube"});

                ros::WallDuration(2).sleep();
            }
        };
    } // namespace place_states
} // namespace sm_moveit_4