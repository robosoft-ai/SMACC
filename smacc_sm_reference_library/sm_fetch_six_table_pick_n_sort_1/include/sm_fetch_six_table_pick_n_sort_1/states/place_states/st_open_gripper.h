#pragma once
namespace sm_fetch_six_table_pick_n_sort_1
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
                configure_orthogonal<OrGripper, CbDetachObject>();
            }

            void onEntry()
            {
                ros::WallDuration(2).sleep();
            }

            void onExit()
            {
                ros::WallDuration(2).sleep();
            }
        };
    } // namespace place_states
} // namespace sm_fetch_six_table_pick_n_sort_1
