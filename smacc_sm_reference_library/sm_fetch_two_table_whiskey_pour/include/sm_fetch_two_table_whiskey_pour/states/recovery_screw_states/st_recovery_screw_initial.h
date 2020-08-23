#pragma once
namespace sm_fetch_two_table_whiskey_pour
{
    namespace recovery_screw
    {
        struct StRecoveryScrewInitial : smacc::SmaccState<StRecoveryScrewInitial, SS>
        {
            using SmaccState::SmaccState;

            // TRANSITION TABLE
            typedef mpl::list<
                Transition<EvCbSuccess<CbMoveCartesianRelative, OrArm>, StRecoveryScrewSecond, SUCCESS>,
                Transition<EvCbFailure<CbMoveCartesianRelative, OrArm>, StRecoveryScrewInitial, ABORT>>
                reactions;

            // STATE FUNCTIONS
            static void staticConfigure()
            {
                geometry_msgs::Vector3 offset;
                offset.x = -0.15;
                configure_orthogonal<OrArm, CbMoveCartesianRelative>(offset);
                configure_orthogonal<OrGripper, CbOpenGripper>();
            }

            void runtimeConfigure()
            {
            }


            void onExit()
            {
                ros::Duration(1.0).sleep();
            }
        };
    } // namespace recovery_screw
} // namespace sm_fetch_two_table_whiskey_pour