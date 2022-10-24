#pragma once
namespace sm_fetch_screw_loop_1
{
    namespace recovery_screw
    {
        struct StRecoveryScrewSecond : smacc::SmaccState<StRecoveryScrewSecond, SS>
        {
            using SmaccState::SmaccState;

            // TRANSITION TABLE
            typedef mpl::list<
                Transition<EvCbSuccess<CbMoveLastTrajectoryInitialState, OrArm>, StRecoveryScrewThird, SUCCESS>,
                Transition<EvCbFailure<CbMoveLastTrajectoryInitialState, OrArm>, StRecoveryScrewSecond, ABORT>>
                reactions;

            // STATE FUNCTIONS
            static void staticConfigure()
            {
                configure_orthogonal<OrArm, CbMoveLastTrajectoryInitialState>();
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
} // namespace sm_fetch_screw_loop_1
