#pragma once
namespace sm_fetch_screw_loop_1
{
    namespace recovery_screw
    {
        struct StRecoveryScrewThird : smacc::SmaccState<StRecoveryScrewThird, SS>
        {
            using SmaccState::SmaccState;

            // TRANSITION TABLE
            typedef mpl::list<
                Transition<EvCbFailure<CbExecuteLastTrajectory, OrArm>, StRecoveryScrewThird, ABORT>>
                reactions;

            // STATE FUNCTIONS
            static void staticConfigure()
            {
                configure_orthogonal<OrArm, CbExecuteLastTrajectory>();
                configure_orthogonal<OrGripper, CbCloseGripper>();
            }

            void runtimeConfigure()
            {
                CbExecuteLastTrajectory *cb = this->getOrthogonal<OrArm>()->getClientBehavior<CbExecuteLastTrajectory>();
                //cb->onSuccess(&StRecoveryScrewInitial::throwSequenceFinishedEvent, this);
                cb->onSuccess(&StRecoveryScrewThird::onSuccess, this);
            }

            void onSuccess()
            {
                this->throwSequenceFinishedEvent();
            }


            void onExit()
            {
            }
        };
    } // namespace recovery_screw
} // namespace sm_fetch_screw_loop_1