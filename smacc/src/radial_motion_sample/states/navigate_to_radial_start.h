#pragma once
#include "radial_motion.h"
#include "plugins/move_base_to_goal.h"
#include "plugins/reel.h"
#include <non_rt_helper/DispenseModeGoal.h>
#include <boost/statechart/transition.hpp>

namespace NavigateToRadialStart
{
    using namespace smacc;

    struct NavigationOrthogonalLine;
    struct ReelOrthogonalLine;

    //--------------------------------------------
    /// State NavigateToRadialStart
    struct State : SmaccState< State, RadialMotionStateMachine,
                                mpl::list< NavigationOrthogonalLine, ReelOrthogonalLine > >
    {
        typedef sc::transition< EvStateFinished, RotateDegress> reactions;


        public:
        State(my_context ctx)
        :SmaccState<State, RadialMotionStateMachine,
                                mpl::list< NavigationOrthogonalLine, ReelOrthogonalLine > >(ctx)
        {
            ROS_INFO("Entering in NavigateToRadialStart State");
        }

        ~State()
        {
            ROS_INFO("Finishing NavigateToRadialStart state");
        }
    };

    //--------------------------------------------------
    // orthogonal line 0
    class MoveBase_MoveGoal_ActionClient;

    struct NavigationOrthogonalLine: public SmaccState<NavigationOrthogonalLine, State::orthogonal< 0 > , MoveBase_MoveGoal_ActionClient>
    {
        NavigationOrthogonalLine(my_context ctx)
            :SmaccState<NavigationOrthogonalLine, State::orthogonal< 0 > , MoveBase_MoveGoal_ActionClient>(ctx)
        {
            ROS_INFO("Entering in move_base orthogonal line");
        }

        ~NavigationOrthogonalLine ()
        {
            ROS_INFO("Finishing move base orthogonal line");
        }
    };

    struct MoveBase_MoveGoal_ActionClient: SmaccState<MoveBase_MoveGoal_ActionClient, NavigationOrthogonalLine >
    {
        typedef sc::custom_reaction< EvActionClientSuccess > reactions;

        public:

        MoveBase_MoveGoal_ActionClient(my_context ctx)
            : SmaccState<MoveBase_MoveGoal_ActionClient, NavigationOrthogonalLine >(ctx)
        {
            ROS_INFO("Entering MoveBase_MoveGoal_ActionClient");
            moveBaseClient_ = context<RadialMotionStateMachine >().requiresActionClient<smacc::SmaccMoveBaseActionClient>("move_base");

            smacc::SmaccMoveBaseActionClient::Goal goal;
            goal.target_pose.pose.position.x=10;
            goal.target_pose.pose.position.y=10;
            moveBaseClient_->sendGoal(goal);
        }

        sc::result react( const EvActionClientSuccess & ev )
        {
            if (ev.client == moveBaseClient_)
            {
                ROS_INFO("Received event to move base client");
                this->post_event(EvStateFinished());
                return this->terminate();
            }
        }

        ~MoveBase_MoveGoal_ActionClient()
        {
            ROS_INFO("Exiting move goal Action Client");
        }

        private:
        smacc::SmaccMoveBaseActionClient* moveBaseClient_;
    };

    //--------------------------------------------------
    // orthogonal line 1

    struct Reel_ActionClient;

    struct ReelOrthogonalLine: SmaccState<ReelOrthogonalLine, State::orthogonal< 1 > , Reel_ActionClient>
    {
        public:
            ReelOrthogonalLine(my_context ctx)
                :SmaccState<ReelOrthogonalLine, State::orthogonal< 1 > , Reel_ActionClient>(ctx)
            {
                ROS_INFO("Entering in reel orthogonal line");
            }

            ~ReelOrthogonalLine ()
            {
                ROS_INFO("Finishing reel orthogonal line");
            }
    };

    struct Reel_ActionClient: SmaccState<Reel_ActionClient, ReelOrthogonalLine >
    {
        typedef sc::custom_reaction< EvActionClientSuccess > reactions;

        public:
            Reel_ActionClient(my_context ctx)
                : SmaccState<Reel_ActionClient, ReelOrthogonalLine >(ctx)
            {
                ROS_INFO("Entering Reel_ActionClient");
                reelActionClient_ = context<RadialMotionStateMachine >().requiresActionClient<smacc::SmaccReelActionClient>("non_rt_helper");
                
                //dispense
                smacc::SmaccReelActionClient::Goal goal;
                goal.dispense_mode= smacc::SmaccReelActionClient::Goal::DISPENSE;
                reelActionClient_->sendGoal(goal);
            }

            sc::result react( const EvActionClientSuccess &  ev)
            {
                if (ev.client == reelActionClient_)
                {
                    ROS_INFO("Received event for reel client");
                    return this->terminate();
                }
            }

            ~Reel_ActionClient()
            {
                ROS_INFO("Exiting Reel_Action Client");
            }

        private:
            smacc::SmaccReelActionClient* reelActionClient_;
    };
}

