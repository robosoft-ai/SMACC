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
    class MoveBaseSubState;

    struct NavigationOrthogonalLine: public SmaccState<NavigationOrthogonalLine, State::orthogonal< 0 > , MoveBaseSubState>
    {
        NavigationOrthogonalLine(my_context ctx)
            :SmaccState<NavigationOrthogonalLine, State::orthogonal< 0 > , MoveBaseSubState>(ctx)
        {
            ROS_INFO("Entering in move_base orthogonal line");
        }

        ~NavigationOrthogonalLine ()
        {
            ROS_INFO("Finishing move base orthogonal line");
        }
    };

    //--------------------------------------------------

    struct MoveBaseSubState: SmaccState<MoveBaseSubState, NavigationOrthogonalLine >
    {
        typedef sc::custom_reaction< EvActionClientSuccess > reactions;

        public:

        MoveBaseSubState(my_context ctx)
            : SmaccState<MoveBaseSubState, NavigationOrthogonalLine >(ctx)
        {
            ROS_INFO("Entering MoveBaseSubState");
            moveBaseClient_ = context<RadialMotionStateMachine >().requiresActionClient<smacc::SmaccMoveBaseActionClient>("move_base");
            
            moveBaseClient_->waitForActionServer();

            smacc::SmaccMoveBaseActionClient::Goal goal;
            goal.target_pose.header.frame_id="/odom";
            goal.target_pose.header.stamp=ros::Time::now();
            
            goal.target_pose.pose.position.x=3;
            goal.target_pose.pose.position.y=0;
            goal.target_pose.pose.orientation.w=1;
            moveBaseClient_->sendGoal(goal);
        }

        sc::result react( const EvActionClientSuccess & ev )
        {
            if (ev.client == moveBaseClient_ && ev.getResult()==actionlib::SimpleClientGoalState::PENDING)
            {
                ROS_INFO("Received event to move base client");
                this->post_event(EvStateFinished());
                return this->terminate();
            }
        }

        ~MoveBaseSubState()
        {
            ROS_INFO("Exiting move goal Action Client");
        }

        private:
        smacc::SmaccMoveBaseActionClient* moveBaseClient_;
    };

    //--------------------------------------------------
    // orthogonal line 1

    struct ReelSubState;

    struct ReelOrthogonalLine: SmaccState<ReelOrthogonalLine, State::orthogonal< 1 > , ReelSubState>
    {
        public:
            ReelOrthogonalLine(my_context ctx)
                :SmaccState<ReelOrthogonalLine, State::orthogonal< 1 > , ReelSubState>(ctx)
            {
                ROS_INFO("Entering in reel orthogonal line");
            }

            ~ReelOrthogonalLine ()
            {
                ROS_INFO("Finishing reel orthogonal line");
            }
    };

    //--------------------------------------------------
    struct ReelSubState: SmaccState<ReelSubState, ReelOrthogonalLine >
    {
        typedef sc::custom_reaction< EvActionClientSuccess > reactions;

        public:
            ReelSubState(my_context ctx)
                : SmaccState<ReelSubState, ReelOrthogonalLine >(ctx)
            {
                ROS_INFO("Entering ReelSubState");
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

            ~ReelSubState()
            {
                ROS_INFO("Exiting Reel_Action Client");
            }

        private:
            smacc::SmaccReelActionClient* reelActionClient_;
    };
}

