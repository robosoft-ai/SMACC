#pragma once
#include "states/common.h"

namespace NavigateToRadialStart
{
    struct NavSubstates;
    struct ReelSubstates;

    //--------------------------------------------
    /// State NavigateToRadialStart
    struct State : SmaccState< State, SmaccStateMachine,
                                mpl::list< NavSubstates, ReelSubstates > >
    {
        typedef sc::custom_reaction< EvStateFinished > reactions;

        public:
        State(my_context ctx)
        :SmaccState<State, SmaccStateMachine,
                                mpl::list< NavSubstates, ReelSubstates > >(ctx)
        {
            ROS_INFO("Entering in NavigateToRadialStart State");
        }


        sc::result Focused::react( const EvShutterFull & )
        {
            return transit< NavigateToRadialStart >();
        }

        ~State()
        {
            ROS_INFO("Finishing NavigateToRadialStart state");
        }
    };

    //--------------------------------------------------
    // orthogonal line 0
    class MoveBase_MoveGoal_ActionClient;

    struct NavSubstates: sc::simple_state<NavSubstates, State::orthogonal< 0 > , MoveBase_MoveGoal_ActionClient>
    {
        NavSubstates()
        {
            ROS_INFO("Entering in move_base orthogonal line");
            //context< SmaccStateMachine >().registerActionClientRequest("move_base");
        }

        ~NavSubstates ()
        {
            ROS_INFO("Finishing move base orthogonal line");
        }
    };

    struct MoveBase_MoveGoal_ActionClient: sc::simple_state<
    MoveBase_MoveGoal_ActionClient, NavSubstates >
    {
        typedef sc::custom_reaction< EvActionClientSuccess > reactions;

        MoveBase_MoveGoal_ActionClient()
        {
            ROS_INFO("Entering MoveBase_MoveGoal_ActionClient");
        }

        sc::result react( const EvActionClientSuccess & )
        {
            ROS_INFO("Received event to move base client");
            return terminate();
        }

        ~MoveBase_MoveGoal_ActionClient()
        {
            ROS_INFO("Exiting move goal Action Client");
        }
    };

    //--------------------------------------------------
    // orthogonal line 1

    struct Reel_ActionClient;

    struct ReelSubstates: sc::simple_state<
    ReelSubstates, State::orthogonal< 1 > , Reel_ActionClient>
    {
        ReelSubstates()
        {
            ROS_INFO("Entering in reel orthogonal line");
        }

        ~ReelSubstates ()
        {
            ROS_INFO("Finishing reel orthogonal line");
        }
    };

    struct Reel_ActionClient: sc::simple_state<
    Reel_ActionClient, ReelSubstates >
    {
        typedef sc::custom_reaction< EvActionClientSuccess > reactions;

        Reel_ActionClient()
        {
            ROS_INFO("Entering Reel_ActionClient");
        }

        sc::result react( const EvActionClientSuccess & )
        {
            ROS_INFO("Received event to reel client");
            return terminate();
        }

        ~Reel_ActionClient()
        {
            ROS_INFO("Exiting Reel_Action Client");
        }
    };
}

