#pragma once

#include <smacc/smacc.h>
#include <tf/tf.h>
namespace sm_moveit_wine_serve
{
    static int count = 0;
    struct EvPlaceGlass : sc::event<EvPlaceGlass>
    {
    };
    struct EvServeWine : sc::event<EvServeWine>
    {
    };

    // STATE DECLARATION
    struct StDecideDestinyTableAction : smacc::SmaccState<StDecideDestinyTableAction, SmMoveitWineFetch>
    {
        using SmaccState::SmaccState;

        // TRANSITION TABLE
        typedef mpl::list<
            Transition<EvPlaceGlass, StPlaceGlassBack, SUCCESS /*SS2:SsPlaceObject*/ /*StPouringPosture*/>,
            Transition<EvServeWine, StPouringPosture> /*retry*/
            >
            reactions;

        // STATE FUNCTIONS
        static void staticConfigure()
        {
        }

        void runtimeConfigure()
        {
            if (count == 0)
            {
                this->postEvent<EvPlaceGlass>();
            }
            else
            {
                this->postEvent<EvServeWine>();
            }
        }

        void onExit(SUCCESS)
        {
            count++;
        }
    };
} // namespace sm_moveit_wine_serve