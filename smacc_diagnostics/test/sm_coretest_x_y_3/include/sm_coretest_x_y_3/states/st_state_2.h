#include <smacc/smacc.h>

namespace sm_coretest_x_y_3
{
// STATE DECLARATION

extern int counter;

struct State2 : smacc::SmaccState<State2, SmCoreTestXY3>
{
    using SmaccState::SmaccState;

    // TRANSITION TABLE
    typedef mpl::list<

        Transition<AutomaticTransitionEvent, State1, SUCCESS>>
        reactions;

    // STATE FUNCTIONS
    static void staticConfigure()
    {
    }

    void onEntry()
    {
        counter++;
        this->postEvent<AutomaticTransitionEvent>();
    }
};
}