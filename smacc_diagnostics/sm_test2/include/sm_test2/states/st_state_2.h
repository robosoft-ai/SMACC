#include <smacc/smacc.h>

namespace sm_test2
{
// STATE DECLARATION

extern int counter;

struct State2 : smacc::SmaccState<State2, SmTest2>
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