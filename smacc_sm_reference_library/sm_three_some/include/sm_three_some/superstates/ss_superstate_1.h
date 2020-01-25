namespace sm_three_some
{
namespace SS1
{

namespace sm_three_some
{
namespace ss1_states
{
//HERE WE MAKE FORWARD DECLARATIONS OF ALL SUBSTATE ROUTINES
class StiState1;
class StiState2;
class StiState3;
} // namespace ss1_states
} // namespace sm_three_some

using namespace sm_three_some::ss1_states;

struct Ss1 : smacc::SmaccState<Ss1, MsRun, StiState1, sc::has_full_history>
{
public:
    using SmaccState::SmaccState;

    typedef mpl::list<
        // Transition<EvSuperstateFinish<StiState3>, StState1>

        // Keyboard events
        Transition<EvLoopEnd<StiState1>, StState1>>
        reactions;

    static constexpr int total_iterations() { return 3; }
    int iteration_count = 0;

    static void staticConfigure()
    {
    }

    void runtimeConfigure()
    {
    }
}; // namespace SS4

//forward declaration for the superstate
using SS = SS1::Ss1;

#include <sm_three_some/states/ss_superstate_1/sti_state_1.h>
#include <sm_three_some/states/ss_superstate_1/sti_state_2.h>
#include <sm_three_some/states/ss_superstate_1/sti_state_3.h>

} // namespace SS1
} // namespace sm_three_some
