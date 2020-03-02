namespace sm_starcraft_ai
{
namespace SS3
{
namespace sm_starcraft_ai
{
namespace inner_states
{
//FORWARD DECLARATIONS OF ALL INNER STATES
class StiState1;
class StiState2;
class StiState3;
} // namespace ss1_states
} // namespace sm_starcraft_ai

using namespace sm_starcraft_ai::inner_states;

// STATE DECLARATION
struct Ss3 : smacc::SmaccState<Ss3, MsRun, StiState1, sc::has_full_history>
{
public:
    using SmaccState::SmaccState;

// TRANSITION TABLE
    typedef mpl::list<

    Transition<EvLoopEnd<StiState1>, StState2>
    
    >reactions;

// STATE VARIABLES
    static constexpr int total_iterations() { return 5; }
    int iteration_count = 0;

// STATE FUNCTIONS
    static void staticConfigure()
    {
    }

    void runtimeConfigure()
    {
    }
}; // namespace SS4

//forward declaration for the superstate
using SS = SS3::Ss3;

#include <sm_starcraft_ai/states/inner_states/sti_state_1.h>
#include <sm_starcraft_ai/states/inner_states/sti_state_2.h>
#include <sm_starcraft_ai/states/inner_states/sti_state_3.h>

} // namespace SS3
} // namespace sm_starcraft_ai
