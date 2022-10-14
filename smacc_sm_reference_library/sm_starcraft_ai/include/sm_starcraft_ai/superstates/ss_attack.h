namespace sm_starcraft_ai
{
namespace SS3
{
namespace sm_starcraft_ai
{
namespace attack_inner_states
{
//FORWARD DECLARATIONS OF ALL INNER STATES
class StiAttack1;
class StiAttack2;
class StiAttack3;
} // namespace ss1_states
} // namespace sm_starcraft_ai

using namespace sm_starcraft_ai::attack_inner_states;

// STATE DECLARATION
struct SsAttack : smacc::SmaccState<SsAttack, MsRun, StiAttack1>
{
public:
    using SmaccState::SmaccState;

// TRANSITION TABLE
    typedef mpl::list<

    Transition<EvLoopEnd<StiAttack1>, StObserve>

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
using SS = SS3::SsAttack;

#include <sm_starcraft_ai/states/attack_inner_states/sti_attack_1.h>
#include <sm_starcraft_ai/states/attack_inner_states/sti_attack_2.h>
#include <sm_starcraft_ai/states/attack_inner_states/sti_attack_3.h>

} // namespace SS3
} // namespace sm_starcraft_ai
