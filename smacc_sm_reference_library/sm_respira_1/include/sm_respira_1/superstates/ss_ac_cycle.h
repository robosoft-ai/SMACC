namespace sm_respira_1
{
namespace SS1
{
namespace sm_respira_1
{
namespace ac_cycle_inner_states
{
//FORWARD DECLARATIONS OF ALL INNER STATES
class StiACCycleLoop;
class StiACCycleInspire;
class StiACCyclePlateau;
class StiACCycleExpire;
class StiACCycleDwell;
} // namespace ss1_states
} // namespace sm_respira_1

using namespace sm_respira_1::ac_cycle_inner_states;

// STATE DECLARATION
struct SsACCycle : smacc::SmaccState<SsACCycle, MsRun, StiACCycleLoop, sc::has_full_history>
{
public:
    using SmaccState::SmaccState;

// TRANSITION TABLE
    typedef mpl::list<

    Transition<EvLoopEnd<StiACCycleLoop>, StObserve>
    
    >reactions;

// STATE VARIABLES
    static constexpr int total_iterations() { return 1000; }
    int iteration_count = 0;

// STATE FUNCTIONS
    static void staticConfigure()
    {
    }

    void runtimeConfigure()
    {
    }
}; // namespace SS1

//forward declaration for the superstate
using SS = SS1::SsACCycle;

#include <sm_respira_1/states/ac_cycle_inner_states/sti_ac_cycle_loop.h>
#include <sm_respira_1/states/ac_cycle_inner_states/sti_ac_cycle_inspire.h>
#include <sm_respira_1/states/ac_cycle_inner_states/sti_ac_cycle_plateau.h>
#include <sm_respira_1/states/ac_cycle_inner_states/sti_ac_cycle_expire.h>
#include <sm_respira_1/states/ac_cycle_inner_states/sti_ac_cycle_dwell.h>

} // namespace SS1
} // namespace sm_respira_1
