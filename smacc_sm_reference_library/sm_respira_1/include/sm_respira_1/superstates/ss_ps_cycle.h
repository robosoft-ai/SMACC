namespace sm_respira_1
{
namespace SS4
{
namespace sm_respira_1
{
namespace ps_cycle_inner_states
{
//FORWARD DECLARATIONS OF ALL INNER STATES
class StiPSCycleLoop;
class StiPSCycleInspire;
class StiPSCyclePlateau;
class StiPSCycleExpire;
class StiPSCycleDwell;
} // namespace ss1_states
} // namespace sm_respira_1

using namespace sm_respira_1::ps_cycle_inner_states;

// STATE DECLARATION
struct SsPSCycle : smacc::SmaccState<SsPSCycle, MsRun, StiPSCycleLoop, sc::has_full_history>
{
public:
    using SmaccState::SmaccState;

// TRANSITION TABLE
    typedef mpl::list<

    Transition<EvLoopEnd<StiPSCycleLoop>, StObserve>
    
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
using SS = SS4::SsPSCycle;

#include <sm_respira_1/states/ps_cycle_inner_states/sti_ps_cycle_loop.h>
#include <sm_respira_1/states/ps_cycle_inner_states/sti_ps_cycle_inspire.h>
#include <sm_respira_1/states/ps_cycle_inner_states/sti_ps_cycle_plateau.h>
#include <sm_respira_1/states/ps_cycle_inner_states/sti_ps_cycle_expire.h>
#include <sm_respira_1/states/ps_cycle_inner_states/sti_ps_cycle_dwell.h>

} // namespace SS1
} // namespace sm_respira_1
