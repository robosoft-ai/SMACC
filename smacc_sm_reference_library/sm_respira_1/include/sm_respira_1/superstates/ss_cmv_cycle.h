namespace sm_respira_1
{
namespace SS2
{
namespace sm_respira_1
{
namespace cmv_cycle_inner_states
{
//FORWARD DECLARATIONS OF ALL INNER STATES
class StiPSCycleLoop;
class StiPSCycleInspire;
class StiPSCyclePlateau;
class StiPSCycleExpire;
class StiPSCycleDwell;
} // namespace ss2_states
} // namespace sm_respira_1

using namespace sm_respira_1::cmv_cycle_inner_states;

// STATE DECLARATION
struct SsCMVCycle : smacc::SmaccState<SsCMVCycle, MsRun, StiPSCycleLoop, sc::has_full_history>
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
}; // namespace SS4

//forward declaration for the superstate
using SS = SS2::SsCMVCycle;

#include <sm_respira_1/states/cmv_cycle_inner_states/sti_cmv_cycle_loop.h>
#include <sm_respira_1/states/cmv_cycle_inner_states/sti_cmv_cycle_inspire.h>
#include <sm_respira_1/states/cmv_cycle_inner_states/sti_cmv_cycle_plateau.h>
#include <sm_respira_1/states/cmv_cycle_inner_states/sti_cmv_cycle_expire.h>
#include <sm_respira_1/states/cmv_cycle_inner_states/sti_cmv_cycle_dwell.h>

} // namespace SS2
} // namespace sm_respira_1
