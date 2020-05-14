namespace sm_respira_1
{
namespace SS3
{
namespace sm_respira_1
{
namespace pc_cycle_inner_states
{
//FORWARD DECLARATIONS OF ALL INNER STATES
class StiPCCycleLoop;
class StiPCCycleInspire;
class StiPCCyclePlateau;
class StiPCCycleExpire;
class StiPCCycleDwell;
} // namespace ss1_states
} // namespace sm_respira_1

using namespace sm_respira_1::pc_cycle_inner_states;

// STATE DECLARATION
struct SsPCCycle : smacc::SmaccState<SsPCCycle, MsRun, StiPCCycleLoop, sc::has_full_history>
{
public:
    using SmaccState::SmaccState;

// TRANSITION TABLE
    typedef mpl::list<

    Transition<EvLoopEnd<StiPCCycleLoop>, StObserve>
    
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
using SS = SS3::SsPCCycle;

#include <sm_respira_1/states/pc_cycle_inner_states/sti_pc_cycle_loop.h>
#include <sm_respira_1/states/pc_cycle_inner_states/sti_pc_cycle_inspire.h>
#include <sm_respira_1/states/pc_cycle_inner_states/sti_pc_cycle_plateau.h>
#include <sm_respira_1/states/pc_cycle_inner_states/sti_pc_cycle_expire.h>
#include <sm_respira_1/states/pc_cycle_inner_states/sti_pc_cycle_dwell.h>

} // namespace SS3
} // namespace sm_respira_1
