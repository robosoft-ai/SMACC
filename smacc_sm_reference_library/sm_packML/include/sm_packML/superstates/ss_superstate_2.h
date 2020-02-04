namespace sm_packML
{
namespace SS2
{
namespace sm_packML
{
namespace ss2_states
{
//FORWARD DECLARATIONS OF ALL INNER STATES
class StiState4;
class StiState5;
class StiState6;
} // namespace ss1_states
} // namespace sm_packML

using namespace sm_packML::ss2_states;

// STATE DECLARATION
struct Ss2 : smacc::SmaccState<Ss2, MsRun, StiState4, sc::has_full_history>
{
public:
    using SmaccState::SmaccState;

// TRANSITION TABLE
    typedef mpl::list<

    Transition<EvLoopEnd<StiState5>, StIdle>
    
    >reactions;

// STATE VARIABLES
    static constexpr int total_iterations() { return 3; }
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
using SS = SS2::Ss2;

#include <sm_packML/states/ss_superstate_2/sti_state_4.h>
#include <sm_packML/states/ss_superstate_2/sti_state_5.h>
#include <sm_packML/states/ss_superstate_2/sti_state_6.h>

} // namespace SS1
} // namespace sm_packML
