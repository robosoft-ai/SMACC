namespace sm_starcraft_ai
{
namespace SS1
{
namespace sm_starcraft_ai
{
namespace move_inner_states
{
//FORWARD DECLARATIONS OF ALL INNER STATES
class StiMove1;
class StiMove2;
class StiMove3;
} // namespace ss1_states
} // namespace sm_starcraft_ai

using namespace sm_starcraft_ai::move_inner_states;

// STATE DECLARATION
struct SsMove : smacc::SmaccState<SsMove, MsRun, StiMove1>
{
public:
    using SmaccState::SmaccState;

// TRANSITION TABLE
    typedef mpl::list<

    Transition<EvLoopEnd<StiMove1>, StObserve>

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
}; // namespace SS1

//forward declaration for the superstate
using SS = SS1::SsMove;

#include <sm_starcraft_ai/states/move_inner_states/sti_move_1.h>
#include <sm_starcraft_ai/states/move_inner_states/sti_move_2.h>
#include <sm_starcraft_ai/states/move_inner_states/sti_move_3.h>

} // namespace SS1
} // namespace sm_starcraft_ai
