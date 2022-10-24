namespace sm_starcraft_ai
{
namespace SS2
{
namespace sm_starcraft_ai
{
namespace build_inner_states
{
//FORWARD DECLARATIONS OF ALL INNER STATES
class StiBuild1;
class StiBuild2;
class StiBuild3;
} // namespace ss2_states
} // namespace sm_starcraft_ai

using namespace sm_starcraft_ai::build_inner_states;

// STATE DECLARATION
struct SsBuild : smacc::SmaccState<SsBuild, MsRun, StiBuild1>
{
public:
    using SmaccState::SmaccState;

// TRANSITION TABLE
    typedef mpl::list<

    Transition<EvLoopEnd<StiBuild1>, StObserve>

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
using SS = SS2::SsBuild;

#include <sm_starcraft_ai/states/build_inner_states/sti_build_1.h>
#include <sm_starcraft_ai/states/build_inner_states/sti_build_2.h>
#include <sm_starcraft_ai/states/build_inner_states/sti_build_3.h>

} // namespace SS2
} // namespace sm_starcraft_ai
