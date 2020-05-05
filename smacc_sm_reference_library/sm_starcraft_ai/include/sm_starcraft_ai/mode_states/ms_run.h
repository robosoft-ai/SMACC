#include <smacc/smacc.h>
namespace sm_starcraft_ai
{
// STATE DECLARATION
class MsRun : public smacc::SmaccState<MsRun, SmStarcraftAI, StObserve, sc::has_full_history>
{
public:
   using SmaccState::SmaccState;
};
} // namespace sm_starcraft_ai