#include <smacc/smacc.h>
namespace sm_starcraft_ai
{
// STATE DECLARATION
class MsRun : public smacc::SmaccState<MsRun, SmStarcraftAI, StObserve>
{
public:
   using SmaccState::SmaccState;
};
} // namespace sm_starcraft_ai
