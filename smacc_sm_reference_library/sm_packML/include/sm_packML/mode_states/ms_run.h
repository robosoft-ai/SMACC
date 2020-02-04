#include <smacc/smacc.h>
namespace sm_packML
{
// STATE DECLARATION
class MsRun : public smacc::SmaccState<MsRun, SmPackML, StIdle, sc::has_full_history>
{
public:
   using SmaccState::SmaccState;
};
} // namespace sm_packML