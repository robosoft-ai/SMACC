#include <smacc/smacc.h>
namespace sm_pr2_plugs{
// STATE DECLARATION
class MsRecharge : public smacc::SmaccState<MsRecharge, SmPR2Plugs, StProcessRechargeCommand, sc::has_full_history>
{
public:
   using SmaccState::SmaccState;
};
} // namespace sm_pr2_plugs