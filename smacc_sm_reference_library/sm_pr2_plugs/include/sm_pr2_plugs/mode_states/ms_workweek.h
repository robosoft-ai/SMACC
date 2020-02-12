#include <smacc/smacc.h>
namespace sm_pr2_plugs{
// STATE DECLARATION
class MsWorkweek : public smacc::SmaccState<MsWorkweek, SmPR2Plugs, StMonday, sc::has_full_history>
{
public:
   using SmaccState::SmaccState;
};
} // namespace sm_pr2_plugs