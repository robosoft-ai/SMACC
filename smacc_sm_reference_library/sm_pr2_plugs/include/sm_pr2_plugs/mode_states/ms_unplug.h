#include <smacc/smacc.h>
namespace sm_pr2_plugs{
// STATE DECLARATION
class MsUnplug : public smacc::SmaccState<MsUnplug, SmPR2Plugs, StUnplug>
{
public:
   using SmaccState::SmaccState;
};
} // namespace sm_pr2_plugs