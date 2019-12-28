#include <smacc/smacc.h>
namespace sm_three_some
{
class MsThreeSomeRunMode : public smacc::SmaccState<MsThreeSomeRunMode, SmThreeSome, StState1>
{
public:
   using SmaccState::SmaccState;
};
} // namespace sm_three_some