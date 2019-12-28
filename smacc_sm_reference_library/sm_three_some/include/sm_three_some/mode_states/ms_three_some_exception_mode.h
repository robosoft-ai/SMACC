#include <smacc/smacc.h>
namespace sm_three_some
{
class MsTheeSomeExceptionMode : public smacc::SmaccState<MsTheeSomeExceptionMode, SmThreeSome, StState1>
{
public:
   using SmaccState::SmaccState;
};
} // namespace sm_three_some