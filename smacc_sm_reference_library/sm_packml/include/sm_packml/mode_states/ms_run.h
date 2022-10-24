#include <smacc/smacc.h>
namespace sm_packml
{
// STATE DECLARATION
class MsRun : public smacc::SmaccState<MsRun, SmPackML, StStarting>
{
public:
   using SmaccState::SmaccState;
};
} // namespace sm_packml
