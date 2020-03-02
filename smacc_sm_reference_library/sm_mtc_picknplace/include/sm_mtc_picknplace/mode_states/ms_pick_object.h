#include <smacc/smacc.h>
namespace sm_mtc_picknplace
{
// STATE DECLARATION
class MsPickObject : public smacc::SmaccState<MsPickObject, SmMTCPickNPlace, StCloseHand, sc::has_full_history>
{
public:
   using SmaccState::SmaccState;
};
} // namespace sm_mtc_picknplace