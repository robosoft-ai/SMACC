#include <smacc/smacc.h>
namespace sm_mtc_picknplace
{
// STATE DECLARATION
class MsWorkweek : public smacc::SmaccState<MsWorkweek, SmMTCPickNPlace, StMonday, sc::has_full_history>
{
public:
   using SmaccState::SmaccState;
};
} // namespace sm_mtc_picknplace