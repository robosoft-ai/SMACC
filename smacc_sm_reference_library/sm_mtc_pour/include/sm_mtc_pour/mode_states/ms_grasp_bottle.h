#include <smacc/smacc.h>
namespace sm_mtc_pour
{
// STATE DECLARATION
class MsGraspBottle : public smacc::SmaccState<MsGraspBottle, SmMTCPour, StAllowGripperObjectCollision, sc::has_full_history>
{
public:
   using SmaccState::SmaccState;
};
} // namespace sm_mtc_pour