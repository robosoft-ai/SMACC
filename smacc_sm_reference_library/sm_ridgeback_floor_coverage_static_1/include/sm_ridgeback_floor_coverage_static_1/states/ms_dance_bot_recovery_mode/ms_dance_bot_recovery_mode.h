#include <smacc/smacc.h>
namespace sm_ridgeback_floor_coverage_static_1
{
// STATE DECLARATION
class MsDanceBotRecoveryMode : public smacc::SmaccState<MsDanceBotRecoveryMode, SmRidgebackFloorCoverageStatic1>
{
public:
   using SmaccState::SmaccState;

// TRANSITION TABLE
   typedef mpl::list< 
   
   Transition<EvGlobalError, sc::deep_history<MsDanceBotRunMode::LastDeepState>> 
   
   >reactions;
   // typedef Transition<EvGlobalError, MsDanceBotRunMode> reactions;
};
}