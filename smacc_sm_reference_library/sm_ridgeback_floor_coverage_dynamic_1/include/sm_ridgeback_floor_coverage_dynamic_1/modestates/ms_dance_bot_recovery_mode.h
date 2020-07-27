#include <smacc/smacc.h>
namespace sm_ridgeback_floor_coverage_dynamic_1
{
// STATE DECLARATION
class MsDanceBotRecoveryMode : public smacc::SmaccState<MsDanceBotRecoveryMode, SmRidgebackFloorCoverageDynamic1>
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