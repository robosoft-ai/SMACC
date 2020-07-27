#include <smacc/smacc.h>
namespace sm_ridgeback_floor_coverage_dynamic_1
{
// STATE DECLARATION
class MsDanceBotRunMode : public smacc::SmaccState<MsDanceBotRunMode, SmRidgebackFloorCoverageDynamic1, StAcquireSensors>
{
public:
   using SmaccState::SmaccState;

// TRANSITION TABLE
   typedef mpl::list<
   
   Transition<EvGlobalError, MsDanceBotRecoveryMode> 
   
   >reactions;
};
}