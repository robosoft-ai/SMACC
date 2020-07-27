#include <smacc/smacc.h>
namespace sm_ridgeback_floor_coverage_static_1
{
// STATE DECLARATION
class MsDanceBotRunMode : public smacc::SmaccState<MsDanceBotRunMode, SmRidgebackFloorCoverageStatic1, StAcquireSensors>
{
public:
   using SmaccState::SmaccState;

// TRANSITION TABLE
   typedef mpl::list<
   
   Transition<EvGlobalError, MsDanceBotRecoveryMode> 
   
   >reactions;
};
}