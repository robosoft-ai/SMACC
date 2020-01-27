#include <smacc/smacc.h>
namespace sm_dance_bot_3
{
// STATE DECLARATION
class MsDanceBotRecoveryMode : public smacc::SmaccState<MsDanceBotRecoveryMode, SmDanceBot>
{
public:
   using SmaccState::SmaccState;

// TRANSITION TABLE
   typedef mpl::list<
   
   Transition<EvGlobalError, sc::deep_history<StAcquireSensors>> 
   
   >reactions;
   // typedef Transition<EvGlobalError, MsDanceBotRunMode> reactions;
};
}