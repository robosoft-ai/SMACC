#include <smacc/smacc.h>
namespace sm_dance_bot_2
{
// STATE DECLARATION
class MsDanceBotRecoveryMode : public smacc::SmaccState<MsDanceBotRecoveryMode, SmDanceBot2>
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