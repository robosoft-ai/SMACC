#include <smacc/smacc.h>
namespace sm_dance_bot
{
class MsDanceBotRecoveryMode : public smacc::SmaccState<MsDanceBotRecoveryMode, SmDanceBot>
{
public:
   using SmaccState::SmaccState;

   typedef Transition<EvGlobalError, sc::deep_history<StAcquireSensors>> reactions;
   // typedef Transition<EvGlobalError, MsDanceBotRunMode> reactions;
};
}