#include <smacc/smacc.h>
namespace sm_dance_bot
{
class MsDanceBotRecoveryMode : public smacc::SmaccState<MsDanceBotRecoveryMode, SmDanceBot>
{
public:
   using SmaccState::SmaccState;

   typedef smacc::Transition<EvGlobalError, sc::deep_history<StAcquireSensors>> reactions;
   // typedef smacc::Transition<EvGlobalError, MsDanceBotRunMode> reactions;
};
}