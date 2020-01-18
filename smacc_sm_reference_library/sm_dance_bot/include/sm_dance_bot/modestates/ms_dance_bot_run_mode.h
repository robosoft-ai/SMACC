#include <smacc/smacc.h>
namespace sm_dance_bot
{
class MsDanceBotRunMode : public smacc::SmaccState<MsDanceBotRunMode, SmDanceBot, StAcquireSensors>
{
public:
   using SmaccState::SmaccState;

   typedef Transition<EvGlobalError, MsDanceBotRecoveryMode> reactions;
};
}