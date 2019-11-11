class MsDanceBotRecoveryMode : public smacc::SmaccState<MsDanceBotRecoveryMode, SmDanceBot>
{
public:
   using SmaccState::SmaccState;

   //typedef smacc::transition<EvGlobalError, sc::deep_history<MsDanceBotRunMode>> reactions;
   typedef smacc::transition<EvGlobalError, MsDanceBotRunMode> reactions;
};