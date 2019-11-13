class MsDanceBotRunMode: public smacc::SmaccState<MsDanceBotRunMode, SmDanceBot, StAcquireSensors>
{
   public:
   using SmaccState::SmaccState;

   typedef smacc::transition<EvGlobalError, MsDanceBotRecoveryMode> reactions;

};