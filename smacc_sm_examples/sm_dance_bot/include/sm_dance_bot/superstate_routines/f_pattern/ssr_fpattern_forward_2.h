struct SsrFPatternForward2: smacc::SmaccState<SsrFPatternForward2,SS>
{
  using SmaccState::SmaccState;

  typedef sc::transition<EvActionSucceded<smacc::SmaccMoveBaseActionClient::Result>, SsFPattern1> reactions; 

  void onInitialize()
  {
    auto& superstate = this->context<SS>();
    this->configure<NavigationOrthogonal>(std::make_shared<SbNavigateForward>(superstate.pitch_lenght_meters));
    this->configure<ToolOrthogonal>(std::make_shared<SbToolStop>());  
  }
};