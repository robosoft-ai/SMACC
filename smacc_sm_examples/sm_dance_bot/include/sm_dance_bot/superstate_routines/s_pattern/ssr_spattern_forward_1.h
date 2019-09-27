struct SsrSPatternForward1: public smacc::SmaccState<SsrSPatternForward1,SS>
{
  using SmaccState::SmaccState;

  typedef sc::transition<EvActionSucceded<smacc::SmaccMoveBaseActionClient::Result>, SsrSPatternRotate2> reactions; 

  void onInitialize()
  {
    auto& superstate = this->context<SS>();
    
    this->configure<NavigationOrthogonal>(std::make_shared<SbNavigateForward>(superstate.pitch_lenght_meters));
    this->configure<ToolOrthogonal>(std::make_shared<SbToolStart>());    
  }
};