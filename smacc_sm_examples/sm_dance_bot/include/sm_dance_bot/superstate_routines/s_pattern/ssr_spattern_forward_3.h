struct SsrSPatternForward3: public smacc::SmaccState<SsrSPatternForward3,SS>
{
  using SmaccState::SmaccState;

  typedef sc::transition<EvActionSucceded<smacc::SmaccMoveBaseActionClient::Result>, SsrSPatternRotate4> reactions; 

  void onInitialize()
  {
    auto& superstate = this->context<SS>();
    
    this->configure<NavigationOrthogonal>(std::make_shared<SbNavigateForward>(superstate.pitch2_lenght_meters));
    this->configure<ToolOrthogonal>(std::make_shared<SbToolStart>());    
  }
};