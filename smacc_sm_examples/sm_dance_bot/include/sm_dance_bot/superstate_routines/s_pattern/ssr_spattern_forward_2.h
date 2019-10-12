struct SsrSPatternForward2: public smacc::SmaccState<SsrSPatternForward2,SS>
{
  using SmaccState::SmaccState;

  typedef sc::transition<EvActionSucceded<smacc::SmaccMoveBaseActionClient>, SsrSPatternRotate3> reactions; 

  void onInitialize()
  {
    auto& superstate = this->context<SS>();
    
    this->configure<NavigationOrthogonal>(std::make_shared<SbNavigateForward>(superstate.pitch1_lenght_meters));
    this->configure<ToolOrthogonal>(std::make_shared<SbToolStart>());    
  }
};