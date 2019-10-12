struct SsrRadialEndPoint: smacc::SmaccState<SsrRadialEndPoint,SS>
{
  using SmaccState::SmaccState;

  typedef sc::transition<EvActionSucceded<smacc::SmaccMoveBaseActionClient>, SsrRadialReturn> reactions; 

  void onInitialize()
  {
    auto& superstate = this->context<SS>();
    
    this->configure<NavigationOrthogonal>(std::make_shared<SbNavigateForward>(superstate.ray_length_meters));
    this->configure<ToolOrthogonal>(std::make_shared<SbToolStop>());    
  }
  
};
