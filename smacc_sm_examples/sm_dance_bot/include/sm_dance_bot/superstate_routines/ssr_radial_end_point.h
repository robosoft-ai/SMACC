struct SsrRadialEndPoint: smacc::SmaccState<SsrRadialEndPoint,SS>
{
  using SmaccState::SmaccState;

  typedef sc::transition<EvActionSucceded<smacc::SmaccMoveBaseActionClient::Result>, SsrRadialReturn> reactions; 

  void onInitialize()
  {
    this->configure<NavigationOrthogonal>(new SbNavigateForward(1));
    this->configure<ToolOrthogonal>(new SbToolStop());    
  }
  
};
