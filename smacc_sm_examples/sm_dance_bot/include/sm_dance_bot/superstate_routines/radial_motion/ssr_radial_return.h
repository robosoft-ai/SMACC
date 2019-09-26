struct SsrRadialReturn: smacc::SmaccState<SsrRadialReturn,SS>
{
  typedef sc::transition<EvActionSucceded<smacc::SmaccMoveBaseActionClient::Result>,  SsrRadialRotate> reactions; 

  using SmaccState::SmaccState;

  void onInitialize()
  {
    this->configure<NavigationOrthogonal>(std::make_shared<SbUndoPathBackwards>());
    this->configure<ToolOrthogonal>(std::make_shared<SbToolStop>());
  }
};