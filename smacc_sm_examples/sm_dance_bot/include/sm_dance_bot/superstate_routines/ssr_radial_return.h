struct SsrRadialReturn: smacc::SmaccState<SsrRadialReturn,SS>
{
  typedef sc::transition<EvActionSucceded<smacc::SmaccMoveBaseActionClient::Result>,  SsrRadialRotate> reactions; 

  using SmaccState::SmaccState;

  void onInitialize()
  {
    this->configure<NavigationOrthogonal>(new SbUndoPathBackwards());
    this->configure<ToolOrthogonal>(new SbToolStop());
  }
};