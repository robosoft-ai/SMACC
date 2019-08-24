struct Spinning1: SmaccState<Spinning1, RadialMotionWaypointsStateMachine>
{
  typedef sc::transition<EvActionSucceded<smacc::SmaccMoveBaseActionClient::Result>,  RadialMotion2::RadialMotion2> reactions; 
  using SmaccState::SmaccState;

  void onInitialize()
  {
    this->configure<NavigationOrthogonal>(new Rotate(360));
    this->configure<ToolOrthogonal>(new ToolStop());
  }

  void onEntry()
  {
  }

  void onExit()
  { 
     
  }
};