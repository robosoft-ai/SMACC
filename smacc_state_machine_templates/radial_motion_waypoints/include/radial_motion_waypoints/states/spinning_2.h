struct Spinning2: SmaccState<Spinning2, RadialMotionWaypointsStateMachine>
{
  typedef sc::transition<EvActionSucceded<smacc::SmaccMoveBaseActionClient::Result>,  RadialMotion3::RadialMotion3> reactions; 

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