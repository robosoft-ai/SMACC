struct Spinning3: SmaccState<Spinning3, RadialMotionWaypointsStateMachine> // <- these are the orthogonal lines of this State 
{
  // when this state is finished then move to the NavigateToEndPoint state
  typedef sc::transition<EvActionSucceded<smacc::SmaccMoveBaseActionClient::Result>,  RadialMotion1::RadialMotion1> reactions; 

public:

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