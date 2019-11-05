/// NavigateToEndPoint State
struct NavigateToEndPoint: SmaccState<NavigateToEndPoint,RadialMotionSuperState>
{
  // when this state is finished move to the ReturnToRadialStart state
  typedef smacc::transition<EvActionSucceded<smacc::SmaccMoveBaseActionClient>, ReturnToRadialStart> reactions; 

public:
  using SmaccState::SmaccState;

  void onInitialize()
  {
    this->configure<NavigationOrthogonal>(std::make_shared<NavigateForward>(3));
    this->configure<ToolOrthogonal>(std::make_shared<ToolStart>());

    // ALTERNATIVE SYNTAX
    // auto fw = new NavigateForward();
    // fw->forwardDistance = 3;
    // fw->forwardSpeed = 1;
    // this->configure<NavigationOrthogonal>(fw);
    
  }

  void onEntry()
  {
  }

  void onExit()
  {
  }
};
