using namespace smacc;
//--------------------------------------------
/// NavigateToEndPoint State
struct NavigateToEndPoint: SmaccState<NavigateToEndPoint,RadialMotionSuperState>
{
  // when this state is finished move to the ReturnToRadialStart state
  typedef sc::transition<EvActionSucceded<smacc::SmaccMoveBaseActionClient::Result>, ReturnToRadialStart> reactions; 

public:
  using SmaccState::SmaccState;

  void onInitialize()
  {
    this->configure<NavigationOrthogonal>(new NavigateForward(3));
    this->configure<ToolOrthogonal>(new ToolStart());

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
