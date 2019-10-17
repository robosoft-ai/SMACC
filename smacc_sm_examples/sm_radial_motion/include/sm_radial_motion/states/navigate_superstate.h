using namespace smacc;

struct RadialMotionSuperState: smacc::SmaccState<RadialMotionSuperState,RadialMotionStateMachine, NavigateToRadialStart>
{
  using SmaccState::SmaccState;

  typedef mpl::list<sc::custom_reaction<EvStateFinish<ReturnToRadialStart>>> reactions;

  int times;

  sc::result react( const EvStateFinish<ReturnToRadialStart> & )
  {
    times++;

    if(times == 4)
    {
      this->throwFinishEvent();
    }
    
    return forward_event();
  }
};
