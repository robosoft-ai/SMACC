using namespace smacc;

struct RadialMotionSuperState: smacc::SmaccState<RadialMotionSuperState,RadialMotionStateMachine, NavigateToRadialStart>
{
  using SmaccState::SmaccState;

  typedef sc::custom_reaction<EvStateFinished<ReturnToRadialStart>> reactions;

  int times;

  sc::result react( const EvStateFinished<ReturnToRadialStart> & )
  {
    times++;

    if(times == 4)
    {
      this->throwFinishEvent();
    }
  }
};
