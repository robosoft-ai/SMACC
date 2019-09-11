
#include <boost/statechart/event.hpp>
#include <boost/statechart/state_machine.hpp>
#include <boost/statechart/simple_state.hpp>
#include <boost/statechart/transition.hpp>
#include <boost/statechart/custom_reaction.hpp>

#include <smacc/smacc.h>
#include <boost/config.hpp>

#include <ctime>
#include <iostream>


namespace sc = boost::statechart;


//////////////////////////////////////////////////////////////////////////////
struct EvReset : sc::event< EvReset > {};

struct IElapsedTime
{
  virtual double ElapsedTime() const = 0;
};

/*

struct StopWatch : sc::state_machine< StopWatch, Active > {};
*/

struct Active;
struct StopWatch
    : public smacc::SmaccStateMachineBase<StopWatch, Active>
{
    StopWatch(my_context ctx, smacc::SignalDetector *signalDetector)
        : SmaccStateMachineBase<StopWatch, Active>(ctx, signalDetector)
    {
    }
};

struct Active : smacc::SmaccState< Active, StopWatch >
{
  public:
    typedef sc::custom_reaction<EvReset> reactions;

    using SmaccState::SmaccState;
    /*Active() 
    {

    }*/

    void onEntry()
    {
        auto ev = new EvReset();
        postEvent( ev );
    }

    sc::result react(const EvReset &ev)
    {
        std::cout << "REACTION!!!!" << std::endl;
    }
    
};


//////////////////////////////////////////////////////////////////////////////
int main(int argc, char** argv)
{
   
  //StopWatch stopWatch;
  //stopWatch.initiate();
  ros::init(argc,argv,"test5");
  smacc::run<StopWatch>();

  return 0;
}