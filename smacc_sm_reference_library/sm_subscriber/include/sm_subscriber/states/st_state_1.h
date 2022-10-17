#include <smacc/smacc.h>

namespace sm_subscriber
{
using namespace smacc::default_transition_tags;

// STATE DECLARATION
struct State1 : smacc::SmaccState<State1, SmSubscriber>
{
  using SmaccState::SmaccState;

  // TRANSITION TABLE
  typedef mpl::list<

      //Transition<EvTopicMessage<ClNumbersSubscription, OrSubscriber>, State2, SUCCESS>
      Transition<EvTimer<CbTimerCountdownOnce, OrTimer>, State2, SUCCESS>
      >
      reactions;

  // STATE FUNCTIONS
  static void staticConfigure()
  {
      configure_orthogonal<OrSubscriber, CbPrintMessageNumber>();  // EvTimer triggers once at 10 client ticks
      configure_orthogonal<OrTimer, CbTimerCountdownOnce>(5); // EvTimer triggers once at 10 client ticks
  }

  void runtimeConfigure()
  {
  }

  void onEntry()
  {
    ROS_INFO("On Entry state1 !");
  }

  void onExit()
  {
    ROS_INFO("On Exit state1!");
  }
};
}  // namespace sm_subscriber
