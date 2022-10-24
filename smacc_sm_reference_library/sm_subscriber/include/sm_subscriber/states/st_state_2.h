#include <smacc/smacc.h>

namespace sm_subscriber
{
// STATE DECLARATION
struct State2 : smacc::SmaccState<State2, SmSubscriber>
{
  using SmaccState::SmaccState;

  // TRANSITION TABLE
  typedef mpl::list<

      Transition<EvImportantMessage<CbPostCustomEvent, OrSubscriber>, State1, SUCCESS>

      >
      reactions;

  // STATE FUNCTIONS
  static void staticConfigure()
  {
    configure_orthogonal<OrSubscriber, CbPostCustomEvent>();  // EvTimer triggers once at 10 client ticks
  }

  void runtimeConfigure()
  {
    ROS_INFO("Entering State2");
  }

  void onEntry()
  {
    ROS_INFO("On Entry!");
  }

  void onExit()
  {
    ROS_INFO("On Exit!");
  }
};
}  // namespace sm_subscriber
