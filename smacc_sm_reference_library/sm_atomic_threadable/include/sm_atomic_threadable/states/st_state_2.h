#include <smacc/smacc.h>

namespace sm_atomic_threadable {
// STATE DECLARATION
struct State2 : smacc::SmaccState<State2, SmAtomicThreadable> {
  using SmaccState::SmaccState;

  // TRANSITION TABLE
  typedef mpl::list<

      Transition<EvExitCb<CbThread, OrTimer>, State1, SUCCESS>

      >
      reactions;

  // STATE FUNCTIONS
  static void staticConfigure() { configure_orthogonal<OrTimer, CbThread>(); }

  void runtimeConfigure() { ROS_INFO("Entering State2"); }

  void onEntry() { ROS_INFO("On Entry!"); }

  void onExit() { ROS_INFO("On Exit!"); }
};
}  // namespace sm_atomic_threadable
