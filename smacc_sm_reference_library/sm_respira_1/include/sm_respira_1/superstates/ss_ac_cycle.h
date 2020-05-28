namespace sm_respira_1
{
using namespace sm_respira_1::ac_cycle_inner_states;

// STATE DECLARATION
struct SsACCycle : smacc::SmaccState<SsACCycle, MsRun, StiACCycleLoop, sc::has_full_history>
{
public:
  using SmaccState::SmaccState;

  // TRANSITION TABLE
  typedef mpl::list<

      Transition<EvLoopEnd<StiACCycleLoop>, StObserve>

      >
      reactions;

  // STATE VARIABLES
  static constexpr int total_iterations()
  {
    return 1000;
  }
  int iteration_count = 0;

  // STATE FUNCTIONS
  static void staticConfigure()
  {
  }

  void runtimeConfigure()
  {
  }
};  // namespace SS1

}  // namespace sm_respira_1
