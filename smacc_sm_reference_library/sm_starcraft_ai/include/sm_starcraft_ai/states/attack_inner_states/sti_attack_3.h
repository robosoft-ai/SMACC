namespace sm_starcraft_ai
{
namespace attack_inner_states
{
// STATE DECLARATION
struct StiAttack3 : smacc::SmaccState<StiAttack3, SS>
{
  using SmaccState::SmaccState;

// DECLARE CUSTOM OBJECT TAGS
  struct TIMEOUT : SUCCESS{};
  struct NEXT : SUCCESS{};
  struct PREVIOUS : ABORT{};

// TRANSITION TABLE
  typedef mpl::list<

  Transition<EvTimer<CbTimerCountdownOnce, OrTimer>, StiAttack1, TIMEOUT>,
  Transition<EvKeyPressP<CbDefaultKeyboardBehavior, OrKeyboard>, StiAttack2, PREVIOUS>,
  Transition<EvKeyPressN<CbDefaultKeyboardBehavior, OrKeyboard>, StiAttack1, NEXT>

  >reactions;

 // STATE FUNCTIONS
  static void staticConfigure()
  {
    configure_orthogonal<OrTimer, CbTimerCountdownOnce>(10);
    configure_orthogonal<OrSubscriber, CbWatchdogSubscriberBehavior>();
    configure_orthogonal<OrUpdatablePublisher, CbDefaultPublishLoop>();
    configure_orthogonal<OrKeyboard, CbDefaultKeyboardBehavior>();
  }

  void runtimeConfigure()
  {
  }

  void onEntry()
  {
    ROS_INFO("On Entry!");
  }

  void onExit()
  {
    ROS_INFO("On Exit!");
  }

  void onTimerClientTickCallback()
  {
    ROS_INFO("timer client tick!");
  }

  void onSingleBehaviorTickCallback()
  {
    ROS_INFO("single behavior tick!");
  }

};
}
}
