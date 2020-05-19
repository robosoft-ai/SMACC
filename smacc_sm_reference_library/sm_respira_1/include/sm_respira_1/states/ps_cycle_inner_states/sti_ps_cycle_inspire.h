namespace sm_respira_1
{
namespace ps_cycle_inner_states
{
// STATE DECLARATION
struct StiPSCycleInspire : smacc::SmaccState<StiPSCycleInspire, SS>
{
  using SmaccState::SmaccState;

// DECLARE CUSTOM OBJECT TAGS
  struct TIMEOUT : SUCCESS{};
  struct NEXT : SUCCESS{};
  struct PREVIOUS : ABORT{};

// TRANSITION TABLE
  typedef mpl::list<
  
  Transition<EvTimer<CbTimerCountdownOnce, OrTimer>, StiPSCyclePlateau, TIMEOUT>,
  Transition<EvKeyPressN<CbDefaultKeyboardBehavior, OrKeyboard>, StiPSCyclePlateau, NEXT>,
  Transition<EvKeyPressP<CbDefaultKeyboardBehavior, OrKeyboard>, StiPSCycleLoop, PREVIOUS>,

  Transition<EvKeyPressY<CbDefaultKeyboardBehavior, OrKeyboard>, MsLeakyLung, ABORT>,
  Transition<EvKeyPressZ<CbDefaultKeyboardBehavior, OrKeyboard>, MsPatientObstruction, ABORT>
  
  >reactions;

  // STATE FUNCTIONS
  static void staticConfigure()
  {
    configure_orthogonal<OrTimer, CbTimerCountdownOnce>(40);
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

};
}
}