namespace sm_three_some
{
namespace ss1_states
{
// STATE DECLARATION
struct StiState2 : smacc::SmaccState<StiState2, SS>
{
  using SmaccState::SmaccState;

// TRANSITION TABLE
  typedef mpl::list<
  
  Transition<EvTimer<CbTimer, OrTimer>, StiState3>,
  Transition<EvKeyPressN<CbDefaultKeyboardBehavior, OrKeyboard>, StiState3>,
  Transition<EvKeyPressP<CbDefaultKeyboardBehavior, OrKeyboard>, StiState1>
  
  >reactions;

  // STATE FUNCTIONS
  static void staticConfigure()
  {
    configure_orthogonal<OrTimer, CbTimer>();
    configure_orthogonal<OrSubscriber, CbWatchdogSubscriberBehavior>();
    configure_orthogonal<OrUpdatablePublisher, CbDefaultPublishLoop>();
    configure_orthogonal<OrKeyboard, CbDefaultKeyboardBehavior>();
  }

  void runtimeConfigure()
  {
  }
};
}
}