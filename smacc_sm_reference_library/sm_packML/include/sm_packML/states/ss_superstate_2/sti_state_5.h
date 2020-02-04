namespace sm_packML
{
namespace ss2_states
{
// STATE DECLARATION
struct StiState5 : smacc::SmaccState<StiState5, SS>
{
  using SmaccState::SmaccState;

// TRANSITION TABLE
  typedef mpl::list<
  
  Transition<EvTimer<CbTimer, OrTimer>, StiState6>,
  Transition<EvKeyPressN<CbDefaultKeyboardBehavior, OrKeyboard>, StiState6>,
  Transition<EvKeyPressP<CbDefaultKeyboardBehavior, OrKeyboard>, StiState4>
  
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