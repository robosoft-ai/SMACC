namespace sm_packML
{
namespace ss2_states
{
// STATE DECLARATION
struct StiState6 : smacc::SmaccState<StiState6, SS>
{
  using SmaccState::SmaccState;

// TRANSITION TABLE
  typedef mpl::list<
    
  Transition<EvKeyPressP<CbDefaultKeyboardBehavior, OrKeyboard>, StiState5>,
  Transition<EvKeyPressN<CbDefaultKeyboardBehavior, OrKeyboard>, StiState4>
      
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
    //this->OnEventReceived<EvKeyPressN<CbDefaultKeyboardBehavior>>(onNextKey);
  }
};
}
}