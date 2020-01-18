namespace sm_three_some
{
namespace ss1_states
{
struct StiState3 : smacc::SmaccState<StiState3, SS>
{
  using SmaccState::SmaccState;

  typedef mpl::list<
      //Transition<EvTopicMessage<CbBehavior1b>, StiState1>,

      Transition<EvKeyPressP<CbDefaultKeyboardBehavior, OrKeyboard>, StiState2>,
      Transition<EvKeyPressN<CbDefaultKeyboardBehavior, OrKeyboard>, StiState1>>
      reactions;

  //-------------------------------------------------------------------------------
  static void onDefinition()
  {
    configure_orthogonal<OrTimer, CbTimer>();
    configure_orthogonal<OrSubscriber, CbWatchdogSubscriberBehavior>();
    configure_orthogonal<OrUpdatablePublisher, CbDefaultPublishLoop>();
    configure_orthogonal<OrKeyboard, CbDefaultKeyboardBehavior>();
  }

  //-------------------------------------------------------------------------------
  void onInitialize()
  {
    //this->OnEventReceived<EvKeyPressN<CbDefaultKeyboardBehavior>>(onNextKey);
  }
};
}
}