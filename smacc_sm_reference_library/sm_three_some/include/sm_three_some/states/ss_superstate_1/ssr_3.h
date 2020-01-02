namespace ss1_states
{
struct Ssr3 : smacc::SmaccState<Ssr3, SS>
{
  using SmaccState::SmaccState;

  typedef mpl::list<
      //smacc::transition<EvTopicMessage<CbBehavior1b>, Ssr1>,

      smacc::transition<EvKeyPressP<CbDefaultKeyboardBehavior, OrKeyboard>, Ssr2>,
      smacc::transition<EvKeyPressN<CbDefaultKeyboardBehavior, OrKeyboard>, Ssr1>>
      reactions;

  //-------------------------------------------------------------------------------
  static void onDefinition()
  {
    static_configure<OrTimer, CbTimer>();
    static_configure<OrSubscriber, CbWatchdogSubscriberBehavior>();
    static_configure<OrUpdatablePublisher, CbDefaultPublishLoop>();
    static_configure<OrKeyboard, CbDefaultKeyboardBehavior>();
  }

  //-------------------------------------------------------------------------------
  void onInitialize()
  {
    //this->OnEventReceived<EvKeyPressN<CbDefaultKeyboardBehavior>>(onNextKey);
  }
};
}