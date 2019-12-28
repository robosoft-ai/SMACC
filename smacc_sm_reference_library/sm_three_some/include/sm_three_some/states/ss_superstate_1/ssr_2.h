struct Ssr2 : smacc::SmaccState<Ssr2, SS>
{
  using SmaccState::SmaccState;

  typedef mpl::list<
      smacc::transition<EvTimer<CbTimer, OrTimer>, Ssr3>,

      smacc::transition<EvKeyPressN<CbDefaultKeyboardBehavior, OrKeyboard>, Ssr3>,
      smacc::transition<EvKeyPressP<CbDefaultKeyboardBehavior, OrKeyboard>, Ssr1>>
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
  }
};
