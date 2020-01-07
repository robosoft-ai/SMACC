namespace sm_three_some
{
namespace ss1_states
{
struct Ssr2 : smacc::SmaccState<Ssr2, SS>
{
  using SmaccState::SmaccState;

  typedef mpl::list<
      smacc::Transition<EvTimer<CbTimer, OrTimer>, Ssr3>,

      smacc::Transition<EvKeyPressN<CbDefaultKeyboardBehavior, OrKeyboard>, Ssr3>,
      smacc::Transition<EvKeyPressP<CbDefaultKeyboardBehavior, OrKeyboard>, Ssr1>>
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
}
}