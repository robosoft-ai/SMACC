struct Ssr3 : smacc::SmaccState<Ssr3, SS>
{
  using SmaccState::SmaccState;

  typedef mpl::list<
      //smacc::transition<EvTopicMessage<SbBehavior1b>, Ssr1>,

      smacc::transition<EvKeyPressP<SbKeyboard>, Ssr2>,
      smacc::transition<EvKeyPressN<SbKeyboard>, Ssr1>>
      reactions;

  //-------------------------------------------------------------------------------
  static void onDefinition()
  {
    static_configure<Orthogonal1, SbBehavior1b>();
    static_configure<Orthogonal2, SbBehavior2b>();
    static_configure<KeyboardOrthogonal, SbKeyboard>();
  }

  //-------------------------------------------------------------------------------
  void onInitialize()
  {
    //this->OnEventReceived<EvKeyPressN<SbKeyboard>>(onNextKey);
  }
};