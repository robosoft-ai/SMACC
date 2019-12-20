struct Ssr3 : smacc::SmaccState<Ssr3, SS>
{
  using SmaccState::SmaccState;

  typedef mpl::list<
      //smacc::transition<EvTopicMessage<CbBehavior1b>, Ssr1>,

      smacc::transition<EvKeyPressP<CbKeyboard, KeyboardOrthogonal>, Ssr2>,
      smacc::transition<EvKeyPressN<CbKeyboard, KeyboardOrthogonal>, Ssr1>>
      reactions;

  //-------------------------------------------------------------------------------
  static void onDefinition()
  {
    static_configure<Orthogonal1, CbBehavior1b>();
    static_configure<Orthogonal2, CbBehavior2b>();
    static_configure<KeyboardOrthogonal, CbKeyboard>();
  }

  //-------------------------------------------------------------------------------
  void onInitialize()
  {
    //this->OnEventReceived<EvKeyPressN<CbKeyboard>>(onNextKey);
  }
};