struct Ssr3 : smacc::SmaccState<Ssr3, SS>
{
  using SmaccState::SmaccState;

  typedef mpl::list<
      //smacc::transition<EvTopicMessage<CbBehavior1b>, Ssr1>,

      smacc::transition<EvKeyPressP<CbKeyboard, OrKeyboard>, Ssr2>,
      smacc::transition<EvKeyPressN<CbKeyboard, OrKeyboard>, Ssr1>>
      reactions;

  //-------------------------------------------------------------------------------
  static void onDefinition()
  {
    static_configure<OrOrthogonal1, CbBehavior1b>();
    static_configure<OrOrthogonal2, CbBehavior2b>();
    static_configure<OrKeyboard, CbKeyboard>();
  }

  //-------------------------------------------------------------------------------
  void onInitialize()
  {
    //this->OnEventReceived<EvKeyPressN<CbKeyboard>>(onNextKey);
  }
};