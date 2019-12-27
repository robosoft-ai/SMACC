struct Ssr2 : smacc::SmaccState<Ssr2, SS>
{
  using SmaccState::SmaccState;

  typedef mpl::list<
      smacc::transition<EvTopicMessage<CbBehavior2b, OrOrthogonal2>, Ssr3>,

      smacc::transition<EvKeyPressN<CbKeyboard, OrKeyboard>, Ssr3>,
      smacc::transition<EvKeyPressP<CbKeyboard, OrKeyboard>, Ssr1>>
      reactions;

  //-------------------------------------------------------------------------------
  static void onDefinition()
  {
    static_configure<OrOrthogonal1, CbBehavior1>();
    static_configure<OrOrthogonal2, CbBehavior2b>();
    static_configure<OrKeyboard, CbKeyboard>();
  }

  //-------------------------------------------------------------------------------
  void onInitialize()
  {
  }
};
