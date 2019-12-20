struct Ssr2 : smacc::SmaccState<Ssr2, SS>
{
  using SmaccState::SmaccState;

  typedef mpl::list<
      smacc::transition<EvTopicMessage<CbBehavior2b, Orthogonal2>, Ssr3>,

      smacc::transition<EvKeyPressN<CbKeyboard, KeyboardOrthogonal>, Ssr3>,
      smacc::transition<EvKeyPressP<CbKeyboard, KeyboardOrthogonal>, Ssr1>>
      reactions;

  //-------------------------------------------------------------------------------
  static void onDefinition()
  {
    static_configure<Orthogonal1, CbBehavior1>();
    static_configure<Orthogonal2, CbBehavior2b>();
    static_configure<KeyboardOrthogonal, CbKeyboard>();
  }

  //-------------------------------------------------------------------------------
  void onInitialize()
  {
  }
};
