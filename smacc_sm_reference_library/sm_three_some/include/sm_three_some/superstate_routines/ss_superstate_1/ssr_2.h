struct Ssr2 : smacc::SmaccState<Ssr2, SS>
{
  using SmaccState::SmaccState;

  typedef mpl::list<
      smacc::transition<EvTopicMessage<SbBehavior2b, Orthogonal2>, Ssr3>,

      smacc::transition<EvKeyPressN<SbKeyboard, KeyboardOrthogonal>, Ssr3>,
      smacc::transition<EvKeyPressP<SbKeyboard, KeyboardOrthogonal>, Ssr1>>
      reactions;

  //-------------------------------------------------------------------------------
  static void onDefinition()
  {
    static_configure<Orthogonal1, SbBehavior1>();
    static_configure<Orthogonal2, SbBehavior2b>();
    static_configure<KeyboardOrthogonal, SbKeyboard>();
  }

  //-------------------------------------------------------------------------------
  void onInitialize()
  {
  }
};
