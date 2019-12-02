struct Ssr3 : smacc::SmaccState<Ssr3, SS>
{
  using SmaccState::SmaccState;

  typedef smacc::transition<EvTopicMessage<SbBehavior1b>, Ssr1> reactions;

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
  }
};
