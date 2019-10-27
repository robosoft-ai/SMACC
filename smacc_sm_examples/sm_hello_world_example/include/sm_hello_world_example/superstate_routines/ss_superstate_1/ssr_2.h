
struct Ssr2 : smacc::SmaccState<Ssr2, SS>
{
  using SmaccState::SmaccState;

  typedef smacc::transition<EvTopicMessage<SbBehavior2b>, Ssr3> reactions;
  
  //-------------------------------------------------------------------------------
  static void onDefinition()
  {
    static_configure<Orthogonal1, SbBehavior1>();
    static_configure<Orthogonal2, SbBehavior2b>();
  }
  
  //-------------------------------------------------------------------------------
  void onInitialize()
  {

  }
};

