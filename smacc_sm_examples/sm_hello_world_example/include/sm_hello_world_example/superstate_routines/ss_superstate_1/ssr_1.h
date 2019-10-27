
struct Ssr1 : smacc::SmaccState<Ssr1, SS>
{
  public:
  using SmaccState::SmaccState;

  typedef smacc::transition<EvTopicMessage<SbBehavior1b>, Ssr2> reactions;
  
  //-------------------------------------------------------------------------------
  static void onDefinition()
  {
    static_configure<Orthogonal1, SbBehavior1b>();
    static_configure<Orthogonal2, SbBehavior2b>();
  }
  
  //-------------------------------------------------------------------------------
  void onInitialize()
  {

  }
};
