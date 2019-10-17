struct SsrSPatternForward1 : public smacc::SmaccState<SsrSPatternForward1, SS>
{
  using SmaccState::SmaccState;

  typedef smacc::transition<EvActionSucceded<smacc::SmaccMoveBaseActionClient>, SsrSPatternRotate2> reactions;

  static void onDefinition()
  {
    static_configure<ToolOrthogonal, SbToolStart>();
    static_configure<NavigationOrthogonal, SbNavigateForward>(SS::pitch2_lenght_meters());
  }

  void onInitialize()
  {
  }
};