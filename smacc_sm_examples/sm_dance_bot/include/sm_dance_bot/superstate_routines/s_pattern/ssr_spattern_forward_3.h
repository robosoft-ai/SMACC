struct SsrSPatternForward3 : public smacc::SmaccState<SsrSPatternForward3, SS>
{
  using SmaccState::SmaccState;

  typedef smacc::transition<EvActionSucceded<smacc::SmaccMoveBaseActionClient>, SsrSPatternRotate4> reactions;

  static void onDefinition()
  {
    static_configure<NavigationOrthogonal, SbNavigateForward>(SS::pitch2_lenght_meters());
    static_configure<ToolOrthogonal, SbToolStart>();
  }

  void onInitialize()
  {

  }
};