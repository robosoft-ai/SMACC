struct SsrFPatternRotate1 : smacc::SmaccState<SsrFPatternRotate1, SS>
{
  using SmaccState::SmaccState;

  typedef smacc::transition<EvActionSucceeded<smacc::SmaccMoveBaseActionClient, OrNavigation>, SsrFPatternForward1> reactions;

  static void onDefinition()
  {
    float angle = 0;
    if (SS::direction() == TDirection::LEFT)
      angle = 90;
    else
      angle = -90;

    static_configure<OrNavigation, CbRotate>(angle);
    static_configure<OrTool, CbToolStop>();
  }

  void onInitialize()
  {
    
  }
};