struct SsrFPatternRotate2 : smacc::SmaccState<SsrFPatternRotate2, SS>
{
  using SmaccState::SmaccState;

  typedef smacc::transition<EvActionSucceeded<smacc::SmaccMoveBaseActionClient, NavigationOrthogonal>, SsrFPatternForward2> reactions;

  static void onDefinition()
  {
    float angle = 0;
    if (SS::direction() == TDirection::LEFT)
      angle = -90;
    else
      angle = 90;

    static_configure<NavigationOrthogonal, CbRotate>(angle);
    static_configure<ToolOrthogonal, CbToolStop>();
  }

  void onInitialize()
  {
  }
};