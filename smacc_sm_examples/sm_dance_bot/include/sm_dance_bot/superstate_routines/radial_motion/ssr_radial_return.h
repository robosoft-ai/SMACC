struct SsrRadialReturn : smacc::SmaccState<SsrRadialReturn, SS>
{
  typedef sc::custom_reaction<EvActionSucceded<smacc::SmaccMoveBaseActionClient>> reactions;

  using SmaccState::SmaccState;

  static void onDefinition()
  {
    static_configure<NavigationOrthogonal, SbUndoPathBackwards>();
    static_configure<ToolOrthogonal, SbToolStop>();
  }

  void onInitialize()
  {
  }

  sc::result react(const EvActionSucceded<smacc::SmaccMoveBaseActionClient> &ev)
  {
    auto &superstate = this->context<SS>();
    ROS_INFO("[SsrRadialRotate] Radial rotate: SS current iteration: %d/%d", superstate.iteration_count, superstate.total_iterations());

    if (++(superstate.iteration_count) == superstate.total_iterations()) // 1 == two times
    {
      ROS_INFO("Breaking radial motion. Throwing superstate finith event.");
      superstate.throwFinishEvent();
    }
    else
    {
      ROS_INFO("LOOPING TO Radial Rotate");
      return transit<SS1::SsrRadialRotate>();
    }

    return discard_event();
  }
};