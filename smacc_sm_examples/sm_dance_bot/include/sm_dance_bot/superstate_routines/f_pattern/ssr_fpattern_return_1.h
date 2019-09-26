namespace SS4
{
struct SsrFPatternReturn1: smacc::SmaccState<SsrFPatternReturn1,SS>
{
  using SmaccState::SmaccState;

  typedef sc::transition<EvActionSucceded<smacc::SmaccMoveBaseActionClient::Result>, SsrFPatternRotate2> reactions; 

  void onInitialize()
  {
    auto& superstate = this->context<SS>();
    this->configure<NavigationOrthogonal>(std::make_shared<SbUndoPathBackwards>());
    this->configure<ToolOrthogonal>(std::make_shared<SbToolStart>());  
  }
};
}