namespace SS4
{
struct SsrFPatternForward1: smacc::SmaccState<SsrFPatternForward1,SS>
{
  using SmaccState::SmaccState;

  typedef sc::transition<EvActionSucceded<smacc::SmaccMoveBaseActionClient::Result>, SsrFPatternReturn1> reactions; 

  void onInitialize()
  {
    auto& superstate = this->context<SS>();
    this->configure<NavigationOrthogonal>(std::make_shared<SbNavigateForward>(superstate.ray_lenght_meters));
    this->configure<ToolOrthogonal>(std::make_shared<SbToolStart>());    
  }
};
}