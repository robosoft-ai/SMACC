struct SsrFPatternRotate1: smacc::SmaccState<SsrFPatternRotate1,SS>
{
  using SmaccState::SmaccState;

  typedef sc::transition<EvActionSucceded<smacc::SmaccMoveBaseActionClient::Result>, SS> reactions; 

  void onInitialize()
  {
    auto& superstate = this->context<SS>();

    float angle = 0;
    if (superstate.direction == TDirection::LEFT)
      angle = 90;
    else
      angle = -90;
      
    this->configure<NavigationOrthogonal>(std::make_shared<SbRotate>(angle));
    this->configure<ToolOrthogonal>(std::make_shared<SbToolStop>());  
  }
};