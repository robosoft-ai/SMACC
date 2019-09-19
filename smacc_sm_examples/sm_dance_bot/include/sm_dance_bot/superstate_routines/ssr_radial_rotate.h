struct SsrRadialRotate: smacc::SmaccState<SsrRadialRotate, SS>
{
  using SmaccState::SmaccState;

  typedef mpl::list<sc::transition<EvActionSucceded<smacc::SmaccMoveBaseActionClient::Result>,  SsrRadialEndPoint>> reactions; 

  void onInitialize()
  {
    auto currentIteration = this->context<SS>().iteration_count ;

    ROS_INFO("Radial rotate: SS current iteration: %d", currentIteration);
    if(currentIteration < this->context<SS>().total_iterations)
    {
        this->configure<NavigationOrthogonal>(std::make_shared<SbRotate>(45));
        this->configure<ToolOrthogonal>(std::make_shared<SbToolStop>());
    }
  }
};