struct SsrRadialRotate: smacc::SmaccState<SsrRadialRotate, SS>
{
  using SmaccState::SmaccState;

  typedef mpl::list<sc::transition<EvActionSucceded<smacc::SmaccMoveBaseActionClient>,  SsrRadialEndPoint>> reactions; 

  void onInitialize()
  {
    auto& superstate = this->context<SS>();
    
    ROS_INFO("[SsrRadialRotate] Radial rotate: SS current iteration: %d/%d", superstate.iteration_count , superstate.total_iterations);
    if(superstate.iteration_count < superstate.total_iterations)
    {
        this->configure<NavigationOrthogonal>(std::make_shared<SbRotate>(superstate.ray_angle_increment_degree));
        this->configure<ToolOrthogonal>(std::make_shared<SbToolStop>());
    }
  }
};