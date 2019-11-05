/// State NavigateToRadialStart
struct NavigateToRadialStart: smacc::SmaccState<NavigateToRadialStart, RadialMotionSuperState> // <- these are the orthogonal lines of this State
{
  // when this state is finished then move to the RotateDegress state
  typedef smacc::transition<EvActionSucceded<smacc::SmaccMoveBaseActionClient>, RotateDegress> reactions; 

public:
  using SmaccState::SmaccState;

  void onInitialize()
  {
     ROS_INFO("ON INITIALIZEEEE");
     this->configure<NavigationOrthogonal>(std::make_shared<NavigateGlobalPosition>(1, 0));
     this->configure<ToolOrthogonal>(std::make_shared<ToolStart>());
  }

  void onEntry()
  {
    ROS_INFO("-------");
    ROS_INFO("Entering in NavigateToRadialStart State");
  }

  void onExit() 
  {
    ROS_INFO("Finishing NavigateToRadialStart state");
  }
};
