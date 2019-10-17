/*****************************************************************************************************************
 * ReelRobotix Inc. - Software License Agreement      Copyright (c) 2018
 * 	 Authors: Pablo Inigo Blasco, Brett Aldrich
 *
 ******************************************************************************************************************/

/// State NavigateToRadialStart
struct RotateDegress: SmaccState<RotateDegress, RadialMotionSuperState> // <- these are the orthogonal lines of this State 
{
  // when this state is finished then move to the NavigateToEndPoint state
  typedef smacc::transition<EvActionSucceded<smacc::SmaccMoveBaseActionClient>,  NavigateToEndPoint> reactions; 

public:

  using SmaccState::SmaccState;

  void onInitialize()
  {
    this->configure<NavigationOrthogonal>(std::make_shared<Rotate>(90));
    this->configure<ToolOrthogonal>(std::make_shared<ToolStop>());
  }

  void onEntry()
  {
    ROS_INFO("-------");
    ROS_INFO("Entering in ROTATE TEN DEGREES STATE");
  }

  // This is the state destructor. This code will be executed when the
  // workflow exits from this state (that is according to statechart the moment when this object is destroyed)
  void onExit()
  { 
    ROS_INFO("Exiting in ROTATE TEN DEGREES STATE"); 
  }
};
