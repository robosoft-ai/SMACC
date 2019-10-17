//--------------------------------------------
/// ReturnToRadialStart State
struct ReturnToRadialStart : SmaccState< ReturnToRadialStart, RadialMotionSuperState > // <- these are the orthogonal lines of this State
{
    // when this state is finished then move to the RotateDegress state
    typedef smacc::transition<EvActionSucceded<smacc::SmaccMoveBaseActionClient>, RotateDegress> reactions; 

public:
  using SmaccState::SmaccState;

   void onInitialize()
   {
      this->configure<NavigationOrthogonal>(std::make_shared<UndoPathBackwards>());
      this->configure<ToolOrthogonal>(std::make_shared<ToolStop>());
   }

    void onEntry()
    {
        ROS_INFO("-------");
        ROS_INFO("Entering State: ReturnToRadialStart");
    }

    void onExit()
    {
        ROS_INFO("Exiting State: ReturnToRadialStart");
    }
};