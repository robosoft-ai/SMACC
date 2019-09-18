struct StNavigateToWaypoint1: smacc::SmaccState<StNavigateToWaypoint1,SmDanceBot>
{
  using SmaccState::SmaccState;

  typedef mpl::list<
              // Expected event
              sc::transition<EvActionSucceded<smacc::SmaccMoveBaseActionClient::Result>, StNavigateToWaypointsX>,

              // Keyboard events
              sc::transition<KeyPressEvent<'p'>,StNavigateReverse2>,

              // Error events
              sc::transition<smacc::EvTopicMessageTimeout<LidarSensor>, StAcquireSensors>,
              sc::transition<EvActionAborted<smacc::SmaccMoveBaseActionClient::Result>, StNavigateToWaypointsX>
            > reactions;

  void onInitialize()
  {
     this->configure<NavigationOrthogonal>(new SbNavigateGlobalPosition(0, 0));
     this->configure<ToolOrthogonal>(new SbToolStart());
     this->configure<KeyboardOrthogonal>(new SbKeyboard());
  }
};
