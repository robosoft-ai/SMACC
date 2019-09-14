struct StNavigateReverse2 : smacc::SmaccState<StNavigateReverse2, SmDanceBot>
{
   using SmaccState::SmaccState;

   typedef mpl::list<
            // Expected event
            sc::transition<EvActionSucceded<smacc::SmaccMoveBaseActionClient::Result>, StNavigateToWaypoint1>,

            // Keyboard events
            sc::transition<KeyPressEvent<'p'>,StRotateDegrees4>,
            sc::transition<smacc::KeyPressEvent<'n'>, StNavigateToWaypoint1>,

            // Error events
            sc::transition<smacc::EvSensorMessageTimeout<LidarSensor>, StAcquireSensors>,
            sc::transition<EvActionAborted<smacc::SmaccMoveBaseActionClient::Result>, StNavigateToWaypointsX>
            > reactions;

   void onInitialize()
   {
      this->configure<NavigationOrthogonal>(new SbNavigateBackwards(2));
      this->configure<ToolOrthogonal>(new SbToolStop());
   }
};
