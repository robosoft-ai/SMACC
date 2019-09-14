struct StNavigateReverse1 : smacc::SmaccState<StNavigateReverse1, SmDanceBot>
{
   using SmaccState::SmaccState;

   typedef mpl::list<
            // Expected event
            sc::transition<EvActionSucceded<smacc::SmaccMoveBaseActionClient::Result>, StRotateDegrees3>,                  

            // Keyboard event
            sc::transition<KeyPressEvent<'p'>,SS2::SsRadialPattern2>,
            sc::transition<KeyPressEvent<'n'>, StRotateDegrees3>,

            // Sensor events
            sc::transition<smacc::EvSensorMessageTimeout<LidarSensor>, StAcquireSensors>,
            sc::transition<EvActionAborted<smacc::SmaccMoveBaseActionClient::Result>, StNavigateToWaypointsX>
            > reactions;

   void onInitialize()
   {
      this->configure<NavigationOrthogonal>(new SbNavigateBackwards(2));
      this->configure<ToolOrthogonal>(new SbToolStop());
   }
};
