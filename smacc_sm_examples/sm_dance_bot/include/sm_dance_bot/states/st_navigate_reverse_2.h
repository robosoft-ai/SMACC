struct StNavigateReverse2 : smacc::SmaccState<StNavigateReverse2, SmDanceBot>
{
   using SmaccState::SmaccState;

   typedef mpl::list<
            sc::transition<smacc::SmaccMoveBaseActionClient::SuccessEv, StNavigateToWaypoint1>,

            sc::transition<smacc::EvSensorMessageTimeout<LidarSensor>, StAcquireSensors>,
            sc::transition<EvActionAborted<smacc::SmaccMoveBaseActionClient::Result>, StNavigateToWaypointsX>
            > reactions;

   void onInitialize()
   {
      this->configure<NavigationOrthogonal>(new SbNavigateBackwards(2));
      this->configure<ToolOrthogonal>(new SbToolStop());
   }
};
