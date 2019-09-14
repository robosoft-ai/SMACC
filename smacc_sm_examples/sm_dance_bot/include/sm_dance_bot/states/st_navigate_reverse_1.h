struct StNavigateReverse1 : smacc::SmaccState<StNavigateReverse1, SmDanceBot>
{
   using SmaccState::SmaccState;

   typedef mpl::list<
            sc::transition<smacc::SmaccMoveBaseActionClient::SuccessEv, StRotateDegrees3>,

            sc::transition<smacc::EvSensorMessageTimeout<LidarSensor>, StAcquireSensors>,
            sc::transition<EvActionAborted<smacc::SmaccMoveBaseActionClient::Result>, StNavigateToWaypointsX>
            > reactions;

   void onInitialize()
   {
      this->configure<NavigationOrthogonal>(new SbNavigateBackwards(2));
      this->configure<ToolOrthogonal>(new SbToolStop());
   }
};
