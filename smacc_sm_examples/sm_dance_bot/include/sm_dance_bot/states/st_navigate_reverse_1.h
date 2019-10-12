struct StNavigateReverse1 : smacc::SmaccState<StNavigateReverse1, SmDanceBot>
{
   using SmaccState::SmaccState;

   typedef mpl::list<
       // Expected event
       sc::transition<EvActionSucceded<smacc::SmaccMoveBaseActionClient::Result>, StRotateDegrees3>,

       // Keyboard event
       sc::transition<EvKeyPressP<SbKeyboard>, SS2::SsRadialPattern2>,
       sc::transition<EvKeyPressN<SbKeyboard>, StRotateDegrees3>,

       // Sensor events
       sc::transition<smacc::EvTopicMessageTimeout<LidarSensor>, StAcquireSensors>,
       sc::transition<EvActionAborted<smacc::SmaccMoveBaseActionClient::Result>, StNavigateToWaypointsX>>
       reactions;

   void onInitialize()
   {
      this->configure<NavigationOrthogonal>(std::make_shared<SbNavigateBackwards>(2));
      this->configure<ToolOrthogonal>(std::make_shared<SbToolStop>());
      this->configure<KeyboardOrthogonal>(std::make_shared<SbKeyboard>());
   }
};
