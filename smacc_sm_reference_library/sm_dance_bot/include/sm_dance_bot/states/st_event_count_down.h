struct StEventCountDown : smacc::SmaccState<StEventCountDown, MsDanceBotRunMode>
{
    using SmaccState::SmaccState;

    typedef mpl::list<
        // Expected event
        transition<EvCountdownEnd<LuEventCountdown>, StNavigateToWaypointsX>,

        smacc::transition<EvGlobalError, sc::deep_history<StAcquireSensors>>>
        reactions;

    static void onDefinition()
    {
        //   static_configure<ObstaclePerceptionOrthogonal, SbLidarSensor>();
        //   static_configure<KeyboardOrthogonal, SbKeyboard>();
        //   static_configure<PublisherOrthogonal, SbStringPublisher>("Hello World!");
        //   static_configure<SensorOrthogonal, SbConditionTemperatureSensor>();
        //   static_configure<Service3Orthogonal, Service3Behavior>(Service3Command::SERVICE3_ON);

        static_createLogicUnit<LuEventCountdown, EvCountdownEnd<LuEventCountdown>, mpl::list<EvTimer<SmaccTimerClient>>>(100);

        static_createLogicUnit<LuEventCountdown, EvCountdownEnd<LuEventCountdown>, mpl::list<EvActionFeedback<MoveBaseActionClient>>>(100);
    }
};