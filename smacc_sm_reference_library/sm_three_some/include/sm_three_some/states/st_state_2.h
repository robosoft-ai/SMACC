namespace sm_three_some
{
struct StState2 : smacc::SmaccState<StState2, MsRun>
{
    using SmaccState::SmaccState;

    typedef mpl::list<
        smacc::Transition<EvAllGo<LuAllEventsGo>, StState3>,

        // Keyboard events
        smacc::Transition<EvKeyPressP<CbDefaultKeyboardBehavior, OrKeyboard>, StState1>,
        smacc::Transition<EvKeyPressN<CbDefaultKeyboardBehavior, OrKeyboard>, StState3>>
        reactions;

    static void onDefinition()
    {
        static_configure<OrTimer, CbTimer>();
        static_configure<OrSubscriber, CbWatchdogSubscriberBehavior>();
        static_configure<OrUpdatablePublisher, CbDefaultPublishLoop>();
        static_configure<OrKeyboard, CbDefaultKeyboardBehavior>();

        static_createLogicUnit<LuAllEventsGo,
                               EvAllGo<LuAllEventsGo>,
                               mpl::list<EvTimer<CbTimer, OrTimer>,
                                         EvKeyPressA<CbDefaultKeyboardBehavior, OrKeyboard>>>();
    }

    void onInitialize()
    {
    }
};
} // namespace sm_three_some