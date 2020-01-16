namespace sm_three_some
{
struct StState2 : smacc::SmaccState<StState2, MsRun>
{
    using SmaccState::SmaccState;

    typedef mpl::list<
        smacc::Transition<EvAllGo<SbAllEventsGo>, StState3>,

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

        // static_createStateBehavior<SbAllEventsGo,
        //                        EvAllGo<SbAllEventsGo>,
        //                        mpl::list<EvTimer<CbTimer, OrTimer>,
        //                                  EvKeyPressA<CbDefaultKeyboardBehavior, OrKeyboard>>>();

        auto sbAll = static_createStateBehavior<SbAllEventsGo>();
        sbAll->addInputEvent<EvTimer<CbTimer, OrTimer>>();
        sbAll->addInputEvent<EvKeyPressA<CbDefaultKeyboardBehavior, OrKeyboard>>();
        sbAll->setOutputEvent<EvAllGo<SbAllEventsGo>>();

        /*auto sb = static_createStateBehavior<SbAllEventsGo>();

        sb->subscribesEvent<EvTimer<CbTimer, OrTimer>>();
        sb->subscribesEvent<EvKeyPressA<CbDefaultKeyboardBehavior, OrKeyboard>>();

        sb->publishesEvent<EvAllGo<SbAllEventsGo>>();*/
    }

    void onInitialize()
    {
    }
};
} // namespace sm_three_some