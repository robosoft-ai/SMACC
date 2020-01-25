namespace sm_three_some
{
struct StState2 : smacc::SmaccState<StState2, MsRun>
{
    using SmaccState::SmaccState;

    typedef mpl::list<
        Transition<EvAllGo<SrAllEventsGo>, StState3>,

        // Keyboard events
        Transition<EvKeyPressP<CbDefaultKeyboardBehavior, OrKeyboard>, StState1>,
        Transition<EvKeyPressN<CbDefaultKeyboardBehavior, OrKeyboard>, StState3>>
        reactions;

    static void staticConfigure()
    {
        configure_orthogonal<OrTimer, CbTimer>();
        configure_orthogonal<OrSubscriber, CbWatchdogSubscriberBehavior>();
        configure_orthogonal<OrUpdatablePublisher, CbDefaultPublishLoop>();
        configure_orthogonal<OrKeyboard, CbDefaultKeyboardBehavior>();

        // static_createStateReactor<SrAllEventsGo,
        //                        EvAllGo<SrAllEventsGo>,
        //                        mpl::list<EvTimer<CbTimer, OrTimer>,
        //                                  EvKeyPressA<CbDefaultKeyboardBehavior, OrKeyboard>>>();

        auto sbAll = static_createStateReactor<SrAllEventsGo>();
        sbAll->addInputEvent<EvTimer<CbTimer, OrTimer>>();
        sbAll->addInputEvent<EvKeyPressA<CbDefaultKeyboardBehavior, OrKeyboard>>();
        sbAll->setOutputEvent<EvAllGo<SrAllEventsGo>>();

        /*auto sb = static_createStateReactor<SrAllEventsGo>();

        sb->subscribesEvent<EvTimer<CbTimer, OrTimer>>();
        sb->subscribesEvent<EvKeyPressA<CbDefaultKeyboardBehavior, OrKeyboard>>();

        sb->publishesEvent<EvAllGo<SrAllEventsGo>>();*/
    }

    void runtimeConfigure()
    {
    }
};
} // namespace sm_three_some