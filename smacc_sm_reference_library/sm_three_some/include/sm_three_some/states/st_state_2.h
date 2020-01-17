namespace sm_three_some
{
struct StState2 : smacc::SmaccState<StState2, MsRun>
{
    using SmaccState::SmaccState;

    typedef mpl::list<
        Transition<EvAllGo<SbAllEventsGo>, StState3>,

        // Keyboard events
        Transition<EvKeyPressP<CbDefaultKeyboardBehavior, OrKeyboard>, StState1>,
        Transition<EvKeyPressN<CbDefaultKeyboardBehavior, OrKeyboard>, StState3>>
        reactions;

    static void onDefinition()
    {
        configure_orthogonal<OrTimer, CbTimer>();
        configure_orthogonal<OrSubscriber, CbWatchdogSubscriberBehavior>();
        configure_orthogonal<OrUpdatablePublisher, CbDefaultPublishLoop>();
        configure_orthogonal<OrKeyboard, CbDefaultKeyboardBehavior>();

        static_createStateBehavior<SbAllEventsGo,
                               EvAllGo<SbAllEventsGo>,
                               mpl::list<EvTimer<CbTimer, OrTimer>,
                                         EvKeyPressA<CbDefaultKeyboardBehavior, OrKeyboard>>>();

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