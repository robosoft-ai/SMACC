namespace sm_packML
{
// STATE DECLARATION
struct StStarting : smacc::SmaccState<StStarting, MsRun>
{
    using SmaccState::SmaccState;

// TRANSITION TABLE
    typedef mpl::list<
   
    // Transition<EvAllGo<SrAllEventsGo>, StExecute>,
    // Keyboard events
    // Transition<EvKeyPressP<CbDefaultKeyboardBehavior, OrKeyboard>, StIdle>,
    Transition<EvKeyPressN<CbDefaultKeyboardBehavior, OrKeyboard>, StExecute>
    
    >reactions;

// STATE FUNCTIONS
    static void staticConfigure()
    {
        configure_orthogonal<OrTimer, CbTimer>();
        configure_orthogonal<OrSubscriber, CbWatchdogSubscriberBehavior>();
        configure_orthogonal<OrUpdatablePublisher, CbDefaultPublishLoop>();
        configure_orthogonal<OrKeyboard, CbDefaultKeyboardBehavior>();

        // Create State Reactor
        auto sbAll = static_createStateReactor<SrAllEventsGo>();
        sbAll->addInputEvent<EvTimer<CbTimer, OrTimer>>();
        sbAll->addInputEvent<EvKeyPressA<CbDefaultKeyboardBehavior, OrKeyboard>>();
        sbAll->setOutputEvent<EvAllGo<SrAllEventsGo>>();
    }

    void runtimeConfigure()
    {
    }
};
} // namespace sm_packML