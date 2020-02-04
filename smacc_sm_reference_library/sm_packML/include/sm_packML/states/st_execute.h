namespace sm_packML
{
// STATE DECLARATION
struct StExecute : smacc::SmaccState<StExecute, MsRun>
{
    using SmaccState::SmaccState;

// TRANSITION TABLE
    typedef mpl::list<
    
    Transition<smacc::EvTopicMessage<CbWatchdogSubscriberBehavior, OrSubscriber>, SS1::Ss1>,
    // Keyboard events
    Transition<EvKeyPressP<CbDefaultKeyboardBehavior, OrKeyboard>, StStarting>,
    Transition<EvKeyPressN<CbDefaultKeyboardBehavior, OrKeyboard>, SS1::Ss1>
    
    >reactions;

// STATE FUNCTIONS
    static void staticConfigure()
    {
        configure_orthogonal<OrTimer, CbTimer>();
        configure_orthogonal<OrSubscriber, CbWatchdogSubscriberBehavior>();
        configure_orthogonal<OrUpdatablePublisher, CbDefaultPublishLoop>();
        configure_orthogonal<OrKeyboard, CbDefaultKeyboardBehavior>();
    }

    void runtimeConfigure()
    {
    }
};
} // namespace sm_packML