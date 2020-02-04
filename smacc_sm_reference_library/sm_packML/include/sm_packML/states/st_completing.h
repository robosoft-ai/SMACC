namespace sm_packML
{
// STATE DECLARATION
struct StCompleting : smacc::SmaccState<StCompleting, MsRun>
{
    using SmaccState::SmaccState;

// TRANSITION TABLE
    typedef mpl::list<
        
    // Transition<smacc::EvTopicMessage<CbWatchdogSubscriberBehavior, OrTimer>, StComplete>,
    // Keyboard events
    // Transition<EvKeyPressP<CbDefaultKeyboardBehavior, OrKeyboard>, SS1::Ss1>,
    Transition<EvKeyPressN<CbDefaultKeyboardBehavior, OrKeyboard>, StComplete> //,
    // Transition<EvFail, MsRecover, smacc::ABORT>
    
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