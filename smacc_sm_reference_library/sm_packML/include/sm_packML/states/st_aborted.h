namespace sm_packML
{
// STATE DECLARATION
struct StAborted : smacc::SmaccState<StAborted, SmPackML>
{
    using SmaccState::SmaccState;

// TRANSITION TABLE
    typedef mpl::list<
        
    Transition<smacc::EvTopicMessage<CbWatchdogSubscriberBehavior, OrTimer>, StStarting>,
    // Keyboard events
    Transition<EvKeyPressP<CbDefaultKeyboardBehavior, OrKeyboard>, SS1::Ss1>,
    Transition<EvKeyPressN<CbDefaultKeyboardBehavior, OrKeyboard>, StStarting>,
    Transition<EvFail, MsRecover, smacc::ABORT>
    
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