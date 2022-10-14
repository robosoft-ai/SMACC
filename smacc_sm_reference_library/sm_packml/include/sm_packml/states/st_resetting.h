namespace sm_packml
{
// STATE DECLARATION
struct StResetting : smacc::SmaccState<StResetting, MsRun>
{
    using SmaccState::SmaccState;

// TRANSITION TABLE
    typedef mpl::list<

    // Transition<smacc::EvTopicMessage<CbWatchdogSubscriberBehavior, OrTimer>, StStarting>,
    // Keyboard events
    Transition<EvKeyPressP<CbDefaultKeyboardBehavior, OrKeyboard>, StIdle, SUCCESS> //,
    // Transition<EvKeyPressN<CbDefaultKeyboardBehavior, OrKeyboard>, StStarting>,
    // Transition<EvFail, MsStop, smacc::ABORT>

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
} // namespace sm_packml
