namespace sm_packml
{
// STATE DECLARATION
struct StStopped : smacc::SmaccState<StStopped, MsStop>
{
    using SmaccState::SmaccState;

// DECLARE CUSTOM OBJECT TAGS
    struct RESET : SUCCESS{};

// TRANSITION TABLE
    typedef mpl::list<

    // Transition<smacc::EvTopicMessage<CbWatchdogSubscriberBehavior, OrTimer>, StStarting>,
    // Keyboard events
    // Transition<EvKeyPressP<CbDefaultKeyboardBehavior, OrKeyboard>, SS1::Ss1>,
    Transition<EvKeyPressN<CbDefaultKeyboardBehavior, OrKeyboard>, StResetting, RESET> //,
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
