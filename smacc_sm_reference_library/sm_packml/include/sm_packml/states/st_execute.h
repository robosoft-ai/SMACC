namespace sm_packml
{
// STATE DECLARATION
struct StExecute : smacc::SmaccState<StExecute, MsRun>
{
    using SmaccState::SmaccState;

// DECLARE CUSTOM OBJECT TAGS
    struct HOLD : PREEMPT{};
    struct SUSPEND : PREEMPT{};
    struct STOP : ABORT{};

// TRANSITION TABLE
    typedef mpl::list<

    // Transition<smacc::EvTopicMessage<CbWatchdogSubscriberBehavior, OrSubscriber>, SS1::Ss1>,
    // Keyboard events
    Transition<EvKeyPressN<CbDefaultKeyboardBehavior, OrKeyboard>, StCompleting, SUCCESS>,
    Transition<EvKeyPressP<CbDefaultKeyboardBehavior, OrKeyboard>, StHolding, HOLD>,
    Transition<EvKeyPressZ<CbDefaultKeyboardBehavior, OrKeyboard>, StSuspending, SUSPEND>,
    Transition<EvKeyPressT<CbDefaultKeyboardBehavior, OrKeyboard>, MsStop, STOP>,
    Transition<EvKeyPressE<CbDefaultKeyboardBehavior, OrKeyboard>, StAborting, ABORT>


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
