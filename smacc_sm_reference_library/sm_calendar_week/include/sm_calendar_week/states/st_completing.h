namespace sm_calendar_week
{
// STATE DECLARATION
struct StCompleting : smacc::SmaccState<StCompleting, MsWorkweek>
{
    using SmaccState::SmaccState;

// TRANSITION TABLE
    typedef mpl::list<
        
    // Transition<smacc::EvTopicMessage<CbWatchdogSubscriberBehavior, OrTimer>, StComplete>,
    // Keyboard events
    // Transition<EvKeyPressP<CbDefaultKeyboardBehavior, OrKeyboard>, SS1::Ss1>,
    Transition<EvKeyPressN<CbDefaultKeyboardBehavior, OrKeyboard>, StComplete, SUCCESS> //,
    // Transition<EvFail, MsWeekend, smacc::ABORT>
    
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
} // namespace sm_calendar_week