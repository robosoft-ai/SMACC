namespace sm_respira_1
{
// STATE DECLARATION
struct StObserve : smacc::SmaccState<StObserve, MsRun>
{
    using SmaccState::SmaccState;

// DECLARE CUSTOM OBJECT TAGS
    struct AC_CYCLE : SUCCESS{};
    struct CMV_CYCLE : SUCCESS{};
    struct PC_CYCLE : SUCCESS{};
    struct PS_CYCLE : SUCCESS{};
    struct SHUTDOWN : SUCCESS{};
    struct CALIBRATION : PREEMPT{};

// TRANSITION TABLE
    typedef mpl::list<
    
    // Transition<EvTimer<CbTimerCountdownOnce, OrTimer>, SS1::SsACCycle, TIMEOUT>,
    // Transition<smacc::EvTopicMessage<CbWatchdogSubscriberBehavior, OrSubscriber>, SS1::SsACCycle>,
    // Keyboard events
    Transition<EvKeyPressA<CbDefaultKeyboardBehavior, OrKeyboard>, SS1::SsACCycle, AC_CYCLE>,
    Transition<EvKeyPressB<CbDefaultKeyboardBehavior, OrKeyboard>, SS2::SsCMVCycle, CMV_CYCLE>,
    Transition<EvKeyPressC<CbDefaultKeyboardBehavior, OrKeyboard>, SS3::SsPCCycle, PC_CYCLE>,
    Transition<EvKeyPressD<CbDefaultKeyboardBehavior, OrKeyboard>, SS4::SsPSCycle, PS_CYCLE>,

    Transition<EvKeyPressL<CbDefaultKeyboardBehavior, OrKeyboard>, MsCalibration, CALIBRATION>,
    Transition<EvKeyPressS<CbDefaultKeyboardBehavior, OrKeyboard>, MsShutdown, SHUTDOWN>
    
    >reactions;

// STATE FUNCTIONS
    static void staticConfigure()
    {
        configure_orthogonal<OrTimer, CbTimerCountdownOnce>(10);
        configure_orthogonal<OrSubscriber, CbWatchdogSubscriberBehavior>();
        configure_orthogonal<OrUpdatablePublisher, CbDefaultPublishLoop>();
        configure_orthogonal<OrKeyboard, CbDefaultKeyboardBehavior>();
    }

    void runtimeConfigure()
    {
    }
    
    void onEntry()
    {
        ROS_INFO("On Entry!");
    }

    void onExit()
    {
        ROS_INFO("On Exit!");
    }

};
} // namespace sm_respira_1