namespace sm_respira_1
{
// STATE DECLARATION
struct StLeakyLungStep3 : smacc::SmaccState<StLeakyLungStep3, MsLeakyLung>
{
    using SmaccState::SmaccState;

// DECLARE CUSTOM OBJECT TAGS
    struct TIMEOUT : SUCCESS{};
    struct NEXT : SUCCESS{};
    struct PREVIOUS : ABORT{}; 

// TRANSITION TABLE
    typedef mpl::list<
    
    // Transition<EvTimer<CbTimerCountdownOnce, OrTimer>, sc::deep_history<MsLeakyLung>, SUCCESS>
    // Transition<smacc::EvTopicMessage<CbWatchdogSubscriberBehavior, OrSubscriber>, SS1::SsACCycle>,
    // Keyboard events
    // Transition<EvKeyPressA<CbDefaultKeyboardBehavior, OrKeyboard>, SS1::SsACCycle, MOVE>,
    // Transition<EvKeyPressB<CbDefaultKeyboardBehavior, OrKeyboard>, SS2::SsCMVCycle, BUILD>,
    // Transition<EvKeyPressC<CbDefaultKeyboardBehavior, OrKeyboard>, SS3::SsPCCycle, ATTACK>
    
    >reactions;

// STATE FUNCTIONS
    static void staticConfigure()
    {
        configure_orthogonal<OrTimer, CbTimerCountdownOnce>(50);
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