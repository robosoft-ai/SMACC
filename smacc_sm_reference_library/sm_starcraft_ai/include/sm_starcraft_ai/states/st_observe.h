namespace sm_starcraft_ai
{
// STATE DECLARATION
struct StObserve : smacc::SmaccState<StObserve, MsRun>
{
    using SmaccState::SmaccState;

// DECLARE CUSTOM OBJECT TAGS
    struct MOVE : SUCCESS{};
    struct BUILD : SUCCESS{};
    struct ATTACK : SUCCESS{};

// TRANSITION TABLE
    typedef mpl::list<
    
    // Transition<EvTimer<CbTimerCountdownOnce, OrTimer>, SS1::SsMove, TIMEOUT>,
    // Transition<smacc::EvTopicMessage<CbWatchdogSubscriberBehavior, OrSubscriber>, SS1::SsMove>,
    // Keyboard events
    Transition<EvKeyPressA<CbDefaultKeyboardBehavior, OrKeyboard>, SS1::SsMove, MOVE>,
    Transition<EvKeyPressB<CbDefaultKeyboardBehavior, OrKeyboard>, SS2::SsBuild, BUILD>,
    Transition<EvKeyPressC<CbDefaultKeyboardBehavior, OrKeyboard>, SS3::SsAttack, ATTACK>
    
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
} // namespace sm_starcraft_ai