namespace sm_respira_1
{
// STATE DECLARATION
struct StLeakyLungStep1 : smacc::SmaccState<StLeakyLungStep1, MsLeakyLung>
{
    using SmaccState::SmaccState;

// DECLARE CUSTOM OBJECT TAGS
    struct MOVE : SUCCESS{};
    struct BUILD : SUCCESS{};
    struct ATTACK : SUCCESS{};

// TRANSITION TABLE
    typedef mpl::list<
    
    Transition<EvTimer<CbTimerCountdownOnce, OrTimer>, StLeakyLungStep2, SUCCESS>,
    // Transition<smacc::EvTopicMessage<CbWatchdogSubscriberBehavior, OrSubscriber>, SS1::SsACCycle>,
    // Keyboard events
    Transition<EvKeyPressN<CbDefaultKeyboardBehavior, OrKeyboard>, StLeakyLungStep2, SUCCESS>
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
        // get reference to the client
        ClRosTimer *client;
        this->requiresClient(client);

        // subscribe to the timer client callback
        client->onTimerTick(&StLeakyLungStep1::onTimerClientTickCallback, this);

        // getting reference to the single countdown behavior
        auto *cbsingle = this->getOrthogonal<OrTimer>()
                             ->getClientBehavior<CbTimerCountdownOnce>();

        // subscribe to the single countdown behavior callback
        cbsingle->onTimerTick(&StLeakyLungStep1::onSingleBehaviorTickCallback, this);
    }
    
    void onEntry()
    {
        ROS_INFO("On Entry!");
    }

    void onExit()
    {
        ROS_INFO("On Exit!");
    }

    void onTimerClientTickCallback()
    {
        ROS_INFO("timer client tick!");
    }

    void onSingleBehaviorTickCallback()
    {
        ROS_INFO("single behavior tick!");
    }
};
} // namespace sm_respira_1