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
        // get reference to the client
        ClRosTimer *client;
        this->requiresClient(client);

        // subscribe to the timer client callback
        client->onTimerTick(&StObserve::onTimerClientTickCallback, this);

        // getting reference to the single countdown behavior
        auto *cbsingle = this->getOrthogonal<OrTimer>()
                             ->getClientBehavior<CbTimerCountdownOnce>();

        // subscribe to the single countdown behavior callback
        cbsingle->onTimerTick(&StObserve::onSingleBehaviorTickCallback, this);
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
} // namespace sm_starcraft_ai