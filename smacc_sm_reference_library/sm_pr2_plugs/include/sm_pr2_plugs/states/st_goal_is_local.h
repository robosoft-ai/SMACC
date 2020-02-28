namespace sm_pr2_plugs{
// STATE DECLARATION
struct StGoalIsLocal : smacc::SmaccState<StGoalIsLocal, MsRecharge>
{
    using SmaccState::SmaccState;

// DECLARE CUSTOM OBJECT TAGS
    struct TRUE : SUCCESS{};
    struct FALSE : ABORT{};

// TRANSITION TABLE
    typedef mpl::list<
        
    Transition<EvKeyPressN<CbDefaultKeyboardBehavior, OrKeyboard>, StSafetyTuck, FALSE>,
    Transition<EvTimer<CbTimerCountdownOnce, OrTimer>, StUntuckAtOutlet, TRUE>
    
    >reactions;

// STATE FUNCTIONS
    static void staticConfigure()
    {
        configure_orthogonal<OrTimer,  CbTimerCountdownOnce>(5);    
        configure_orthogonal<OrKeyboard, CbDefaultKeyboardBehavior>();
    }

       void runtimeConfigure()
    {
        // get reference to the client
        ClRosTimer *client;
        this->requiresClient(client);

        // subscribe to the timer client callback
        client->onTimerTick(&StGoalIsLocal::onTimerClientTickCallback, this);

        // getting reference to the single countdown behavior
        auto *cbsingle = this->getOrthogonal<OrTimer>()
                             ->getClientBehavior<CbTimerCountdownOnce>();

        // subscribe to the single countdown behavior callback
        cbsingle->onTimerTick(&StGoalIsLocal::onSingleBehaviorTickCallback, this);
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
} // namespace sm_pr2_plugs