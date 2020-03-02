namespace sm_mtc_picknplace
{

// STATE DECLARATION
struct StOpenHand : smacc::SmaccState<StOpenHand, SmMTCPickNPlace>
{
    using SmaccState::SmaccState;

// TRANSITION TABLE
    typedef mpl::list<
   
    Transition<EvTimer<CbTimerCountdownOnce, OrTimer>, StMoveToPick, SUCCESS>,
    Transition<EvKeyPressN<CbDefaultKeyboardBehavior, OrKeyboard>, StMoveToPick, PREEMPT>
    
    >reactions;

// STATE FUNCTIONS
    static void staticConfigure()
    {
        configure_orthogonal<OrTimer, CbTimerCountdownOnce>(5); // EvTimer triggers once at 10 client ticks
        configure_orthogonal<OrKeyboard, CbDefaultKeyboardBehavior>();
    }

    void runtimeConfigure()
    {
        // get reference to the client
        ClRosTimer *client;
        this->requiresClient(client);

        // subscribe to the timer client callback
        client->onTimerTick(&StOpenHand::onTimerClientTickCallback, this);

        // getting reference to the single countdown behavior
        auto *cbsingle = this->getOrthogonal<OrTimer>()
                             ->getClientBehavior<CbTimerCountdownOnce>();

        // subscribe to the single countdown behavior callback
        cbsingle->onTimerTick(&StOpenHand::onSingleBehaviorTickCallback, this);
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
} // namespace sm_mtc_picknplace