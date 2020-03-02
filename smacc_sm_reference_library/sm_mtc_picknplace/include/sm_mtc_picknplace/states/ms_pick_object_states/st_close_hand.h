namespace sm_mtc_picknplace
{
// STATE DECLARATION
struct StCloseHand : smacc::SmaccState<StCloseHand, MsPickObject>
{
    using SmaccState::SmaccState;

// TRANSITION TABLE
    typedef mpl::list<
        
    Transition<EvKeyPressN<CbDefaultKeyboardBehavior, OrKeyboard>, MsPlaceObject, PREEMPT>,
    Transition<EvTimer<CbTimerCountdownOnce, OrTimer>, MsPlaceObject, SUCCESS>
    
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
        client->onTimerTick(&StCloseHand::onTimerClientTickCallback, this);

        // getting reference to the single countdown behavior
        auto *cbsingle = this->getOrthogonal<OrTimer>()
                             ->getClientBehavior<CbTimerCountdownOnce>();

        // subscribe to the single countdown behavior callback
        cbsingle->onTimerTick(&StCloseHand::onSingleBehaviorTickCallback, this);
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