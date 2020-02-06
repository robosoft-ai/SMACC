namespace sm_calendar_week
{
// STATE DECLARATION
struct StSunday : smacc::SmaccState<StSunday, MsWeekend>
{
    using SmaccState::SmaccState;

// DECLARE CUSTOM OBJECT TAGS
    struct RESET : SUCCESS{};

// TRANSITION TABLE
    typedef mpl::list<
        
    Transition<EvKeyPressN<CbDefaultKeyboardBehavior, OrKeyboard>, MsWorkweek, PREEMPT>,
    Transition<EvTimer<CbTimerCountdownOnce, OrTimer>, MsWorkweek, SUCCESS> //,
    
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
        client->onTimerTick(&StSunday::onTimerClientTickCallback, this);

        // getting reference to the single countdown behavior
        auto *cbsingle = this->getOrthogonal<OrTimer>()
                             ->getClientBehavior<CbTimerCountdownOnce>();

        // subscribe to the single countdown behavior callback
        cbsingle->onTimerTick(&StSunday::onSingleBehaviorTickCallback, this);
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
} // namespace sm_calendar_week